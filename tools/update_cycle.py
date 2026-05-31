#!/usr/bin/env python3
"""Board-safe recurring Mazge update workflow.

Sequence:
1. List remote folders and compute new recent folders.
2. Pull recent meta.json gently.
3. Pull missing JPGs gently for recent folders.
4. Rebuild dataset and refresh reports/statistics.
5. Launch labeling phases sequentially.
6. Launch cat-ID review (Benis/Mazge) as a final manual phase.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import shutil
import socket
import subprocess
import sys
import threading
import time
import urllib.request
import webbrowser
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SD = REPO / "captures" / "sd"
CAPTURES = REPO / "captures"
TOOLS = REPO / "tools"
HOST = "192.168.0.41"
LABEL_PORT = 8765
CATID_PORT = 8766


def run(cmd: list[str], *, cwd: Path = REPO, stdout_path: Path | None = None) -> None:
    print("$", " ".join(cmd), flush=True)
    if stdout_path is None:
        subprocess.run(cmd, cwd=cwd, check=True)
        return
    stdout_path.parent.mkdir(parents=True, exist_ok=True)
    with stdout_path.open("w") as f:
        subprocess.run(cmd, cwd=cwd, check=True, stdout=f)


def board_reachable(host: str, port: int = 80, timeout: float = 3.0) -> bool:
    """Quick TCP connect check — returns False instead of blocking on a dead board."""
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def read_with_deadline(response: object, deadline_s: float) -> bytes:
    """Read an HTTP response body with a hard wall-clock deadline.

    urllib's socket timeout governs individual recv() calls, not total transfer
    time.  If the ESP sends data in slow trickles the socket timeout never fires
    even though the overall read takes minutes.  This wrapper runs the read in a
    daemon thread and closes the response socket if the deadline elapses, which
    unblocks the thread immediately.
    """
    result: list[bytes] = []
    exc_box: list[BaseException] = []

    def _read() -> None:
        try:
            result.append(response.read())  # type: ignore[attr-defined]
        except BaseException as e:  # noqa: BLE001
            exc_box.append(e)

    t = threading.Thread(target=_read, daemon=True)
    t.start()
    t.join(deadline_s)
    if t.is_alive():
        try:
            response.close()  # type: ignore[attr-defined]
        except Exception:  # noqa: BLE001
            pass
        t.join(2)  # give the thread a moment to notice the close
        raise TimeoutError(f"response.read() exceeded {deadline_s:.0f}s deadline")
    if exc_box:
        raise exc_box[0]
    return result[0]


def fetch_remote_folders(host: str, port: int = 80, per_recv_timeout: float = 8.0,
                         wall_clock_limit: float = 150.0) -> list[str]:
    """/sdfolders streams ~1 KB/s as tiny HTTP chunks.  Waiting for complete JSON
    before parsing means any hiccup kills the whole call.  Instead we stream-parse:
    open a raw socket, read with short per-recv timeouts, accumulate bytes, and
    extract folder names via regex as they arrive.  We return whatever we collected
    if the connection closes normally OR if the wall-clock limit is reached (partial
    list is better than nothing for the missing-folder diff).
    """
    import re as _re
    request = (
        f"GET /sdfolders HTTP/1.1\r\n"
        f"Host: {host}\r\n"
        f"Connection: close\r\n"
        f"\r\n"
    ).encode()

    start = time.monotonic()
    buf = bytearray()
    sock = socket.create_connection((host, port), timeout=per_recv_timeout)
    try:
        sock.settimeout(per_recv_timeout)
        sock.sendall(request)
        while True:
            if time.monotonic() - start > wall_clock_limit:
                print(
                    f"fetch_remote_folders: wall-clock limit {wall_clock_limit:.0f}s reached, "
                    f"using partial response ({len(buf)} bytes)",
                    flush=True,
                )
                break
            try:
                chunk = sock.recv(4096)
            except TimeoutError:
                # No data for per_recv_timeout seconds — board stalled or done
                print(
                    f"fetch_remote_folders: recv timeout after {time.monotonic()-start:.0f}s "
                    f"({len(buf)} bytes so far)",
                    flush=True,
                )
                break
            if not chunk:
                break  # clean EOF
            buf += chunk
    finally:
        try:
            sock.close()
        except Exception:  # noqa: BLE001
            pass

    elapsed = time.monotonic() - start
    # Strip HTTP headers (everything before first blank line)
    raw = bytes(buf)
    header_end = raw.find(b"\r\n\r\n")
    body = raw[header_end + 4:] if header_end != -1 else raw
    # HTTP/1.1 chunked: strip chunk-size lines (hex\r\n ... \r\n)
    # Simple approach: extract all quoted strings that look like burst folder names
    folders = _re.findall(rb'"([^"]{8,50})"', body)
    result = [f.decode(errors="replace") for f in folders
              if len(f) >= 8 and (f[:8].isdigit() or f.startswith(b"burst_"))]
    print(
        f"fetch_remote_folders: {len(result)} folders in {elapsed:.1f}s "
        f"({len(buf)} bytes)",
        flush=True,
    )
    if not result:
        raise RuntimeError(f"fetch_remote_folders: got {len(buf)} bytes but found no folder names")
    return result


def is_ts_folder(name: str) -> bool:
    return len(name) >= 15 and name[8] == "_" and name[:8].isdigit() and name[9:15].isdigit()


def compute_bounds(hours: int, since: str | None, until: str | None) -> tuple[str, str | None]:
    if since:
        since_prefix = since
    else:
        since_prefix = (dt.datetime.now() - dt.timedelta(hours=hours)).strftime("%Y%m%d_%H%M%S")
    return since_prefix, until


def list_recent_missing(host: str, since_prefix: str, until_prefix: str | None) -> list[str]:
    remote = fetch_remote_folders(host)
    local = {d.name for d in SD.iterdir() if d.is_dir()}
    missing = []
    for folder in sorted(remote):
        if not is_ts_folder(folder):
            continue
        if folder[:15] < since_prefix:
            continue
        if until_prefix and folder[:8] > until_prefix:
            continue
        if folder not in local:
            missing.append(folder)
    print(f"remote_total={len(remote)} local_total={len(local)} missing_recent={len(missing)}", flush=True)
    for folder in missing:
        print(f"  missing: {folder}", flush=True)
    return missing


def pull_meta_gently(host: str, folders: list[str]) -> None:
    ok = err = 0
    for i, folder in enumerate(folders, 1):
        dst = SD / folder
        dst.mkdir(parents=True, exist_ok=True)
        url = f"http://{host}/sdget?f={folder}/meta.json"
        success = False
        for attempt in range(3):
            try:
                with urllib.request.urlopen(url, timeout=12) as response:
                    data = read_with_deadline(response, deadline_s=20)
                (dst / "meta.json").write_bytes(data)
                meta = json.loads(data)
                print(
                    f"  [{i}/{len(folders)}] {folder} prey={meta.get('apiResult')} dir={meta.get('direction')} imgs={len(meta.get('images', []))}",
                    flush=True,
                )
                ok += 1
                success = True
                break
            except Exception as exc:  # noqa: BLE001
                if attempt == 2:
                    err += 1
                    print(f"  [{i}/{len(folders)}] {folder} ERR {exc}", flush=True)
                else:
                    time.sleep(1.5 * (attempt + 1))
        if success:
            time.sleep(0.6)
    print(f"meta_pull_ok={ok} err={err}", flush=True)


def recent_local_folders(since_prefix: str, until_prefix: str | None) -> list[Path]:
    out = []
    for d in sorted(SD.iterdir()):
        if not d.is_dir() or not is_ts_folder(d.name):
            continue
        if d.name[:15] < since_prefix:
            continue
        if until_prefix and d.name[:8] > until_prefix:
            continue
        if (d / "meta.json").exists():
            out.append(d)
    return out


def fetch_with_retries(url: str, retries: int = 3, timeout: float = 15.0) -> bytes:
    last_exc: Exception | None = None
    for attempt in range(1, retries + 1):
        try:
            with urllib.request.urlopen(url, timeout=timeout) as response:
                return read_with_deadline(response, deadline_s=timeout * 1.5)
        except Exception as exc:  # noqa: BLE001
            last_exc = exc
            if attempt < retries:
                time.sleep(1.2 * attempt)
    raise RuntimeError(str(last_exc))


def pull_recent_jpgs_gently(host: str, since_prefix: str, until_prefix: str | None) -> None:
    burst_count = file_count = byte_count = error_count = 0
    for folder in recent_local_folders(since_prefix, until_prefix):
        try:
            meta = json.loads((folder / "meta.json").read_text())
        except Exception:
            continue
        image_names = [x.get("f") for x in meta.get("images", []) if x.get("f")]
        existing = {p.name for p in folder.glob("*.jpg")}
        missing = [name for name in image_names if name not in existing]
        if not missing:
            continue
        burst_count += 1
        print(f"  {folder.name}: pulling {len(missing)} jpgs", flush=True)
        for name in missing:
            url = f"http://{host}/sdget?f={folder.name}/{name}"
            try:
                data = fetch_with_retries(url)
                if len(data) < 1000:
                    raise RuntimeError(f"short_response:{len(data)}")
                (folder / name).write_bytes(data)
                file_count += 1
                byte_count += len(data)
            except Exception as exc:  # noqa: BLE001
                error_count += 1
                print(f"    ERR {name}: {exc}", flush=True)
            time.sleep(0.35)
    print(
        f"jpg_sync_bursts={burst_count} files={file_count} bytes={byte_count} errors={error_count}",
        flush=True,
    )


def ensure_prey_review_source() -> None:
    review = CAPTURES / "prey_review"
    review.mkdir(parents=True, exist_ok=True)

    prey_bursts = []
    for d in sorted(SD.iterdir()):
        if not d.is_dir():
            continue
        meta_path = d / "meta.json"
        if not meta_path.exists():
            continue
        try:
            meta = json.loads(meta_path.read_text())
        except Exception:
            continue
        if meta.get("apiResult") == 1:
            prey_bursts.append(d.name)

    extra = []
    fn_path = CAPTURES / "human_vs_fw_false_negatives_last48h.json"
    if fn_path.exists():
        try:
            extra = [r.get("burst_id") for r in json.loads(fn_path.read_text()) if r.get("burst_id")]
        except Exception:
            extra = []

    selected = sorted(set(prey_bursts + extra))
    copied = 0
    for burst_id in selected:
        src = SD / burst_id
        dst = review / burst_id
        if not src.exists():
            continue
        dst.mkdir(parents=True, exist_ok=True)
        for f in src.iterdir():
            if not f.is_file():
                continue
            out = dst / f.name
            if out.exists() and out.stat().st_size == f.stat().st_size:
                continue
            shutil.copy2(f, out)
            copied += 1
    print(f"prey_review_selected={len(selected)} files_copied={copied}", flush=True)


def write_catid_pending(since_prefix: str, until_prefix: str | None) -> Path:
    rows = list(csv.DictReader((REPO / "dataset" / "bursts.csv").open()))
    pending = []
    idx = 0
    ts = dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds")
    for row in rows:
        burst_id = row.get("burst_id", "")
        if len(burst_id) < 8 or not burst_id[:8].isdigit():
            continue
        if burst_id[:15] < since_prefix:
            continue
        if until_prefix and burst_id[:8] > until_prefix:
            continue
        if row.get("human_subject") != "cat":
            continue
        if row.get("cat_id"):
            continue
        burst_dir = SD / burst_id
        if not burst_dir.exists():
            continue
        frame = None
        for cand in sorted(burst_dir.glob("f05*.jpg")):
            frame = cand
            break
        if frame is None:
            jpgs = sorted(burst_dir.glob("f*.jpg"))
            if not jpgs:
                continue
            frame = jpgs[min(len(jpgs) - 1, 5)]
        pending.append(
            {
                "idx": idx,
                "burst_id": burst_id,
                "frame_path": str(frame.relative_to(REPO)).replace("\\", "/"),
                "frame_file": frame.name,
                "auto_label": "unknown",
                "label": "unknown",
                "ts": ts,
            }
        )
        idx += 1
    out = TOOLS / "catid_pending.json"
    out.write_text(json.dumps(pending, indent=2))
    print(f"catid_pending={len(pending)} -> {out}", flush=True)
    return out


def maybe_import_catid_export() -> bool:
    if not sys.stdin.isatty():
        return False

    default_path = Path.home() / "Downloads" / "catid_labels.jsonl"
    prompt = (
        f"Import cat-ID export now? Enter path [{default_path}] or press Enter to skip: "
    )
    try:
        entered = input(prompt).strip()
    except EOFError:
        return False

    if entered.lower() in {"", "skip", "no"}:
        return False

    src = Path(entered) if entered else default_path
    if not src.exists():
        print(f"cat-id export not found: {src}", flush=True)
        return False

    labels_path = REPO / "dataset" / "labels.jsonl"
    existing = set()
    if labels_path.exists():
        with labels_path.open() as f:
            existing = {line.rstrip("\n") for line in f}

    added = 0
    with src.open() as inp, labels_path.open("a") as out:
        for line in inp:
            rec = line.rstrip("\n")
            if not rec or rec in existing:
                continue
            out.write(rec + "\n")
            existing.add(rec)
            added += 1

    print(f"imported_catid_labels={added} from {src}", flush=True)
    return added > 0


def run_reports_and_stats() -> None:
    run(["uv", "run", "python", "tools/build_dataset.py"])
    ensure_prey_review_source()
    run(["uv", "run", "python", "tools/prey_review_html.py"])
    run(["uv", "run", "python", "tools/prey_review_pdf.py"])
    run(["uv", "run", "python", "tools/cat_activity_html_report.py"])
    run(["uv", "run", "python", "tools/cat_activity_report.py"], stdout_path=CAPTURES / "cat_activity_report.txt")
    run(["uv", "run", "python", "tools/show_human_labels.py"], stdout_path=CAPTURES / "human_label_summary.txt")
    run(["uv", "run", "python", "tools/show_training_stats.py"], stdout_path=CAPTURES / "training_stats.txt")
    if (TOOLS / "report_false_negatives_last48h.py").exists():
        run(["uv", "run", "python", "tools/report_false_negatives_last48h.py"])
    if (TOOLS / "summarize_non_prey_last48h.py").exists():
        run(["uv", "run", "python", "tools/summarize_non_prey_last48h.py"])


def wait_for_url(url: str, timeout_s: float = 20.0) -> None:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            with urllib.request.urlopen(url, timeout=2):
                return
        except Exception:
            time.sleep(0.5)


def _free_port(port: int) -> None:
    """Kill any process currently listening on *port* so a fresh server can bind."""
    import subprocess as _sp
    try:
        result = _sp.run(
            ["lsof", "-ti", f"tcp:{port}"],
            capture_output=True, text=True,
        )
        pids = result.stdout.split()
        if pids:
            _sp.run(["kill", "-9"] + pids, capture_output=True)
            time.sleep(0.5)
            print(f"freed port {port} (killed pids: {' '.join(pids)})", flush=True)
    except Exception:  # noqa: BLE001
        pass


def run_server_phase(cmd: list[str], url: str, title: str, cwd: Path = REPO, open_browser: bool = True) -> None:
    print(f"\n=== {title} ===", flush=True)
    print("Stop the server with Ctrl+C when you are done with this phase.", flush=True)
    # Free the port before binding so stale servers from prior runs never block us.
    import urllib.parse as _up
    _parsed = _up.urlparse(url)
    if _parsed.port:
        _free_port(_parsed.port)

    def open_later() -> None:
        wait_for_url(url)
        webbrowser.open(url)

    timer = None
    if open_browser:
        timer = threading.Thread(target=open_later, daemon=True)
        timer.start()

    subprocess.run(cmd, cwd=cwd, check=False)


def run_labeling_phases(since_date: str | None, until_date: str | None, open_browser: bool) -> None:
    cmd = ["uv", "run", "python", "tools/label_bursts.py", "--filter", "prey-positive", "--port", str(LABEL_PORT)]
    if since_date:
        cmd += ["--since", since_date]
    if until_date:
        cmd += ["--until", until_date]
    run_server_phase(cmd, f"http://127.0.0.1:{LABEL_PORT}/", "Label prey-positive bursts", open_browser=open_browser)

    cmd = ["uv", "run", "python", "tools/label_bursts.py", "--filter", "unlabelled", "--port", str(LABEL_PORT)]
    if since_date:
        cmd += ["--since", since_date]
    if until_date:
        cmd += ["--until", until_date]
    run_server_phase(
        cmd,
        f"http://127.0.0.1:{LABEL_PORT}/",
        "Label remaining no-prey / direction / subject bursts",
        open_browser=open_browser,
    )

    write_catid_pending((since_date or "00000000") + "_000000", until_date)
    catid_url = f"http://127.0.0.1:{CATID_PORT}/tools/cat_id_review.html?data=tools/catid_pending.json"
    run_server_phase(
        [sys.executable, "-m", "http.server", str(CATID_PORT), "--directory", str(REPO)],
        catid_url,
        "Review Benis / Mazge cat IDs",
        cwd=REPO,
        open_browser=open_browser,
    )
    if maybe_import_catid_export():
        print("\n=== Refresh after cat-ID import ===", flush=True)
        run_reports_and_stats()
    else:
        print(
            "Cat-ID export not imported automatically. If you exported catid_labels.jsonl later, rerun with --skip-sync --skip-labeling to refresh reports.",
            flush=True,
        )


def main() -> None:
    ap = argparse.ArgumentParser(description="Run the recurring Mazge update workflow safely")
    ap.add_argument("--host", default=HOST)
    ap.add_argument("--hours", type=int, default=72, help="Recent window for sync and labeling")
    ap.add_argument("--since", help="YYYYMMDD override for labeling/report window")
    ap.add_argument("--until", help="YYYYMMDD inclusive upper bound")
    ap.add_argument("--skip-sync", action="store_true", help="Do not touch the board; use local files only")
    ap.add_argument("--skip-labeling", action="store_true", help="Prepare data and reports only")
    ap.add_argument("--no-browser", action="store_true", help="Do not auto-open browser tabs for labeling phases")
    args = ap.parse_args()

    since_prefix, until_prefix = compute_bounds(args.hours, args.since, args.until)
    since_date = args.since or since_prefix[:8]

    print("=== Mazge update cycle ===", flush=True)
    print(f"window_since={since_prefix} until={until_prefix or '(none)'}", flush=True)

    if not args.skip_sync:
        print("\n=== Board connectivity check ===", flush=True)
        if not board_reachable(args.host):
            print(
                f"WARNING: {args.host} unreachable (TCP connect timed out). "
                "Skipping SD sync — rerun without --skip-sync when board is online.",
                flush=True,
            )
        else:
            print(f"Board {args.host} reachable. Starting sync.", flush=True)
            try:
                missing = list_recent_missing(args.host, since_prefix, until_prefix)
                if missing:
                    print("\n=== Pull recent metadata ===", flush=True)
                    pull_meta_gently(args.host, missing)
                else:
                    print("\nNo new recent folders to sync.", flush=True)
                print("\n=== Pull missing JPGs for recent folders ===", flush=True)
                pull_recent_jpgs_gently(args.host, since_prefix, until_prefix)
            except Exception as exc:  # noqa: BLE001
                print(
                    f"\nWARNING: Sync failed ({exc}). "
                    "Continuing with local data for reports and labeling.",
                    flush=True,
                )

    print("\n=== Refresh reports, statistics, dashboard inputs ===", flush=True)
    run_reports_and_stats()

    if not args.skip_labeling:
        run_labeling_phases(since_date, args.until, open_browser=not args.no_browser)

    print("\nUpdate cycle complete.", flush=True)


if __name__ == "__main__":
    main()
