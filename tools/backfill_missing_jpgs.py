"""Backfill missing JPGs for burst folders that only have meta.json locally.

Walks captures/sd/ and for each folder where meta.json lists images that
don't exist on disk, downloads them from the device via /sdget. Works
folder-by-folder so a crash mid-run is recoverable.
"""
from __future__ import annotations

import argparse
import json
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent / "captures" / "sd"
HOST_DEFAULT = "192.168.0.41"


def missing_for_folder(folder: Path) -> list[str]:
    meta_p = folder / "meta.json"
    if not meta_p.exists():
        return []
    try:
        m = json.loads(meta_p.read_text())
    except Exception:
        return []
    out = []
    for img in m.get("images", []):
        name = img.get("f")
        if not name:
            continue
        if not (folder / name).exists():
            out.append(name)
    return out


def download(host: str, remote: str, dst: Path, retries: int = 3) -> int:
    url = f"http://{host}/sdget?f={remote}"
    for attempt in range(retries):
        try:
            with urllib.request.urlopen(url, timeout=60) as r:
                data = r.read()
            dst.parent.mkdir(parents=True, exist_ok=True)
            dst.write_bytes(data)
            return len(data)
        except Exception as e:
            if attempt == retries - 1:
                print(f"    ERR {remote}: {e}", flush=True)
                return 0
            time.sleep(1.0 * (attempt + 1))
    return 0


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=HOST_DEFAULT)
    ap.add_argument("--limit", type=int, default=0,
                    help="Stop after N folders (0 = all)")
    args = ap.parse_args()

    folders = sorted(d for d in ROOT.iterdir() if d.is_dir())
    todo: list[tuple[Path, list[str]]] = []
    for d in folders:
        miss = missing_for_folder(d)
        if miss:
            todo.append((d, miss))

    total_files = sum(len(m) for _, m in todo)
    print(f"Folders needing backfill: {len(todo)}  (total files: {total_files})",
          flush=True)
    if not todo:
        return

    if args.limit:
        todo = todo[:args.limit]

    grand_bytes = 0
    grand_count = 0
    t0 = time.time()
    for i, (folder, names) in enumerate(todo, 1):
        f_t0 = time.time()
        f_bytes = 0
        ok = 0
        for name in names:
            n = download(args.host, f"{folder.name}/{name}", folder / name)
            if n > 0:
                ok += 1
                f_bytes += n
        grand_bytes += f_bytes
        grand_count += ok
        elapsed = time.time() - t0
        rate = (grand_bytes / 1024) / elapsed if elapsed > 0 else 0
        f_dur = time.time() - f_t0
        print(f"  [{i}/{len(todo)}] {folder.name}: {ok}/{len(names)} files "
              f"({f_bytes//1024}KB in {f_dur:.1f}s)  -- avg {rate:.1f}KB/s",
              flush=True)

    dur = time.time() - t0
    print(f"\nDone. {grand_count} files, {grand_bytes//1024}KB in {dur:.0f}s "
          f"({(grand_bytes/1024)/dur:.1f}KB/s)")


if __name__ == "__main__":
    main()
