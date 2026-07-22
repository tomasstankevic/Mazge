"""Compare server v2 decisions to local YOLO11n + human labels for last 24h.

Joins three sources per (burst_id, frame_idx):
  1. server JSONL log: cat_id, prey_score, door_action, severity
  2. local manifest.csv: yolo_subject, human_subject, has_jpg
  3. local bursts.csv: human_prey, human_direction (per-burst rollup)

Outputs:
  - per-frame: server cat_id vs YOLO subject (confusion)
  - per-burst: server modal cat_id vs human burst-level cat (if labelled)
  - "unknown" frames analysis: what does YOLO say for frames the server skipped?
  - prey disagreements
"""

from __future__ import annotations

import csv
import datetime as dt
import json
import re
from collections import Counter, defaultdict
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
LOG_DIR = REPO / "logs" / "server"
MANIFEST = REPO / "dataset" / "manifest.csv"
BURSTS = REPO / "dataset" / "bursts.csv"

CUTOFF_HOURS = 24


def load_log() -> list[dict]:
    rows = []
    for p in sorted(LOG_DIR.glob("server.jsonl*")):
        with p.open() as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    r = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if r.get("ep") != "/v2/frame":
                    continue
                if r.get("status_code") != 200:
                    continue
                rows.append(r)
    return rows


# burst_id in the server log is "<triggerMs>_genN".
# burst_id in the manifest is "YYYYMMDD_HHMMSS_genN".
# Bridge: extract _genN suffix and the ts_ms of the FIRST frame in each server burst,
# match against manifest burst dirs by gen + closest timestamp.

def normalize_log(rows: list[dict]) -> dict:
    """Return dict (canonical_burst_key, frame_idx) -> log_row."""
    # Group by raw burst_id, take min ts_ms per burst as the canonical trigger time.
    by_raw: dict[str, list[dict]] = defaultdict(list)
    for r in rows:
        by_raw[r["burst_id"]].append(r)
    out: dict = {}
    burst_meta: dict[str, dict] = {}  # raw burst_id -> {trigger_dt, gen}
    for raw, frames in by_raw.items():
        ts0 = min(f["ts_ms"] for f in frames)
        m = re.match(r"(\d+)_gen(\d+)", raw)
        gen = int(m.group(2)) if m else -1
        burst_meta[raw] = {
            "trigger_ms": ts0,
            "trigger_dt": dt.datetime.fromtimestamp(ts0 / 1000, dt.UTC),
            "gen": gen,
            "n_frames": len(frames),
        }
        for f in frames:
            out[(raw, f["frame_index"])] = f
    return {"frames": out, "bursts": burst_meta}


LOCAL_TZ_OFFSET_HOURS = 2  # ESP32 logs CEST timestamps in dir names; server log is UTC


def burst_id_from_dir(name: str) -> tuple[dt.datetime, int] | None:
    m = re.match(r"(\d{8})_(\d{6})_gen(\d+)", name)
    if not m:
        return None
    d, t, g = m.groups()
    try:
        local = dt.datetime.strptime(d + t, "%Y%m%d%H%M%S")
        as_utc = local.replace(tzinfo=dt.UTC) - dt.timedelta(hours=LOCAL_TZ_OFFSET_HOURS)
        return as_utc, int(g)
    except ValueError:
        return None


def match_server_to_local(server_bursts: dict, local_bursts: list[str]) -> dict[str, str]:
    """Match raw server burst_id -> local burst dir name by gen + closest time (<=120s)."""
    mapping: dict[str, str] = {}
    local_parsed = []
    for name in local_bursts:
        p = burst_id_from_dir(name)
        if p:
            local_parsed.append((p[0], p[1], name))
    for raw, meta in server_bursts.items():
        gen = meta["gen"]
        ts = meta["trigger_dt"]
        best = None
        best_dt = dt.timedelta(seconds=120)
        for ldt, lgen, lname in local_parsed:
            if lgen != gen:
                continue
            delta = abs(ldt - ts)
            if delta <= best_dt:
                best_dt = delta
                best = lname
        if best:
            mapping[raw] = best
    return mapping


def load_manifest() -> dict[tuple[str, int], dict]:
    out = {}
    if not MANIFEST.exists():
        return out
    with MANIFEST.open() as fh:
        rdr = csv.DictReader(fh)
        for row in rdr:
            try:
                fi = int(row["frame_idx"])
            except (KeyError, ValueError):
                continue
            out[(row["burst_id"], fi)] = row
    return out


def load_bursts_csv() -> dict[str, dict]:
    out = {}
    if not BURSTS.exists():
        return out
    with BURSTS.open() as fh:
        rdr = csv.DictReader(fh)
        for row in rdr:
            out[row["burst_id"]] = row
    return out


def main() -> None:
    rows = load_log()
    norm = normalize_log(rows)
    print(f"server log: {len(rows)} v2 frames in {len(norm['bursts'])} bursts")

    # Filter to last CUTOFF_HOURS
    now = dt.datetime.now(dt.UTC)
    cutoff = now - dt.timedelta(hours=CUTOFF_HOURS)
    recent = {k: v for k, v in norm["bursts"].items() if v["trigger_dt"] >= cutoff}
    print(f"last {CUTOFF_HOURS}h: {len(recent)} bursts")

    manifest = load_manifest()
    bursts_csv = load_bursts_csv()
    print(f"manifest: {len(manifest)} frames; bursts.csv: {len(bursts_csv)} rows")

    # Filter local burst dirs to last 24h based on parsed timestamp.
    local_recent = []
    for name in bursts_csv:
        p = burst_id_from_dir(name)
        if p and p[0] >= cutoff:
            local_recent.append(name)
    print(f"local bursts (last 24h, parsed from name): {len(local_recent)}")

    mapping = match_server_to_local(recent, local_recent)
    print(f"server <-> local match (gen+time <=120s): {len(mapping)} bursts")
    unmatched = [b for b in recent if b not in mapping]
    if unmatched:
        print(f"unmatched server bursts: {len(unmatched)} (showing first 5)")
        for b in unmatched[:5]:
            m = recent[b]
            print(f"  {b}  trigger={m['trigger_dt'].isoformat(timespec='seconds')}  n_frames={m['n_frames']}")

    # === Per-frame: server cat_id vs YOLO subject ===
    confusion: Counter = Counter()
    server_unknown_yolo: Counter = Counter()
    yolo_cat_server_says: Counter = Counter()
    for (raw, fi), srv in norm["frames"].items():
        local_burst = mapping.get(raw)
        if not local_burst:
            continue
        mrow = manifest.get((local_burst, fi))
        if not mrow:
            continue
        yolo = mrow.get("yolo_subject", "")
        # yolo_subject in manifest is the YOLO-derived per-frame label (0..3 or label name)
        # Normalise to {empty, cat, human, other}.
        yolo_label = yolo if yolo in ("empty", "cat", "human", "other") else ""
        if not yolo_label:
            try:
                yi = int(float(yolo))
                yolo_label = ["empty", "cat", "human", "other"][yi]
            except Exception:
                yolo_label = "?"
        srv_cat = srv.get("cat_id") or "unknown"
        confusion[(srv_cat, yolo_label)] += 1
        if srv_cat == "unknown":
            server_unknown_yolo[yolo_label] += 1
        if yolo_label == "cat":
            yolo_cat_server_says[srv_cat] += 1

    print("\n=== per-frame: server cat_id × YOLO11n subject ===")
    print(f"{'server_cat':>10} {'yolo':>8} {'n':>6}")
    for (sc, yl), n in sorted(confusion.items(), key=lambda x: -x[1])[:20]:
        print(f"{sc:>10} {yl:>8} {n:>6}")

    print("\n=== When server says cat_id=unknown, YOLO says: ===")
    total = sum(server_unknown_yolo.values())
    for k, v in server_unknown_yolo.most_common():
        pct = 100.0 * v / total if total else 0
        print(f"  {k:>8} {v:>5}  ({pct:5.1f}%)")

    print("\n=== When YOLO says cat, server says: ===")
    total = sum(yolo_cat_server_says.values())
    for k, v in yolo_cat_server_says.most_common():
        pct = 100.0 * v / total if total else 0
        print(f"  {k:>8} {v:>5}  ({pct:5.1f}%)")

    # === Per-burst: server modal cat_id vs human label ===
    print("\n=== per-burst: server modal cat_id × human_subject ===")
    per_burst_srv = defaultdict(Counter)  # raw_burst -> Counter(cat_id)
    for (raw, fi), srv in norm["frames"].items():
        if raw not in mapping:
            continue
        if srv.get("cat_recognized"):
            per_burst_srv[raw][srv.get("cat_id")] += 1
        else:
            per_burst_srv[raw]["unknown"] += 1

    burst_confusion = Counter()
    for raw, counts in per_burst_srv.items():
        local = mapping[raw]
        # Modal recognized cat for this burst (ignore "unknown" if any cat present)
        recd = Counter({k: v for k, v in counts.items() if k != "unknown"})
        if recd:
            srv_cat = recd.most_common(1)[0][0]
        else:
            srv_cat = "unknown"
        human_subj = bursts_csv.get(local, {}).get("human_subject", "")
        # human_subject is "cat"/"empty"/"human"/"other"/"unclear" but doesn't say WHICH cat
        burst_confusion[(srv_cat, human_subj)] += 1

    print(f"{'server_cat':>10} {'human':>8} {'n':>6}")
    for (sc, hs), n in sorted(burst_confusion.items(), key=lambda x: -x[1]):
        print(f"{sc:>10} {hs:>8} {n:>6}")

    # === Prey disagreement ===
    print("\n=== bursts with server prey_score>=0.5 OR human_prey=1 ===")
    # Pre-group frames by raw burst for efficiency
    frames_by_raw: dict[str, list[tuple[int, dict]]] = defaultdict(list)
    for (raw, fi), srv in norm["frames"].items():
        frames_by_raw[raw].append((fi, srv))
    flagged_bursts = []
    for raw, frames in per_burst_srv.items():
        local = mapping[raw]
        srv_frames = sorted(frames_by_raw[raw], key=lambda x: x[0])
        max_prey = max((f[1].get("prey_score") or 0) for f in srv_frames)
        b_row = bursts_csv.get(local, {})
        h_prey = b_row.get("human_prey", "")
        if max_prey >= 0.5 or h_prey == "1":
            flagged_bursts.append((local, raw, max_prey, h_prey, b_row, srv_frames))
            print(f"  {local}  server_max_prey={max_prey:.3f}  human_prey={h_prey!r}  human_subj={b_row.get('human_subject', '')!r}")

    # Drill into each flagged burst: per-frame scores + door actions
    for local, raw, max_prey, h_prey, b_row, srv_frames in flagged_bursts:
        print(f"\n  --- {local} ({raw}) ---")
        print(f"    human_subject={b_row.get('human_subject','')!r}  human_prey={h_prey!r}  human_direction={b_row.get('human_direction','')!r}")
        print(f"    {'fi':>3} {'prey':>6} {'sev':>8} {'cat':>8} {'cat_conf':>8} {'door':>7} {'reason':>30}")
        for fi, srv in srv_frames:
            print(f"    {fi:>3} {srv.get('prey_score',0):>6.3f} {str(srv.get('severity','')):>8} {str(srv.get('cat_id','')):>8} {srv.get('cat_confidence',0):>8.3f} {str(srv.get('door_action','')):>7} {str(srv.get('reason',''))[:30]:>30}")

    # === Bursts where server said "unknown" but human said "cat" ===
    print("\n=== bursts: server modal=unknown, human=cat (cat-detection misses) ===")
    for raw, frames in per_burst_srv.items():
        local = mapping[raw]
        recd = Counter({k: v for k, v in frames.items() if k != "unknown"})
        srv_cat = recd.most_common(1)[0][0] if recd else "unknown"
        if srv_cat != "unknown":
            continue
        b_row = bursts_csv.get(local, {})
        if b_row.get("human_subject") != "cat":
            continue
        srv_frames = sorted(frames_by_raw[raw], key=lambda x: x[0])
        n_frames = len(srv_frames)
        n_with_cat_attempt = sum(1 for _, s in srv_frames if (s.get("cat_softmax") or [0, 0])[1] > 0.01)
        print(f"  {local}  n_frames={n_frames}  any-body-detected={n_with_cat_attempt}")
        for fi, srv in srv_frames:
            cs = srv.get("cat_softmax") or [0, 0]
            print(f"    fi={fi}  prey={srv.get('prey_score',0):.3f}  cat_softmax=[{cs[0]:.3f},{cs[1]:.3f}]  cat_conf={srv.get('cat_confidence',0):.3f}  reason={srv.get('reason','')}")

    # === Cat-ID split for cat bursts ===
    print("\n=== per-burst breakdown for HUMAN=cat bursts (cat-ID skew check) ===")
    print(f"  {'burst':>30} {'modal':>7} {'mazge':>5} {'benis':>5} {'unk':>5} {'max_cat_conf':>12}")
    for raw, frames in per_burst_srv.items():
        local = mapping[raw]
        b_row = bursts_csv.get(local, {})
        if b_row.get("human_subject") != "cat":
            continue
        srv_frames = frames_by_raw[raw]
        max_cat_conf = max(s.get("cat_confidence", 0) for _, s in srv_frames)
        recd = Counter({k: v for k, v in frames.items() if k != "unknown"})
        modal = recd.most_common(1)[0][0] if recd else "unknown"
        print(f"  {local:>30} {modal:>7} {frames.get('mazge',0):>5} {frames.get('benis',0):>5} {frames.get('unknown',0):>5} {max_cat_conf:>12.3f}")


if __name__ == "__main__":
    main()
