"""Aggregate server log stats — bursts, latency, cat IDs, hourly distribution."""

import argparse
import glob
import json
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--log-glob",
        default="logs/server/server.jsonl*",
        help="glob for JSONL log files",
    )
    args = parser.parse_args()

    reqs: list[dict] = []
    for path in sorted(glob.glob(args.log_glob)):
        with open(path) as fh:
            for line in fh:
                line = line.strip()
                if not line.startswith("{"):
                    continue
                try:
                    r = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if r.get("ep") in ("/v2/frame", "/v1/compat"):
                    reqs.append(r)

    print(f"TOTAL: {len(reqs)} inference requests")
    if not reqs:
        return

    # Bursts
    bursts: dict[tuple[str, str], list[dict]] = defaultdict(list)
    for r in reqs:
        bursts[(r.get("device_id", ""), r.get("burst_id", ""))].append(r)
    print(f"BURSTS: {len(bursts)}")

    cat_bursts = 0
    prey_bursts = 0
    allow_bursts = 0
    deny_bursts = 0
    cat_counter: Counter[str] = Counter()
    deny_no_cat = 0
    for (_dev, _bid), frames in bursts.items():
        any_cat = any(f.get("cat_recognized") for f in frames)
        any_prey = any(f.get("detected") for f in frames)
        if any_cat:
            cat_bursts += 1
            ids = Counter(
                f.get("cat_id") for f in frames if f.get("cat_recognized")
            )
            cat_counter[ids.most_common(1)[0][0]] += 1
        if any_prey:
            prey_bursts += 1
        da = Counter(f.get("door_action") for f in frames).most_common(1)[0][0]
        if da == "allow":
            allow_bursts += 1
        else:
            deny_bursts += 1
            if not any_cat:
                deny_no_cat += 1

    print(f"  cat recognized:  {cat_bursts}")
    print(f"  door allow:      {allow_bursts}")
    print(f"  door deny:       {deny_bursts}  (of which {deny_no_cat} had no cat at all)")
    print(f"  prey detected:   {prey_bursts}")
    print(f"  cat ids:         {dict(cat_counter)}")
    print()

    # Latency
    lat = sorted(r["decision_ms"] for r in reqs if "decision_ms" in r)
    def pct(p: float) -> int:
        return lat[int(p * (len(lat) - 1))] if lat else 0

    print("LATENCY (ms)")
    print(f"  all  n={len(lat):4d}  min={lat[0]:4d}  p50={pct(0.5):4d}  p95={pct(0.95):4d}  p99={pct(0.99):4d}  max={lat[-1]:5d}  mean={sum(lat)/len(lat):4.0f}")

    det_lat = sorted(r["decision_ms"] for r in reqs if r.get("cat_recognized"))
    no_lat = sorted(r["decision_ms"] for r in reqs if not r.get("cat_recognized"))
    if det_lat:
        print(f"  cat  n={len(det_lat):4d}  min={det_lat[0]:4d}  p50={det_lat[len(det_lat)//2]:4d}  p95={det_lat[int(0.95*(len(det_lat)-1))]:4d}  max={det_lat[-1]:5d}")
    if no_lat:
        print(f"  ncat n={len(no_lat):4d}  min={no_lat[0]:4d}  p50={no_lat[len(no_lat)//2]:4d}  p95={no_lat[int(0.95*(len(no_lat)-1))]:4d}  max={no_lat[-1]:5d}")
    print()

    # JPEG sizes
    sizes = sorted(r["jpeg_bytes"] for r in reqs if "jpeg_bytes" in r)
    if sizes:
        print(f"JPEG bytes  min={sizes[0]}  p50={sizes[len(sizes)//2]}  p95={sizes[int(0.95*(len(sizes)-1))]}  max={sizes[-1]}")
        print()

    # Hourly (UTC)
    hours: Counter[int] = Counter()
    cat_hours: Counter[int] = Counter()
    for r in reqs:
        ts = r.get("ts_ms", 0) / 1000
        h = datetime.fromtimestamp(ts, tz=timezone.utc).hour
        hours[h] += 1
        if r.get("cat_recognized"):
            cat_hours[h] += 1
    print("REQUESTS by hour (UTC):")
    print("  hour    reqs   cat_frames")
    for h in sorted(hours):
        bar = "#" * min(40, int(hours[h] / 2))
        print(f"  {h:02d}:00  {hours[h]:5d}  {cat_hours[h]:5d}  {bar}")
    print()

    # Cat-positive bursts with image paths for review
    print("CAT-POSITIVE BURSTS (review):")
    cat_burst_summary = []
    for (dev, bid), frames in bursts.items():
        cat_frames = [f for f in frames if f.get("cat_recognized")]
        if not cat_frames:
            continue
        ts = min(f.get("ts_ms", 0) for f in frames)
        ids = Counter(f.get("cat_id") for f in cat_frames)
        max_prey = max(f.get("prey_score", 0) for f in frames)
        first_img = next(
            (f.get("image_path") for f in sorted(frames, key=lambda x: x.get("frame_index", 99)) if f.get("image_path")),
            None,
        )
        cat_burst_summary.append((ts, bid, ids.most_common(1)[0][0], len(frames), len(cat_frames), max_prey, first_img))
    for ts, bid, cid, n, ncat, mp, img in sorted(cat_burst_summary):
        when = datetime.fromtimestamp(ts / 1000, tz=timezone.utc).strftime("%m-%d %H:%M:%S")
        print(f"  {when}  burst={bid:>15s}  cat={cid}  frames={ncat}/{n}  max_prey={mp:.3f}")
        if img:
            print(f"      img: {img}")


if __name__ == "__main__":
    main()
