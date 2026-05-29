#!/usr/bin/env python3
"""Rebuild ESP32 event log from local SD mirror (captures/sd/<dir>/meta.json).

Scans last N hours, picks each burst's meta.json, packs into the binary
EventEntry format the firmware expects, and POSTs to /setevents.
"""
import argparse
import json
import struct
import sys
import time
import urllib.request
from datetime import datetime, timezone
from pathlib import Path

# Must match `struct __attribute__((packed)) EventEntry` in main.cpp:
#   uint32_t uptimeMs;   int32_t epochSec;
#   int16_t  gen;        int8_t  frameCount;  int8_t result;
#   int16_t  distMin;    int16_t distMax;
#   uint8_t  mode;       int8_t  trend;
#   uint16_t latencyMs;
# Total 20 bytes, little-endian, no padding.
EVENT_FMT = "<IiHbbhhBbH"
EVENT_SIZE = struct.calcsize(EVENT_FMT)
assert EVENT_SIZE == 20, f"unexpected struct size: {EVENT_SIZE}"

MAX_EVENTS = 50

DEFAULT_ESP = "192.168.0.41"
DEFAULT_CAPTURES = Path(__file__).resolve().parent.parent / "captures" / "sd"


def classify_trend(images):
    """Match firmware classifyDistTrend exactly.

    - min_dist = min valid dist; first_dist = first valid dist.
    - exit if min<180 and first<230 -> 2
    - any valid readings -> 1 enter
    - else 0 unknown
    """
    min_d = None
    first_d = None
    for img in images:
        d = img.get("dist", -1)
        if d is None or d < 0:
            continue
        if first_d is None:
            first_d = d
        if min_d is None or d < min_d:
            min_d = d
    if min_d is None:
        return 0
    if min_d < 180 and first_d < 230:
        return 2
    return 1


def build_event(meta_path: Path):
    try:
        meta = json.loads(meta_path.read_text())
    except Exception as e:
        return None, f"meta read failed: {e}"
    epoch = int(meta.get("epoch", 0))
    if epoch < 1_000_000_000:
        return None, "no/bad epoch in meta"
    gen = int(meta.get("gen", 0))
    frames = int(meta.get("frames", 0))
    api_results = meta.get("apiResults") or []

    # result = -1 if no api summary, else number of frames flagged prey
    if not api_results or all(r == -1 for r in api_results):
        result = -1
    else:
        result = sum(1 for r in api_results if r == 1)

    images = meta.get("images") or []
    dist_vals = [img.get("dist", -1) for img in images if img.get("dist", -1) >= 0]
    if dist_vals:
        dist_min = min(dist_vals)
        dist_max = max(dist_vals)
    else:
        dist_min = -1
        dist_max = -1

    trend_val = meta.get("direction")
    if trend_val is None:
        trend_val = classify_trend(images)

    uptime_ms = int(meta.get("uptimeMs", 0)) & 0xFFFFFFFF
    # API processing latency (apiDoneMs - apiCallMs)
    api_call = int(meta.get("apiCallMs", 0))
    api_done = int(meta.get("apiDoneMs", 0))
    lat_ms = max(0, api_done - api_call) if (api_call and api_done) else 0
    if lat_ms > 65535:
        lat_ms = 65535
    return (uptime_ms, epoch, gen, frames, result, dist_min, dist_max, 1, int(trend_val), lat_ms), None


def pack_events(events):
    out = bytearray()
    for ev in events:
        out += struct.pack(EVENT_FMT, *ev)
    return bytes(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=DEFAULT_ESP)
    ap.add_argument("--captures", default=str(DEFAULT_CAPTURES))
    ap.add_argument("--hours", type=int, default=72)
    ap.add_argument("--max", type=int, default=MAX_EVENTS)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    captures_dir = Path(args.captures)
    if not captures_dir.is_dir():
        sys.exit(f"captures dir not found: {captures_dir}")

    cutoff = time.time() - args.hours * 3600
    print(f"Scanning {captures_dir} for bursts within last {args.hours}h "
          f"(epoch >= {int(cutoff)})...")

    candidates = []  # (epoch, dir_name, meta_path)
    for entry in captures_dir.iterdir():
        if not entry.is_dir():
            continue
        meta = entry / "meta.json"
        if not meta.is_file():
            continue
        try:
            meta_epoch = int(json.loads(meta.read_text()).get("epoch", 0))
        except Exception:
            continue
        if meta_epoch < cutoff:
            continue
        candidates.append((meta_epoch, entry.name, meta))

    candidates.sort()  # oldest first (matches firmware's append order)
    if len(candidates) > args.max:
        candidates = candidates[-args.max:]  # keep newest N
    print(f"  matched {len(candidates)} bursts (cap = {args.max})")

    events = []
    for ep, name, meta_path in candidates:
        ev, err = build_event(meta_path)
        if err:
            print(f"  skip {name}: {err}")
            continue
        events.append(ev)
        ts = datetime.fromtimestamp(ev[1]).strftime("%Y-%m-%d %H:%M:%S")
        res = ev[4]
        res_str = "PENDING" if res < 0 else ("CLEAR" if res == 0 else f"PREY x{res}")
        print(f"  {ts}  gen{ev[2]:<2} {ev[3]}f  {res_str}")

    if not events:
        sys.exit("no events to upload")

    payload = pack_events(events)
    print(f"\nPacked {len(events)} events = {len(payload)} bytes "
          f"({EVENT_SIZE} bytes/entry)")

    if args.dry_run:
        print("(dry run — not sending)")
        return

    url = f"http://{args.host}/setevents"
    print(f"POSTing to {url}...")
    req = urllib.request.Request(url, data=payload,
                                 headers={"Content-Type": "application/octet-stream"},
                                 method="POST")
    with urllib.request.urlopen(req, timeout=15) as resp:
        body = resp.read().decode(errors="replace")
        print(f"ESP says: {body}")


if __name__ == "__main__":
    main()
