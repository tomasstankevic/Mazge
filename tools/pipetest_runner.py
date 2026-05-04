#!/usr/bin/env python3
"""Run pipeline timing benchmarks on the ESP32 against a known prey burst.

For each frame in the burst, runs each pipeline variant N times on the ESP,
collects per-iteration timings, optionally fetches the processed JPEG and
sends it to the prey API to verify hits.

Usage:
  python tools/pipetest_runner.py [--burst captures/sd/20260501_015148_gen1]
                                  [--host 192.168.0.41]
                                  [--variants A,B,C,D,E,F,H]
                                  [--iters 5]
                                  [--api-verify]
                                  [--save-out outdir/]
"""
import argparse
import base64
import json
import os
import statistics
import sys
import time
from pathlib import Path

import requests

API_URL = "https://prey-detection.florian-mutel.workers.dev"


def load_api_key():
    p = Path(__file__).resolve().parent.parent / "API_key_prey_detector.txt"
    return p.read_text().strip()


def run_pipe(host, variant, jpg_bytes, iters):
    """Run one pipeline N times. Returns dict from ESP."""
    url = f"http://{host}/pipetest?pipe={variant}&iters={iters}&out=0"
    r = requests.post(url, data=jpg_bytes,
                      headers={"Content-Type": "application/octet-stream"},
                      timeout=60)
    r.raise_for_status()
    return r.json()


def get_pipe_output(host, variant, jpg_bytes):
    """Run pipeline once and fetch the processed JPEG bytes."""
    url = f"http://{host}/pipetest?pipe={variant}&iters=1&out=1"
    r = requests.post(url, data=jpg_bytes,
                      headers={"Content-Type": "application/octet-stream"},
                      timeout=60)
    r.raise_for_status()
    t_us = int(r.headers.get("X-Pipe-Time-Us", "0"))
    return r.content, t_us


def call_prey_api(jpg_bytes, api_key):
    """POST processed JPEG to prey API. Returns (detected_bool, raw_response)."""
    b64 = base64.b64encode(jpg_bytes).decode()
    r = requests.post(API_URL,
                      headers={"Content-Type": "application/json",
                               "Authorization": f"Bearer {api_key}"},
                      json={"image_base64": b64},
                      timeout=20)
    r.raise_for_status()
    j = r.json()
    return bool(j.get("detected", False)), j


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--burst", default="captures/sd/20260501_015148_gen1",
                    help="Path to burst directory containing f00.jpg..f09.jpg")
    ap.add_argument("--host", default="192.168.0.41", help="ESP32 host")
    ap.add_argument("--variants", default="A,B,C,D,E,F,H",
                    help="Comma-separated pipeline IDs")
    ap.add_argument("--iters", type=int, default=5)
    ap.add_argument("--api-verify", action="store_true",
                    help="Send each processed frame to prey API")
    ap.add_argument("--save-out", default=None,
                    help="Save processed JPEGs to this directory")
    ap.add_argument("--frames", default=None,
                    help="Comma-separated frame indices to test (default: all)")
    args = ap.parse_args()

    burst = Path(args.burst)
    if not burst.is_dir():
        print(f"ERROR: burst dir not found: {burst}", file=sys.stderr)
        return 1

    frames = sorted(burst.glob("f??.jpg"))
    if args.frames:
        wanted = set(int(x) for x in args.frames.split(","))
        frames = [f for f in frames if int(f.stem[1:]) in wanted]
    if not frames:
        print("ERROR: no frames found", file=sys.stderr)
        return 1

    variants = args.variants.split(",")
    api_key = load_api_key() if args.api_verify else None
    save_dir = Path(args.save_out) if args.save_out else None
    if save_dir:
        save_dir.mkdir(parents=True, exist_ok=True)

    # Per-variant aggregated stats: list of (frame, median_us, out_len, hit)
    results = {v: [] for v in variants}

    print(f"Burst: {burst}")
    print(f"Frames: {[f.name for f in frames]}")
    print(f"Variants: {variants}, iters={args.iters}, api_verify={args.api_verify}")
    print()

    for f in frames:
        jpg = f.read_bytes()
        print(f"=== {f.name} ({len(jpg)} bytes) ===")
        for v in variants:
            try:
                rj = run_pipe(args.host, v, jpg, args.iters)
            except Exception as e:
                print(f"  {v}: ERROR {e}")
                results[v].append((f.name, None, None, None))
                continue
            times_us = rj["times_us"]
            out_len = rj["out_lens"][-1] if rj["out_lens"] else 0
            med_us = statistics.median(times_us)
            min_us = min(times_us)

            hit = None
            if args.api_verify or save_dir:
                try:
                    out_bytes, _ = get_pipe_output(args.host, v, jpg)
                    if save_dir:
                        out_path = save_dir / f"{f.stem}_{v}.jpg"
                        out_path.write_bytes(out_bytes)
                    if args.api_verify:
                        hit, _ = call_prey_api(out_bytes, api_key)
                except Exception as e:
                    print(f"  {v}: out/API error: {e}")

            hit_str = ""
            if hit is not None:
                hit_str = f" hit={'YES' if hit else 'no'}"
            print(f"  {v}: med={med_us/1000:7.1f}ms min={min_us/1000:7.1f}ms "
                  f"out={out_len:6}B{hit_str}")
            results[v].append((f.name, med_us, out_len, hit))

    print()
    print("=" * 70)
    print(f"Summary (median across {len(frames)} frames)")
    print("=" * 70)
    print(f"{'Pipe':<5} {'Med ms':>10} {'Min ms':>10} {'Max ms':>10} "
          f"{'OutB med':>10} {'Hits':>8}")
    for v in variants:
        rows = results[v]
        valid = [r for r in rows if r[1] is not None]
        if not valid:
            print(f"{v:<5} (no valid runs)")
            continue
        meds = [r[1] for r in valid]
        outs = [r[2] for r in valid]
        med_med = statistics.median(meds) / 1000
        min_med = min(meds) / 1000
        max_med = max(meds) / 1000
        out_med = statistics.median(outs)
        hits = sum(1 for r in valid if r[3] is True)
        total_with_hit = sum(1 for r in valid if r[3] is not None)
        hits_str = f"{hits}/{total_with_hit}" if total_with_hit else "n/a"
        print(f"{v:<5} {med_med:>10.1f} {min_med:>10.1f} {max_med:>10.1f} "
              f"{out_med:>10.0f} {hits_str:>8}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
