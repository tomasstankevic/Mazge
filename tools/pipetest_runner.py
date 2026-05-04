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
    """POST processed JPEG to prey API. Returns (detected, raw_response, elapsed_ms)."""
    b64 = base64.b64encode(jpg_bytes).decode()
    t0 = time.perf_counter()
    r = requests.post(API_URL,
                      headers={"Content-Type": "application/json",
                               "Authorization": f"Bearer {api_key}"},
                      json={"image_base64": b64},
                      timeout=20)
    elapsed_ms = (time.perf_counter() - t0) * 1000
    if r.status_code != 200:
        return None, {"http": r.status_code, "text": r.text[:80]}, elapsed_ms
    j = r.json()
    return bool(j.get("detected", False)), j, elapsed_ms


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
    ap.add_argument("--qvga-input", action="store_true",
                    help="Pre-resize VGA frames to QVGA (320x240) on host "
                         "before sending to ESP")
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
        if args.qvga_input:
            from PIL import Image
            import io as _io
            img = Image.open(_io.BytesIO(jpg)).resize((320, 240))
            buf = _io.BytesIO()
            # subsampling=1 -> 4:2:2 to match OV2640 output
            img.save(buf, format="JPEG", quality=80, subsampling=1)
            jpg = buf.getvalue()
        print(f"=== {f.name} ({len(jpg)} bytes) ===")
        for v in variants:
            try:
                rj = run_pipe(args.host, v, jpg, args.iters)
            except Exception as e:
                print(f"  {v}: ERROR {e}")
                results[v].append((f.name, None, None, None, None))
                continue
            times_us = rj["times_us"]
            out_len = rj["out_lens"][-1] if rj["out_lens"] else 0
            med_us = statistics.median(times_us)
            min_us = min(times_us)

            hit = None
            api_ms = None
            if args.api_verify or save_dir:
                try:
                    out_bytes, _ = get_pipe_output(args.host, v, jpg)
                    if save_dir:
                        out_path = save_dir / f"{f.stem}_{v}.jpg"
                        out_path.write_bytes(out_bytes)
                    if args.api_verify:
                        hit, _, api_ms = call_prey_api(out_bytes, api_key)
                except Exception as e:
                    print(f"  {v}: out/API error: {e}")

            hit_str = ""
            if hit is not None:
                hit_str = f" hit={'YES' if hit else 'no'}"
            api_str = f" api={api_ms:6.0f}ms" if api_ms is not None else ""
            print(f"  {v}: prep={med_us/1000:6.1f}ms{api_str} out={out_len:6}B{hit_str}")
            results[v].append((f.name, med_us, out_len, hit, api_ms))

    print()
    print("=" * 80)
    print(f"Summary (median across {len(frames)} frames)")
    print("=" * 80)
    print(f"{'Pipe':<5} {'Prep ms':>10} {'API ms':>10} {'E2E ms':>10} "
          f"{'OutB med':>10} {'Hits':>8}")
    for v in variants:
        rows = results[v]
        valid = [r for r in rows if r[1] is not None]
        if not valid:
            print(f"{v:<5} (no valid runs)")
            continue
        meds = [r[1] for r in valid]
        outs = [r[2] for r in valid]
        api_times = [r[4] for r in valid if r[4] is not None]
        prep_med = statistics.median(meds) / 1000
        out_med = statistics.median(outs)
        api_med = statistics.median(api_times) if api_times else 0
        e2e_med = prep_med + api_med if api_times else prep_med
        hits = sum(1 for r in valid if r[3] is True)
        total_with_hit = sum(1 for r in valid if r[3] is not None)
        hits_str = f"{hits}/{total_with_hit}" if total_with_hit else "n/a"
        api_str = f"{api_med:>10.1f}" if api_times else f"{'-':>10}"
        e2e_str = f"{e2e_med:>10.1f}" if api_times else f"{'-':>10}"
        print(f"{v:<5} {prep_med:>10.1f} {api_str} {e2e_str} "
              f"{out_med:>10.0f} {hits_str:>8}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
