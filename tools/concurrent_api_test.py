"""Measure concurrent vs sequential API call latency on laptop.

Sends a 384x384 grayscale JPEG (from variant R output) to the prey API at
varying concurrency levels and reports per-call median + total time.
"""
import argparse
import base64
import statistics
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import requests

API_URL = "https://prey-detection.florian-mutel.workers.dev"
API_KEY = (Path(__file__).resolve().parent.parent /
           "API_key_prey_detector.txt").read_text().strip()


def call(jpg):
    t0 = time.time()
    try:
        r = requests.post(
            API_URL,
            headers={"Authorization": f"Bearer {API_KEY}",
                     "Content-Type": "application/json"},
            json={"image_base64": base64.b64encode(jpg).decode()},
            timeout=20,
        )
    except Exception as e:
        return None, (time.time() - t0) * 1000, f"err {e}"
    elapsed = (time.time() - t0) * 1000
    if r.status_code != 200:
        return None, elapsed, f"HTTP{r.status_code}"
    return r.json().get("detected"), elapsed, ""


def run_batch(jpg, n, parallel):
    """Send n calls, with up to `parallel` concurrent connections."""
    t0 = time.time()
    results = []
    with ThreadPoolExecutor(max_workers=parallel) as ex:
        futures = [ex.submit(call, jpg) for _ in range(n)]
        for f in as_completed(futures):
            results.append(f.result())
    total = (time.time() - t0) * 1000
    return total, [r[1] for r in results]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--burst", default="captures/prey_review/20260501_015148_gen1")
    ap.add_argument("--frame", default="f05.jpg")
    ap.add_argument("--n", type=int, default=10, help="frames per batch (one per API call)")
    ap.add_argument("--parallels", default="1,2,3,4,5,8",
                    help="comma-separated parallelism levels to test")
    args = ap.parse_args()

    src = Path(args.burst) / args.frame
    print(f"Source: {src}")
    # Use the production lossless rotate to produce a representative payload
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from jpeg_lossless_crop_rotate_gray import process
    jpg = process(src.read_bytes(), 64, 48, 384, 384)
    print(f"Payload: {len(jpg)} bytes (base64 ≈ {len(jpg)*4//3} chars)\n")

    # Warm up TLS
    print("Warm-up call...")
    _, warm_ms, err = call(jpg)
    print(f"  warm: {warm_ms:.0f}ms{(' ' + err) if err else ''}\n")

    print(f"{'parallel':>9} {'total_ms':>10} {'avg_per_call':>13} {'min':>7} {'med':>7} {'max':>7}")
    parallels = [int(x) for x in args.parallels.split(",")]
    for p in parallels:
        # Run twice and take the second to get stable warm numbers
        run_batch(jpg, args.n, p)
        total, per = run_batch(jpg, args.n, p)
        per_sorted = sorted(per)
        avg = total / args.n
        print(f"{p:>9} {total:>10.0f} {avg:>13.0f} "
              f"{per_sorted[0]:>7.0f} {statistics.median(per):>7.0f} "
              f"{per_sorted[-1]:>7.0f}")


if __name__ == "__main__":
    main()
