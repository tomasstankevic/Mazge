"""Latency micro-benchmark for an Ultralytics YOLO model on this machine.

Usage:
    .venv-server/bin/python tools/bench_yolo_local.py yolo11x.pt --imgsz 640 --warmup 5 --runs 30
"""

import argparse
import statistics
import time

import numpy as np

import torch
from ultralytics import YOLO


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("model", help="path or name (e.g. yolo11x.pt) of an Ultralytics model")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument("--runs", type=int, default=30)
    parser.add_argument("--threads", type=int, default=0, help="torch.set_num_threads (0 = leave default)")
    args = parser.parse_args()

    if args.threads:
        torch.set_num_threads(args.threads)

    print(f"torch={torch.__version__} threads={torch.get_num_threads()}")
    print(f"loading {args.model} ...")
    t0 = time.perf_counter()
    model = YOLO(args.model)
    print(f"  loaded in {time.perf_counter() - t0:.2f}s")

    rng = np.random.default_rng(0)
    frame = rng.integers(0, 256, (args.imgsz, args.imgsz, 3), dtype=np.uint8)

    print(f"warmup x{args.warmup} ...")
    for _ in range(args.warmup):
        model.predict(frame, imgsz=args.imgsz, verbose=False)

    print(f"timed runs x{args.runs} ...")
    samples_ms: list[float] = []
    for _ in range(args.runs):
        t0 = time.perf_counter()
        model.predict(frame, imgsz=args.imgsz, verbose=False)
        samples_ms.append((time.perf_counter() - t0) * 1000.0)

    samples_ms.sort()
    p50 = statistics.median(samples_ms)
    p95 = samples_ms[int(0.95 * (len(samples_ms) - 1))]
    p99 = samples_ms[int(0.99 * (len(samples_ms) - 1))]
    print()
    print(f"model={args.model} imgsz={args.imgsz} runs={args.runs}")
    print(f"  min={samples_ms[0]:7.1f} ms")
    print(f"  p50={p50:7.1f} ms")
    print(f"  p95={p95:7.1f} ms")
    print(f"  p99={p99:7.1f} ms")
    print(f"  max={samples_ms[-1]:7.1f} ms")
    print(f"  mean={statistics.mean(samples_ms):7.1f} ms")


if __name__ == "__main__":
    main()
