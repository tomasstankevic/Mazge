#!/usr/bin/env python3
"""Simulate quadratic ramp on recent bursts."""
import json, os, glob

CAP = 300  # nightExposureCap
POST = 5   # POST_TRIGGER_FRAMES

bursts = [
    "burst_20260421_221339_gen3",
    "burst_20260421_221323_gen2",
    "burst_20260421_221219_gen1",
    "burst_20260421_221010_gen6",
    "burst_20260421_220948_gen5",
    "burst_20260421_220933_gen4",
    "burst_20260421_220917_gen3",
    "burst_20260421_220901_gen2",
]

for bname in bursts:
    path = f"/Users/ruta/Tomas/repos/Mazge/captures/{bname}/burst_meta.json"
    if not os.path.exists(path):
        continue
    m = json.load(open(path))
    cap = m.get("esp32_capture", {})
    trigger_ms = cap.get("trigger_ms", 0)
    frames_ms = cap.get("frame_capture_ms", [])
    dists = m["frame_distance_mm"]

    files = sorted(glob.glob(f"/Users/ruta/Tomas/repos/Mazge/captures/{bname}/frame_*.jpg"))
    sizes = [os.path.getsize(f) for f in files]

    n = min(len(frames_ms), len(dists), len(sizes))
    print(f"=== {bname} ({n} frames) ===")

    # Count post-trigger frames
    post_frames = [i for i in range(n) if frames_ms[i] > trigger_ms]
    num_post = len(post_frames)

    for i in range(n):
        is_post = frames_ms[i] > trigger_ms if trigger_ms else False

        if is_post:
            # remaining counts down: last post-trigger frame has remaining=1
            idx_in_post = post_frames.index(i)
            remaining = num_post - idx_in_post
            quad_aec = CAP * remaining * remaining // (POST * POST)
            label = f"POST r={remaining}"
        else:
            quad_aec = CAP
            label = "PRE"

        blown = "BLOWN" if sizes[i] > 25000 else ("dark" if sizes[i] < 10000 else "ok")
        print(f"  f{i:02d}: dist={dists[i]:4d}mm  {sizes[i]//1024:3d}K  {label:12s}  quad_aec={quad_aec:3d}  [{blown}]")
    print()
