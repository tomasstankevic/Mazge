#!/usr/bin/env python3
"""Analyze distance progression across bursts to correlate with entering/exiting."""
import json, glob, os

bursts = sorted(glob.glob("captures/burst_*/burst_meta.json"))
for bp in bursts:
    m = json.load(open(bp))
    name = os.path.basename(os.path.dirname(bp))
    dists = m.get("frame_distance_mm", [])

    pa_path = os.path.join(os.path.dirname(bp), "prey_analysis.json")
    direction = "?"
    prey = "?"
    cat_count = "?"
    face_det = "?"
    if os.path.exists(pa_path):
        pa = json.load(open(pa_path))
        if pa:
            r = pa[0] if isinstance(pa, list) else pa
            direction = r.get("direction", "?")
            p = r.get("prey_detected")
            prey = "Y" if p is True else ("N" if p is False else "?")
            cat_count = r.get("cat_detections", "?")
            face_det = r.get("face_detected", "?")

    valid = [d for d in dists if d > 0]
    trend = ""
    if len(valid) >= 2:
        if valid[-1] < valid[0] - 30:
            trend = "CLOSER"
        elif valid[-1] > valid[0] + 30:
            trend = "FARTHER"
        else:
            trend = "FLAT"

    d_str = ",".join(str(d) for d in dists)
    print(f"{name}  dists=[{d_str}]  valid={len(valid)}/{len(dists)}  trend={trend}  dir={direction}  prey={prey}  cats={cat_count}  face={face_det}")
