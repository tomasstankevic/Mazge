#!/usr/bin/env python3
"""Compute image stats for all burst best-frames to calibrate quality filters."""
import cv2
import numpy as np
import json
from pathlib import Path

data = json.loads(Path("captures/prey_analysis.json").read_text())

header = f"{'burst':45s}  {'frm':>3s}  {'mean':>5s}  {'std':>5s}  {'lap_v':>8s}  {'brt%':>5s}  {'drk%':>5s}  {'hot%':>5s}  cat"
print(header)
print("-" * len(header))

for r in data:
    bdir = Path("captures") / r["burst"]
    cat = r.get("best_cat", {})
    fidx = cat.get("frame", r.get("best_frame_by_variance", 0))
    fpath = bdir / f"frame_{fidx:02d}.jpg"
    if not fpath.exists():
        continue
    img = cv2.imread(str(fpath), cv2.IMREAD_GRAYSCALE)
    h, w = img.shape
    total = h * w

    mean_v = img.mean()
    std_v = img.std()
    lap_var = cv2.Laplacian(img, cv2.CV_64F).var()

    bright_pct = (img > 240).sum() / total * 100
    dark_pct = (img < 30).sum() / total * 100

    _, bright_mask = cv2.threshold(img, 220, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(bright_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    max_hot = 0
    if contours:
        max_hot = max(cv2.contourArea(c) for c in contours)
    hot_pct = max_hot / total * 100

    has_cat = r.get("cat_detections", 0) > 0
    conf = cat.get("confidence", 0)
    cat_str = f"cat {conf:.2f}" if has_cat else "---"
    print(f"{r['burst']:45s}  f{fidx:<2d}  {mean_v:5.1f}  {std_v:5.1f}  {lap_var:8.1f}  {bright_pct:5.1f}  {dark_pct:5.1f}  {hot_pct:5.2f}  {cat_str}")
