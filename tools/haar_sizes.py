#!/usr/bin/env python3
"""Analyze Haar detection sizes to find false positive threshold."""
import cv2
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from prey_analyzer import CatFaceDetector

face_det = CatFaceDetector()
CAPTURES = Path(__file__).resolve().parent.parent / "captures"

detections = []
for bdir in sorted(CAPTURES.iterdir()):
    if not bdir.is_dir() or not (bdir / "frame_00.jpg").exists():
        continue
    for fp in sorted(bdir.glob("frame_*.jpg")):
        img = cv2.imread(str(fp))
        if img is None:
            continue
        h_img, w_img = img.shape[:2]
        faces = face_det.detect_face(img)
        for x, y, w, h in faces:
            area_pct = (w * h) / (h_img * w_img) * 100
            detections.append((bdir.name, fp.stem, w, h, area_pct))

print("Total Haar detections:", len(detections))
print()
print("Sorted by area (smallest = most likely false positive):")
print(f"{'burst':45s}  {'frame':10s}  {'wxh':>8s}  {'area%':>6s}")
print("-" * 75)
for burst, frame, w, h, area_pct in sorted(detections, key=lambda x: x[4]):
    tag = " <<< tiny" if area_pct < 3.0 else ""
    print(f"{burst:45s}  {frame:10s}  {w:3d}x{h:<3d}  {area_pct:5.1f}%{tag}")

print()
print("Distribution:")
for thresh in [1, 2, 3, 5, 8, 10, 15, 20]:
    count = sum(1 for _, _, _, _, a in detections if a >= thresh)
    print(f"  >= {thresh:2d}% area: {count:3d} detections")
