#!/usr/bin/env python3
"""Test Haar cascade with various settings to maximize cat face detection."""
import cv2
import json
from pathlib import Path

data = json.loads(Path('captures/prey_analysis.json').read_text())

cc_ext = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalcatface_extended.xml')
cc_basic = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalcatface.xml')
clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))

configs = [
    ("ext_mn3",   cc_ext,   1.1, 3, 30),
    ("ext_mn2",   cc_ext,   1.1, 2, 30),
    ("ext_mn1",   cc_ext,   1.1, 1, 30),
    ("ext_s1.05", cc_ext,   1.05, 2, 30),
    ("bas_mn2",   cc_basic, 1.1, 2, 30),
    ("bas_mn1",   cc_basic, 1.1, 1, 30),
]

# Track per-burst results
results = {}
for r in data:
    bdir = Path('captures') / r['burst']
    frames = sorted(bdir.glob('frame_*.jpg'))
    yolo = r.get('cat_detections', 0) > 0
    burst_results = {}
    
    for fp in frames:
        img = cv2.imread(str(fp))
        if img is None: continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_clahe = clahe.apply(gray)
        gray_eq = cv2.equalizeHist(gray)
        
        for name, cascade, sf, mn, ms in configs:
            for prep_name, prep_img in [("raw", gray), ("clahe", gray_clahe), ("eq", gray_eq)]:
                key = f"{name}_{prep_name}"
                faces = cascade.detectMultiScale(prep_img, scaleFactor=sf, minNeighbors=mn, minSize=(ms, ms))
                if len(faces) > 0:
                    if key not in burst_results:
                        burst_results[key] = []
                    burst_results[key].append(fp.stem)
    
    results[r['burst']] = (yolo, burst_results)

# Find the config that detects the most bursts while minimizing false positives
print("Config effectiveness (bursts with face detected):")
config_counts = {}
for burst, (yolo, br) in results.items():
    for key in br:
        if key not in config_counts:
            config_counts[key] = {'total': 0, 'new_over_yolo': 0}
        config_counts[key]['total'] += 1
        if not yolo:
            config_counts[key]['new_over_yolo'] += 1

for key in sorted(config_counts, key=lambda k: config_counts[k]['total'], reverse=True):
    c = config_counts[key]
    print(f"  {key:25s}  total={c['total']:2d}  new_over_yolo={c['new_over_yolo']:2d}")

# Show the best config results per burst
print("\n\nPer-burst: best config = ext_mn1_clahe (or whichever is best)")
best_key = max(config_counts, key=lambda k: config_counts[k]['new_over_yolo'])
print(f"Using: {best_key}")
for burst, (yolo, br) in results.items():
    yolo_str = "YOLO" if yolo else "noYOLO"
    face_str = ','.join(br.get(best_key, [])) if best_key in br else "---"
    if face_str != "---" or not yolo:
        print(f"  {burst:45s}  {yolo_str:8s}  {face_str}")
