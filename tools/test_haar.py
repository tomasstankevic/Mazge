#!/usr/bin/env python3
"""Test Haar cascade cat face detection on various frames."""
import cv2
import time

cc = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalcatface_extended.xml')

test_frames = [
    ('captures/burst_20260419_031049_gen6/frame_04.jpg', 'night face YOLO cat'),
    ('captures/burst_20260418_221655_gen7/frame_01.jpg', 'night cat YOLO 62%'),
    ('captures/burst_20260418_221704_gen8/frame_01.jpg', 'night cat YOLO 65%'),
    ('captures/burst_20260419_122135_gen3/frame_08.jpg', 'day closeup no YOLO'),
    ('captures/burst_20260419_121456_gen2/frame_07.jpg', 'day closeup no YOLO'),
    ('captures/burst_20260419_121359_gen1/frame_07.jpg', 'day no YOLO'),
    ('captures/burst_20260418_184513_gen1/frame_00.jpg', 'day clear cat noYOLO'),
    ('captures/burst_20260418_184638_gen2/frame_01.jpg', 'day no YOLO'),
    ('captures/burst_20260418_230639_gen1/frame_01.jpg', 'night fur/butt?'),
    ('captures/burst_20260418_230652_gen3/frame_08.jpg', 'night fur/butt?'),
    ('captures/burst_20260418_230700_gen4/frame_00.jpg', 'night fur?'),
    ('captures/burst_20260418_230728_gen8/frame_03.jpg', 'night fur/butt?'),
    ('captures/burst_20260418_230739_gen9/frame_02.jpg', 'night overexposed'),
    ('captures/burst_20260419_031044_gen1/frame_02.jpg', 'night close'),
    ('captures/burst_20260419_031045_gen2/frame_02.jpg', 'night dark'),
    ('captures/burst_20260419_031047_gen4/frame_00.jpg', 'night dark close'),
    ('captures/burst_20260418_193142_gen1/frame_05.jpg', 'night blob'),
    ('captures/burst_20260418_204758_gen12/frame_05.jpg', 'night dark blob'),
    ('captures/burst_20260418_225059_gen1/frame_00.jpg', 'night ?'),
    ('captures/burst_20260418_230707_gen5/frame_06.jpg', 'night ?'),
    ('captures/burst_20260418_230713_gen6/frame_01.jpg', 'night ?'),
    ('captures/burst_20260418_230747_gen10/frame_07.jpg', 'night ?'),
]

# Also test ALL frames in a burst, not just the best one
print("=== Single frame tests ===")
for path, desc in test_frames:
    img = cv2.imread(path)
    if img is None:
        print(f'  SKIP {path}')
        continue
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Also try with histogram equalization
    gray_eq = cv2.equalizeHist(gray)
    
    t0 = time.perf_counter()
    faces = cc.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=3, minSize=(30, 30))
    ms1 = (time.perf_counter() - t0) * 1000
    
    t0 = time.perf_counter()
    faces_eq = cc.detectMultiScale(gray_eq, scaleFactor=1.1, minNeighbors=3, minSize=(30, 30))
    ms2 = (time.perf_counter() - t0) * 1000
    
    n1, n2 = len(faces), len(faces_eq)
    s1 = f'{n1}face' if n1 > 0 else '---'
    s2 = f'{n2}face(eq)' if n2 > 0 else '---'
    if n1 > 0:
        biggest = max(faces, key=lambda f: f[2]*f[3])
        s1 += f' {biggest[2]}x{biggest[3]}'
    if n2 > 0:
        biggest = max(faces_eq, key=lambda f: f[2]*f[3])
        s2 += f' {biggest[2]}x{biggest[3]}'
    print(f'  {s1:18s} {s2:18s} {ms1+ms2:5.1f}ms  {desc:25s}  {path}')

# Now test scanning all frames in select bursts
print("\n=== Full burst scan (all frames) ===")
import json
from pathlib import Path
data = json.loads(Path('captures/prey_analysis.json').read_text())
for r in data:
    bdir = Path('captures') / r['burst']
    frames = sorted(bdir.glob('frame_*.jpg'))
    face_frames = []
    for fp in frames:
        img = cv2.imread(str(fp))
        if img is None: continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_eq = cv2.equalizeHist(gray)
        f1 = cc.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=3, minSize=(30, 30))
        f2 = cc.detectMultiScale(gray_eq, scaleFactor=1.1, minNeighbors=3, minSize=(30, 30))
        if len(f1) > 0 or len(f2) > 0:
            face_frames.append(fp.stem)
    yolo_cats = r.get('cat_detections', 0)
    status = f'YOLO={yolo_cats}' if yolo_cats > 0 else 'noYOLO'
    face_str = ','.join(face_frames) if face_frames else '---'
    print(f'  {r["burst"]:45s}  {status:10s}  haar: {face_str}')
