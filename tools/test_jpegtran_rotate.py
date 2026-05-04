#!/usr/bin/env python3
"""Test lossless DCT rotation (jpegtran -rotate 270) on the prey burst.

Validates that DCT-domain 90 CCW rotation produces images the prey API
recognizes — confirming the algorithm before porting to C for ESP32.

Two outputs per frame:
  rot       - jpegtran -rotate 270 (color, retains chroma)
  rot_gray  - jpegtran -rotate 270 + -grayscale (drop chroma like our
              proposed ESP implementation will)
"""
import base64
import os
import subprocess
import sys
import tempfile
from pathlib import Path

import requests

API_URL = "https://prey-detection.florian-mutel.workers.dev"
API_KEY = (Path(__file__).resolve().parent.parent / "API_key_prey_detector.txt").read_text().strip()
JPEGTRAN = "/opt/homebrew/bin/jpegtran"

burst = Path(sys.argv[1] if len(sys.argv) > 1 else "captures/sd/20260501_015148_gen1")


def jpegtran(src: Path, *args) -> bytes:
    with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tmp:
        tmp_path = tmp.name
    try:
        subprocess.run([JPEGTRAN, *args, "-outfile", tmp_path, str(src)],
                       check=True, capture_output=True)
        return Path(tmp_path).read_bytes()
    finally:
        os.unlink(tmp_path)


def call_api(jpg: bytes) -> bool:
    b64 = base64.b64encode(jpg).decode()
    r = requests.post(API_URL,
                      headers={"Authorization": f"Bearer {API_KEY}",
                               "Content-Type": "application/json"},
                      json={"image_base64": b64}, timeout=20)
    if r.status_code != 200:
        return f"HTTP{r.status_code}"
    return bool(r.json().get("detected", False))


frames = sorted(burst.glob("f??.jpg"))
print(f"Burst: {burst}, {len(frames)} frames\n")
print(f"{'Frame':<8} {'Orig B':>8} {'Rot B':>8} {'RotGr B':>8} "
      f"{'Orig':>5} {'Rot':>5} {'RotGr':>6}")

orig_hits = rot_hits = rot_gray_hits = 0
for f in frames:
    orig = f.read_bytes()
    rot = jpegtran(f, "-rotate", "270")
    rot_gray = jpegtran(f, "-rotate", "270", "-grayscale")

    h_orig = call_api(orig)
    h_rot = call_api(rot)
    h_gray = call_api(rot_gray)
    if h_orig is True: orig_hits += 1
    if h_rot is True: rot_hits += 1
    if h_gray is True: rot_gray_hits += 1

    def fmt(h):
        if h is True: return "YES"
        if h is False: return "-"
        return str(h)
    print(f"{f.name:<8} {len(orig):>8} {len(rot):>8} {len(rot_gray):>8} "
          f"{fmt(h_orig):>5} {fmt(h_rot):>7} {fmt(h_gray):>7}")

print()
print(f"Hits: orig={orig_hits}/{len(frames)}  "
      f"rot={rot_hits}/{len(frames)}  "
      f"rot_gray={rot_gray_hits}/{len(frames)}")
