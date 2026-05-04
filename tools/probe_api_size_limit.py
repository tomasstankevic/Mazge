#!/usr/bin/env python3
"""Probe the prey API payload size limit by testing rotated frames at various crop sizes."""
import base64, os, subprocess, sys, tempfile
from pathlib import Path
import requests

API_URL = "https://prey-detection.florian-mutel.workers.dev"
API_KEY = (Path(__file__).resolve().parent.parent / "API_key_prey_detector.txt").read_text().strip()
JPEGTRAN = "/opt/homebrew/bin/jpegtran"

src = Path(sys.argv[1] if len(sys.argv) > 1 else "captures/sd/20260501_015148_gen1/f05.jpg")


def jt(*args) -> bytes:
    with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tmp:
        tmp_path = tmp.name
    try:
        subprocess.run([JPEGTRAN, *args, "-outfile", tmp_path, str(src)],
                       check=True, capture_output=True)
        return Path(tmp_path).read_bytes()
    finally:
        os.unlink(tmp_path)


def call(jpg: bytes):
    b64 = base64.b64encode(jpg).decode()
    r = requests.post(API_URL,
                      headers={"Authorization": f"Bearer {API_KEY}",
                               "Content-Type": "application/json"},
                      json={"image_base64": b64}, timeout=30)
    if r.status_code != 200:
        return f"HTTP{r.status_code} ({len(b64)}b b64)"
    return "YES" if r.json().get("detected") else "no"


# Source is 640x480 (landscape). After rotate 270 CCW -> 480x640 (portrait).
# Crops are in original orientation (jpegtran applies them BEFORE rotate)
# Format: (label, args)
tests = [
    ("rot_full_color",        ["-rotate", "270"]),
    ("rot_full_gray",         ["-rotate", "270", "-grayscale"]),
    # Crop original 640x480 to remove right occlusion: 512x480 -> rot -> 480x512
    ("rot_cropR128_color",    ["-crop", "512x480+0+0", "-rotate", "270"]),
    ("rot_cropR128_gray",     ["-crop", "512x480+0+0", "-rotate", "270", "-grayscale"]),
    # Smaller: 384x384 center crop -> rot -> 384x384
    ("rot_384_color",         ["-crop", "384x384+64+48", "-rotate", "270"]),
    ("rot_384_gray",          ["-crop", "384x384+64+48", "-rotate", "270", "-grayscale"]),
    # Smaller still: 256x256
    ("rot_256_color",         ["-crop", "256x256+128+112", "-rotate", "270"]),
    ("rot_256_gray",          ["-crop", "256x256+128+112", "-rotate", "270", "-grayscale"]),
]

print(f"Source: {src} ({src.stat().st_size}B)\n")
print(f"{'Variant':<22} {'Size':>8} {'B64':>8}  {'API'}")
for label, args in tests:
    try:
        out = jt(*args)
        b64_len = ((len(out) + 2) // 3) * 4
        result = call(out)
        print(f"{label:<22} {len(out):>8} {b64_len:>8}  {result}")
    except subprocess.CalledProcessError as e:
        print(f"{label:<22} jpegtran error: {e.stderr.decode()[:60]}")
