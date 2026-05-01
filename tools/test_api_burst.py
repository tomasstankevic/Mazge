#!/usr/bin/env python3
"""Send burst frames to the prey detection API matching ESP32 exactly.

ESP32 behaviour:
  1. Sort frames by JPEG file size descending
  2. Pick top 3 (API_FRAMES_PER_BURST=3)
  3. Lossless JPEG crop 640x480 -> 384x384 at (64,48) using MCU-aligned jpegtran
  4. POST {"image_base64":"<b64>"} to API
  5. First "detected":true -> apiResult=1

Usage: python test_api_burst.py [burst_dir] [--all]
  --all  send ALL frames (not just top 3)
"""
import base64, json, os, sys, subprocess, tempfile, requests

API_URL = "https://prey-detection.florian-mutel.workers.dev"
API_KEY = open(os.path.join(os.path.dirname(__file__), "..", "API_key_prey_detector.txt")).read().strip()

JPEGTRAN = "/opt/homebrew/bin/jpegtran"

burst_dir = sys.argv[1] if len(sys.argv) > 1 else "captures/sd/20260501_015148_gen1"
send_all = "--all" in sys.argv


def lossless_crop(src_path: str) -> bytes:
    """Lossless JPEG crop matching ESP32: 384x384 at offset (64,48)."""
    with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tmp:
        tmp_path = tmp.name
    try:
        subprocess.run(
            [JPEGTRAN, "-crop", "384x384+64+48", "-outfile", tmp_path, src_path],
            check=True, capture_output=True,
        )
        with open(tmp_path, "rb") as f:
            return f.read()
    finally:
        os.unlink(tmp_path)


# Gather all frames with sizes
frames = []
for i in range(10):
    fname = f"f{i:02d}.jpg"
    fpath = os.path.join(burst_dir, fname)
    if os.path.exists(fpath):
        frames.append((fname, fpath, os.path.getsize(fpath)))

# Sort by file size descending (ESP32 picks largest first)
frames.sort(key=lambda x: x[2], reverse=True)

if not send_all:
    selected = frames[:3]
    skipped = {f[0] for f in frames} - {f[0] for f in selected}
    print(f"ESP32 mode: sending top 3 by size (of {len(frames)}), skipping: {sorted(skipped)}")
else:
    selected = frames
    print(f"Sending ALL {len(frames)} frames")

print()
api_result = 0  # overall: 1 if any frame is prey

for fname, fpath, fsize in selected:
    crop_bytes = lossless_crop(fpath)

    b64 = base64.b64encode(crop_bytes).decode()
    resp = requests.post(
        API_URL,
        headers={
            "Content-Type": "application/json",
            "Authorization": f"Bearer {API_KEY}",
        },
        json={"image_base64": b64},
        timeout=15,
    )
    data = resp.json()
    detected = data.get("detected", False)
    if detected:
        api_result = 1
    tag = "PREY!" if detected else "no prey"
    print(f"{fname} (raw {fsize}B -> crop {len(crop_bytes)}B): {tag}")

print(f"\napiResult = {api_result}")
