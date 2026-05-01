#!/usr/bin/env python3
"""Test: crop right occlusion (128px), then resize to 384x384."""
import base64, io, os, requests
from PIL import Image

API_URL = "https://prey-detection.florian-mutel.workers.dev"
API_KEY = open(os.path.join(os.path.dirname(__file__), "..", "API_key_prey_detector.txt")).read().strip()

PREY = "captures/sd/20260501_015148_gen1"
CLEAN = "captures/sd/20260430_011516_gen3"


def test_burst(burst_dir, label):
    print(f"=== {label}: {burst_dir} ===")
    for fi in range(10):
        fpath = os.path.join(burst_dir, f"f{fi:02d}.jpg")
        if not os.path.exists(fpath):
            continue
        img = Image.open(fpath)
        # Crop out right 128px occlusion: 640x480 -> 512x480
        cropped = img.crop((0, 0, 512, 480))
        # Rotate 90 CCW (camera is sideways) -> 480x512
        rotated = cropped.rotate(90, expand=True)
        # Proportional resize to fit 384
        w, h = rotated.size
        new_h = 384
        new_w = int(w * 384 / h)
        resized = rotated.resize((new_w, new_h), Image.Resampling.LANCZOS)
        buf = io.BytesIO()
        resized.save(buf, format="JPEG", quality=85)
        img_bytes = buf.getvalue()
        b64 = base64.b64encode(img_bytes).decode()
        resp = requests.post(
            API_URL,
            headers={"Content-Type": "application/json", "Authorization": f"Bearer {API_KEY}"},
            json={"image_base64": b64},
            timeout=15,
        )
        detected = resp.json().get("detected", False)
        tag = "PREY!" if detected else "no prey"
        print(f"  f{fi:02d} (512x480 -> 384x384, {len(img_bytes)}B): {tag}")


test_burst(PREY, "PREY")
print()
test_burst(CLEAN, "CLEAN")
