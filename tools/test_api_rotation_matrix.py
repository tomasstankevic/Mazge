#!/usr/bin/env python3
"""Test all crop strategies WITH 90° CCW rotation on prey and clean bursts.

Strategies (all with 90° CCW rotation applied first):
  esp32_rot      - Rotate, then lossless-style center crop 384x384 (PIL, not jpegtran)
  fullpil_rot    - Rotate full frame, proportional resize to 384
  full384sq_rot  - Rotate full frame, stretch to 384x384
  cropR_rot      - Crop right 128px, rotate, proportional resize to 384
  cropR_sq_rot   - Crop right 128px, rotate, stretch to 384x384
  center320_rot  - Rotate, center 320x320, resize to 384x384
  esp32_norot    - Original ESP32 method (lossless crop, no rotation) for comparison
  fullpil_norot  - Full frame proportional resize, no rotation (previous best w/o rotation)
"""
import base64, io, os, subprocess, sys, tempfile
import requests
from PIL import Image

API_URL = "https://prey-detection.florian-mutel.workers.dev"
API_KEY = open(os.path.join(os.path.dirname(__file__), "..", "API_key_prey_detector.txt")).read().strip()
JPEGTRAN = "/opt/homebrew/bin/jpegtran"

PREY = "captures/sd/20260501_015148_gen1"
CLEAN = "captures/sd/20260430_011516_gen3"

STRATEGIES = [
    "esp32_norot",
    "fullpil_norot",
    "esp32_rot",
    "fullpil_rot",
    "full384sq_rot",
    "cropR_rot",
    "cropR_sq_rot",
    "center320_rot",
]

FRAMES = list(range(10))


def pil_to_jpeg(img, quality=85):
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=quality)
    return buf.getvalue()


def resize_prop(img, target=384):
    w, h = img.size
    if w > h:
        new_w, new_h = target, int(h * target / w)
    else:
        new_h, new_w = target, int(w * target / h)
    return img.resize((new_w, new_h), Image.Resampling.LANCZOS)


def lossless_crop(src_path):
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


def prepare(fpath, strategy):
    img = Image.open(fpath)
    w, h = img.size  # 640x480

    if strategy == "esp32_norot":
        return lossless_crop(fpath)

    elif strategy == "fullpil_norot":
        return pil_to_jpeg(resize_prop(img, 384))

    elif strategy == "esp32_rot":
        # Simulate ESP32 crop region but with rotation
        # Crop (64,48)-(448,432) = 384x384, then rotate
        cropped = img.crop((64, 48, 448, 432))
        rotated = cropped.rotate(90, expand=True)
        return pil_to_jpeg(rotated)

    elif strategy == "fullpil_rot":
        rotated = img.rotate(90, expand=True)  # 480x640
        return pil_to_jpeg(resize_prop(rotated, 384))

    elif strategy == "full384sq_rot":
        rotated = img.rotate(90, expand=True)
        return pil_to_jpeg(rotated.resize((384, 384), Image.Resampling.LANCZOS))

    elif strategy == "cropR_rot":
        # Crop right 128px, rotate CCW, proportional resize
        cropped = img.crop((0, 0, 512, 480))
        rotated = cropped.rotate(90, expand=True)  # 480x512
        return pil_to_jpeg(resize_prop(rotated, 384))

    elif strategy == "cropR_sq_rot":
        # Crop right 128px, rotate CCW, stretch to 384x384
        cropped = img.crop((0, 0, 512, 480))
        rotated = cropped.rotate(90, expand=True)
        return pil_to_jpeg(rotated.resize((384, 384), Image.Resampling.LANCZOS))

    elif strategy == "center320_rot":
        # Rotate, then center 320x320, upscale to 384
        rotated = img.rotate(90, expand=True)  # 480x640
        rw, rh = rotated.size
        cx, cy = rw // 2, rh // 2
        cropped = rotated.crop((cx - 160, cy - 160, cx + 160, cy + 160))
        return pil_to_jpeg(cropped.resize((384, 384), Image.Resampling.LANCZOS))

    raise ValueError(strategy)


def call_api(image_bytes):
    b64 = base64.b64encode(image_bytes).decode()
    resp = requests.post(
        API_URL,
        headers={"Content-Type": "application/json", "Authorization": f"Bearer {API_KEY}"},
        json={"image_base64": b64},
        timeout=15,
    )
    return resp.json().get("detected", False)


results = {}

for burst_label, burst_dir in [("PREY", PREY), ("CLEAN", CLEAN)]:
    print(f"\n{'='*60}")
    print(f"  {burst_label}: {burst_dir}")
    print(f"{'='*60}")
    for strat in STRATEGIES:
        hits = 0
        total = 0
        details = []
        for fi in FRAMES:
            fpath = os.path.join(burst_dir, f"f{fi:02d}.jpg")
            if not os.path.exists(fpath):
                continue
            img_bytes = prepare(fpath, strat)
            detected = call_api(img_bytes)
            total += 1
            if detected:
                hits += 1
            details.append("P" if detected else ".")
        key = (burst_label, strat)
        results[key] = (hits, total)
        det_str = " ".join(details)
        print(f"  {strat:16s}: {hits}/{total}  [{det_str}]")

# Final summary
print(f"\n{'='*70}")
print(f"{'Strategy':18s} | {'PREY hits':>10s} | {'CLEAN FP':>10s} | {'Net score':>10s}")
print(f"{'-'*70}")
for strat in STRATEGIES:
    ph, pt = results[("PREY", strat)]
    ch, ct = results[("CLEAN", strat)]
    score = ph - 3 * ch
    print(f"{strat:18s} | {ph:>2d}/{pt:<2d}       | {ch:>2d}/{ct:<2d}       | {score:>+3d}")
print(f"{'='*70}")
