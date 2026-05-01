#!/usr/bin/env python3
"""Test multiple crop/resize strategies on prey and non-prey bursts.

Strategies tested:
  esp32       - Lossless jpegtran crop 384x384 at (64,48) [current ESP32 method]
  fullpil     - Full 640x480 -> proportional resize to max 384 (384x288) via PIL
  full384sq   - Full 640x480 -> square resize 384x384 (stretch) via PIL
  top384      - Top half crop 640x240 -> resize to 384x184 via PIL
  center320   - Center crop 320x320 -> resize to 384x384 via PIL
  center256   - Center crop 256x256 -> resize to 384x384 via PIL
  left384     - Left crop 384x480 -> proportional resize via PIL
  nocrop      - Raw 640x480 JPEG sent as-is (exceeds 384 max but let's see)
"""
import base64, io, json, os, subprocess, sys, tempfile
import requests
from PIL import Image

API_URL = "https://prey-detection.florian-mutel.workers.dev"
API_KEY = open(os.path.join(os.path.dirname(__file__), "..", "API_key_prey_detector.txt")).read().strip()
JPEGTRAN = "/opt/homebrew/bin/jpegtran"

STRATEGIES = ["esp32", "fullpil", "full384sq", "top384", "center320", "center256", "left384", "nocrop"]

prey_burst = "captures/sd/20260501_015148_gen1"
clean_burst = "captures/sd/20260430_011516_gen3"

# Allow override
if len(sys.argv) > 1:
    prey_burst = sys.argv[1]
if len(sys.argv) > 2:
    clean_burst = sys.argv[2]

FRAMES = [5, 6, 7, 8]  # focus on trigger frames


def pil_to_jpeg(img: Image.Image, quality: int = 85) -> bytes:
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=quality)
    return buf.getvalue()


def resize_prop(img: Image.Image, target: int = 384) -> Image.Image:
    w, h = img.size
    if w > h:
        new_w, new_h = target, int(h * target / w)
    else:
        new_h, new_w = target, int(w * target / h)
    return img.resize((new_w, new_h), Image.Resampling.LANCZOS)


def lossless_crop(src_path: str) -> bytes:
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


def prepare_image(fpath: str, strategy: str) -> tuple[bytes, str]:
    """Returns (jpeg_bytes, description)."""
    if strategy == "esp32":
        return lossless_crop(fpath), "lossless 384x384"

    img = Image.open(fpath)
    w, h = img.size  # 640x480

    if strategy == "fullpil":
        out = resize_prop(img, 384)
        return pil_to_jpeg(out), f"{out.size[0]}x{out.size[1]}"

    elif strategy == "full384sq":
        out = img.resize((384, 384), Image.Resampling.LANCZOS)
        return pil_to_jpeg(out), "384x384 stretched"

    elif strategy == "top384":
        # Top half: likely shows cat approaching
        crop = img.crop((0, 0, w, h // 2))  # 640x240
        out = resize_prop(crop, 384)
        return pil_to_jpeg(out), f"top half -> {out.size[0]}x{out.size[1]}"

    elif strategy == "center320":
        # Center 320x320, then upscale to 384
        cx, cy = w // 2, h // 2
        crop = img.crop((cx - 160, cy - 160, cx + 160, cy + 160))
        out = crop.resize((384, 384), Image.Resampling.LANCZOS)
        return pil_to_jpeg(out), "center 320->384"

    elif strategy == "center256":
        # Tighter center 256x256, upscale to 384
        cx, cy = w // 2, h // 2
        crop = img.crop((cx - 128, cy - 128, cx + 128, cy + 128))
        out = crop.resize((384, 384), Image.Resampling.LANCZOS)
        return pil_to_jpeg(out), "center 256->384"

    elif strategy == "left384":
        # Left 384 columns (skip right 256px which may be occluded)
        crop = img.crop((0, 0, 384, h))  # 384x480
        out = resize_prop(crop, 384)
        return pil_to_jpeg(out), f"left 384 -> {out.size[0]}x{out.size[1]}"

    elif strategy == "nocrop":
        return pil_to_jpeg(img), f"raw {w}x{h}"

    raise ValueError(f"Unknown strategy: {strategy}")


def call_api(image_bytes: bytes) -> bool:
    b64 = base64.b64encode(image_bytes).decode()
    resp = requests.post(
        API_URL,
        headers={
            "Content-Type": "application/json",
            "Authorization": f"Bearer {API_KEY}",
        },
        json={"image_base64": b64},
        timeout=15,
    )
    return resp.json().get("detected", False)


def test_burst(burst_dir: str, label: str) -> dict:
    """Test all strategies on a burst. Returns {strategy: {frame: bool}}."""
    results = {}
    for strat in STRATEGIES:
        results[strat] = {}
        for fi in FRAMES:
            fpath = os.path.join(burst_dir, f"f{fi:02d}.jpg")
            if not os.path.exists(fpath):
                results[strat][fi] = None
                continue
            img_bytes, desc = prepare_image(fpath, strat)
            detected = call_api(img_bytes)
            results[strat][fi] = detected
            tag = "PREY" if detected else "  - "
            print(f"  [{label}] {strat:12s} f{fi:02d} ({desc:>25s}, {len(img_bytes):5d}B): {tag}")
    return results


print(f"PREY burst:  {prey_burst}")
print(f"CLEAN burst: {clean_burst}")
print(f"Frames: {FRAMES}")
print(f"Strategies: {STRATEGIES}")
print()

print("=== PREY BURST ===")
prey_results = test_burst(prey_burst, "PREY ")

print()
print("=== CLEAN BURST (no prey expected) ===")
clean_results = test_burst(clean_burst, "CLEAN")

# Summary table
print()
print("=" * 80)
print(f"{'Strategy':14s} | {'PREY burst hits':>16s} | {'CLEAN false pos':>16s} | Score")
print("-" * 80)
for strat in STRATEGIES:
    prey_hits = sum(1 for v in prey_results[strat].values() if v is True)
    prey_total = sum(1 for v in prey_results[strat].values() if v is not None)
    clean_fp = sum(1 for v in clean_results[strat].values() if v is True)
    clean_total = sum(1 for v in clean_results[strat].values() if v is not None)
    # Score: hits - 3*false_positives
    score = prey_hits - 3 * clean_fp
    print(f"{strat:14s} | {prey_hits:>2d}/{prey_total:<2d}            | {clean_fp:>2d}/{clean_total:<2d}            | {score:+d}")
print("=" * 80)
