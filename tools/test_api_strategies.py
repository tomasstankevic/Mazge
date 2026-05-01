#!/usr/bin/env python3
"""Test prey API with different crop strategies on burst frames.

Strategies:
  1. esp32    - Fixed center crop 384x384 at (64,48), lossless jpegtran
  2. rpi      - YOLO cat bbox crop, proportional resize to 384, PIL JPEG (like RPi)  
  3. fullpil  - Full frame resized proportionally to 384, PIL JPEG
"""
import base64, json, os, sys, subprocess, tempfile, io
import requests
import numpy as np
from PIL import Image

API_URL = "https://prey-detection.florian-mutel.workers.dev"
API_KEY = open(os.path.join(os.path.dirname(__file__), "..", "API_key_prey_detector.txt")).read().strip()
JPEGTRAN = "/opt/homebrew/bin/jpegtran"

burst_dir = sys.argv[1] if len(sys.argv) > 1 else "captures/sd/20260501_015148_gen1"
strategy = sys.argv[2] if len(sys.argv) > 2 else "rpi"


def resize_proportionally(img: Image.Image, target: int = 384) -> Image.Image:
    """Resize proportionally to fit within target (same as RPi common.py)."""
    w, h = img.size
    if w > h:
        new_w = target
        new_h = int(h * (target / w))
    else:
        new_h = target
        new_w = int(w * (target / h))
    return img.resize((new_w, new_h), Image.Resampling.LANCZOS)


def pil_to_jpeg_bytes(img: Image.Image) -> bytes:
    buf = io.BytesIO()
    img.save(buf, format="JPEG")
    buf.seek(0)
    return buf.read()


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


def call_api(image_bytes: bytes) -> dict:
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
    return resp.json()


# Try to use YOLO for cat detection (optional, for rpi strategy)
yolo_net = None
if strategy == "rpi":
    try:
        import cv2
        # Use the NCNN model from catflap-prey-detector
        model_base = os.path.join(os.path.dirname(__file__), "..", "..",
                                   "catflap-prey-detector", "models",
                                   "yolo11n_ncnn_model_384_640", "model.ncnn")
        param_path = model_base + ".param"
        bin_path = model_base + ".bin"
        if os.path.exists(param_path) and os.path.exists(bin_path):
            yolo_net = cv2.dnn.readNet(param_path, bin_path)
            print(f"Loaded YOLO NCNN model")
        else:
            print(f"YOLO model not found at {model_base}, falling back to simple center crop for rpi")
    except Exception as e:
        print(f"Could not load YOLO: {e}, using simple crop")


def detect_cat_bbox_simple(img_pil: Image.Image) -> tuple[int, int, int, int] | None:
    """Simple brightness-based cat detection for IR images."""
    import cv2
    img_np = np.array(img_pil.convert("L"))
    # Threshold for bright areas (cat in IR)
    _, thresh = cv2.threshold(img_np, 80, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    # Largest contour
    c = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(c)
    # Pad by 10%
    pad_x = int(w * 0.1)
    pad_y = int(h * 0.1)
    img_w, img_h = img_pil.size
    x1 = max(0, x - pad_x)
    y1 = max(0, y - pad_y)
    x2 = min(img_w, x + w + pad_x)
    y2 = min(img_h, y + h + pad_y)
    return (x1, y1, x2, y2)


print(f"Strategy: {strategy}")
print(f"Burst: {burst_dir}")
print()

for i in range(10):
    fname = f"f{i:02d}.jpg"
    fpath = os.path.join(burst_dir, fname)
    if not os.path.exists(fpath):
        continue

    if strategy == "esp32":
        crop_bytes = lossless_crop(fpath)
        desc = f"lossless 384x384"
    elif strategy == "rpi":
        img = Image.open(fpath)
        bbox = detect_cat_bbox_simple(img)
        if bbox:
            cat_crop = img.crop(bbox)
            desc = f"cat bbox {bbox[2]-bbox[0]}x{bbox[3]-bbox[1]}"
        else:
            cat_crop = img
            desc = f"no cat found, full frame"
        resized = resize_proportionally(cat_crop, 384)
        crop_bytes = pil_to_jpeg_bytes(resized)
        desc += f" -> {resized.size[0]}x{resized.size[1]}"
    elif strategy == "fullpil":
        img = Image.open(fpath)
        resized = resize_proportionally(img, 384)
        crop_bytes = pil_to_jpeg_bytes(resized)
        desc = f"full {img.size[0]}x{img.size[1]} -> {resized.size[0]}x{resized.size[1]}"
    else:
        print(f"Unknown strategy: {strategy}")
        sys.exit(1)

    data = call_api(crop_bytes)
    detected = data.get("detected", False)
    tag = "PREY!" if detected else "no prey"
    print(f"{fname} ({desc}, {len(crop_bytes)}B): {tag}")
