"""Confirm API accepts non-square images where both dims <= 384."""
import base64, io
from pathlib import Path
import requests
from PIL import Image

API_URL = "https://prey-detection.florian-mutel.workers.dev"
API_KEY = (Path(__file__).resolve().parent.parent / "API_key_prey_detector.txt").read_text().strip()


def call(jpg):
    r = requests.post(API_URL,
        headers={"Authorization": f"Bearer {API_KEY}", "Content-Type": "application/json"},
        json={"image_base64": base64.b64encode(jpg).decode()}, timeout=30)
    if r.status_code != 200:
        return f"HTTP{r.status_code} {r.text[:80]}"
    return "YES" if r.json().get("detected") else "no"


# 480x640 rotated source
src = Image.open("captures/sd/20260501_015148_gen1/f05.jpg").rotate(90, expand=True)

tests = [
    # Rectangular, both dims <= 384
    ("288x384 (3:4 portrait)",  src.crop((0, 0, 288, 384))),
    ("384x288 (4:3 landscape)", src.resize((384, 288))),  # squeeze
    ("240x384 (5:8 portrait)",  src.crop((0, 0, 240, 384))),
    ("384x240 (8:5)",           src.resize((384, 240))),
    # Boundary
    ("384x384 (square)",        src.crop((0, 0, 384, 384))),
    ("384x385 (one over)",      src.resize((384, 385))),
]

for label, img in tests:
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=80)
    data = buf.getvalue()
    print(f"{label:<32} dims={img.size[0]}x{img.size[1]:<4} {len(data):>6}B -> {call(data)}")
