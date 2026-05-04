"""Probe API limits to determine if 384x384 is dimensional or file-size limit."""
import base64
import io
from pathlib import Path
import requests
from PIL import Image

API_URL = "https://prey-detection.florian-mutel.workers.dev"
API_KEY = (Path(__file__).resolve().parent.parent / "API_key_prey_detector.txt").read_text().strip()


def call(jpg):
    r = requests.post(
        API_URL,
        headers={"Authorization": f"Bearer {API_KEY}", "Content-Type": "application/json"},
        json={"image_base64": base64.b64encode(jpg).decode()},
        timeout=30,
    )
    if r.status_code != 200:
        return f"HTTP{r.status_code} {r.text[:80]}"
    return "YES" if r.json().get("detected") else "no"


# Source rotated 90 CCW: 480x640
src = Image.open("captures/sd/20260501_015148_gen1/f05.jpg").rotate(90, expand=True)

# Tests: vary dims and quality independently to see what triggers HTTP 400
tests = [
    ("384x384 q=100 (large file)",    src.crop((0, 0, 384, 384)), 100),
    ("384x384 q=80",                  src.crop((0, 0, 384, 384)), 80),
    ("400x400 q=80 (slightly bigger)", src.crop((0, 0, 400, 400)), 80),
    ("480x480 q=80",                  src.crop((0, 0, 480, 480)), 80),
    ("384x480 q=80 (W=384, H=480)",   src.crop((0, 0, 384, 480)), 80),
    ("480x640 q=10 (small file)",     src,                          10),
    ("480x640 q=80 (full)",           src,                          80),
]
for label, img, q in tests:
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=q)
    data = buf.getvalue()
    print(f"{label:<32} dims={img.size[0]}x{img.size[1]:<4} {len(data):>6}B -> {call(data)}")
