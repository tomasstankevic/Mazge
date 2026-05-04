"""Simulate the proposed 'lossless crop + 2x downscale + drop chroma + rotate' pipeline
using PIL (proxy for what the lossless DCT version would produce).

Pipeline:
  640x480 source
    -> crop right 128px (PIL: lossless approximation)
    -> 2x downscale to 256x240
    -> grayscale
    -> rotate 90 CCW
    -> 240x256 grayscale JPEG
"""
import base64, io
from pathlib import Path
import requests
from PIL import Image

API_URL = "https://prey-detection.florian-mutel.workers.dev"
API_KEY = (Path(__file__).resolve().parent.parent / "API_key_prey_detector.txt").read_text().strip()


def call(jpg):
    r = requests.post(API_URL,
        headers={"Authorization": f"Bearer {API_KEY}", "Content-Type": "application/json"},
        json={"image_base64": base64.b64encode(jpg).decode()}, timeout=20)
    if r.status_code != 200:
        return f"HTTP{r.status_code}"
    return "YES" if r.json().get("detected") else "no"


burst = Path("captures/sd/20260501_015148_gen1")
frames = sorted(burst.glob("f??.jpg"))

hits = 0
for f in frames:
    img = Image.open(f)
    # Crop right 128 occlusion: 640x480 -> 512x480
    cropped = img.crop((0, 0, 512, 480))
    # 2x downscale: 512x480 -> 256x240
    smaller = cropped.resize((256, 240), Image.BOX)  # BOX = 2x average ~ DCT downscale
    # Grayscale
    gray = smaller.convert("L")
    # Rotate 90 CCW
    rotated = gray.rotate(90, expand=True)  # -> 240x256
    # Encode
    buf = io.BytesIO()
    rotated.save(buf, format="JPEG", quality=80)
    data = buf.getvalue()
    r = call(data)
    if r == "YES":
        hits += 1
    print(f"{f.name:<8} dims={rotated.size[0]}x{rotated.size[1]:<3} {len(data):>5}B -> {r}")

print(f"\nHits: {hits}/{len(frames)}")
