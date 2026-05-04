"""Validate Python lossless 2x crop+rotate against API on full prey burst."""
import base64
import sys
from pathlib import Path
import requests

sys.path.insert(0, str(Path(__file__).resolve().parent))
from jpeg_lossless_crop_2x_rotate_gray import process

API_URL = "https://prey-detection.florian-mutel.workers.dev"
API_KEY = (Path(__file__).resolve().parent.parent / "API_key_prey_detector.txt").read_text().strip()


def call(jpg):
    r = requests.post(API_URL,
        headers={"Authorization": f"Bearer {API_KEY}", "Content-Type": "application/json"},
        json={"image_base64": base64.b64encode(jpg).decode()}, timeout=20)
    if r.status_code != 200:
        return f"HTTP{r.status_code}"
    return "YES" if r.json().get("detected") else "no"


burst = Path(sys.argv[1] if len(sys.argv) > 1 else "captures/sd/20260501_015148_gen1")
frames = sorted(burst.glob("f??.jpg"))

hits = 0
for f in frames:
    src = f.read_bytes()
    out = process(src)
    r = call(out)
    if r == "YES":
        hits += 1
    print(f"{f.name:<8} src={len(src):>6} out={len(out):>5}  -> {r}")

print(f"\nHits: {hits}/{len(frames)}")
