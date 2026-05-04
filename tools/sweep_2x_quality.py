"""Sweep output JPEG quality for the lossless 2x rotate pipeline."""
import sys
import base64
from pathlib import Path
import requests

sys.path.insert(0, str(Path(__file__).resolve().parent))
import jpeg_lossless_crop_2x_rotate_gray as m

API_URL = "https://prey-detection.florian-mutel.workers.dev"
API_KEY = (Path(__file__).resolve().parent.parent / "API_key_prey_detector.txt").read_text().strip()


def call(jpg):
    r = requests.post(
        API_URL,
        headers={"Authorization": f"Bearer {API_KEY}", "Content-Type": "application/json"},
        json={"image_base64": base64.b64encode(jpg).decode()},
        timeout=20,
    )
    if r.status_code != 200:
        return f"HTTP{r.status_code}"
    return "YES" if r.json().get("detected") else "no"


frames = sorted(Path("captures/sd/20260501_015148_gen1").glob("f??.jpg"))
orig = m.quality_qt_natural


def make_override(qq):
    return lambda _ignored: orig(qq)


for q in [80, 85, 90, 95]:
    m.quality_qt_natural = make_override(q)
    hits = 0
    sizes = []
    for f in frames:
        out = m.process(f.read_bytes())
        sizes.append(len(out))
        if call(out) == "YES":
            hits += 1
    print(f"q={q:3d}: hits={hits}/10, avg_size={sum(sizes)//len(sizes)}B")
m.quality_qt_natural = orig
