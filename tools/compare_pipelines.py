#!/usr/bin/env python3
"""Compare pipeline timing: full bilinear vs JPG_SCALE_2X+rotate."""
import base64, io, os, subprocess, requests, time
from PIL import Image

API_URL = "https://prey-detection.florian-mutel.workers.dev"
API_KEY = open(os.path.join(os.path.dirname(__file__), "..", "API_key_prey_detector.txt")).read().strip()

PREY_BURST = "captures/sd/20260501_015148_gen1"
OUT_DIR = "captures/pipeline_comparison"
JPEGTRAN = "/opt/homebrew/bin/jpegtran"

os.makedirs(OUT_DIR, exist_ok=True)


def api_check(img_bytes):
    b64 = base64.b64encode(img_bytes).decode()
    resp = requests.post(
        API_URL,
        headers={"Content-Type": "application/json", "Authorization": f"Bearer {API_KEY}"},
        json={"image_base64": b64},
        timeout=15,
    )
    return resp.json().get("detected", False)


def timed_api_call(img_bytes):
    """Returns (detected, b64_ms, api_ms, total_ms, payload_size)."""
    t0 = time.time()
    b64 = base64.b64encode(img_bytes).decode()
    t1 = time.time()
    resp = requests.post(
        API_URL,
        headers={"Content-Type": "application/json", "Authorization": f"Bearer {API_KEY}"},
        json={"image_base64": b64},
        timeout=15,
    )
    t2 = time.time()
    detected = resp.json().get("detected", False)
    return detected, (t1-t0)*1000, (t2-t1)*1000, (t2-t0)*1000, len(img_bytes)


def pipeline_full(fpath):
    """Full decode: crop right 128px, rotate 90 CCW, bilinear resize to 360x384."""
    img = Image.open(fpath)
    cropped = img.crop((0, 0, 512, 480))
    rotated = cropped.rotate(90, expand=True)
    resized = rotated.resize((360, 384), Image.Resampling.LANCZOS)
    buf = io.BytesIO()
    resized.save(buf, format="JPEG", quality=80)
    return buf.getvalue()


def pipeline_scale2x_rotate(fpath):
    """JPG_SCALE_2X: decode at 320x240, rotate 90 CCW -> 240x320."""
    img = Image.open(fpath)
    half = img.resize((320, 240), Image.Resampling.BOX)
    rotated = half.rotate(90, expand=True)  # -> 240x320
    buf = io.BytesIO()
    rotated.save(buf, format="JPEG", quality=80)
    return buf.getvalue()


PIPELINES = [
    ("full(360x384)", pipeline_full),
    ("scale2x+rot(240x320)", pipeline_scale2x_rotate),
]

print(f"Timing comparison on: {PREY_BURST}")
print(f"Each frame: preprocess + base64 + API call")
print()

for label, fn in PIPELINES:
    print(f"--- {label} ---")
    print(f"  {'Frame':<5} {'Prep ms':>8} {'B64 ms':>8} {'API ms':>8} {'Total ms':>9} {'Size':>7} {'Det'}")
    prep_times = []
    api_times = []
    total_times = []
    sizes = []
    hits = 0

    for fi in range(10):
        fpath = os.path.join(PREY_BURST, f"f{fi:02d}.jpg")
        if not os.path.exists(fpath):
            continue

        tp0 = time.time()
        img_bytes = fn(fpath)
        tp1 = time.time()
        prep_ms = (tp1 - tp0) * 1000

        detected, b64_ms, api_ms, call_total_ms, size = timed_api_call(img_bytes)
        total_ms = prep_ms + call_total_ms

        prep_times.append(prep_ms)
        api_times.append(api_ms)
        total_times.append(total_ms)
        sizes.append(size)
        if detected:
            hits += 1

        tag = "PREY" if detected else "—"
        print(f"  f{fi:02d}   {prep_ms:>7.1f} {b64_ms:>7.1f} {api_ms:>7.1f} {total_ms:>8.1f} {size:>6} {tag}")

    print(f"  ---")
    print(f"  AVG   {sum(prep_times)/len(prep_times):>7.1f} {'':>8} {sum(api_times)/len(api_times):>7.1f} {sum(total_times)/len(total_times):>8.1f} {sum(sizes)//len(sizes):>6} {hits}/10")
    print(f"  Payload: avg {sum(sizes)//len(sizes)} B, b64 avg {int(sum(sizes)*4/3)//len(sizes)} B")
    print()
