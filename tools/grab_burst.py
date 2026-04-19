#!/usr/bin/env python3
"""One-shot burst downloader."""
import requests, os, io
from datetime import datetime
from PIL import Image, ImageOps

IP = "192.168.0.41"
OUTDIR = "/Users/ruta/Tomas/repos/Mazge/captures"

stats = requests.get(f"http://{IP}/stats", timeout=5).json()
n_archives = stats["burstArchives"]
gen = stats["burstGen"]
counts = stats["burstCounts"]
print(f"Archives: {n_archives}, gen: {gen}, counts: {counts}")

for a in range(n_archives):
    count = counts[a]
    print(f"\nArchive {a}: {count} frames")
    frames = []
    for i in range(count):
        r = requests.get(f"http://{IP}/burst?a={a}&i={i}", timeout=10)
        if r.status_code == 200 and len(r.content) > 100:
            frames.append(r.content)
            print(f"  frame {i}: {len(r.content)} bytes")
        else:
            print(f"  frame {i}: FAILED ({r.status_code})")

    if frames:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        dirname = f"burst_{ts}_gen{gen}"
        path = os.path.join(OUTDIR, dirname)
        os.makedirs(path, exist_ok=True)
        for i, jpg in enumerate(frames):
            img = Image.open(io.BytesIO(jpg))
            img = img.rotate(90, expand=True)
            img = ImageOps.autocontrast(img)
            fname = os.path.join(path, f"frame_{i:02d}.jpg")
            img.save(fname, quality=95)
            print(f"  Saved {fname} ({img.size[0]}x{img.size[1]})")
        print(f"Done: {len(frames)} frames -> {path}")
