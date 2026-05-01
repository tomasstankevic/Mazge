#!/usr/bin/env python3
"""Save all crop strategy images for visual inspection."""
import io, os, subprocess, sys, tempfile
from PIL import Image

JPEGTRAN = "/opt/homebrew/bin/jpegtran"
STRATEGIES = ["esp32", "fullpil", "full384sq", "top384", "center320", "center256", "left384", "nocrop"]
FRAMES = [5, 6, 7, 8]

prey_burst = "captures/sd/20260501_015148_gen1"
clean_burst = "captures/sd/20260430_011516_gen3"
out_dir = "captures/crop_comparison"


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


def prepare_image(fpath, strategy):
    if strategy == "esp32":
        return lossless_crop(fpath)
    img = Image.open(fpath)
    w, h = img.size
    if strategy == "fullpil":
        return pil_to_jpeg(resize_prop(img, 384))
    elif strategy == "full384sq":
        return pil_to_jpeg(img.resize((384, 384), Image.Resampling.LANCZOS))
    elif strategy == "top384":
        return pil_to_jpeg(resize_prop(img.crop((0, 0, w, h // 2)), 384))
    elif strategy == "center320":
        cx, cy = w // 2, h // 2
        return pil_to_jpeg(img.crop((cx-160, cy-160, cx+160, cy+160)).resize((384, 384), Image.Resampling.LANCZOS))
    elif strategy == "center256":
        cx, cy = w // 2, h // 2
        return pil_to_jpeg(img.crop((cx-128, cy-128, cx+128, cy+128)).resize((384, 384), Image.Resampling.LANCZOS))
    elif strategy == "left384":
        return pil_to_jpeg(resize_prop(img.crop((0, 0, 384, h)), 384))
    elif strategy == "nocrop":
        return pil_to_jpeg(img)


# API results from previous run
API_RESULTS = {
    "prey": {
        "esp32":     {5: False, 6: True,  7: False, 8: False},
        "fullpil":   {5: False, 6: True,  7: True,  8: True},
        "full384sq": {5: False, 6: True,  7: True,  8: True},
        "top384":    {5: False, 6: False, 7: False, 8: False},
        "center320": {5: False, 6: False, 7: True,  8: False},
        "center256": {5: False, 6: False, 7: False, 8: False},
        "left384":   {5: False, 6: False, 7: False, 8: False},
        "nocrop":    {5: False, 6: False, 7: False, 8: False},
    },
    "clean": {
        "esp32":     {5: False, 6: False, 7: False, 8: False},
        "fullpil":   {5: False, 6: False, 7: False, 8: False},
        "full384sq": {5: False, 6: False, 7: False, 8: False},
        "top384":    {5: False, 6: False, 7: False, 8: False},
        "center320": {5: False, 6: False, 7: False, 8: False},
        "center256": {5: False, 6: False, 7: False, 8: False},
        "left384":   {5: False, 6: False, 7: False, 8: False},
        "nocrop":    {5: False, 6: False, 7: False, 8: False},
    },
}

for burst_label, burst_dir in [("prey", prey_burst), ("clean", clean_burst)]:
    for strat in STRATEGIES:
        for fi in FRAMES:
            fpath = os.path.join(burst_dir, f"f{fi:02d}.jpg")
            if not os.path.exists(fpath):
                continue
            img_bytes = prepare_image(fpath, strat)
            detected = API_RESULTS[burst_label][strat].get(fi, False)
            tag = "PREY" if detected else "noprey"
            subdir = os.path.join(out_dir, burst_label, strat)
            os.makedirs(subdir, exist_ok=True)
            fname = f"f{fi:02d}_{tag}.jpg"
            with open(os.path.join(subdir, fname), "wb") as f:
                f.write(img_bytes)

# Now create contact sheets: one per burst, rows=strategies, cols=frames
from PIL import ImageDraw

for burst_label in ["prey", "clean"]:
    cell_w, cell_h = 200, 200
    margin = 2
    label_w = 100
    cols = len(FRAMES)
    rows = len(STRATEGIES)
    sheet_w = label_w + cols * (cell_w + margin) + margin
    sheet_h = 20 + rows * (cell_h + margin + 14) + margin
    sheet = Image.new("RGB", (sheet_w, sheet_h), (30, 30, 30))
    draw = ImageDraw.Draw(sheet)

    # Header
    draw.text((label_w + 2, 2), f"{burst_label.upper()} burst", fill=(255, 255, 100))
    for c, fi in enumerate(FRAMES):
        x = label_w + margin + c * (cell_w + margin)
        draw.text((x + 2, 2), f"f{fi:02d}", fill=(200, 200, 200))

    for r, strat in enumerate(STRATEGIES):
        y = 20 + margin + r * (cell_h + margin + 14)
        draw.text((2, y + cell_h // 2 - 6), strat, fill=(200, 200, 200))
        for c, fi in enumerate(FRAMES):
            x = label_w + margin + c * (cell_w + margin)
            img_path = os.path.join(out_dir, burst_label, strat, f"f{fi:02d}_{'PREY' if API_RESULTS[burst_label][strat].get(fi) else 'noprey'}.jpg")
            if os.path.exists(img_path):
                img = Image.open(img_path)
                # Fit into cell_w x cell_h maintaining aspect
                img.thumbnail((cell_w, cell_h), Image.Resampling.LANCZOS)
                # Center in cell
                px = x + (cell_w - img.width) // 2
                py = y + (cell_h - img.height) // 2
                sheet.paste(img, (px, py))
                # Border color: green=PREY, gray=noprey
                detected = API_RESULTS[burst_label][strat].get(fi, False)
                color = (0, 255, 0) if detected else (80, 80, 80)
                draw.rectangle([x-1, y-1, x + cell_w, y + cell_h], outline=color, width=2)
                # Label
                tag = "PREY" if detected else ""
                if tag:
                    draw.text((x + 2, y + cell_h - 12), tag, fill=(255, 0, 0))

    out_path = os.path.join(out_dir, f"contact_{burst_label}.jpg")
    sheet.save(out_path, quality=95)
    print(f"Saved {out_path}")

print("Done — individual images in", out_dir)
