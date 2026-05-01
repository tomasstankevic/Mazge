#!/usr/bin/env python3
"""Create contact sheets of f05-f07 from all bursts for visual prey inspection."""
from PIL import Image, ImageDraw
import os, sys

sd = sys.argv[1] if len(sys.argv) > 1 else "captures/sd"
bursts = sorted([d for d in os.listdir(sd) if d.startswith("2026") and os.path.isdir(os.path.join(sd, d))])

batch_size = 10
thumb_w, thumb_h = 213, 160
cols = 3  # f05, f06, f07
margin = 2
label_h = 16

for batch_idx in range(0, len(bursts), batch_size):
    batch = bursts[batch_idx : batch_idx + batch_size]
    rows = len(batch)
    sheet_w = cols * (thumb_w + margin) + margin
    sheet_h = rows * (thumb_h + label_h + margin) + margin
    sheet = Image.new("RGB", (sheet_w, sheet_h), (30, 30, 30))
    draw = ImageDraw.Draw(sheet)

    for r, burst in enumerate(batch):
        y = margin + r * (thumb_h + label_h + margin)
        draw.text((margin + 2, y), burst, fill=(255, 255, 100))
        for c, fi in enumerate([5, 6, 7]):
            x = margin + c * (thumb_w + margin)
            fpath = os.path.join(sd, burst, f"f{fi:02d}.jpg")
            if os.path.exists(fpath):
                img = Image.open(fpath).resize((thumb_w, thumb_h))
                sheet.paste(img, (x, y + label_h))
            else:
                draw.text((x + 10, y + label_h + 60), f"f{fi:02d} missing", fill=(255, 0, 0))

    out = os.path.join(sd, f"contact_f05f07_batch{batch_idx // batch_size}.jpg")
    sheet.save(out, quality=90)
    print(f"Saved {out} ({len(batch)} bursts)")

print("Done")
