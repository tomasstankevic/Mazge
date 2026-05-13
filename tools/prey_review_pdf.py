"""Generate PDF report of all prey-positive bursts.

For each burst:
  - Header: folder name, CET timestamp, overall verdict
  - Grid: 10 thumbnails (rotated upright, ~50mm wide)
  - Per-frame label: f00..f09 + verdict (PREY/clear/-)

Requires: reportlab, Pillow.
"""
import datetime
import io
import json
import sys
import zoneinfo
from pathlib import Path

from PIL import Image
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas

ROOT = Path(sys.argv[1] if len(sys.argv) > 1 else "captures/prey_review")
OUT = ROOT / "report.pdf"
TZ = zoneinfo.ZoneInfo("Europe/Berlin")

PAGE_W, PAGE_H = A4
MARGIN = 12 * mm
COLS = 10
THUMB_W = 16 * mm
GAP = 1.5 * mm
HEADER_H = 13 * mm
ROW_H = THUMB_W * (4 / 3) + 2 * mm  # thumbs are landscape -> rotated portrait
BURST_H = HEADER_H + ROW_H


def load_bursts(root: Path):
    bursts = []
    for d in sorted(root.iterdir()):
        if not d.is_dir():
            continue
        meta = d / "meta.json"
        if not meta.exists():
            continue
        try:
            m = json.loads(meta.read_text())
        except Exception:
            continue
        epoch = m.get("epoch")
        when = (datetime.datetime.fromtimestamp(epoch, tz=TZ)
                .strftime("%Y-%m-%d %H:%M:%S")
                if epoch else "(no epoch)")
        api = (m.get("apiResults") or [-1] * 10)[:10]
        # Map frame index -> filename
        img_by_idx = {}
        for f in d.iterdir():
            if f.suffix.lower() != ".jpg":
                continue
            n = f.stem
            if not n.startswith("f"):
                continue
            try:
                i = int(n[1:3])
            except ValueError:
                continue
            img_by_idx[i] = f
        bursts.append({
            "name": d.name, "when": when, "api": api,
            "img": img_by_idx, "result": m.get("apiResult"),
        })
    return bursts


def load_thumb(path: Path, target_w_px: int = 240):
    """Open image, rotate 90 CCW, downscale to target width, return PIL Image."""
    try:
        img = Image.open(path)
    except Exception:
        return None
    # Camera is sideways: source is 640x480 landscape, rotate 90 CCW -> 480x640 portrait
    rot = img.rotate(90, expand=True)
    w, h = rot.size
    if w > target_w_px:
        scale = target_w_px / w
        rot = rot.resize((target_w_px, int(h * scale)), Image.LANCZOS)
    return rot


def draw_burst(c: canvas.Canvas, burst: dict, top_y: float):
    """Draw one burst block whose top edge is at top_y. Returns bottom_y."""
    x0 = MARGIN
    # Header
    c.setFont("Helvetica-Bold", 9)
    c.setFillColorRGB(0.6, 0.8, 1.0)
    c.drawString(x0, top_y - 4 * mm, burst["name"])
    c.setFont("Helvetica", 8)
    c.setFillColorRGB(0.7, 0.7, 0.7)
    c.drawString(x0 + 55 * mm, top_y - 4 * mm,
                 f"{burst['when']} CET")
    verdict = "PREY" if burst["result"] == 1 else "no prey" if burst["result"] == 0 else "?"
    color = (1.0, 0.4, 0.4) if verdict == "PREY" else (0.4, 1.0, 0.4)
    c.setFillColorRGB(*color)
    c.setFont("Helvetica-Bold", 8)
    c.drawRightString(PAGE_W - MARGIN, top_y - 4 * mm, verdict)
    # Thumbnails row
    grid_y = top_y - HEADER_H
    avail_w = PAGE_W - 2 * MARGIN
    cell_w = (avail_w - GAP * (COLS - 1)) / COLS
    cell_h = cell_w * (4 / 3)  # rotated frames: 480/640 = 0.75, so portrait
    for i in range(10):
        cx = x0 + i * (cell_w + GAP)
        cy = grid_y - cell_h
        flag = burst["api"][i] if i < len(burst["api"]) else -1
        # Border colour
        if flag == 1:
            c.setStrokeColorRGB(1.0, 0.3, 0.3)
            c.setLineWidth(1.5)
        elif flag == 0:
            c.setStrokeColorRGB(0.4, 0.9, 0.4)
            c.setLineWidth(0.5)
        else:
            c.setStrokeColorRGB(0.4, 0.4, 0.4)
            c.setLineWidth(0.3)
        c.rect(cx, cy, cell_w, cell_h)
        # Image
        img_path = burst["img"].get(i)
        if img_path is not None:
            try:
                pil = load_thumb(img_path)
                buf = io.BytesIO()
                pil.save(buf, format="JPEG", quality=70)
                buf.seek(0)
                from reportlab.lib.utils import ImageReader
                c.drawImage(ImageReader(buf), cx + 0.5, cy + 3 * mm,
                            cell_w - 1, cell_h - 3.5 * mm,
                            preserveAspectRatio=True, anchor='c')
            except Exception:
                pass
        # Label below image
        c.setFont("Helvetica-Bold" if flag == 1 else "Helvetica", 6)
        if flag == 1:
            c.setFillColorRGB(1.0, 0.5, 0.5)
        elif flag == 0:
            c.setFillColorRGB(0.5, 0.9, 0.5)
        else:
            c.setFillColorRGB(0.6, 0.6, 0.6)
        label = f"f{i:02d} "
        if flag == 1:
            label += "PREY"
        elif flag == 0:
            label += "ok"
        else:
            label += "-"
        c.drawString(cx + 1, cy + 0.5 * mm, label)
    return grid_y - cell_h - 3 * mm


def main():
    bursts = load_bursts(ROOT)
    if not bursts:
        print("No bursts found.")
        return
    print(f"Generating PDF for {len(bursts)} bursts -> {OUT}")
    c = canvas.Canvas(str(OUT), pagesize=A4)
    c.setTitle("Prey detections review")

    def paint_bg():
        c.setFillColorRGB(0.07, 0.07, 0.13)
        c.rect(0, 0, PAGE_W, PAGE_H, stroke=0, fill=1)

    paint_bg()

    # Cover header on first page
    y = PAGE_H - MARGIN
    c.setFont("Helvetica-Bold", 14)
    c.setFillColorRGB(0.4, 1.0, 0.7)
    c.drawString(MARGIN, y, "Prey detection review")
    y -= 6 * mm
    c.setFont("Helvetica", 9)
    c.setFillColorRGB(0.7, 0.7, 0.7)
    c.drawString(MARGIN, y,
        f"{len(bursts)} prey-positive bursts. Red-bordered = API flagged prey, "
        "green = checked clear, gray = not checked (early exit).")
    y -= 8 * mm

    for i, b in enumerate(bursts):
        # Page break if not enough space
        if y - BURST_H < MARGIN:
            c.showPage()
            paint_bg()
            y = PAGE_H - MARGIN
        y = draw_burst(c, b, y)

    c.save()
    print(f"Wrote {OUT} ({OUT.stat().st_size // 1024} KB)")


if __name__ == "__main__":
    main()
