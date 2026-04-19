#!/usr/bin/env python3
"""Generate a visual report of cat/prey detections from prey_analysis.json.

Produces:
  report/
    01_burst_XXXXXXXX_genN_cat65pct_noprey.jpg   — annotated full frame
    01_burst_XXXXXXXX_genN_api_crop.jpg           — cropped image sent to API
    ...
    contact_sheet.jpg                              — all detections side-by-side

Usage:
  uv run python tools/generate_report.py [captures/prey_analysis.json]
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


CAPTURES_DIR = Path(__file__).resolve().parent.parent / "captures"
REPORT_DIR = CAPTURES_DIR / "report"


def draw_label(draw: ImageDraw.ImageDraw, x: int, y: int, text: str, fill: str, bg: str = "black"):
    """Draw text with a background rectangle."""
    bbox = draw.textbbox((x, y), text)
    draw.rectangle([bbox[0]-2, bbox[1]-1, bbox[2]+2, bbox[3]+1], fill=bg)
    draw.text((x, y), text, fill=fill)


def make_annotated(burst_dir: Path, result: dict, idx: int) -> tuple[Image.Image | None, Image.Image | None]:
    """Create annotated full-frame and crop images for one burst."""
    best_cat = result.get("best_cat")

    # Pick frame: YOLO best cat frame > variance-selected frame > frame 0
    if best_cat:
        frame_idx = best_cat["frame"]
    elif "best_frame_by_variance" in result:
        frame_idx = result["best_frame_by_variance"]
    else:
        frame_idx = 0

    frame_path = burst_dir / f"frame_{frame_idx:02d}.jpg"
    if not frame_path.exists():
        return None, None

    # -- Full annotated frame --
    img = Image.open(frame_path)
    draw = ImageDraw.Draw(img)

    # Cat bounding box (if detected)
    if best_cat:
        bx, by, bw, bh = best_cat["box"]
        conf = best_cat["confidence"]
        draw.rectangle([bx, by, bx+bw, by+bh], outline="lime", width=2)
        draw_label(draw, bx, max(0, by-14),
                   f'cat {conf:.0%}', fill="lime")

    # Motion box
    mbox = result.get("motion_box")
    if mbox:
        mx, my, mw, mh = mbox
        draw.rectangle([mx, my, mx+mw, my+mh], outline="yellow", width=1)

    # Prey result banner
    prey = result.get("prey_detected")
    api_ms = result.get("api_latency_ms", 0)
    if prey is True:
        banner = f"PREY DETECTED  ({api_ms:.0f}ms)"
        color = "red"
    elif prey is False:
        banner = f"No prey  ({api_ms:.0f}ms)"
        color = "lime"
    else:
        banner = "API skipped"
        color = "gray"

    # Cat status tag
    if best_cat:
        cat_tag = f"cat {best_cat['confidence']:.0%}"
    else:
        cat_tag = "no YOLO cat"

    # Bottom banner
    iw, ih = img.size
    draw.rectangle([0, ih-18, iw, ih], fill="black")
    draw.text((4, ih-16), f"{cat_tag} | {banner}", fill=color)

    # Top-left burst label
    draw_label(draw, 2, 2, f'{idx:02d} {result["burst"]}  f{frame_idx}', fill="white")

    # -- API crop --
    crop_path = burst_dir / "debug" / "api_crop.jpg"
    crop_img = None
    if crop_path.exists():
        crop_img = Image.open(crop_path)
        cdraw = ImageDraw.Draw(crop_img)
        cw, ch = crop_img.size
        cdraw.rectangle([0, ch-18, cw, ch], fill="black")
        cdraw.text((4, ch-16), f"API crop → {banner}", fill=color)

    return img, crop_img


def make_contact_sheet(images: list[tuple[int, str, Image.Image]], cols: int = 4, thumb_w: int = 280) -> Image.Image:
    """Tile thumbnails into a contact sheet."""
    thumbs = []
    for idx, label, img in images:
        w, h = img.size
        scale = thumb_w / w
        thumb = img.resize((thumb_w, int(h * scale)), Image.Resampling.LANCZOS)
        thumbs.append((idx, label, thumb))

    if not thumbs:
        return Image.new("RGB", (400, 100), "black")

    th = max(t[2].height for t in thumbs)
    rows = (len(thumbs) + cols - 1) // cols
    sheet = Image.new("RGB", (cols * thumb_w, rows * (th + 4)), (30, 30, 30))

    for i, (idx, label, thumb) in enumerate(thumbs):
        r, c = divmod(i, cols)
        x = c * thumb_w
        y = r * (th + 4)
        sheet.paste(thumb, (x, y))

    return sheet


def main():
    json_path = Path(sys.argv[1]) if len(sys.argv) > 1 else CAPTURES_DIR / "prey_analysis.json"
    if not json_path.exists():
        print(f"Not found: {json_path}")
        print("Run prey_analyzer.py first.")
        sys.exit(1)

    with open(json_path) as f:
        results = json.load(f)

    REPORT_DIR.mkdir(exist_ok=True)

    # Include ALL bursts, sort: cat-positive first (by conf desc), then no-cat
    cat_results = [r for r in results if r.get("cat_detections", 0) > 0]
    nocat_results = [r for r in results if r.get("cat_detections", 0) == 0]
    cat_results.sort(key=lambda r: r.get("best_cat", {}).get("confidence", 0), reverse=True)
    all_sorted = cat_results + nocat_results

    contact_items = []
    saved = 0

    for idx, r in enumerate(all_sorted, 1):
        burst_name = r["burst"]
        burst_dir = CAPTURES_DIR / burst_name
        conf = r.get("best_cat", {}).get("confidence", 0)
        prey = r.get("prey_detected", False)
        has_cat = r.get("cat_detections", 0) > 0
        prey_tag = "PREY" if prey else "noprey"
        cat_tag = f"cat{conf:.0%}".replace("%", "pct") if has_cat else "nocat"

        full_img, crop_img = make_annotated(burst_dir, r, idx)
        if full_img is None:
            continue

        # Save annotated full frame
        fname = f"{idx:02d}_{burst_name}_{cat_tag}_{prey_tag}.jpg"
        full_img.save(REPORT_DIR / fname, quality=92)

        # Save API crop
        if crop_img:
            cname = f"{idx:02d}_{burst_name}_api_crop.jpg"
            crop_img.save(REPORT_DIR / cname, quality=92)

        label = f"{burst_name}\n{'cat '+f'{conf:.0%}' if has_cat else 'no cat'} {prey_tag}"
        contact_items.append((idx, label, full_img))
        saved += 1
        cat_str = f"cat={conf:.0%}" if has_cat else "no cat"
        print(f"  [{idx:2d}] {burst_name}  {cat_str}  prey={prey}  api={r.get('api_latency_ms',0):.0f}ms")

    # Contact sheet
    if contact_items:
        sheet = make_contact_sheet(contact_items, cols=4)
        sheet.save(REPORT_DIR / "contact_sheet.jpg", quality=92)
        print(f"\nContact sheet: {REPORT_DIR / 'contact_sheet.jpg'}")

    print(f"\nReport: {saved} cat detections saved to {REPORT_DIR}/")


if __name__ == "__main__":
    main()
