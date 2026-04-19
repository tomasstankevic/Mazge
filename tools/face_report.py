#!/usr/bin/env python3
"""Generate a visual report of ALL frames where cat face/body was detected.

Runs YOLO + Haar cascade on every frame of every burst, annotates hits,
and generates:
  1. Individual annotated JPGs for every frame with a detection
  2. A per-burst summary strip (horizontal row of all 10 frames, hits highlighted)
  3. A master contact sheet of all bursts

Usage:
    cd Mazge && uv run python tools/face_report.py
"""
import cv2
import json
import logging
import sys
import time
import numpy as np
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

# Import from our prey_analyzer
sys.path.insert(0, str(Path(__file__).parent))
from prey_analyzer import YOLODetector, CatFaceDetector, CAT_CLASS_ID  # noqa: E402

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-5s %(message)s", datefmt="%H:%M:%S")
log = logging.getLogger("face_report")

CAPTURES = Path(__file__).resolve().parent.parent / "captures"
REPORT_DIR = CAPTURES / "face_report"

# ── Annotation helpers ────────────────────────────────────────────────

def annotate_frame(frame_bgr, yolo_dets, haar_faces, burst_name, frame_idx):
    """Draw YOLO and Haar boxes on frame. Returns annotated PIL Image."""
    img = Image.fromarray(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(img)
    w, h = img.size

    # YOLO detections (green)
    for d in yolo_dets:
        if d["class_id"] == CAT_CLASS_ID:
            x, y, bw, bh = d["x"], d["y"], d["w"], d["h"]
            draw.rectangle([x, y, x + bw, y + bh], outline="lime", width=2)
            draw.text((x, max(0, y - 12)), f'YOLO {d["confidence"]:.0%}', fill="lime")

    # Haar faces (cyan)
    for fx, fy, fw, fh in haar_faces:
        draw.rectangle([fx, fy, fx + fw, fy + fh], outline="cyan", width=2)
        draw.text((fx, max(0, fy - 12)), "Haar face", fill="cyan")

    # Banner
    has_yolo = any(d["class_id"] == CAT_CLASS_ID for d in yolo_dets)
    has_haar = len(haar_faces) > 0
    if has_yolo and has_haar:
        banner = "YOLO+Haar"
        color = "lime"
    elif has_yolo:
        banner = "YOLO only"
        color = "lime"
    elif has_haar:
        banner = "Haar only"
        color = "cyan"
    else:
        banner = "no detection"
        color = "gray"

    # Top label
    draw.text((2, 2), f"{burst_name} f{frame_idx}", fill="white")
    # Bottom banner
    draw.rectangle([0, h - 16, w, h], fill="black")
    draw.text((2, h - 14), banner, fill=color)

    return img


def make_burst_strip(frames_pil, burst_name, thumb_w=120, thumb_h=90):
    """Create a horizontal strip of all frames in a burst.
    
    Frames with detections get a colored border. Others get dimmed.
    """
    n = len(frames_pil)
    margin = 2
    strip_w = n * (thumb_w + margin) + margin
    strip_h = thumb_h + 20 + margin * 2  # 20px for label

    strip = Image.new("RGB", (strip_w, strip_h), (30, 30, 30))
    draw = ImageDraw.Draw(strip)
    # Burst label
    draw.text((4, 2), burst_name, fill="white")

    for i, (pil_img, has_det) in enumerate(frames_pil):
        thumb = pil_img.resize((thumb_w, thumb_h), Image.LANCZOS)
        if not has_det:
            # Dim non-detection frames
            arr = np.array(thumb).astype(np.float32) * 0.35
            thumb = Image.fromarray(arr.astype(np.uint8))
        x = margin + i * (thumb_w + margin)
        y = 20
        strip.paste(thumb, (x, y))

    return strip


# ── Main ──────────────────────────────────────────────────────────────

def main():
    log.info("Loading models...")
    yolo = YOLODetector()
    face_det = CatFaceDetector()

    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    (REPORT_DIR / "frames").mkdir(exist_ok=True)

    burst_dirs = sorted(
        p for p in CAPTURES.iterdir()
        if p.is_dir() and (p / "frame_00.jpg").exists()
    )
    log.info("Found %d bursts", len(burst_dirs))

    all_strips = []
    summary = []
    total_frames = 0
    total_yolo = 0
    total_haar = 0
    total_both = 0

    for bi, bdir in enumerate(burst_dirs):
        burst_name = bdir.name
        frame_paths = sorted(bdir.glob("frame_*.jpg"))
        frames_pil_for_strip = []
        burst_yolo = 0
        burst_haar = 0
        burst_both = 0

        for fp in frame_paths:
            total_frames += 1
            frame_bgr = cv2.imread(str(fp))
            if frame_bgr is None:
                continue
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            fidx = int(fp.stem.split("_")[1])

            # YOLO
            raw_dets = yolo.detect(frame_rgb)
            yolo_dets = yolo.detect_to_image_coords(frame_rgb, raw_dets)
            yolo_cats = [d for d in yolo_dets if d["class_id"] == CAT_CLASS_ID]

            # Haar
            haar_faces = face_det.detect_face(frame_bgr)

            has_yolo = len(yolo_cats) > 0
            has_haar = len(haar_faces) > 0
            has_any = has_yolo or has_haar

            if has_yolo:
                burst_yolo += 1
                total_yolo += 1
            if has_haar:
                burst_haar += 1
                total_haar += 1
            if has_yolo and has_haar:
                burst_both += 1
                total_both += 1

            # Annotate
            ann = annotate_frame(frame_bgr, yolo_dets, haar_faces, burst_name, fidx)
            frames_pil_for_strip.append((ann, has_any))

            # Save individual annotated frame if detection found
            if has_any:
                out = REPORT_DIR / "frames" / f"{burst_name}_f{fidx:02d}.jpg"
                ann.save(str(out), quality=90)

        # Make strip for this burst
        if frames_pil_for_strip:
            strip = make_burst_strip(frames_pil_for_strip, burst_name)
            all_strips.append(strip)

        has_anything = burst_yolo > 0 or burst_haar > 0
        summary.append({
            "burst": burst_name,
            "frames": len(frame_paths),
            "yolo_frames": burst_yolo,
            "haar_frames": burst_haar,
            "both_frames": burst_both,
            "has_detection": has_anything,
        })
        tag = ""
        if burst_yolo and burst_haar:
            tag = "YOLO+Haar"
        elif burst_yolo:
            tag = "YOLO only"
        elif burst_haar:
            tag = "Haar only"
        else:
            tag = "---"
        log.info("[%2d/%d] %s  YOLO=%d Haar=%d  %s",
                 bi + 1, len(burst_dirs), burst_name, burst_yolo, burst_haar, tag)

    # ── Master contact sheet ──
    log.info("Creating contact sheet from %d strips...", len(all_strips))
    if all_strips:
        max_w = max(s.width for s in all_strips)
        total_h = sum(s.height for s in all_strips)
        sheet = Image.new("RGB", (max_w, total_h), (20, 20, 20))
        y = 0
        for s in all_strips:
            sheet.paste(s, (0, y))
            y += s.height
        sheet_path = REPORT_DIR / "contact_sheet.jpg"
        sheet.save(str(sheet_path), quality=90)
        log.info("Contact sheet: %s (%dx%d)", sheet_path, max_w, total_h)

    # ── Summary ──
    with open(REPORT_DIR / "face_detection_summary.json", "w") as f:
        json.dump(summary, f, indent=2)

    bursts_with_det = sum(1 for s in summary if s["has_detection"])
    bursts_yolo_only = sum(1 for s in summary if s["yolo_frames"] > 0 and s["haar_frames"] == 0)
    bursts_haar_only = sum(1 for s in summary if s["haar_frames"] > 0 and s["yolo_frames"] == 0)
    bursts_both = sum(1 for s in summary if s["yolo_frames"] > 0 and s["haar_frames"] > 0)
    bursts_neither = sum(1 for s in summary if not s["has_detection"])

    print("\n" + "=" * 60)
    print(f"Total frames scanned:  {total_frames}")
    print(f"Frames with YOLO cat:  {total_yolo}")
    print(f"Frames with Haar face: {total_haar}")
    print(f"Frames with both:      {total_both}")
    print(f"")
    print(f"Bursts: {len(summary)} total")
    print(f"  YOLO+Haar:  {bursts_both}")
    print(f"  YOLO only:  {bursts_yolo_only}")
    print(f"  Haar only:  {bursts_haar_only}")
    print(f"  Neither:    {bursts_neither}  (butt/fur/empty)")
    print(f"")
    print(f"Report: {REPORT_DIR}")
    print("=" * 60)

    # Print the "neither" bursts for reference
    if bursts_neither:
        print(f"\nBursts with NO detection (likely exit/butt/fur):")
        for s in summary:
            if not s["has_detection"]:
                print(f"  {s['burst']}")


if __name__ == "__main__":
    main()
