#!/usr/bin/env python3
"""Combined detection report: Motion mask + YOLO + Haar.

Strategy:
  1. Use frame 0 as background reference
  2. For each subsequent frame, compute motion mask vs frame 0
  3. Run YOLO + Haar on frames with significant motion
  4. Validate detections: box must overlap motion area
  5. Generate visual report showing motion, YOLO, Haar, and combined result

Usage:
    cd Mazge && uv run python tools/combined_report.py
"""
import cv2
import json
import logging
import sys
import time
import numpy as np
from pathlib import Path
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).parent))
from prey_analyzer import YOLODetector, CatFaceDetector, CAT_CLASS_ID  # noqa: E402

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-5s %(message)s", datefmt="%H:%M:%S")
log = logging.getLogger("combined")

CAPTURES = Path(__file__).resolve().parent.parent / "captures"
REPORT_DIR = CAPTURES / "combined_report"

# ── Motion detection ──────────────────────────────────────────────────

def compute_motion_mask(bg_gray, frame_gray, threshold=30):
    """Binary mask of pixels that changed significantly from background."""
    diff = cv2.absdiff(frame_gray, bg_gray)
    _, mask = cv2.threshold(diff, threshold, 255, cv2.THRESH_BINARY)
    # Dilate to fill gaps
    mask = cv2.dilate(mask, np.ones((7, 7), np.uint8), iterations=2)
    return mask


def motion_fraction(mask):
    """Fraction of image covered by motion (0-1)."""
    return mask.sum() / 255.0 / (mask.shape[0] * mask.shape[1])


def box_motion_overlap(mask, x, y, w, h):
    """Fraction of a detection box that overlaps with the motion mask."""
    h_img, w_img = mask.shape
    x0 = max(0, x)
    y0 = max(0, y)
    x1 = min(w_img, x + w)
    y1 = min(h_img, y + h)
    if x1 <= x0 or y1 <= y0:
        return 0.0
    roi = mask[y0:y1, x0:x1]
    return (roi > 0).sum() / max(roi.size, 1)


# ── Annotation ────────────────────────────────────────────────────────

def annotate_combined(frame_bgr, motion_mask, yolo_dets, haar_faces,
                      burst_name, frame_idx, motion_frac):
    """Draw motion overlay + validated detections."""
    img = Image.fromarray(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(img)
    w_img, h_img = img.size

    # Semi-transparent motion overlay (red tint on motion areas)
    motion_rgb = np.array(img)
    motion_3ch = cv2.cvtColor(motion_mask, cv2.COLOR_GRAY2RGB)
    # Red tint where motion is
    motion_overlay = motion_rgb.copy()
    motion_overlay[motion_3ch[:, :, 0] > 0] = (
        motion_overlay[motion_3ch[:, :, 0] > 0] * 0.6 +
        np.array([100, 0, 0]) * 0.4
    ).astype(np.uint8)
    img = Image.fromarray(motion_overlay)
    draw = ImageDraw.Draw(img)

    validated_yolo = []
    validated_haar = []

    # YOLO detections
    for d in yolo_dets:
        if d["class_id"] != CAT_CLASS_ID:
            continue
        x, y, bw, bh = d["x"], d["y"], d["w"], d["h"]
        overlap = box_motion_overlap(motion_mask, x, y, bw, bh)
        valid = overlap > 0.15
        if valid:
            validated_yolo.append(d)
        color = "lime" if valid else "red"
        draw.rectangle([x, y, x + bw, y + bh], outline=color, width=2)
        label = f'YOLO {d["confidence"]:.0%}'
        if not valid:
            label += " (static!)"
        draw.text((x, max(0, y - 12)), label, fill=color)

    # Haar faces
    for fx, fy, fw, fh in haar_faces:
        overlap = box_motion_overlap(motion_mask, fx, fy, fw, fh)
        valid = overlap > 0.15
        if valid:
            validated_haar.append((fx, fy, fw, fh))
        color = "cyan" if valid else "red"
        draw.rectangle([fx, fy, fx + fw, fy + fh], outline=color, width=2)
        label = "Haar"
        if not valid:
            label += " (static!)"
        draw.text((fx, max(0, fy - 12)), label, fill=color)

    # Banner
    parts = []
    if validated_yolo:
        parts.append("YOLO")
    if validated_haar:
        parts.append("Haar")
    if parts:
        banner = "+".join(parts)
        color = "lime"
    else:
        banner = "no cat"
        color = "gray"

    # Top label
    draw.text((2, 2), f"{burst_name} f{frame_idx}  motion={motion_frac:.0%}", fill="white")
    # Bottom banner
    draw.rectangle([0, h_img - 16, w_img, h_img], fill="black")
    draw.text((2, h_img - 14), banner, fill=color)

    return img, len(validated_yolo) > 0, len(validated_haar) > 0


def make_burst_strip(frames_data, burst_name, thumb_w=120, thumb_h=90):
    """Horizontal strip: all frames, detections highlighted, others dimmed."""
    n = len(frames_data)
    margin = 2
    strip_w = n * (thumb_w + margin) + margin
    strip_h = thumb_h + 20 + margin * 2

    strip = Image.new("RGB", (strip_w, strip_h), (30, 30, 30))
    draw = ImageDraw.Draw(strip)
    draw.text((4, 2), burst_name, fill="white")

    for i, (pil_img, has_det) in enumerate(frames_data):
        thumb = pil_img.resize((thumb_w, thumb_h), Image.LANCZOS)
        if not has_det:
            arr = np.array(thumb).astype(np.float32) * 0.35
            thumb = Image.fromarray(arr.astype(np.uint8))
        x = margin + i * (thumb_w + margin)
        strip.paste(thumb, (x, 20))

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
    totals = {"frames": 0, "yolo_valid": 0, "haar_valid": 0,
              "yolo_static": 0, "haar_static": 0}

    for bi, bdir in enumerate(burst_dirs):
        burst_name = bdir.name
        frame_paths = sorted(bdir.glob("frame_*.jpg"))
        if not frame_paths:
            continue

        # Load background (frame 0)
        bg_bgr = cv2.imread(str(frame_paths[0]))
        if bg_bgr is None:
            continue
        bg_gray = cv2.cvtColor(bg_bgr, cv2.COLOR_BGR2GRAY)
        bg_gray = cv2.GaussianBlur(bg_gray, (5, 5), 0)

        burst_strip_data = []
        b_yolo_v = 0
        b_haar_v = 0
        b_yolo_s = 0
        b_haar_s = 0

        for fp in frame_paths:
            totals["frames"] += 1
            frame_bgr = cv2.imread(str(fp))
            if frame_bgr is None:
                continue
            fidx = int(fp.stem.split("_")[1])
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            frame_gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
            frame_gray_blur = cv2.GaussianBlur(frame_gray, (5, 5), 0)

            # Motion mask vs background
            motion_mask = compute_motion_mask(bg_gray, frame_gray_blur)
            m_frac = motion_fraction(motion_mask)

            # YOLO
            raw_dets = yolo.detect(frame_rgb)
            yolo_dets = yolo.detect_to_image_coords(frame_rgb, raw_dets)

            # Haar
            haar_faces = face_det.detect_face(frame_bgr)

            # Count raw detections
            raw_yolo_cats = sum(1 for d in yolo_dets if d["class_id"] == CAT_CLASS_ID)
            raw_haar = len(haar_faces)

            # Annotate with motion validation
            ann, has_valid_yolo, has_valid_haar = annotate_combined(
                frame_bgr, motion_mask, yolo_dets, haar_faces,
                burst_name, fidx, m_frac
            )

            if has_valid_yolo:
                b_yolo_v += 1
                totals["yolo_valid"] += 1
            elif raw_yolo_cats > 0:
                b_yolo_s += 1
                totals["yolo_static"] += 1

            if has_valid_haar:
                b_haar_v += 1
                totals["haar_valid"] += 1
            elif raw_haar > 0:
                b_haar_s += 1
                totals["haar_static"] += 1

            has_any = has_valid_yolo or has_valid_haar
            burst_strip_data.append((ann, has_any))

            # Save individual frame if any detection
            if has_any or raw_yolo_cats > 0 or raw_haar > 0:
                out = REPORT_DIR / "frames" / f"{burst_name}_f{fidx:02d}.jpg"
                ann.save(str(out), quality=90)

        # Strip
        if burst_strip_data:
            strip = make_burst_strip(burst_strip_data, burst_name)
            all_strips.append(strip)

        # Summary
        has_valid = b_yolo_v > 0 or b_haar_v > 0
        tag = ""
        if b_yolo_v and b_haar_v:
            tag = "YOLO+Haar"
        elif b_yolo_v:
            tag = "YOLO"
        elif b_haar_v:
            tag = "Haar"
        else:
            tag = "---"

        rejected = ""
        if b_yolo_s or b_haar_s:
            rejected = f" (rejected: {b_yolo_s} YOLO + {b_haar_s} Haar as static)"
        log.info("[%2d/%d] %s  valid: YOLO=%d Haar=%d  %s%s",
                 bi + 1, len(burst_dirs), burst_name, b_yolo_v, b_haar_v, tag, rejected)

        summary.append({
            "burst": burst_name,
            "frames": len(frame_paths),
            "yolo_valid": b_yolo_v,
            "haar_valid": b_haar_v,
            "yolo_static_rejected": b_yolo_s,
            "haar_static_rejected": b_haar_s,
            "direction": "entering" if has_valid else "exiting",
        })

    # Contact sheet
    log.info("Creating contact sheet...")
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

    with open(REPORT_DIR / "summary.json", "w") as f:
        json.dump(summary, f, indent=2)

    entering = sum(1 for s in summary if s["direction"] == "entering")
    exiting = sum(1 for s in summary if s["direction"] == "exiting")

    print()
    print("=" * 65)
    print(f"Frames scanned:     {totals['frames']}")
    print(f"YOLO valid:         {totals['yolo_valid']:3d}  (rejected as static: {totals['yolo_static']})")
    print(f"Haar valid:         {totals['haar_valid']:3d}  (rejected as static: {totals['haar_static']})")
    print()
    print(f"Bursts: {len(summary)} total")
    print(f"  Entering (face/body + motion): {entering}")
    print(f"  Exiting  (no valid detection):  {exiting}")
    print()
    print(f"Report: {REPORT_DIR}")
    print("=" * 65)

    if exiting:
        print(f"\nExiting/no-detection bursts:")
        for s in summary:
            if s["direction"] == "exiting":
                rej = s["yolo_static_rejected"] + s["haar_static_rejected"]
                rej_str = f"  ({rej} rejected as static)" if rej else ""
                print(f"  {s['burst']}{rej_str}")


if __name__ == "__main__":
    main()
