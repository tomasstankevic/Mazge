#!/usr/bin/env python3
"""Prey detection pipeline for Mazge ESP32-CAM burst captures.

Runs on the laptop. Pipeline:
  1. YOLO11n (NCNN) — detect cat bounding box in each frame
  2. Motion crop — frame-differencing to find the active region (for fisheye)
  3. Prey Detection API — send cropped cat image for prey classification
  4. Benchmarks each stage

Usage:
  # Analyze a single burst directory
  uv run python tools/prey_analyzer.py captures/burst_20260419_132554_gen6

  # Analyze all bursts
  uv run python tools/prey_analyzer.py captures/

  # Watch for new bursts (live mode alongside burst_saver.py)
  uv run python tools/prey_analyzer.py --watch captures/

  # Skip API call, just run YOLO + motion crop
  uv run python tools/prey_analyzer.py --no-api captures/burst_20260419_132554_gen6

Env vars:
  PREY_DETECTOR_API_KEY  — Bearer token for the prey detection API
  PREY_API_URL           — override API endpoint (default: fl2o's worker)
"""
import argparse
import asyncio
import base64
import datetime
import io
import json
import logging
import os
import sys
import time
from pathlib import Path

import ssl

import aiohttp
import certifi
import cv2
import ncnn
import numpy as np
from ncnn.utils.functional import nms, xywh2xyxy
from ncnn.utils.objects import Detect_Object
from PIL import Image, ImageDraw, ImageFont
from skimage.metrics import structural_similarity as ssim

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-5s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("prey")

# ── paths ──────────────────────────────────────────────────────────────
PROJECT_ROOT = Path(__file__).resolve().parent.parent
CATFLAP_REPO = PROJECT_ROOT.parent / "catflap-prey-detector"
MODEL_DIR = CATFLAP_REPO / "models" / "yolo11n_ncnn_model_384_640"

PREY_API_URL = os.environ.get(
    "PREY_API_URL",
    "https://prey-detection.florian-mutel.workers.dev",
)
PREY_API_KEY = os.environ.get("PREY_DETECTOR_API_KEY", "")
if not PREY_API_KEY:
    api_key_path = PROJECT_ROOT / "API_key_prey_detector.txt"
    if api_key_path.exists():
        PREY_API_KEY = api_key_path.read_text().strip()

# COCO class 15 = cat
CAT_CLASS_ID = 15
BIRD_CLASS_ID = 14
# Classes we care about detecting
CLASSES_OF_INTEREST = {CAT_CLASS_ID: "cat", BIRD_CLASS_ID: "bird"}
CAT_CONF_THRESHOLD = 0.15  # very low — our ESP32 IR images are grainy/overexposed
BIRD_CONF_THRESHOLD = 0.15

# ── Haar cascade cat face detector ─────────────────────────────────────

class CatFaceDetector:
    """OpenCV Haar cascade for detecting cat faces.

    Used as a fallback when YOLO can't find the cat body (close-ups, IR).
    Face found → cat is entering/approaching → check for prey.
    No face   → likely cat butt/fur/exiting → low priority.
    """

    def __init__(self):
        xml = cv2.data.haarcascades + "haarcascade_frontalcatface.xml"
        self.cascade = cv2.CascadeClassifier(xml)
        if self.cascade.empty():
            raise FileNotFoundError(f"Haar cascade not found: {xml}")
        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        log.info("Haar cat face cascade loaded")

    def detect_face(self, frame_bgr: np.ndarray) -> list[tuple[int, int, int, int]]:
        """Detect cat faces in a single frame.

        Returns list of (x, y, w, h) bounding boxes.
        Uses CLAHE preprocessing for better IR/night performance.
        """
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        gray_clahe = self.clahe.apply(gray)
        faces = self.cascade.detectMultiScale(
            gray_clahe, scaleFactor=1.1, minNeighbors=2, minSize=(50, 50)
        )
        if len(faces) == 0:
            return []
        return [(int(x), int(y), int(w), int(h)) for x, y, w, h in faces]

    def pick_best_face_frame(
        self, frames_bgr: list[np.ndarray]
    ) -> tuple[int, tuple[int, int, int, int] | None]:
        """Scan all frames, return (frame_idx, biggest_face_box) or (-1, None)."""
        best_idx, best_box, best_area = -1, None, 0
        for i, frame in enumerate(frames_bgr):
            faces = self.detect_face(frame)
            for x, y, w, h in faces:
                area = w * h
                if area > best_area:
                    best_idx, best_box, best_area = i, (x, y, w, h), area
        return best_idx, best_box


# ── YOLO detector ──────────────────────────────────────────────────────

class YOLODetector:
    """Lightweight NCNN YOLO11n wrapper for cat detection."""

    INPUT_H, INPUT_W = 384, 640

    def __init__(self, model_dir: Path = MODEL_DIR):
        param = str(model_dir / "model.ncnn.param")
        binn = str(model_dir / "model.ncnn.bin")
        if not Path(param).exists():
            raise FileNotFoundError(f"Model not found: {param}")
        self.net = ncnn.Net()
        self.net.load_param(param)
        self.net.load_model(binn)
        log.info("YOLO model loaded from %s", model_dir.name)

    def detect(self, image_rgb: np.ndarray) -> list[Detect_Object]:
        h, w = image_rgb.shape[:2]
        mat_in = ncnn.Mat.from_pixels_resize(
            image_rgb, ncnn.Mat.PixelType.PIXEL_RGB,
            w, h, self.INPUT_W, self.INPUT_H,
        )
        mat_in.substract_mean_normalize([0, 0, 0], [1/255.0, 1/255.0, 1/255.0])

        with self.net.create_extractor() as ex:
            ex.input("in0", mat_in)
            _, mat_out = ex.extract("out0")

        pred = np.array(mat_out).T  # (N, 84) — 4 box + 80 class scores
        boxes = pred[:, :4]
        scores = pred[:, 4:]

        detections = []
        for cls_id, cls_name in CLASSES_OF_INTEREST.items():
            conf = scores[:, cls_id]
            threshold = CAT_CONF_THRESHOLD if cls_id == CAT_CLASS_ID else BIRD_CONF_THRESHOLD
            mask = conf > threshold
            if not mask.any():
                continue
            cls_boxes = xywh2xyxy(boxes[mask])
            cls_conf = conf[mask]
            picked = nms(cls_boxes, cls_conf, 0.3)
            for idx in picked:
                b = cls_boxes[idx]
                d = Detect_Object()
                d.label = int(cls_id)
                d.prob = float(cls_conf[idx])
                d.rect.x = float(b[0])
                d.rect.y = float(b[1])
                d.rect.w = float(b[2] - b[0])
                d.rect.h = float(b[3] - b[1])
                detections.append(d)
        return detections

    def detect_to_image_coords(self, image_rgb: np.ndarray, detections: list[Detect_Object]):
        """Scale YOLO output coords (model-space) back to original image pixels."""
        h, w = image_rgb.shape[:2]
        sx, sy = w / self.INPUT_W, h / self.INPUT_H
        results = []
        for d in detections:
            results.append({
                "class_id": d.label,
                "class_name": CLASSES_OF_INTEREST.get(d.label, f"cls{d.label}"),
                "confidence": float(d.prob),
                "x": int(d.rect.x * sx),
                "y": int(d.rect.y * sy),
                "w": int(d.rect.w * sx),
                "h": int(d.rect.h * sy),
            })
        return results


# ── Motion crop (frame differencing) ──────────────────────────────────

def motion_roi(frames_gray: list[np.ndarray], threshold: int = 25, min_area_frac: float = 0.02) -> tuple[int, int, int, int] | None:
    """Find the bounding box of motion across a burst of grayscale frames.

    Returns (x, y, w, h) in pixel coords of the first frame, or None.
    """
    if len(frames_gray) < 2:
        return None

    union_mask = np.zeros_like(frames_gray[0], dtype=np.uint8)
    for i in range(1, len(frames_gray)):
        diff = cv2.absdiff(frames_gray[i], frames_gray[i - 1])
        _, mask = cv2.threshold(diff, threshold, 255, cv2.THRESH_BINARY)
        # dilate to close small gaps
        mask = cv2.dilate(mask, np.ones((5, 5), np.uint8), iterations=2)
        union_mask = cv2.bitwise_or(union_mask, mask)

    contours, _ = cv2.findContours(union_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    # Union all contour bounding boxes
    h_img, w_img = frames_gray[0].shape[:2]
    min_area = min_area_frac * h_img * w_img
    boxes = []
    for c in contours:
        area = cv2.contourArea(c)
        if area >= min_area:
            boxes.append(cv2.boundingRect(c))

    if not boxes:
        return None

    x0 = min(b[0] for b in boxes)
    y0 = min(b[1] for b in boxes)
    x1 = max(b[0] + b[2] for b in boxes)
    y1 = max(b[1] + b[3] for b in boxes)
    return (x0, y0, x1 - x0, y1 - y0)


def compute_motion_mask(bg_gray: np.ndarray, frame_gray: np.ndarray, threshold: int = 30) -> np.ndarray:
    """Binary mask of pixels that changed significantly from the background frame."""
    diff = cv2.absdiff(frame_gray, bg_gray)
    _, mask = cv2.threshold(diff, threshold, 255, cv2.THRESH_BINARY)
    return cv2.dilate(mask, np.ones((7, 7), np.uint8), iterations=2)


def box_motion_overlap(mask: np.ndarray, x: int, y: int, w: int, h: int) -> float:
    """Fraction of a box covered by motion pixels."""
    h_img, w_img = mask.shape
    x0 = max(0, x)
    y0 = max(0, y)
    x1 = min(w_img, x + w)
    y1 = min(h_img, y + h)
    if x1 <= x0 or y1 <= y0:
        return 0.0
    roi = mask[y0:y1, x0:x1]
    return float((roi > 0).sum()) / max(roi.size, 1)


def crop_with_padding(image: np.ndarray, x: int, y: int, w: int, h: int, pad: float = 0.15) -> np.ndarray:
    """Crop image around (x,y,w,h) with percentage padding, clipped to bounds."""
    ih, iw = image.shape[:2]
    px, py = int(w * pad), int(h * pad)
    x0 = max(0, x - px)
    y0 = max(0, y - py)
    x1 = min(iw, x + w + px)
    y1 = min(ih, y + h + py)
    return image[y0:y1, x0:x1]


# ── Prey Detection API ────────────────────────────────────────────────

def ssim_dedup(frames_bgr: list, indices: list[int], threshold: float = 0.90) -> list[int]:
    """Return subset of indices with SSIM < threshold vs previously kept frames (dedup)."""
    kept: list[int] = []
    kept_grays: list[np.ndarray] = []
    for idx in indices:
        gray = cv2.cvtColor(frames_bgr[idx], cv2.COLOR_BGR2GRAY)
        if kept_grays:
            # Compare against last kept frame only (sequential burst)
            h, w = gray.shape
            ref = cv2.resize(kept_grays[-1], (w, h))
            score = ssim(gray, ref, data_range=255)
            if score >= threshold:
                log.debug("Frame %d skipped (SSIM=%.3f vs fr%d)", idx, score, kept[-1])
                continue
        kept.append(idx)
        kept_grays.append(gray)
    return kept


async def call_prey_api(image_bytes: bytes, *, retries: int = 3) -> dict:
    """POST image to the prey detection API. Returns {"detected": bool, "latency_ms": float}."""
    if not PREY_API_KEY:
        return {"detected": None, "latency_ms": 0, "error": "PREY_DETECTOR_API_KEY not set"}

    b64 = base64.b64encode(image_bytes).decode()
    ssl_ctx = ssl.create_default_context(cafile=certifi.where())
    t0 = time.perf_counter()
    last_err = None
    for attempt in range(retries):
        try:
            async with aiohttp.ClientSession() as sess:
                async with sess.post(
                    PREY_API_URL,
                    headers={
                        "Content-Type": "application/json",
                        "Authorization": f"Bearer {PREY_API_KEY}",
                    },
                    json={"image_base64": b64},
                    timeout=aiohttp.ClientTimeout(total=15),
                    ssl=ssl_ctx,
                ) as resp:
                    resp.raise_for_status()
                    data = await resp.json()
            latency = (time.perf_counter() - t0) * 1000
            return {"detected": data.get("detected", False), "latency_ms": latency}
        except Exception as exc:
            last_err = exc
            if attempt < retries - 1:
                log.warning("API attempt %d failed: %s — retrying", attempt + 1, exc)
                await asyncio.sleep(1.0)
    latency = (time.perf_counter() - t0) * 1000
    return {"detected": None, "latency_ms": latency, "error": str(last_err)}


async def analyze_api_candidate(
    burst_dir: Path,
    frame_idx: int,
    frame_bgr: np.ndarray,
    crop_box: dict,
    motion_box: tuple | None,
    *,
    save_debug: bool,
    bg_bgr: np.ndarray | None = None,
) -> dict:
    """Run the prey API for a single candidate frame."""
    api_image = prepare_api_image(frame_bgr, crop_box, motion_box, bg_bgr=bg_bgr)

    if save_debug:
        debug_dir = burst_dir / "debug"
        debug_dir.mkdir(exist_ok=True)
        with open(debug_dir / f"api_crop_frame{frame_idx:02d}.jpg", "wb") as f:
            f.write(api_image)

    api_result = await call_prey_api(api_image)
    return {
        "frame": frame_idx,
        "detected": api_result.get("detected"),
        "latency_ms": api_result.get("latency_ms", 0),
        "error": api_result.get("error"),
        "image_bytes": len(api_image),
        "crop_box": crop_box,  # passed through for annotation
    }


# ── Burst analyzer ────────────────────────────────────────────────────

def load_burst_frames(burst_dir: Path) -> list[np.ndarray]:
    """Load all frame_XX.jpg files from a burst directory, sorted."""
    paths = sorted(burst_dir.glob("frame_*.jpg"))
    frames = []
    for p in paths:
        img = cv2.imread(str(p))
        if img is not None:
            frames.append(img)
    return frames


def load_burst_meta(burst_dir: Path) -> dict | None:
    """Load burst timing metadata saved by burst_saver, if present."""
    meta_path = burst_dir / "burst_meta.json"
    if not meta_path.exists():
        return None
    try:
        return json.loads(meta_path.read_text())
    except Exception as exc:
        log.warning("Failed to read burst_meta.json for %s: %s", burst_dir.name, exc)
        return None


def enrich_latency_summary(result: dict, burst_meta: dict | None, analysis_started_ms: float, analysis_finished_ms: float) -> None:
    """Add end-to-end latency summary from capture to decision."""
    result["analysis_started_ms"] = round(analysis_started_ms, 1)
    result["analysis_finished_ms"] = round(analysis_finished_ms, 1)
    result["analysis_duration_ms"] = round(analysis_finished_ms - analysis_started_ms, 1)

    if not burst_meta:
        return

    result["burst_meta"] = {
        "generation": burst_meta.get("generation"),
        "archive_index": burst_meta.get("archive_index"),
        "distance_mm": burst_meta.get("distance_mm"),
    }

    esp32_capture = burst_meta.get("esp32_capture", {})
    latency = dict(burst_meta.get("latency_ms", {}))
    approx_trigger_ms = esp32_capture.get("approx_local_trigger_ms")
    approx_archive_ms = esp32_capture.get("approx_local_archive_ms")
    download = burst_meta.get("download", {})

    if download.get("finished_ms") is not None:
        latency["download_to_analysis_start_ms"] = round(analysis_started_ms - download["finished_ms"], 1)
    if approx_archive_ms is not None:
        latency["archive_to_analysis_start_ms"] = round(analysis_started_ms - approx_archive_ms, 1)
        latency["archive_to_decision_ms"] = round(analysis_finished_ms - approx_archive_ms, 1)
    if approx_trigger_ms is not None:
        latency["trigger_to_analysis_start_ms"] = round(analysis_started_ms - approx_trigger_ms, 1)
        latency["trigger_to_decision_ms"] = round(analysis_finished_ms - approx_trigger_ms, 1)

    result["latency_ms"] = latency


def pick_best_cat_frame(yolo: YOLODetector, frames_rgb: list[np.ndarray]) -> tuple[int, dict | None, list]:
    """Run YOLO on each frame, return (best_frame_idx, best_detection, all_frame_detections).

    'Best' = highest confidence cat detection.
    """
    best_idx, best_det = -1, None
    all_dets = []
    for i, frame in enumerate(frames_rgb):
        raw_dets = yolo.detect(frame)
        mapped = yolo.detect_to_image_coords(frame, raw_dets)
        all_dets.append(mapped)
        for d in mapped:
            if d["class_id"] == CAT_CLASS_ID:
                if best_det is None or d["confidence"] > best_det["confidence"]:
                    best_idx, best_det = i, d
    return best_idx, best_det, all_dets


def collect_frame_candidates(
    yolo: YOLODetector,
    face_det: CatFaceDetector,
    frames_bgr: list[np.ndarray],
    frames_rgb: list[np.ndarray],
    frames_gray: list[np.ndarray],
    *,
    motion_overlap_threshold: float = 0.15,
) -> tuple[list[dict], int, dict | None, list[list[dict]]]:
    """Collect motion-validated YOLO/face candidates for each frame.

    Returns:
      - per-frame candidate records
      - best YOLO frame index
      - best YOLO detection
      - mapped YOLO detections for all frames
    """
    all_dets = []
    candidates = []
    best_idx, best_det = -1, None
    bg_gray = frames_gray[0] if frames_gray else None

    for i, (frame_bgr, frame_rgb, frame_gray) in enumerate(zip(frames_bgr, frames_rgb, frames_gray)):
        raw_dets = yolo.detect(frame_rgb)
        mapped = yolo.detect_to_image_coords(frame_rgb, raw_dets)
        all_dets.append(mapped)

        motion_mask = compute_motion_mask(bg_gray, frame_gray) if bg_gray is not None else np.zeros_like(frame_gray)

        valid_cats = []
        for det in mapped:
            if det["class_id"] != CAT_CLASS_ID:
                continue
            overlap = box_motion_overlap(motion_mask, det["x"], det["y"], det["w"], det["h"])
            det_with_motion = {**det, "motion_overlap": round(overlap, 3)}
            if overlap >= motion_overlap_threshold:
                valid_cats.append(det_with_motion)
                if best_det is None or det["confidence"] > best_det["confidence"]:
                    best_idx, best_det = i, det

        valid_faces = []
        for x, y, w, h in face_det.detect_face(frame_bgr):
            overlap = box_motion_overlap(motion_mask, x, y, w, h)
            if overlap >= motion_overlap_threshold:
                valid_faces.append({
                    "x": x,
                    "y": y,
                    "w": w,
                    "h": h,
                    "motion_overlap": round(overlap, 3),
                })

        candidates.append({
            "frame": i,
            "cats": valid_cats,
            "faces": valid_faces,
            "has_candidate": bool(valid_cats or valid_faces),
        })

    return candidates, best_idx, best_det, all_dets


def _process_one_frame(
    yolo: YOLODetector,
    face_det: CatFaceDetector,
    frame_bgr: np.ndarray,
    frame_idx: int,
    bg_gray: np.ndarray | None,
    *,
    motion_overlap_threshold: float = 0.15,
) -> tuple[dict, dict | None, list[dict]]:
    """Run YOLO + Haar on a single frame.

    Returns (candidate_record, best_cat_det_or_None, mapped_dets).
    Used by the streaming analyzer to process frames one-by-one during download.
    """
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    frame_gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    motion_mask = (
        compute_motion_mask(bg_gray, frame_gray)
        if bg_gray is not None
        else np.zeros_like(frame_gray)
    )

    raw_dets = yolo.detect(frame_rgb)
    mapped = yolo.detect_to_image_coords(frame_rgb, raw_dets)

    valid_cats, frame_best = [], None
    for det in mapped:
        if det["class_id"] != CAT_CLASS_ID:
            continue
        overlap = box_motion_overlap(motion_mask, det["x"], det["y"], det["w"], det["h"])
        if overlap >= motion_overlap_threshold:
            valid_cats.append({**det, "motion_overlap": round(overlap, 3)})
            if frame_best is None or det["confidence"] > frame_best["confidence"]:
                frame_best = det

    valid_faces = []
    for x, y, w, h in face_det.detect_face(frame_bgr):
        overlap = box_motion_overlap(motion_mask, x, y, w, h)
        if overlap >= motion_overlap_threshold:
            valid_faces.append({"x": x, "y": y, "w": w, "h": h, "motion_overlap": round(overlap, 3)})

    candidate = {
        "frame": frame_idx,
        "cats": valid_cats,
        "faces": valid_faces,
        "has_candidate": bool(valid_cats or valid_faces),
    }
    return candidate, frame_best, mapped


def prepare_api_image(
    frame_bgr: np.ndarray,
    cat_det: dict | None,
    motion_box: tuple | None,
    max_size: int = 384,
    *,
    bg_bgr: np.ndarray | None = None,
) -> bytes:
    """Crop the best region and encode as JPEG for the API.

    Priority: YOLO cat box > motion ROI > full frame.
    Target: ≤384×384 JPEG.
    If bg_bgr is supplied, pixels that haven't changed vs the background are
    blacked out — the resulting image has lower entropy and compresses better.
    """
    # Background subtraction: zero-out static pixels to lower JPEG entropy
    if bg_bgr is not None:
        diff = cv2.absdiff(
            cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY),
            cv2.cvtColor(bg_bgr, cv2.COLOR_BGR2GRAY),
        )
        kernel = np.ones((5, 5), np.uint8)
        motion_mask = cv2.dilate((diff > 20).astype(np.uint8), kernel).astype(bool)
        frame_for_crop = frame_bgr.copy()
        frame_for_crop[~motion_mask] = 0
    else:
        frame_for_crop = frame_bgr

    if cat_det:
        crop = crop_with_padding(frame_for_crop, cat_det["x"], cat_det["y"], cat_det["w"], cat_det["h"], pad=0.20)
    elif motion_box:
        crop = crop_with_padding(frame_for_crop, *motion_box, pad=0.10)
    else:
        crop = frame_for_crop

    # Resize proportionally to fit within max_size
    h, w = crop.shape[:2]
    scale = min(max_size / w, max_size / h, 1.0)
    if scale < 1.0:
        crop = cv2.resize(crop, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)

    _, buf = cv2.imencode(".jpg", crop, [cv2.IMWRITE_JPEG_QUALITY, 90])
    return buf.tobytes()


def save_annotated(
    frame_bgr: np.ndarray,
    detections: list[dict],
    motion_box: tuple | None,
    out_path: Path,
    *,
    decision: str = "none",
    frame_idx: int = -1,
    burst_name: str = "",
    total_ms: float = 0,
    trigger_to_decision_ms: float | None = None,
):
    """Save an annotated frame with detection boxes, decision banner, and timing."""
    img = Image.fromarray(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
    W, H = img.size

    # Decision banner colour
    banner_color = {"green": (0, 180, 0), "red": (200, 0, 0)}.get(decision, (80, 80, 80))
    decision_text = {"green": "GREEN — NO PREY", "red": "RED — PREY!", "none": "NO DECISION"}.get(decision, decision.upper())

    # Draw banner strip at top
    banner_h = max(24, H // 12)
    banner = Image.new("RGB", (W, banner_h), banner_color)
    img = Image.fromarray(
        np.vstack([np.array(banner), np.array(img)])
    )
    H_new = H + banner_h

    draw = ImageDraw.Draw(img)

    # Decision text centred in banner
    draw.text((W // 2, banner_h // 2), decision_text, fill="white", anchor="mm")

    # Detection boxes (offset by banner_h)
    for d in detections:
        color = "lime" if d["class_id"] == CAT_CLASS_ID else "cyan"
        x, y, w, h = d["x"], d["y"] + banner_h, d["w"], d["h"]
        draw.rectangle([x, y, x+w, y+h], outline=color, width=2)
        label = f'{d["class_name"]} {d["confidence"]:.0%}'
        draw.text((x + 2, y + 2), label, fill=color)

    # Motion box (offset by banner_h)
    if motion_box:
        mx, my, mw, mh = motion_box
        draw.rectangle([mx, my + banner_h, mx+mw, my+mh + banner_h], outline="yellow", width=1)

    # Timestamp + burst info bottom bar
    ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    t2d = f"  trig→dec={trigger_to_decision_ms:.0f}ms" if trigger_to_decision_ms is not None else ""
    info_lines = [
        f"{burst_name}  fr{frame_idx}",
        f"{ts}  {total_ms:.0f}ms{t2d}",
    ]
    line_h = 14
    bar_h = line_h * len(info_lines) + 6
    for i, line in enumerate(info_lines):
        draw.text((4, H_new - bar_h + 3 + i * line_h), line, fill="white")

    img.save(out_path, quality=92)


async def _run_decision_phase(
    burst_dir: Path,
    frames_bgr: list[np.ndarray],
    frame_candidates: list[dict],
    all_dets: list[list[dict]],
    best_idx: int,
    best_cat: dict | None,
    detector_ms: float,
    burst_meta: dict | None,
    analysis_started_ms: float,
    *,
    call_api: bool,
    save_debug: bool,
    last_green_time: float | None = None,
) -> dict:
    """Direction classification → motion ROI → API → annotation.

    Shared by both the batch path (analyze_burst) and the streaming path
    (analyze_burst_streaming).  Receives pre-built per-frame detections.
    """
    result: dict = {"burst": burst_dir.name, "frames": len(frames_bgr)}
    result["detector_ms"] = round(detector_ms, 1)
    result["detector_ms_per_frame"] = round(detector_ms / len(frames_bgr), 1) if frames_bgr else 0

    cat_count = sum(len(fc["cats"]) for fc in frame_candidates)
    face_count = sum(len(fc["faces"]) for fc in frame_candidates)
    result["cat_detections"] = cat_count
    result["face_detections"] = face_count
    result["candidate_frames"] = [fc["frame"] for fc in frame_candidates if fc["has_candidate"]]
    if best_cat:
        result["best_cat"] = {
            "frame": best_idx,
            "confidence": round(best_cat["confidence"], 3),
            "box": [best_cat["x"], best_cat["y"], best_cat["w"], best_cat["h"]],
        }
        log.info("Cat found in frame %d (%.0f%% conf, box %dx%d)",
                 best_idx, best_cat["confidence"]*100, best_cat["w"], best_cat["h"])
    else:
        log.info("No cat detected in any frame")

    # Pick best face among motion-validated candidates
    face_idx, face_box, face_area = -1, None, 0
    for fc in frame_candidates:
        for face in fc["faces"]:
            area = face["w"] * face["h"]
            if area > face_area:
                face_idx, face_box, face_area = fc["frame"], face, area

    if face_box:
        result["face_detected"] = True
        result["face_frame"] = face_idx
        result["face_box"] = [face_box["x"], face_box["y"], face_box["w"], face_box["h"]]
        log.info("Haar face in frame %d (box %dx%d)", face_idx, face_box["w"], face_box["h"])
    else:
        result["face_detected"] = False

    if best_cat or face_box:
        result["direction"] = "entering"
    else:
        result["direction"] = "unknown"  # extreme angle — no face / body visible
        log.info("No cat face or body detected → direction unknown (extreme angle?)")

    # ── Stage 2: Motion ROI ──
    t0 = time.perf_counter()
    grays = [cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) for f in frames_bgr]
    mbox = motion_roi(grays)
    motion_ms = (time.perf_counter() - t0) * 1000
    result["motion_ms"] = round(motion_ms, 1)
    if mbox:
        result["motion_box"] = list(mbox)
        log.info("Motion ROI: x=%d y=%d %dx%d", *mbox)

    # ── Stage 3: Prey API on candidate frames ──
    # Use frame 0 as static background for JPEG entropy reduction
    bg_bgr = frames_bgr[0] if len(frames_bgr) > 1 else None
    batch_signal = "none"
    completed: list[dict] = []
    result["prey_detected"] = None
    result["api_frames_checked"] = []
    result["api_positive_frames"] = []
    result["api_negative_frames"] = []

    # Skip API for unknown-direction bursts if a recent GREEN was issued
    RECENT_GREEN_WINDOW_S = 30
    if (call_api
            and result["direction"] == "unknown"
            and last_green_time is not None
            and (time.time() - last_green_time) < RECENT_GREEN_WINDOW_S):
        call_api = False
        batch_signal = "green"
        result["api_skipped"] = "recent_green"
        elapsed = time.time() - last_green_time
        log.info("Skipping API — recent GREEN %.1fs ago, unknown direction (likely same cat)", elapsed)

    # Skip API entirely if no cat face/body was detected at all
    if call_api and cat_count == 0 and face_count == 0:
        call_api = False
        result["api_skipped"] = "no_detections"
        log.info("Skipping API — zero cat/face detections across all frames")

    if call_api and result["direction"] in ("entering", "unknown"):
        api_tasks = []
        candidate_idxs = [fc["frame"] for fc in frame_candidates if fc["has_candidate"]]
        deduped_idxs = ssim_dedup(frames_bgr, candidate_idxs)
        skipped_ssim = len(candidate_idxs) - len(deduped_idxs)
        if skipped_ssim:
            log.info("SSIM dedup: skipped %d duplicate frame(s), sending %d", skipped_ssim, len(deduped_idxs))
        for frame_idx in deduped_idxs:
            fc = next(f for f in frame_candidates if f["frame"] == frame_idx)
            crop_box = fc["cats"][0] if fc["cats"] else fc["faces"][0]
            result["api_frames_checked"].append(frame_idx)
            api_tasks.append(analyze_api_candidate(
                burst_dir, frame_idx, frames_bgr[frame_idx], crop_box, mbox,
                save_debug=save_debug, bg_bgr=bg_bgr,
            ))

        # Fallback for unknown direction: send 2 middle-ish frames (no detections available)
        if not api_tasks and result["direction"] == "unknown" and frames_bgr:
            n = len(frames_bgr)
            mid = n // 2
            idxs = [i for i in (mid, min(mid + 1, n - 1)) if i < n]
            idxs = list(dict.fromkeys(idxs))  # dedupe if burst is tiny
            for frame_idx in idxs:
                result["api_frames_checked"].append(frame_idx)
                api_tasks.append(analyze_api_candidate(
                    burst_dir, frame_idx, frames_bgr[frame_idx],
                    None, mbox, save_debug=save_debug, bg_bgr=bg_bgr,
                ))
            log.info("Unknown direction: sending frames %s to API (no detections)", idxs)

        if api_tasks:
            # Sequential with early exit on first positive — saves API quota
            completed = []
            for task in api_tasks:
                try:
                    item = await task
                except Exception as exc:
                    log.error("API task failed: %s", exc)
                    if "api_error" not in result:
                        result["api_error"] = str(exc)
                    continue
                completed.append(item)
                if item.get("error") and "api_error" not in result:
                    result["api_error"] = item["error"]
                if item.get("detected") is True:
                    result["api_positive_frames"].append(item["frame"])
                    log.warning("PREY in frame %d — stopping early", item["frame"])
                    for remaining in api_tasks[len(completed):]:
                        remaining.close()
                    break
                elif item.get("detected") is False:
                    result["api_negative_frames"].append(item["frame"])

            api_total_ms = sum(item.get("latency_ms", 0) for item in completed)
            result["api_image_bytes"] = sum(item.get("image_bytes", 0) for item in completed)
            result["api_frames_checked"] = [item["frame"] for item in completed]
            result["api_latency_ms"] = round(api_total_ms, 1)
            result["api_latency_ms_per_checked_frame"] = round(api_total_ms / len(completed), 1) if completed else 0

            if result["api_positive_frames"]:
                result["prey_detected"] = True
                batch_signal = "red"
                log.warning("PREY DETECTED in %s frames %s", burst_dir.name, result["api_positive_frames"])
            elif result["api_negative_frames"]:
                result["prey_detected"] = False
                batch_signal = "green"
    elif call_api:
        log.info("Skipping API — cat exiting (face/body detected, direction=exiting)")
        result["api_skipped"] = "exiting"
    elif "api_skipped" not in result:
        result["api_skipped"] = True

    result["light_signal"] = batch_signal
    if batch_signal == "green":
        log.info("GREEN light for %s", burst_dir.name)
    elif batch_signal == "red":
        log.warning("RED light for %s", burst_dir.name)

    total_ms = result["detector_ms"] + result["motion_ms"] + result.get("api_latency_ms", 0)
    result["total_ms"] = round(total_ms, 1)
    enrich_latency_summary(result, burst_meta, analysis_started_ms, time.time() * 1000)

    # ── Save annotated contact sheet ──
    if save_debug and frames_bgr:
        debug_dir = burst_dir / "debug"
        debug_dir.mkdir(exist_ok=True)
        t2d = result.get("latency_ms", {}).get("trigger_to_decision_ms")
        t2d_f = float(t2d) if t2d is not None else None
        ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        frame_api: dict[int, dict] = {item["frame"]: item for item in completed if isinstance(item, dict)}
        _fallback = best_idx if best_idx >= 0 else len(frames_bgr) // 2
        show_idxs = result.get("api_frames_checked") or [_fallback]

        TILE_H = 320
        # Scale tile width to match frame aspect ratio (may vary with crop/resolution)
        if frames_bgr:
            fh, fw = frames_bgr[0].shape[:2]
            TILE_W = int(TILE_H * fw / fh) if fh > 0 else 240
        else:
            TILE_W = 240
        tiles = []
        for fi in show_idxs:
            if fi >= len(frames_bgr):
                continue
            frame = frames_bgr[fi]
            img = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)).resize((TILE_W, TILE_H))
            draw = ImageDraw.Draw(img)
            sx = TILE_W / frame.shape[1]
            sy = TILE_H / frame.shape[0]
            for d in (all_dets[fi] if fi < len(all_dets) else []):
                color = "lime" if d["class_id"] == CAT_CLASS_ID else "cyan"
                x, y, w, h = int(d["x"]*sx), int(d["y"]*sy), int(d["w"]*sx), int(d["h"]*sy)
                draw.rectangle([x, y, x+w, y+h], outline=color, width=2)
            api_item = frame_api.get(fi)
            if api_item and api_item.get("crop_box"):
                cb = api_item["crop_box"]
                x, y, w, h = int(cb["x"]*sx), int(cb["y"]*sy), int(cb["w"]*sx), int(cb["h"]*sy)
                draw.rectangle([x, y, x+w, y+h], outline="magenta", width=2)
            api_result_str = "?"
            if api_item:
                api_result_str = "PREY" if api_item.get("detected") else ("clear" if api_item.get("detected") is False else "err")
            bar_color = (180, 0, 0) if api_result_str == "PREY" else (0, 140, 0) if api_result_str == "clear" else (60, 60, 60)
            draw.rectangle([0, TILE_H - 18, TILE_W, TILE_H], fill=bar_color)
            draw.text((4, TILE_H - 16), f"fr{fi}  {api_result_str}", fill="white")
            tiles.append(np.array(img))

        if tiles:
            contact_img = Image.fromarray(np.hstack(tiles))
            CW, _ = contact_img.size
            banner_color = {"green": (0, 180, 0), "red": (200, 0, 0)}.get(batch_signal, (80, 80, 80))
            decision_text = {"green": "GREEN — NO PREY", "red": "RED — PREY!", "none": "NO DECISION"}.get(batch_signal, batch_signal.upper())
            banner_h = 28
            final_img = Image.fromarray(np.vstack([
                np.array(Image.new("RGB", (CW, banner_h), banner_color)),
                np.array(contact_img),
            ]))
            draw2 = ImageDraw.Draw(final_img)
            draw2.text((CW // 2, banner_h // 2), decision_text, fill="white", anchor="mm")
            t2d_str = f"  trig→dec={t2d_f:.0f}ms" if t2d_f else ""
            info = f"{burst_dir.name}  {ts}  {total_ms:.0f}ms{t2d_str}"
            FH = final_img.size[1]
            draw2.rectangle([0, FH - 16, CW, FH], fill=(30, 30, 30))
            draw2.text((4, FH - 14), info, fill="white")
            final_img.save(burst_dir / "annotated.jpg", quality=92)
            log.info("Annotated contact sheet saved → %s/annotated.jpg (%d frames)", burst_dir.name, len(tiles))
        else:
            ann_idx2 = best_idx if best_idx >= 0 else len(frames_bgr) // 2
            save_annotated(
                frames_bgr[ann_idx2],
                all_dets[ann_idx2] if ann_idx2 < len(all_dets) else [],
                mbox,
                burst_dir / "annotated.jpg",
                decision=batch_signal,
                frame_idx=ann_idx2,
                burst_name=burst_dir.name,
                total_ms=total_ms,
                trigger_to_decision_ms=t2d_f,
            )
            log.info("Annotated image saved → %s/annotated.jpg", burst_dir.name)

    log.info("Total: %.0fms (detectors %.0f + motion %.0f + API %.0f)",
             total_ms, result["detector_ms"], result["motion_ms"], result.get("api_latency_ms", 0))
    if "latency_ms" in result:
        log.info(
            "Latency: trigger->decision=%sms archive->decision=%sms download->analysis=%sms",
            result["latency_ms"].get("trigger_to_decision_ms"),
            result["latency_ms"].get("archive_to_decision_ms"),
            result["latency_ms"].get("download_to_analysis_start_ms"),
        )
    return result


async def analyze_burst(
    burst_dir: Path,
    yolo: YOLODetector,
    face_det: CatFaceDetector,
    *,
    call_api: bool = True,
    save_debug: bool = True,
) -> dict:
    """Batch mode: load all frames from disk, then run full pipeline."""
    log.info("─── Analyzing %s ───", burst_dir.name)
    analysis_started_ms = time.time() * 1000
    burst_meta = load_burst_meta(burst_dir)

    frames_bgr = load_burst_frames(burst_dir)
    if not frames_bgr:
        log.warning("No frames in %s", burst_dir)
        return {"burst": burst_dir.name, "frames": 0, "error": "no frames"}

    frames_rgb = [cv2.cvtColor(f, cv2.COLOR_BGR2RGB) for f in frames_bgr]
    grays = [cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) for f in frames_bgr]

    t0 = time.perf_counter()
    frame_candidates, best_idx, best_cat, all_dets = collect_frame_candidates(
        yolo, face_det, frames_bgr, frames_rgb, grays
    )
    detector_ms = (time.perf_counter() - t0) * 1000

    return await _run_decision_phase(
        burst_dir, frames_bgr, frame_candidates, all_dets,
        best_idx, best_cat, detector_ms, burst_meta, analysis_started_ms,
        call_api=call_api, save_debug=save_debug,
    )


async def analyze_burst_streaming(
    burst_dir: Path,
    yolo: YOLODetector,
    face_det: CatFaceDetector,
    *,
    call_api: bool,
    save_debug: bool,
    last_green_time: float | None = None,
) -> dict:
    """Streaming mode: run YOLO+Haar on each frame the moment it lands on disk.

    burst_saver writes frame_00.jpg … frame_NN.jpg one by one (~200ms/frame).
    Each frame takes ~20ms to detect.  By overlapping detection with download
    we avoid the full per-burst download wait before analysis starts.
    burst_meta.json (written last by burst_saver) is the completion sentinel.
    """
    log.info("─── Streaming %s ───", burst_dir.name)
    analysis_started_ms = time.time() * 1000

    frames_bgr: list[np.ndarray] = []
    frame_candidates: list[dict] = []
    all_dets: list[list[dict]] = []
    best_idx, best_cat = -1, None
    bg_gray: np.ndarray | None = None

    frame_idx = 0
    burst_meta: dict | None = None
    total_frames: int | None = None
    detect_cpu_ms = 0.0

    while True:
        frame_path = burst_dir / f"frame_{frame_idx:02d}.jpg"
        if frame_path.exists():
            img = cv2.imread(str(frame_path))
            if img is not None:
                t_frame = time.perf_counter()
                frame_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                if bg_gray is None:
                    bg_gray = frame_gray  # first frame = static background reference
                candidate, frame_best, mapped = _process_one_frame(
                    yolo, face_det, img, frame_idx, bg_gray
                )
                detect_cpu_ms += (time.perf_counter() - t_frame) * 1000
                frames_bgr.append(img)
                frame_candidates.append(candidate)
                all_dets.append(mapped)
                if frame_best is not None:
                    if best_cat is None or frame_best["confidence"] > best_cat["confidence"]:
                        best_idx, best_cat = frame_idx, frame_best
                frame_idx += 1
                continue  # immediately try next frame — no sleep needed

        # No new frame yet — check for completion sentinel
        meta_path = burst_dir / "burst_meta.json"
        if meta_path.exists():
            if burst_meta is None:
                try:
                    burst_meta = json.loads(meta_path.read_text())
                    total_frames = burst_meta.get("frame_count")
                except Exception:
                    pass
            if total_frames is not None and frame_idx >= total_frames:
                break  # all frames processed

        elapsed_ms = time.time() * 1000 - analysis_started_ms
        if elapsed_ms > 20_000:
            log.warning(
                "Streaming timeout for %s after %.0fms (%d frames processed)",
                burst_dir.name, elapsed_ms, frame_idx,
            )
            if frame_idx > 0:
                break
            return {"burst": burst_dir.name, "frames": 0, "error": "streaming timeout"}

        await asyncio.sleep(0.03)  # wait 30ms before checking for next frame

    detector_ms = detect_cpu_ms
    if not frames_bgr:
        return {"burst": burst_dir.name, "frames": 0, "error": "no frames"}

    return await _run_decision_phase(
        burst_dir, frames_bgr, frame_candidates, all_dets,
        best_idx, best_cat, detector_ms, burst_meta, analysis_started_ms,
        call_api=call_api, save_debug=save_debug,
        last_green_time=last_green_time,
    )


# ── Watch mode ────────────────────────────────────────────────────────

async def watch_loop(captures_dir: Path, yolo: YOLODetector, face_det: CatFaceDetector, call_api: bool):
    """Watch for new burst directories and stream-analyze them as they appear.

    Starts streaming detection immediately when a new dir is created — no wait.
    burst_saver signals completion by writing burst_meta.json last.
    Tracks last GREEN time to skip redundant API calls for subsequent bursts.
    """
    seen = set(p.name for p in captures_dir.iterdir() if p.is_dir())
    log.info("Watching %s for new bursts (%d existing)...", captures_dir, len(seen))
    in_progress: set[str] = set()
    last_green_time: dict[str, float] = {"t": 0.0}  # mutable container for closures

    while True:
        current = set(p.name for p in captures_dir.iterdir() if p.is_dir())
        new_dirs = sorted(current - seen)
        for name in new_dirs:
            seen.add(name)
            in_progress.add(name)

            async def _run(n: str = name) -> None:
                try:
                    green_t = last_green_time["t"] or None
                    result = await analyze_burst_streaming(
                        captures_dir / n, yolo, face_det, call_api=call_api, save_debug=True,
                        last_green_time=green_t,
                    )
                    signal = result.get("light_signal", "none").upper()
                    if signal == "GREEN":
                        last_green_time["t"] = time.time()
                    print(
                        f"[{n}] signal={signal} direction={result.get('direction')} "
                        f"checked={result.get('api_frames_checked', [])} "
                        f"trigger_to_decision_ms={result.get('latency_ms', {}).get('trigger_to_decision_ms')}"
                    )
                    print(json.dumps(result, indent=2))
                    # Save per-burst result JSON
                    (captures_dir / n / "prey_analysis.json").write_text(
                        json.dumps([result], indent=2)
                    )
                except Exception as exc:
                    log.error("Error analyzing %s: %s", n, exc)
                finally:
                    in_progress.discard(n)

            asyncio.create_task(_run())

        await asyncio.sleep(0.5)


# ── CLI ───────────────────────────────────────────────────────────────

async def main():
    parser = argparse.ArgumentParser(description="Prey detection pipeline for ESP32-CAM bursts")
    parser.add_argument("path", help="Burst directory, or captures/ parent to scan all")
    parser.add_argument("--no-api", action="store_true", help="Skip prey detection API call")
    parser.add_argument("--watch", action="store_true", help="Watch for new bursts continuously")
    parser.add_argument("--no-debug", action="store_true", help="Skip saving debug images")
    args = parser.parse_args()

    target = Path(args.path)
    if not target.exists():
        log.error("Path not found: %s", target)
        sys.exit(1)

    log.info("Loading YOLO model...")
    yolo = YOLODetector()
    face_det = CatFaceDetector()

    call_api = not args.no_api
    save_debug = not args.no_debug

    if args.watch:
        await watch_loop(target, yolo, face_det, call_api)
        return

    # Single burst or scan all
    if (target / "frame_00.jpg").exists():
        burst_dirs = [target]
    else:
        burst_dirs = sorted(p for p in target.iterdir() if p.is_dir() and (p / "frame_00.jpg").exists())

    results = []
    for bd in burst_dirs:
        r = await analyze_burst(bd, yolo, face_det, call_api=call_api, save_debug=save_debug)
        results.append(r)

    # Summary
    print("\n" + "=" * 60)
    cats = sum(1 for r in results if r.get("cat_detections", 0) > 0)
    faces = sum(1 for r in results if r.get("face_detected"))
    entering = sum(1 for r in results if r.get("direction") == "entering")
    exiting = sum(1 for r in results if r.get("direction") == "exiting")
    prey = sum(1 for r in results if r.get("prey_detected"))
    green = sum(1 for r in results if r.get("light_signal") == "green")
    red = sum(1 for r in results if r.get("light_signal") == "red")
    detector_avg = np.mean([r["detector_ms_per_frame"] for r in results if "detector_ms_per_frame" in r])
    print(f"Analyzed {len(results)} bursts: {cats} YOLO cat, {faces} Haar face, {entering} entering, {exiting} exiting, {prey} prey")
    print(f"Signals: {green} green, {red} red")
    print(f"Avg detector time: {detector_avg:.1f}ms/frame")
    trigger_latencies = [r["latency_ms"]["trigger_to_decision_ms"] for r in results if r.get("latency_ms", {}).get("trigger_to_decision_ms") is not None]
    if trigger_latencies:
        print(f"Avg trigger->decision: {np.mean(trigger_latencies):.0f}ms (min {min(trigger_latencies):.0f}, max {max(trigger_latencies):.0f})")
    if any("api_latency_ms" in r for r in results):
        api_times = [r["api_latency_ms"] for r in results if "api_latency_ms" in r]
        print(f"Avg API latency: {np.mean(api_times):.0f}ms (min {min(api_times):.0f}, max {max(api_times):.0f})")
    print("=" * 60)

    # Dump full results as JSON
    out_path = target / "prey_analysis.json" if target.is_dir() else target.parent / "prey_analysis.json"
    with open(out_path, "w") as f:
        json.dump(results, f, indent=2)
    log.info("Results saved to %s", out_path)


if __name__ == "__main__":
    asyncio.run(main())
