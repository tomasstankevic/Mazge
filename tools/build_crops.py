#!/usr/bin/env python3
"""Generate cat body+snout crops via two detection pipelines for comparison.

Pipelines:
  A. mdv6_raw    — MegaDetector V6 yolov10-c on raw 640x480 frames
                   (best detection: 95% per-frame, 0% empty-FP)
  B. yolo11x_rot — Ultralytics yolo11x_640 on rotated CCW + top-160 cropped
                   frames (100% per-burst, 2% empty-FP, 4× faster)

Both pipelines:
  - work in the UPRIGHT (rotated CCW + top-cropped) coordinate space for cropping
  - apply niciBume's snout extension heuristic:
      entering → snout = top of bbox + 40% extension downward
      exiting  → snout = bottom of bbox + 40% extension upward
      unknown  → assume entering
  - save body and snout crops as 224x224 RGB JPGs
  - mark exit bursts as label=0 regardless of human_prey (cat can't carry out)

Outputs (per pipeline):
  dataset/<pipeline>/<burst>/f{N:02d}_body.jpg     # body bbox + padding
  dataset/<pipeline>/<burst>/f{N:02d}_snout.jpg    # snout region
  dataset/<pipeline>/<burst>/preview.jpg            # upright frame with
                                                    # green body + cyan snout
                                                    # rectangles for the
                                                    # highest-confidence frame
  dataset/<pipeline>/<burst>/bboxes.json
  dataset/<pipeline>/_index.csv
  dataset/<pipeline>/_dashboard.html  ← live progress + previews

Where <pipeline> is `crops_mdv6_raw` or `crops_yolo11x_rotcrop`.

Usage:
  cd Mazge && uv run python tools/build_crops.py --pipeline mdv6_raw --only-prey
  uv run python tools/build_crops.py --pipeline yolo11x_rotcrop --only-prey
  uv run python tools/build_crops.py --pipeline mdv6_raw            # all bursts
"""
from __future__ import annotations

import argparse
import csv
import json
import logging
import os
import time
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

os.environ.setdefault("SSL_CERT_FILE", __import__("certifi").where())

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s %(levelname)-5s %(message)s",
                    datefmt="%H:%M:%S")
log = logging.getLogger("crops")

REPO = Path(__file__).resolve().parent.parent
SD = REPO / "captures" / "sd"
DATASET = REPO / "dataset"

# Crop geometry
OUT_SIZE = 224
BODY_PAD_FRAC = 0.10        # 10% padding around cat bbox
# Snout region: portion of the body bbox along the entering-direction axis.
# We take the leading 45 % of the bbox (i.e. the head end), extending only
# slightly past the bbox edge to capture anything held in front of the mouth
# (a mouse hanging from a jaw, etc.).
SNOUT_HEAD_FRAC = 0.45      # 45 % of body bbox height = head + shoulders
SNOUT_EXTEND_FRAC = 0.10    # small 10 % extension past leading edge
CAT_CONF = 0.15
CROP_TOP_AFTER_ROT = 160    # pixels to strip from top of rotated frame
                            # (removes the bright door occlusion)
DASHBOARD_REFRESH_EVERY = 10  # frames


# ── Frame upright transform ───────────────────────────────────────────

def to_upright(raw_bgr: np.ndarray) -> np.ndarray:
    """Rotate CCW + crop top to get the upright 480x480 view."""
    rot = cv2.rotate(raw_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return rot[CROP_TOP_AFTER_ROT:, :, :]


def raw_bbox_to_upright(bbox_raw: tuple[int, int, int, int],
                        raw_shape: tuple[int, int]) -> tuple[int, int, int, int]:
    """Map a bbox from raw 640x480 coords to upright (rotated+cropped) coords.

    After CCW 90° rotation of an HxW image (here 480x640):
      x_rot = y_raw
      y_rot = (W - 1) - x_raw         # raw image width = 640
    Then we strip CROP_TOP_AFTER_ROT pixels from the top:
      y_upr = y_rot - CROP_TOP_AFTER_ROT

    bbox_raw is (x, y, w, h) in raw frame coordinates.
    Returns (x, y, w, h) in upright frame coordinates.
    """
    H_raw, W_raw = raw_shape[:2]
    x, y, w, h = bbox_raw
    # Raw corners
    x1, y1 = x, y
    x2, y2 = x + w, y + h
    # After CCW rotation: (x', y') = (y, W - 1 - x). Corners swap.
    rx1 = y1
    ry1 = (W_raw - 1) - x2
    rx2 = y2
    ry2 = (W_raw - 1) - x1
    # Sort
    rx0, rx1 = sorted([rx1, rx2])
    ry0, ry1 = sorted([ry1, ry2])
    # Crop top
    ry0 -= CROP_TOP_AFTER_ROT
    ry1 -= CROP_TOP_AFTER_ROT
    return int(rx0), int(ry0), int(rx1 - rx0), int(ry1 - ry0)


# ── Detector pipelines ────────────────────────────────────────────────

class MDv6Pipeline:
    """MegaDetector V6 yolov10-c on RAW frames."""
    name = "mdv6_raw"
    dirname = "crops_mdv6_raw"

    def __init__(self):
        from PytorchWildlife.models import detection as pw_detection
        self.model = pw_detection.MegaDetectorV6(
            device="mps", version="MDV6-yolov10-c")
        import ultralytics.utils
        ultralytics.utils.LOGGER.setLevel(logging.WARNING)

    def detect_cat(self, raw_bgr: np.ndarray) -> tuple[float, int, int, int, int] | None:
        """Detect on raw frame, return bbox in raw coords."""
        res = self.model.single_image_detection(
            raw_bgr, det_conf_thres=CAT_CONF)
        det = res["detections"]
        best = None
        for box, conf, cls in zip(det.xyxy, det.confidence, det.class_id):
            if int(cls) != 0:  # 0 = animal
                continue
            x1, y1, x2, y2 = box.tolist()
            area = (x2 - x1) * (y2 - y1)
            cand = (float(conf), int(x1), int(y1),
                    int(x2 - x1), int(y2 - y1), area)
            if best is None or cand[5] > best[5]:
                best = cand
        return best[:5] if best else None

    def bbox_to_upright(self, bbox_raw, raw_shape):
        return raw_bbox_to_upright(bbox_raw, raw_shape)


class YOLOxRotCropPipeline:
    """Ultralytics yolo11x_640 on rotated CCW + top-160 cropped frames."""
    name = "yolo11x_rotcrop"
    dirname = "crops_yolo11x_rotcrop"

    def __init__(self):
        from ultralytics import YOLO
        import ultralytics.utils
        ultralytics.utils.LOGGER.setLevel(logging.WARNING)
        self.model = YOLO("yolo11x.pt")
        self.imgsz = 640
        import torch
        self.device = ("mps" if torch.backends.mps.is_available()
                       else ("cuda" if torch.cuda.is_available() else "cpu"))

    def detect_cat(self, raw_bgr: np.ndarray) -> tuple[float, int, int, int, int] | None:
        """Detect on upright frame, return bbox already in upright coords."""
        upright = to_upright(raw_bgr)
        res = self.model.predict(
            upright, imgsz=self.imgsz, conf=CAT_CONF,
            classes=[15],  # COCO cat
            device=self.device, verbose=False)[0]
        if res.boxes is None or len(res.boxes) == 0:
            return None
        best = None
        for b, c in zip(res.boxes.xyxy.cpu().numpy(),
                        res.boxes.conf.cpu().numpy()):
            x1, y1, x2, y2 = b.tolist()
            area = (x2 - x1) * (y2 - y1)
            cand = (float(c), int(x1), int(y1),
                    int(x2 - x1), int(y2 - y1), area)
            if best is None or cand[5] > best[5]:
                best = cand
        return best[:5] if best else None

    def bbox_to_upright(self, bbox_upright, raw_shape):
        # Already in upright coords
        return bbox_upright


PIPELINES = {
    "mdv6_raw": MDv6Pipeline,
    "yolo11x_rotcrop": YOLOxRotCropPipeline,
}


# ── Crop helpers (all operating on UPRIGHT frame) ─────────────────────

def square_crop(img: np.ndarray, cx: int, cy: int, side: int) -> np.ndarray:
    H, W = img.shape[:2]
    half = side // 2
    x0, y0 = cx - half, cy - half
    x1, y1 = x0 + side, y0 + side
    pad_l = max(0, -x0)
    pad_t = max(0, -y0)
    pad_r = max(0, x1 - W)
    pad_b = max(0, y1 - H)
    if pad_l or pad_t or pad_r or pad_b:
        img = cv2.copyMakeBorder(img, pad_t, pad_b, pad_l, pad_r,
                                  cv2.BORDER_CONSTANT, value=(114, 114, 114))
        x0 += pad_l
        y0 += pad_t
        x1 += pad_l
        y1 += pad_t
    return img[y0:y1, x0:x1]


def body_crop(upright: np.ndarray,
              bbox: tuple[int, int, int, int]) -> np.ndarray:
    x, y, w, h = bbox
    side = int(max(w, h) * (1 + 2 * BODY_PAD_FRAC))
    cx, cy = x + w // 2, y + h // 2
    crop = square_crop(upright, cx, cy, side)
    return cv2.resize(crop, (OUT_SIZE, OUT_SIZE), interpolation=cv2.INTER_AREA)


def snout_rect(bbox: tuple[int, int, int, int],
               direction: str | None) -> tuple[int, int, int, int]:
    """Compute the snout rectangle in upright coords.

    Entering: cat enters head-first (head at TOP of body bbox).
              Snout box = top SNOUT_HEAD_FRAC of bbox, extended slightly
              UP past the bbox top to catch anything held in front of mouth.
    Exiting:  cat leaves butt-first (head at BOTTOM, opposite direction).
              Snout box = bottom SNOUT_HEAD_FRAC of bbox, extended slightly DOWN.
    Unknown:  treat as entering (safer for prey detection).

    The snout box covers ~45% of the bbox height — enough to include the head
    and the front paws / mouse-in-mouth, while excluding the back half of the
    body (which is mostly fur for the classifier to ignore).
    """
    x, y, w, h = bbox
    head_h = int(h * SNOUT_HEAD_FRAC)
    extend = int(h * SNOUT_EXTEND_FRAC)
    if direction == "exiting":
        # Bottom of bbox + extend further down
        rx = x
        ry = y + h - head_h
        rw = w
        rh = head_h + extend
    else:
        # Top of bbox + extend slightly up
        rx = x
        ry = y - extend
        rw = w
        rh = head_h + extend
    return rx, ry, rw, rh


def snout_crop(upright: np.ndarray,
               bbox: tuple[int, int, int, int],
               direction: str | None) -> np.ndarray:
    sx, sy, sw, sh = snout_rect(bbox, direction)
    side = max(sw, sh)
    cx, cy = sx + sw // 2, sy + sh // 2
    crop = square_crop(upright, cx, cy, side)
    return cv2.resize(crop, (OUT_SIZE, OUT_SIZE), interpolation=cv2.INTER_AREA)


def render_preview(upright: np.ndarray,
                   body_bbox: tuple[int, int, int, int],
                   snout_bbox: tuple[int, int, int, int],
                   text: str) -> np.ndarray:
    """Draw the body (green) and snout (cyan) rectangles on the upright frame.

    Rectangles are CLAMPED to frame bounds so off-screen extensions appear
    as flush-to-edge instead of invisible. We also draw a label on each.
    """
    img = upright.copy()
    H, W = img.shape[:2]

    def clamp(box):
        x, y, w, h = box
        x0 = max(0, x)
        y0 = max(0, y)
        x1 = min(W - 1, x + w)
        y1 = min(H - 1, y + h)
        return x0, y0, x1, y1

    # Body (green)
    bx0, by0, bx1, by1 = clamp(body_bbox)
    cv2.rectangle(img, (bx0, by0), (bx1, by1), (0, 255, 0), 2)
    cv2.putText(img, "BODY", (bx0 + 3, max(by0 + 14, 14)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1, cv2.LINE_AA)

    # Snout (cyan in BGR -> appears cyan on screen)
    sx0, sy0, sx1, sy1 = clamp(snout_bbox)
    cv2.rectangle(img, (sx0, sy0), (sx1, sy1), (255, 255, 0), 2)
    cv2.putText(img, "SNOUT", (sx0 + 3, max(sy0 + 14, 14)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1, cv2.LINE_AA)

    # Top banner with metadata
    cv2.rectangle(img, (0, 0), (W, 18), (0, 0, 0), -1)
    cv2.putText(img, text, (4, 13),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
    return img


# ── Effective label (exits forced to 0) ──────────────────────────────

def effective_prey_label(human_prey: str, human_direction: str) -> tuple[str, str]:
    """Override human_prey to 0 on exit bursts.

    Returns (effective_label, reason_tag) where reason_tag is one of:
      "as_human"  — used human_prey verbatim
      "exit_zero" — forced to 0 because direction=exiting
      "unclear"   — human_prey was unclear, leave as-is
    """
    if human_direction == "exiting":
        return "0", "exit_zero"
    if human_prey == "unclear":
        return "unclear", "unclear"
    return human_prey, "as_human"


# ── Dashboard ─────────────────────────────────────────────────────────

DASH_CSS = """
body{background:#1a1a1a;color:#eee;font-family:-apple-system,monospace;margin:0;padding:16px;}
h1{margin:0 0 8px 0;font-size:18px;}
h2{font-size:14px;margin:16px 0 6px 0;border-bottom:1px solid #444;padding-bottom:4px;}
.bar{background:#333;height:18px;border-radius:9px;overflow:hidden;margin:8px 0;}
.fill{background:linear-gradient(90deg,#0c8,#0fa);height:100%;transition:width .4s;}
.stat{display:inline-block;background:#262626;padding:6px 12px;border-radius:6px;margin:4px 6px 4px 0;}
.stat b{color:#0fa;}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(320px,1fr));gap:6px;}
.burst{background:#222;border-radius:4px;padding:6px;font-size:11px;}
.burst img{width:100%;display:block;border-radius:3px;background:#333;}
.burst.prey{outline:2px solid #f44;}
.burst.exit{outline:2px solid #48a;}
.row{display:flex;gap:2px;margin:4px 0;}
.row .cell{flex:1;}
.row img{width:100%;display:block;}
.row small{font-size:9px;color:#aaa;display:block;text-align:center;}
.tag{padding:1px 4px;border-radius:2px;font-size:9px;color:#000;background:#888;display:inline-block;margin-right:2px;}
.tag.prey{background:#f88;color:#400;}
.tag.exit{background:#8af;color:#024;}
.tag.entering{background:#9f9;color:#040;}
.updated{color:#888;font-size:11px;float:right;}
"""


def write_dashboard(out_dir: Path, pipeline_name: str, state: dict,
                    preview_bursts: list[str]) -> None:
    total = state["total"]
    done = state["done"]
    found = state["found_cat"]
    skipped = state["skipped"]
    failed = state["failed"]
    elapsed = time.time() - state["t0"]
    rate = (done / elapsed) if elapsed > 0 else 0
    pct = (done / total * 100) if total else 0
    eta = ((total - done) / rate) if rate else 0
    by_subject = state["by_subject"]

    pieces = [
        "<!doctype html><html><head><meta charset='utf-8'>",
        "<meta http-equiv='refresh' content='15'>",
        f"<title>{pipeline_name} crops</title>",
        f"<style>{DASH_CSS}</style></head><body>",
        f"<h1>Crops — pipeline <b>{pipeline_name}</b></h1>",
        f"<span class='updated'>refreshes every 15 s · {time.strftime('%H:%M:%S')}</span>",
        f"<div class='bar'><div class='fill' style='width:{pct:.1f}%'></div></div>",
        f"<div class='stat'>Frames <b>{done}/{total}</b> ({pct:.1f} %)</div>",
        f"<div class='stat'>Cat found <b>{found}</b> ({100*found/max(done,1):.0f} %)</div>",
        f"<div class='stat'>Skipped <b>{skipped}</b></div>",
        f"<div class='stat'>Failed <b>{failed}</b></div>",
        f"<div class='stat'>Rate <b>{rate:.1f} fps</b></div>",
        f"<div class='stat'>Elapsed <b>{elapsed/60:.1f} min</b></div>",
        f"<div class='stat'>ETA <b>{eta/60:.1f} min</b></div>",
    ]

    pieces.append("<h2>Cat-found rate by human_subject</h2>")
    for subj, (n_tot, n_cat) in sorted(by_subject.items()):
        if n_tot == 0:
            continue
        pct_s = 100 * n_cat / n_tot
        pieces.append(f"<div class='stat'>{subj or '?'}: <b>{n_cat}/{n_tot}</b> "
                      f"({pct_s:.0f} %)</div>")

    pieces.append(f"<h2>Preview bursts ({len(preview_bursts)}, entering only) — "
                  f"<span style='color:#0f0'>green=body</span>, "
                  f"<span style='color:#0ff'>cyan=snout</span></h2>")
    pieces.append("<div class='grid'>")
    for entry in preview_bursts:
        bid = entry["burst_id"]
        bdir = out_dir / bid
        if not bdir.exists():
            continue
        prev = bdir / "preview.jpg"
        if not prev.exists():
            continue
        css_class = "burst prey" if entry["human_prey"] == "1" else "burst"
        tag_html = ""
        if entry["human_prey"] == "1":
            tag_html += "<span class='tag prey'>PREY</span>"
        tag_html += "<span class='tag entering'>ENTER</span>"
        pieces.append(
            f"<div class='{css_class}'>"
            f"{tag_html} {bid}<br>"
            f"<img src='{bid}/{prev.name}'>"
            f"</div>"
        )
    pieces.append("</div></body></html>")

    dash = out_dir / "_dashboard.html"
    tmp = dash.with_suffix(".html.tmp")
    tmp.write_text("".join(pieces))
    tmp.replace(dash)


# ── Main ──────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--pipeline", required=True, choices=list(PIPELINES),
                    help="Which detector pipeline to run")
    ap.add_argument("--only-prey", action="store_true",
                    help="Only process prey bursts (smoke test)")
    ap.add_argument("--bursts", help="Comma-separated burst IDs")
    ap.add_argument("--limit", type=int, default=0)
    ap.add_argument("--skip-existing", type=lambda s: s.lower() == "true",
                    default=True)
    ap.add_argument(
        "--label-mode", default="human",
        choices=["human", "all"],
        help="human: only bursts with full human labels — cat (entering or "
             "exiting) + non-cat (empty/human/other) as hard negatives. "
             "all: every burst with JPGs (legacy).")
    ap.add_argument("--include-unclear-direction", action="store_true",
                    help="In --label-mode=human, also include cat bursts where "
                         "human_direction is 'unclear' (assume entering).")
    ap.add_argument("--include-unclear-prey", action="store_true",
                    help="In --label-mode=human, also include cat bursts where "
                         "human_prey is 'unclear'.")
    args = ap.parse_args()

    pipeline_cls = PIPELINES[args.pipeline]
    out_dir = DATASET / pipeline_cls.dirname
    out_dir.mkdir(parents=True, exist_ok=True)
    index_csv = out_dir / "_index.csv"
    existing_index_rows: dict[tuple[str, str, str], dict[str, str]] = {}
    if args.skip_existing and index_csv.exists():
        with open(index_csv, newline="") as f:
            for row in csv.DictReader(f):
                key = (row.get("burst_id", ""), row.get("frame_idx", ""),
                       row.get("image_id", ""))
                existing_index_rows[key] = row

    manifest_rows = list(csv.DictReader(open(DATASET / "manifest.csv")))
    bursts_rows = {r["burst_id"]: r
                   for r in csv.DictReader(open(DATASET / "bursts.csv"))}

    # ── Burst-level filtering (the new label-aware part) ─────────────
    def keep_burst(bid: str) -> bool:
        b = bursts_rows.get(bid)
        if not b or b.get("has_jpgs") != "1":
            return False
        subj = b["human_subject"]
        if args.label_mode == "all":
            return True
        # human mode
        if subj == "":
            return False  # unlabeled, skip
        if subj == "cat":
            direction = b["human_direction"]
            prey = b["human_prey"]
            if direction == "":
                return False
            if direction == "unclear" and not args.include_unclear_direction:
                return False
            if prey == "unclear" and not args.include_unclear_prey:
                return False
            return True
        # empty / human / other → always include as hard negatives
        # (label will be forced to 0 in effective_prey_label)
        return subj in ("empty", "human", "other")

    if args.bursts:
        wanted = set(args.bursts.split(","))
        manifest_rows = [r for r in manifest_rows if r["burst_id"] in wanted]
    elif args.only_prey:
        prey_set = {b for b, r in bursts_rows.items() if r["human_prey"] == "1"}
        manifest_rows = [r for r in manifest_rows if r["burst_id"] in prey_set]
    else:
        manifest_rows = [r for r in manifest_rows if keep_burst(r["burst_id"])]

    if args.limit:
        manifest_rows = manifest_rows[:args.limit]
    manifest_rows = [r for r in manifest_rows if r["has_jpg"] == "1"]

    # Log breakdown of what we're going to process
    kept_bursts = sorted({r["burst_id"] for r in manifest_rows})
    bd_summary = defaultdict(int)
    for bid in kept_bursts:
        b = bursts_rows[bid]
        key = (b["human_subject"] or "?", b["human_direction"] or "?",
               b["human_prey"] or "?")
        bd_summary[key] += 1
    log.info("Pipeline=%s, %d bursts × %d frames after filtering "
             "(label_mode=%s, unclear_dir=%s, unclear_prey=%s):",
             args.pipeline, len(kept_bursts), len(manifest_rows),
             args.label_mode, args.include_unclear_direction,
             args.include_unclear_prey)
    for k, n in sorted(bd_summary.items()):
        log.info("    subj=%-6s dir=%-9s prey=%-8s n=%d", *k, n)

    # Pre-compute which bursts to feature on the dashboard.
    # Show ENTERING cat bursts only (prey first, then non-prey).
    all_bursts = sorted({r["burst_id"] for r in manifest_rows})
    entering_cat = [b for b in all_bursts
                    if bursts_rows.get(b, {}).get("human_subject") == "cat"
                    and bursts_rows.get(b, {}).get("human_direction") == "entering"]
    prey_bids = sorted(b for b in entering_cat
                       if bursts_rows[b]["human_prey"] == "1")
    nonprey_bids = sorted(b for b in entering_cat
                          if bursts_rows[b]["human_prey"] != "1")
    preview_bids = prey_bids + nonprey_bids[:30]

    def preview_entry(bid: str) -> dict:
        return {
            "burst_id": bid,
            "human_prey": bursts_rows.get(bid, {}).get("human_prey", ""),
            "human_direction": bursts_rows.get(bid, {}).get("human_direction", ""),
            "preview_fidx": 9,  # last frame = cat closest to camera
        }
    preview_entries = [preview_entry(b) for b in preview_bids]

    log.info("Output dir: %s", out_dir)
    log.info("Dashboard: file://%s/_dashboard.html", out_dir)

    state = {
        "total": len(manifest_rows),
        "done": 0,
        "found_cat": 0,
        "skipped": 0,
        "failed": 0,
        "by_subject": defaultdict(lambda: [0, 0]),
        "t0": time.time(),
    }
    write_dashboard(out_dir, args.pipeline, state, preview_entries)

    new_index = not index_csv.exists()
    idx_f = open(index_csv, "a", newline="")
    idx_w = csv.DictWriter(idx_f, fieldnames=[
        "burst_id", "frame_idx", "image_id", "human_prey",
        "effective_prey", "label_source",
        "human_subject", "human_direction", "split",
        "cat_found", "cat_conf",
        "bbox_x", "bbox_y", "bbox_w", "bbox_h",
        "body_path", "snout_path",
    ])
    if new_index:
        idx_w.writeheader()

    pipeline = None
    bbox_jsons: dict[str, dict] = {}
    best_preview: dict[str, tuple[float, int, np.ndarray,
                                   tuple[int, int, int, int],
                                   tuple[int, int, int, int], str]] = {}
    last_burst = None
    last_dash_at = 0

    def flush_burst(bid: str) -> None:
        if bid in bbox_jsons:
            (out_dir / bid).mkdir(parents=True, exist_ok=True)
            (out_dir / bid / "bboxes.json").write_text(
                json.dumps(bbox_jsons[bid], indent=2, sort_keys=True))
        # Render the best-frame preview for this burst
        if bid in best_preview:
            _, _, upright, bbox, sbox, text = best_preview[bid]
            prev = render_preview(upright, bbox, sbox, text)
            cv2.imwrite(str(out_dir / bid / "preview.jpg"), prev,
                        [cv2.IMWRITE_JPEG_QUALITY, 80])

    for i, row in enumerate(manifest_rows, 1):
        bid = row["burst_id"]
        fidx = int(row["frame_idx"])
        index_key = (bid, row["frame_idx"], row["image_id"])
        burst = bursts_rows.get(bid, {})
        split = row.get("split", "")
        subj = burst.get("human_subject", "")
        direction = burst.get("human_direction", "")
        human_prey = burst.get("human_prey", "")
        eff_prey, label_src = effective_prey_label(human_prey, direction)

        if bid != last_burst:
            if last_burst is not None:
                flush_burst(last_burst)
                bbox_jsons.pop(last_burst, None)
                best_preview.pop(last_burst, None)
            last_burst = bid
            cdir = out_dir / bid
            cdir.mkdir(parents=True, exist_ok=True)
            existing = cdir / "bboxes.json"
            if existing.exists():
                try:
                    bbox_jsons[bid] = json.loads(existing.read_text())
                except Exception:
                    bbox_jsons[bid] = {}
            else:
                bbox_jsons[bid] = {}

        body_path = out_dir / bid / f"f{fidx:02d}_body.jpg"
        snout_path = out_dir / bid / f"f{fidx:02d}_snout.jpg"

        if args.skip_existing and index_key in existing_index_rows:
            prev = existing_index_rows[index_key]
            # If we already have a body crop path in the index, skip to avoid
            # appending duplicate rows. If body_path is empty, keep processing
            # so this run can still recover a missed detection.
            if prev.get("body_path", ""):
                state["skipped"] += 1
                state["done"] += 1
                if prev.get("cat_found") == "1":
                    state["found_cat"] += 1
                    state["by_subject"][subj][1] += 1
                state["by_subject"][subj][0] += 1
                continue

        if args.skip_existing and body_path.exists():
            state["skipped"] += 1
            state["done"] += 1
            entry = bbox_jsons[bid].get(str(fidx))
            if entry and entry.get("cat_found"):
                state["found_cat"] += 1
                state["by_subject"][subj][1] += 1
            state["by_subject"][subj][0] += 1
            continue

        if pipeline is None:
            log.info("Loading pipeline %s…", args.pipeline)
            pipeline = pipeline_cls()

        img_path = REPO / row["image_id"]
        raw = cv2.imread(str(img_path))
        if raw is None:
            log.warning("Failed reading %s", img_path)
            state["failed"] += 1
            state["done"] += 1
            continue

        try:
            best = pipeline.detect_cat(raw)
        except Exception as e:
            log.warning("Detect failed on %s: %s", img_path, e)
            state["failed"] += 1
            state["done"] += 1
            continue

        upright = to_upright(raw)
        bbox_upright = None
        if best:
            conf, x, y, w, h = best
            bbox_upright = pipeline.bbox_to_upright((x, y, w, h), raw.shape)

        entry = {
            "cat_found": bool(best),
            "cat_conf": round(best[0], 4) if best else 0.0,
            "bbox_upright": list(bbox_upright) if bbox_upright else None,
            "direction_used": direction or "unknown",
            "label_source": label_src,
            "effective_prey": eff_prey,
            "ts": int(time.time()),
        }

        body_p = ""
        snout_p = ""
        if bbox_upright:
            try:
                body = body_crop(upright, bbox_upright)
                cv2.imwrite(str(body_path), body, [cv2.IMWRITE_JPEG_QUALITY, 88])
                body_p = f"{pipeline_cls.dirname}/{bid}/{body_path.name}"
                snout = snout_crop(upright, bbox_upright, direction)
                cv2.imwrite(str(snout_path), snout, [cv2.IMWRITE_JPEG_QUALITY, 88])
                snout_p = f"{pipeline_cls.dirname}/{bid}/{snout_path.name}"
                state["found_cat"] += 1
                state["by_subject"][subj][1] += 1

                # Track best preview for this burst
                cur_best = best_preview.get(bid)
                if cur_best is None or entry["cat_conf"] > cur_best[0]:
                    sbox = snout_rect(bbox_upright, direction)
                    text = (f"{bid} f{fidx:02d} conf={entry['cat_conf']:.2f} "
                            f"dir={direction or '?'} prey={eff_prey} ({label_src})")
                    best_preview[bid] = (entry["cat_conf"], fidx,
                                          upright.copy(), bbox_upright, sbox, text)
            except Exception as e:
                log.warning("Crop failed on %s: %s", img_path, e)
                state["failed"] += 1

        bbox_jsons[bid][str(fidx)] = entry
        idx_w.writerow({
            "burst_id": bid, "frame_idx": fidx, "image_id": row["image_id"],
            "human_prey": human_prey, "effective_prey": eff_prey,
            "label_source": label_src,
            "human_subject": subj, "human_direction": direction, "split": split,
            "cat_found": int(bool(best)),
            "cat_conf": entry["cat_conf"],
            "bbox_x": bbox_upright[0] if bbox_upright else "",
            "bbox_y": bbox_upright[1] if bbox_upright else "",
            "bbox_w": bbox_upright[2] if bbox_upright else "",
            "bbox_h": bbox_upright[3] if bbox_upright else "",
            "body_path": body_p, "snout_path": snout_p,
        })
        existing_index_rows[index_key] = {
            "cat_found": str(int(bool(best))),
            "body_path": body_p,
        }
        idx_f.flush()

        state["done"] += 1
        state["by_subject"][subj][0] += 1

        if state["done"] - last_dash_at >= DASHBOARD_REFRESH_EVERY:
            flush_burst(bid)
            write_dashboard(out_dir, args.pipeline, state, preview_entries)
            last_dash_at = state["done"]
            log.info("[%d/%d] %s f%02d cat=%s  rate=%.1f fps",
                     state["done"], state["total"], bid, fidx,
                     "Y" if best else "N",
                     state["done"] / max(time.time() - state["t0"], 1))

    if last_burst is not None:
        flush_burst(last_burst)
    idx_f.close()
    write_dashboard(out_dir, args.pipeline, state, preview_entries)

    log.info("Done. %d processed (%d cat-found, %d failed, %d skipped) in %.1f min",
             state["done"], state["found_cat"], state["failed"],
             state["skipped"], (time.time() - state["t0"]) / 60)
    log.info("Dashboard: file://%s/_dashboard.html", out_dir)


if __name__ == "__main__":
    main()
