"""Auto-label burst frames with subject class using YOLO11n.

For each frame in captures/sd/, runs YOLO and emits one label record per
detected subject category to dataset/labels.jsonl using
source = "model:yolo11n_subject_v1". The label values are integers:
  0 = empty   (no detection above threshold)
  1 = cat
  2 = human
  3 = other   (dog, bird, anything else of interest)

A separate "model:yolo11n_subject_v1:detail" record carries free-form notes
about the detected classes + confidences (so we don't lose info when
multiple classes are present).

Idempotent: each (image_id, source, label) appears at most once thanks to
build_dataset.py's append-only dedup. To re-run with a newer model bump
the source string (e.g. yolo11n_subject_v2).

Usage:
  uv run python tools/autolabel_subject.py             # all frames
  uv run python tools/autolabel_subject.py --since 20260501
  uv run python tools/autolabel_subject.py --burst 20260515_031942_gen14
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import sys
import time
from pathlib import Path

import cv2
import ncnn
import numpy as np
from ncnn.utils.functional import nms, xywh2xyxy

REPO = Path(__file__).resolve().parent.parent
SD = REPO / "captures" / "sd"
LABELS = REPO / "dataset" / "labels.jsonl"
MODEL_DIR = (REPO.parent / "catflap-prey-detector" / "models"
             / "yolo11n_ncnn_model_384_640")

# Subject class enum
SUBJECT_EMPTY = 0
SUBJECT_CAT = 1
SUBJECT_HUMAN = 2
SUBJECT_OTHER = 3
SUBJECT_NAME = {0: "empty", 1: "cat", 2: "human", 3: "other"}

# Per-class confidence thresholds (deliberately low — this is a labeling
# AID, not ground truth. Humans will review/correct via tools/label_bursts.py).
# IR night frames are very hard for COCO-trained YOLO11n.
THRESHOLDS = {
    0:  0.30,  # person
    14: 0.30,  # bird
    15: 0.25,  # cat (lowest — this is mostly a cat flap)
    16: 0.35,  # dog
    17: 0.40,  # horse (sanity check, basically never expected)
}
# Map COCO class id -> our SUBJECT_*
COCO_TO_SUBJECT = {
    0: SUBJECT_HUMAN,
    14: SUBJECT_OTHER,
    15: SUBJECT_CAT,
    16: SUBJECT_OTHER,
    17: SUBJECT_OTHER,
}
COCO_NAMES = {0: "person", 14: "bird", 15: "cat", 16: "dog", 17: "horse"}

SOURCE = "model:yolo11n_subject_v1"
SOURCE_DETAIL = "model:yolo11n_subject_v1:detail"


class YOLO:
    INPUT_H, INPUT_W = 384, 640

    def __init__(self, model_dir: Path = MODEL_DIR):
        self.net = ncnn.Net()
        self.net.load_param(str(model_dir / "model.ncnn.param"))
        self.net.load_model(str(model_dir / "model.ncnn.bin"))

    def detect(self, image_bgr: np.ndarray) -> tuple[list[dict], dict[int, float]]:
        """Return (above_threshold_detections, max_conf_by_class_id_for_classes_of_interest).
        max_conf is useful for diagnostics: 'YOLO almost saw a cat at 0.22' tells
        a human reviewer where to look first.
        """
        h, w = image_bgr.shape[:2]
        rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        mat_in = ncnn.Mat.from_pixels_resize(
            rgb, ncnn.Mat.PixelType.PIXEL_RGB,
            w, h, self.INPUT_W, self.INPUT_H,
        )
        mat_in.substract_mean_normalize([0, 0, 0], [1/255.0, 1/255.0, 1/255.0])
        with self.net.create_extractor() as ex:
            ex.input("in0", mat_in)
            _, mat_out = ex.extract("out0")
        pred = np.array(mat_out).T  # (N, 84)
        boxes = pred[:, :4]
        scores = pred[:, 4:]

        out = []
        max_conf: dict[int, float] = {}
        for cls_id, thr in THRESHOLDS.items():
            conf = scores[:, cls_id]
            if conf.size:
                max_conf[cls_id] = float(conf.max())
            mask = conf > thr
            if not mask.any():
                continue
            cls_boxes = xywh2xyxy(boxes[mask])
            cls_conf = conf[mask]
            picked = nms(cls_boxes, cls_conf, 0.3)
            for idx in picked:
                out.append({
                    "class_id": int(cls_id),
                    "class_name": COCO_NAMES.get(cls_id, f"cls{cls_id}"),
                    "confidence": float(cls_conf[idx]),
                })
        return out, max_conf


def consolidate(detections: list[dict],
                max_conf: dict[int, float]) -> tuple[int, str]:
    """Return (subject_label, detail_string) for one frame."""
    detail_parts = []
    if detections:
        # Priority: cat > human > other
        has_cat = any(d["class_id"] == 15 for d in detections)
        has_human = any(d["class_id"] == 0 for d in detections)
        if has_cat:
            subj = SUBJECT_CAT
        elif has_human:
            subj = SUBJECT_HUMAN
        else:
            subj = SUBJECT_OTHER
        for d in detections:
            detail_parts.append(f"{d['class_name']}={d['confidence']:.2f}")
    else:
        subj = SUBJECT_EMPTY

    # Add max-conf for cat/person even if below threshold — helps a human
    # reviewer notice 'YOLO almost saw a cat at 0.22 here'.
    diag = []
    for cid in (15, 0):  # cat, person
        c = max_conf.get(cid, 0.0)
        if c > 0.10 and not any(d["class_id"] == cid for d in detections):
            diag.append(f"~{COCO_NAMES[cid]}={c:.2f}")
    if diag:
        detail_parts.append("(" + ", ".join(diag) + ")")
    return subj, ", ".join(detail_parts)


def load_existing_labels() -> set[tuple[str, str, int]]:
    seen = set()
    if not LABELS.exists():
        return seen
    with LABELS.open() as f:
        for line in f:
            try:
                rec = json.loads(line)
            except Exception:
                continue
            try:
                lbl = int(rec.get("label", -1))
            except (TypeError, ValueError):
                # Non-integer labels (e.g. cat_id strings like "mazge") belong
                # to other label sources and don't collide with the int-only
                # autolabel namespace this function tracks.
                continue
            seen.add((rec.get("image_id", ""), rec.get("source", ""), lbl))
    return seen


def burst_date(burst_id: str):
    try:
        return dt.datetime.strptime(burst_id[:8], "%Y%m%d").date()
    except ValueError:
        return None


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--since", help="YYYYMMDD")
    ap.add_argument("--burst", help="Single burst id")
    ap.add_argument("--limit", type=int, default=0)
    args = ap.parse_args()

    if not MODEL_DIR.exists():
        sys.exit(f"YOLO model not found at {MODEL_DIR}")
    if not LABELS.parent.exists():
        LABELS.parent.mkdir(parents=True)

    print("Loading YOLO11n...", flush=True)
    yolo = YOLO()

    burst_dirs = sorted(d for d in SD.iterdir() if d.is_dir())
    if args.burst:
        burst_dirs = [d for d in burst_dirs if d.name == args.burst]
    if args.since:
        since = dt.datetime.strptime(args.since, "%Y%m%d").date()
        burst_dirs = [d for d in burst_dirs
                      if (bd := burst_date(d.name)) and bd >= since]
    if args.limit:
        burst_dirs = burst_dirs[:args.limit]

    print(f"Processing {len(burst_dirs)} burst folders", flush=True)
    seen = load_existing_labels()
    now = dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds")

    total_frames = 0
    new_labels = 0
    counts = {0: 0, 1: 0, 2: 0, 3: 0}
    t0 = time.time()
    with LABELS.open("a") as out:
        for bi, bd in enumerate(burst_dirs, 1):
            jpgs = sorted(bd.glob("f*.jpg"))
            if not jpgs:
                continue
            for jp in jpgs:
                total_frames += 1
                image_id = f"captures/sd/{bd.name}/{jp.name}"
                # Skip if any subject label exists for this image+source.
                if any((image_id, SOURCE, v) in seen for v in (0, 1, 2, 3)):
                    continue

                img = cv2.imread(str(jp), cv2.IMREAD_COLOR)
                if img is None:
                    continue
                detections, max_conf = yolo.detect(img)
                subj, detail = consolidate(detections, max_conf)
                counts[subj] += 1

                rec = {
                    "image_id": image_id,
                    "source": SOURCE,
                    "label": int(subj),
                    "confidence": max((d["confidence"] for d in detections), default=0.0),
                    "ts": now,
                    "notes": detail or None,
                }
                out.write(json.dumps(rec, sort_keys=True) + "\n")
                new_labels += 1
                seen.add((image_id, SOURCE, int(subj)))

            if bi % 25 == 0 or bi == len(burst_dirs):
                elapsed = time.time() - t0
                rate = total_frames / elapsed if elapsed > 0 else 0
                print(f"  [{bi}/{len(burst_dirs)}] {bd.name} | "
                      f"{total_frames} frames @ {rate:.1f} fps | "
                      f"new={new_labels} | empty={counts[0]} cat={counts[1]} "
                      f"human={counts[2]} other={counts[3]}",
                      flush=True)

    dur = time.time() - t0
    print(f"\nDone. {new_labels} new labels in {dur:.0f}s "
          f"({total_frames/dur:.1f} fps)")
    print(f"Subject distribution this run: empty={counts[0]} cat={counts[1]} "
          f"human={counts[2]} other={counts[3]}")


if __name__ == "__main__":
    main()
