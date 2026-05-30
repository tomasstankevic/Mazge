#!/usr/bin/env python3
"""Compare cat-detection variants on the same stratified burst sample.

Variants tested:
  A. nano_ncnn_384x640    — existing on-device pipeline (NCNN, stretched)
  B. yolo11n_letterbox    — same nano but via ultralytics with proper letterbox
  C. yolo11s_640          — small (9M params)
  D. yolo11m_640          — medium (20M params)
  E. yolo11x_640          — extra-large (57M params, COCO upper bound)
  F. yolo11x_960          — extra-large at higher resolution

For each: per-frame and per-burst cat detection rate on the same sample
already evaluated in `models/subject_detection_eval/summary.csv`.

The .pt models are auto-downloaded by ultralytics on first use (cached in
~/.config/Ultralytics or the current dir).

Usage:
  cd Mazge && uv run python tools/test_yolo_variants.py
  uv run python tools/test_yolo_variants.py --variants nano_ncnn,yolo11x_640
  uv run python tools/test_yolo_variants.py --max-bursts 30  # quick smoke
"""
from __future__ import annotations

import argparse
import csv
import logging
import sys
import time
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from prey_analyzer import YOLODetector, CAT_CLASS_ID  # noqa: E402

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s %(levelname)-5s %(message)s",
                    datefmt="%H:%M:%S")
log = logging.getLogger("yolo_variants")

REPO = Path(__file__).resolve().parent.parent
SD = REPO / "captures" / "sd"
OUT = REPO / "models" / "subject_detection_eval" / "variants"
SAMPLE_CSV = REPO / "models" / "subject_detection_eval" / "summary.csv"
MANIFEST_CSV = REPO / "dataset" / "manifest.csv"

CAT_CONF = 0.15


# ────────────────────────────────────────────────────────────────────────
# Variant adapters: each .detect(bgr) returns list[(conf, x, y, w, h)]
# ────────────────────────────────────────────────────────────────────────

class NcnnNano:
    name = "nano_ncnn_384x640"
    def __init__(self):
        self.det = YOLODetector()
    def detect(self, bgr):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        raws = self.det.detect(rgb)
        out = self.det.detect_to_image_coords(rgb, raws)
        return [(d["confidence"], d["x"], d["y"], d["w"], d["h"])
                for d in out if d["class_id"] == CAT_CLASS_ID]


class UltralyticsModel:
    """Adapter for ultralytics YOLO11{n,s,m,l,x}.pt at chosen imgsz."""
    def __init__(self, weights: str, imgsz: int = 640, name: str | None = None,
                 device: str = "auto"):
        from ultralytics import YOLO
        self.model = YOLO(weights)
        self.imgsz = imgsz
        self.name = name or f"{Path(weights).stem}_{imgsz}"
        # Pick device
        import torch
        if device == "auto":
            if torch.backends.mps.is_available():
                self.device = "mps"
            elif torch.cuda.is_available():
                self.device = "cuda"
            else:
                self.device = "cpu"
        else:
            self.device = device

    def detect(self, bgr):
        # ultralytics handles letterbox/normalize internally; pass conf=CAT_CONF
        res = self.model.predict(bgr, imgsz=self.imgsz, conf=CAT_CONF,
                                 classes=[CAT_CLASS_ID], device=self.device,
                                 verbose=False)[0]
        out = []
        if res.boxes is None or len(res.boxes) == 0:
            return out
        for b, c in zip(res.boxes.xyxy.cpu().numpy(),
                        res.boxes.conf.cpu().numpy()):
            x1, y1, x2, y2 = b.tolist()
            out.append((float(c), int(x1), int(y1),
                        int(x2 - x1), int(y2 - y1)))
        return out


class MegaDetector:
    """PytorchWildlife MegaDetector V6 (animal/person/vehicle).

    Class id 0 = animal — used as our 'cat' proxy. MegaDetector is
    trained on millions of camera-trap images incl. IR night frames,
    so it should out-perform COCO YOLO on this domain.
    """
    def __init__(self, version: str = "MDV6-yolov9-c",
                 name: str | None = None):
        import os
        os.environ.setdefault("SSL_CERT_FILE",
                              __import__("certifi").where())
        from PytorchWildlife.models import detection as pw_detection
        # device='mps' is accepted but the base class fallback to cpu;
        # we pass mps anyway in case a newer version respects it.
        self.model = pw_detection.MegaDetectorV6(device="mps",
                                                  version=version)
        self.name = name or version

    def detect(self, bgr):
        res = self.model.single_image_detection(bgr,
                                                 det_conf_thres=CAT_CONF)
        det = res["detections"]
        out = []
        for box, conf, cls in zip(det.xyxy, det.confidence, det.class_id):
            if int(cls) != 0:  # 0 = animal
                continue
            x1, y1, x2, y2 = box.tolist()
            out.append((float(conf), int(x1), int(y1),
                        int(x2 - x1), int(y2 - y1)))
        return out


VARIANT_BUILDERS = {
    "nano_ncnn":      lambda: NcnnNano(),
    "yolo11n_640":    lambda: UltralyticsModel("yolo11n.pt", 640),
    "yolo11s_640":    lambda: UltralyticsModel("yolo11s.pt", 640),
    "yolo11m_640":    lambda: UltralyticsModel("yolo11m.pt", 640),
    "yolo11l_640":    lambda: UltralyticsModel("yolo11l.pt", 640),
    "yolo11x_640":    lambda: UltralyticsModel("yolo11x.pt", 640),
    "yolo11x_960":    lambda: UltralyticsModel("yolo11x.pt", 960,
                                                name="yolo11x_960"),
    "mdv6_yolov9c":   lambda: MegaDetector("MDV6-yolov9-c",
                                            name="MDv6_yolov9c_1280"),
    "mdv6_yolov10c":  lambda: MegaDetector("MDV6-yolov10-c",
                                            name="MDv6_yolov10c_1280"),
    "mdv6_rtdetrc":   lambda: MegaDetector("MDV6-rtdetr-c",
                                            name="MDv6_rtdetrc_1280"),
}

DEFAULT_VARIANTS = ["nano_ncnn", "yolo11m_640", "yolo11x_960",
                    "mdv6_yolov9c", "mdv6_yolov10c"]


def load_sample(max_bursts: int | None) -> list[dict]:
    rows = list(csv.DictReader(open(SAMPLE_CSV)))
    if max_bursts:
        rows = rows[:max_bursts]
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--variants", default=",".join(DEFAULT_VARIANTS),
                    help=f"Comma-separated variant names. "
                         f"Choices: {','.join(VARIANT_BUILDERS)}")
    ap.add_argument("--max-bursts", type=int, default=0)
    ap.add_argument("--rotate", action="store_true",
                    help="Rotate frames 90° CCW before detection "
                         "(matches firmware preprocessing; cat ends up upright)")
    ap.add_argument("--crop-top", type=int, default=0,
                    help="Crop this many pixels off the top of the (rotated) "
                         "frame, where the bright door occlusion sits")
    ap.add_argument("--tag", default="",
                    help="Suffix to append to output files (e.g. 'rot')")
    args = ap.parse_args()

    OUT.mkdir(parents=True, exist_ok=True)
    variant_names = [v.strip() for v in args.variants.split(",") if v.strip()]
    for v in variant_names:
        if v not in VARIANT_BUILDERS:
            sys.exit(f"Unknown variant '{v}'. "
                     f"Choices: {list(VARIANT_BUILDERS)}")

    sample = load_sample(args.max_bursts or None)
    log.info("Sample: %d bursts from %s", len(sample), SAMPLE_CSV.name)

    # Load night flag per (burst, fidx) from manifest
    night = {(r["burst_id"], int(r["frame_idx"])): r["night"] == "1"
             for r in csv.DictReader(open(MANIFEST_CSV))}

    # Pre-load all images once (so we measure model time only)
    log.info("Pre-loading frames into memory (rotate=%s, crop_top=%d)...",
             args.rotate, args.crop_top)
    cache: list[tuple[str, int, str, str, bool, np.ndarray]] = []
    for r in sample:
        bdir = SD / r["burst_id"]
        if not bdir.exists():
            continue
        for jp in sorted(bdir.glob("f*.jpg")):
            fidx = int(jp.stem.split("_")[0][1:])
            img = cv2.imread(str(jp))
            if img is None:
                continue
            if args.rotate:
                img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
            if args.crop_top > 0:
                img = img[args.crop_top:, :, :]
            cache.append((r["burst_id"], fidx, r["human_subject"],
                          r["human_prey"],
                          night.get((r["burst_id"], fidx), False), img))
    log.info("Cached %d frames (shape after preprocessing: %s)",
             len(cache), cache[0][5].shape if cache else "-")

    # Run each variant
    all_results: dict[str, list[dict]] = {}
    timings: dict[str, list[float]] = {}
    for vname in variant_names:
        log.info("─── Building %s ───", vname)
        t = time.time()
        try:
            model = VARIANT_BUILDERS[vname]()
        except Exception as e:
            log.error("Could not build %s: %s", vname, e)
            continue
        log.info("Loaded in %.1fs. Running on %d frames...",
                 time.time() - t, len(cache))

        rows: list[dict] = []
        ms_list: list[float] = []
        t0 = time.time()
        for i, (bid, fidx, subj, prey, is_night, img) in enumerate(cache):
            ts = time.perf_counter()
            dets = model.detect(img)
            ms = (time.perf_counter() - ts) * 1000
            ms_list.append(ms)

            best_conf = max((d[0] for d in dets), default=0.0)
            h, w = img.shape[:2]
            best_box = max(dets, key=lambda d: d[3] * d[4], default=None)
            box_frac = (best_box[3] * best_box[4] / (w * h)
                        if best_box else 0.0)

            rows.append({
                "variant": model.name,
                "burst_id": bid,
                "frame_idx": fidx,
                "night": int(is_night),
                "human_subject": subj,
                "human_prey": prey,
                "n_cats": len(dets),
                "max_conf": round(best_conf, 3),
                "box_frac": round(box_frac, 4),
            })
            if (i + 1) % 200 == 0:
                rate = (i + 1) / (time.time() - t0)
                eta = (len(cache) - i - 1) / rate
                log.info("  %s: %d/%d (%.0f/s, ETA %.0fs)",
                         model.name, i + 1, len(cache), rate, eta)
        log.info("  %s done: %.1fs total, mean %.1f ms/frame",
                 model.name, time.time() - t0, np.mean(ms_list))
        all_results[model.name] = rows
        timings[model.name] = ms_list

    # ── Write per-frame CSV (long format) ─────────────────────────────
    suffix = f"_{args.tag}" if args.tag else ""
    all_rows = [r for rows in all_results.values() for r in rows]
    with open(OUT / f"per_frame{suffix}.csv", "w", newline="") as f:
        if all_rows:
            w = csv.DictWriter(f, fieldnames=list(all_rows[0].keys()))
            w.writeheader()
            w.writerows(all_rows)

    # ── Aggregate comparison ──────────────────────────────────────────
    lines: list[str] = []
    def out(s=""):
        lines.append(s)
        print(s)

    out("=" * 92)
    out(f"YOLO variant comparison  ({len(sample)} bursts, "
        f"{len(cache)} frames, conf≥{CAT_CONF})")
    out("=" * 92)
    out()

    out(f"{'variant':22s}  {'ms/frame':>9s}  "
        f"{'cat_frame%':>11s}  {'cat_burst%':>11s}  "
        f"{'cat_night%':>11s}  {'cat_f5-9%':>11s}  "
        f"{'mean_conf':>9s}  {'mean_box%':>9s}  "
        f"{'empty_FP%':>10s}")
    for vname, rows in all_results.items():
        ms = np.mean(timings[vname])
        cat_rows = [r for r in rows if r["human_subject"] == "cat"]
        empty_rows = [r for r in rows if r["human_subject"] == "empty"]
        cat_night = [r for r in cat_rows if r["night"] == 1]
        cat_late = [r for r in cat_rows if r["frame_idx"] >= 5]

        # Per-burst (any cat detection in any frame)
        bursts_with_cat = defaultdict(int)
        bursts_total = set()
        for r in cat_rows:
            bursts_total.add(r["burst_id"])
            if r["n_cats"]:
                bursts_with_cat[r["burst_id"]] += 1
        burst_hit = sum(1 for b in bursts_total if bursts_with_cat[b] > 0)

        frame_pct = (100 * sum(1 for r in cat_rows if r["n_cats"]) /
                     max(len(cat_rows), 1))
        burst_pct = 100 * burst_hit / max(len(bursts_total), 1)
        night_pct = (100 * sum(1 for r in cat_night if r["n_cats"]) /
                     max(len(cat_night), 1))
        late_pct = (100 * sum(1 for r in cat_late if r["n_cats"]) /
                    max(len(cat_late), 1))
        cat_with = [r for r in cat_rows if r["n_cats"]]
        mean_conf = (np.mean([r["max_conf"] for r in cat_with])
                     if cat_with else 0.0)
        mean_box = (np.mean([r["box_frac"] for r in cat_with]) * 100
                    if cat_with else 0.0)
        empty_fp = (100 * sum(1 for r in empty_rows if r["n_cats"]) /
                    max(len(empty_rows), 1))

        out(f"{vname:22s}  {ms:>8.1f}   "
            f"{frame_pct:>10.1f}%  {burst_pct:>10.1f}%  "
            f"{night_pct:>10.1f}%  {late_pct:>10.1f}%  "
            f"{mean_conf:>9.3f}  {mean_box:>8.1f}%  "
            f"{empty_fp:>9.1f}%")
    out()

    # Per-frame-index for the best variant
    if all_results:
        out("Per-frame-idx detection rate (CAT bursts only):")
        out(f"  {'variant':22s}  "
            + "  ".join(f"f{i}" for i in range(10)))
        for vname, rows in all_results.items():
            cat_rows = [r for r in rows if r["human_subject"] == "cat"]
            by_idx = defaultdict(list)
            for r in cat_rows:
                by_idx[r["frame_idx"]].append(r)
            cells = []
            for i in range(10):
                rs = by_idx.get(i, [])
                if not rs:
                    cells.append("  - ")
                    continue
                pct = 100 * sum(1 for r in rs if r["n_cats"]) / len(rs)
                cells.append(f"{pct:>3.0f}%")
            out(f"  {vname:22s}  " + "  ".join(cells))
    out()

    (OUT / f"comparison{suffix}.txt").write_text("\n".join(lines) + "\n")
    out(f"Wrote {OUT}/comparison{suffix}.txt and {OUT}/per_frame{suffix}.csv")


if __name__ == "__main__":
    main()
