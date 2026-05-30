#!/usr/bin/env python3
"""Evaluate prey_v2 at per-image (frame) level.

This complements burst-level metrics by scoring every frame in the crop index.
Labels are inherited from human burst labels using the same effective-prey rule:
  effective_prey = 0 if human_direction == exiting else human_prey

Outputs in models/prey_v2/<run>/vs_cloud_api_frames/:
  - per_frame.csv        one row per frame
  - confusion.csv        threshold sweep (overall + grouped subsets)
  - hard_fp.csv          highest-confidence false-positive frames
  - hard_fn.csv          lowest-confidence false-negative frames

Usage:
  cd Mazge
  uv run python tools/eval_prey_v2_per_frame.py
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

os.environ.setdefault("SSL_CERT_FILE", __import__("certifi").where())

import torch
from PIL import Image
from torchvision import transforms

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s %(levelname)-5s %(message)s",
                    datefmt="%H:%M:%S")
log = logging.getLogger("eval-frame")

REPO = Path(__file__).resolve().parent.parent
DATASET = REPO / "dataset"
DEFAULT_RUN = REPO / "models" / "prey_v2" / "bodyA"
DEFAULT_CROPS = DATASET / "crops_yolo11x_rotcrop"
IMG_SIZE = 224


def load_model(ckpt_path: Path, device):
    import sys
    sys.path.insert(0, str(REPO / "tools"))
    from train_prey_v2 import PreyClassifier
    model = PreyClassifier().to(device)
    ckpt = torch.load(ckpt_path, map_location=device, weights_only=False)
    model.load_state_dict(ckpt["model"])
    model.eval()
    return model, ckpt


def eval_tf():
    return transforms.Compose([
        transforms.Resize((IMG_SIZE, IMG_SIZE)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],
                             std=[0.229, 0.224, 0.225]),
    ])


def score_frame(model, device, tf, body_path: Path) -> float:
    img = tf(Image.open(body_path).convert("RGB")).unsqueeze(0).to(device)
    with torch.no_grad():
        return float(torch.sigmoid(model(img)).item())


def confusion(rows: list[dict], threshold: float, score_key: str = "v2_frame_p"):
    tp = fp = fn = tn = 0
    for r in rows:
        yt = int(r["effective_human"])
        yp = 1 if float(r[score_key]) >= threshold else 0
        if yt == 1 and yp == 1:
            tp += 1
        elif yt == 1 and yp == 0:
            fn += 1
        elif yt == 0 and yp == 1:
            fp += 1
        else:
            tn += 1
    n = tp + fp + fn + tn
    p = tp / max(tp + fp, 1)
    r = tp / max(tp + fn, 1)
    f1 = 2 * p * r / max(p + r, 1e-8)
    return tp, fp, fn, tn, n, p, r, f1


def daypart(burst_id: str) -> str:
    try:
        hh = int(burst_id.split("_")[1][:2])
    except Exception:
        return "unknown"
    return "day" if 7 <= hh < 19 else "night"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", default=str(DEFAULT_RUN))
    ap.add_argument("--ckpt", default="best_burst_f1")
    ap.add_argument("--crops", default=str(DEFAULT_CROPS))
    ap.add_argument("--device", default="auto")
    ap.add_argument("--score-missing-as-zero", action="store_true", default=True,
                    help="Frames without body crop get score=0.0 (default on)")
    args = ap.parse_args()

    run_dir = Path(args.run_dir)
    crops_dir = Path(args.crops)
    out_dir = run_dir / "vs_cloud_api_frames"
    out_dir.mkdir(parents=True, exist_ok=True)

    device = (torch.device("mps") if args.device == "auto"
                                       and torch.backends.mps.is_available()
              else torch.device(args.device if args.device != "auto"
                                              else "cpu"))
    ckpt_path = run_dir / f"{args.ckpt}.pt"
    if not ckpt_path.exists():
        raise SystemExit(f"No checkpoint at {ckpt_path}")
    model, ckpt = load_model(ckpt_path, device)
    tf = eval_tf()

    split_json = run_dir / "split_assignment.json"
    split_assign = json.loads(split_json.read_text()) if split_json.exists() else {}

    bursts_meta = {r["burst_id"]: r
                   for r in csv.DictReader(open(DATASET / "bursts.csv"))}

    crop_rows = list(csv.DictReader(open(crops_dir / "_index.csv")))
    per_frame = []

    t0 = time.time()
    for r in crop_rows:
        bid = r["burst_id"]
        bm = bursts_meta.get(bid)
        if not bm:
            continue
        if bm["human_prey"] not in ("0", "1"):
            continue

        effective_human = "0" if bm["human_direction"] == "exiting" else bm["human_prey"]
        body_rel = r.get("body_path", "")
        has_body = bool(body_rel)
        if has_body:
            score = score_frame(model, device, tf, DATASET / body_rel)
        else:
            score = 0.0 if args.score_missing_as_zero else float("nan")

        per_frame.append({
            "burst_id": bid,
            "frame_idx": int(r["frame_idx"]),
            "image_id": r.get("image_id", ""),
            "body_path": body_rel,
            "has_body": int(has_body),
            "cat_found": int(r.get("cat_found", "0") or 0),
            "cat_conf": float(r.get("cat_conf", "0") or 0.0),
            "split_train": split_assign.get(bid, ""),
            "human_subject": bm.get("human_subject", ""),
            "human_direction": bm.get("human_direction", ""),
            "cat_id": bm.get("cat_id", "") or "?",
            "effective_human": effective_human,
            "v2_frame_p": round(score, 6),
            "daypart": daypart(bid),
        })

    elapsed = time.time() - t0
    log.info("Scored %d frames in %.1fs", len(per_frame), elapsed)

    per_frame_path = out_dir / "per_frame.csv"
    with open(per_frame_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(per_frame[0].keys()))
        w.writeheader()
        w.writerows(per_frame)

    thresholds = (0.30, 0.50, 0.70, 0.80, 0.90)
    subsets = {
        "ALL": per_frame,
        "train": [r for r in per_frame if r["split_train"] == "train"],
        "val": [r for r in per_frame if r["split_train"] == "val"],
        "test": [r for r in per_frame if r["split_train"] == "test"],
        "cat_found=1": [r for r in per_frame if int(r["cat_found"]) == 1],
        "cat_found=0": [r for r in per_frame if int(r["cat_found"]) == 0],
        "day": [r for r in per_frame if r["daypart"] == "day"],
        "night": [r for r in per_frame if r["daypart"] == "night"],
    }

    rows_table = []
    print("\n" + "=" * 78)
    print(f"Per-frame eval ({args.ckpt}@ep{ckpt['epoch']}) on {len(per_frame)} frames")
    print("=" * 78)
    print(("{:<12} {:>7} {:>12} " + "{:>5} " * 5 + " {:>5} {:>5} {:>5}").format(
          "scope", "thr", "subset", "TP", "FP", "FN", "TN", "N", "P", "R", "F1"))
    print("-" * 102)

    for subset_name, subset in subsets.items():
        if not subset:
            continue
        for thr in thresholds:
            tp, fp, fn, tn, n, p, rec, f1 = confusion(subset, thr)
            print(("{:<12} {:>7.2f} {:>12} " + "{:>5} " * 5 + " {:>5.2f} {:>5.2f} {:>5.2f}").format(
                  "v2_frame", thr, subset_name,
                  tp, fp, fn, tn, n, p, rec, f1))
            rows_table.append({
                "scope": "v2_frame",
                "threshold": thr,
                "subset": subset_name,
                "tp": tp,
                "fp": fp,
                "fn": fn,
                "tn": tn,
                "n": n,
                "precision": p,
                "recall": rec,
                "f1": f1,
            })

    with open(out_dir / "confusion.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows_table[0].keys()))
        w.writeheader()
        w.writerows(rows_table)

    hard_fp = [r for r in per_frame if r["effective_human"] == "0" and r["v2_frame_p"] >= 0.5]
    hard_fp = sorted(hard_fp, key=lambda r: r["v2_frame_p"], reverse=True)
    hard_fn = [r for r in per_frame if r["effective_human"] == "1" and r["v2_frame_p"] < 0.5]
    hard_fn = sorted(hard_fn, key=lambda r: r["v2_frame_p"])

    for name, rows in (("hard_fp.csv", hard_fp), ("hard_fn.csv", hard_fn)):
        with open(out_dir / name, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(per_frame[0].keys()))
            w.writeheader()
            w.writerows(rows)

    print("\nWrote:")
    print(f"  {per_frame_path}")
    print(f"  {out_dir / 'confusion.csv'}")
    print(f"  {out_dir / 'hard_fp.csv'} ({len(hard_fp)} rows)")
    print(f"  {out_dir / 'hard_fn.csv'} ({len(hard_fn)} rows)")


if __name__ == "__main__":
    main()
