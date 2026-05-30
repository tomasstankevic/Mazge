#!/usr/bin/env python3
"""Run prey_v2 on every labeled burst and compare against cloud API.

Compares per-burst predictions:
  * cloud_api: full_burst_label if present else fw_burst_label
  * prey_v2:   max P(prey) over the burst's frames, threshold sweep

Outputs a confusion matrix per method + a per-burst CSV showing
where each method agreed/disagreed with human ground truth.

Usage:
  cd Mazge
  uv run python tools/eval_prey_v2_vs_api.py
  uv run python tools/eval_prey_v2_vs_api.py --ckpt best_burst_f1
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
log = logging.getLogger("eval")

REPO = Path(__file__).resolve().parent.parent
DATASET = REPO / "dataset"
DEFAULT_RUN = REPO / "models" / "prey_v2" / "bodyA"
DEFAULT_CROPS = DATASET / "crops_yolo11x_rotcrop"

IMG_SIZE = 224


def load_model(ckpt_path: Path, device):
    """Load classifier from a training checkpoint."""
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


@torch.no_grad()
def predict_burst(model, device, body_paths, tf):
    """Return list of per-frame P(prey) for one burst."""
    if not body_paths:
        return []
    imgs = torch.stack([tf(Image.open(p).convert("RGB")) for p in body_paths])
    imgs = imgs.to(device)
    logits = model(imgs)
    probs = torch.sigmoid(logits).cpu().numpy()
    return [float(p) for p in probs]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", default=str(DEFAULT_RUN))
    ap.add_argument("--ckpt", default="best_burst_f1",
                    help="best_burst_f1 | best_loss | last")
    ap.add_argument("--crops", default=str(DEFAULT_CROPS))
    ap.add_argument("--device", default="auto")
    args = ap.parse_args()

    run_dir = Path(args.run_dir)
    crops_dir = Path(args.crops)
    out_dir = run_dir / "vs_cloud_api"
    out_dir.mkdir(parents=True, exist_ok=True)

    device = (torch.device("mps") if args.device == "auto"
                                       and torch.backends.mps.is_available()
              else torch.device(args.device if args.device != "auto"
                                              else "cpu"))
    log.info("Device: %s", device)

    ckpt_path = run_dir / f"{args.ckpt}.pt"
    if not ckpt_path.exists():
        raise SystemExit(f"No checkpoint at {ckpt_path}")
    model, ckpt = load_model(ckpt_path, device)
    log.info("Loaded %s (epoch %d, val_burst_f1=%.3f)",
             ckpt_path.name, ckpt["epoch"],
             ckpt["val_metrics"]["burst_f1"])

    # Load split assignment used during training so we can split metrics
    split_json = run_dir / "split_assignment.json"
    split_assign = json.loads(split_json.read_text()) if split_json.exists() else {}

    # Load crops index → frames per burst
    crop_rows = list(csv.DictReader(open(crops_dir / "_index.csv")))
    by_burst: dict[str, list[dict]] = defaultdict(list)
    for r in crop_rows:
        if r["body_path"]:
            by_burst[r["burst_id"]].append(r)

    # Load burst-level human labels and cloud API decisions
    bursts_meta = {r["burst_id"]: r
                   for r in csv.DictReader(open(DATASET / "bursts.csv"))}

    # Build the comparison set: every burst with a HARD human label
    # and at least one body crop (so prey_v2 can predict on it).
    comparison_rows: list[dict] = []
    tf = eval_tf()
    t0 = time.time()
    for bid, frames in sorted(by_burst.items()):
        bm = bursts_meta.get(bid)
        if not bm:
            continue
        if bm["human_prey"] not in ("0", "1"):
            continue  # skip unclear / empty human_prey
        # Cat exit: cat cannot carry prey out -> ground truth is 0
        effective_human = "0" if bm["human_direction"] == "exiting" \
                          else bm["human_prey"]

        body_paths = sorted(DATASET / r["body_path"] for r in frames)
        probs = predict_burst(model, device, body_paths, tf)
        max_p = max(probs) if probs else 0.0
        topk = sorted(probs, reverse=True)[:3]
        top3_mean = sum(topk) / len(topk) if topk else 0.0

        # Cloud API: prefer full_burst_label, fall back to fw_burst_label
        full = bm.get("full_burst_label", "")
        fw = bm.get("fw_burst_label", "")
        cloud = full if full in ("0", "1") else (fw if fw in ("0", "1") else "")

        comparison_rows.append({
            "burst_id": bid,
            "split_train": split_assign.get(bid, ""),
            "human_prey": bm["human_prey"],
            "human_direction": bm["human_direction"],
            "human_subject": bm["human_subject"],
            "cat_id": bm.get("cat_id", ""),
            "effective_human": effective_human,
            "cloud_api": cloud,
            "v2_max_p": round(max_p, 4),
            "v2_top3_mean": round(top3_mean, 4),
            "n_frames": len(frames),
        })

    elapsed = time.time() - t0
    log.info("Predicted on %d bursts in %.1fs (%.2f bursts/s)",
             len(comparison_rows), elapsed, len(comparison_rows) / elapsed)

    # Write per-burst comparison CSV
    csv_path = out_dir / "per_burst.csv"
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(comparison_rows[0].keys()))
        w.writeheader()
        w.writerows(comparison_rows)
    log.info("Wrote %s", csv_path)

    # ── Confusion analysis ────────────────────────────────────────────
    print()
    print("=" * 72)
    print(f"prey_v2 ({args.ckpt} @ ep {ckpt['epoch']}) vs cloud API "
          f"on {len(comparison_rows)} hard-labeled bursts")
    print("=" * 72)
    print(f"  cloud_api was called on: "
          f"{sum(1 for r in comparison_rows if r['cloud_api'] in ('0','1'))}")
    print()

    def confusion(rows, pred_fn):
        tp = fp = fn = tn = 0
        for r in rows:
            yt = r["effective_human"]
            yp = pred_fn(r)
            if yp is None:
                continue
            if yt == "1" and yp == 1:
                tp += 1
            elif yt == "1" and yp == 0:
                fn += 1
            elif yt == "0" and yp == 1:
                fp += 1
            elif yt == "0" and yp == 0:
                tn += 1
        n = tp + fp + fn + tn
        prec = tp / max(tp + fp, 1)
        rec = tp / max(tp + fn, 1)
        f1 = 2 * prec * rec / max(prec + rec, 1e-8)
        return tp, fp, fn, tn, n, prec, rec, f1

    def pred_cloud(r):
        v = r["cloud_api"]
        return None if v not in ("0", "1") else int(v)

    def pred_v2_at(threshold):
        def f(r):
            return 1 if r["v2_max_p"] >= threshold else 0
        return f

    def pred_v2_top3_at(threshold):
        def f(r):
            return 1 if r["v2_top3_mean"] >= threshold else 0
        return f

    headers = ("method", "thresh", "subset", "TP", "FP", "FN", "TN",
               "N", "P", "R", "F1")
    print(("{:<22} {:>7} {:>10} " + "{:>4} " * 5 +
           " {:>5} {:>5} {:>5}").format(*headers))
    print("-" * 92)

    subsets = {
        "ALL": comparison_rows,
        "train": [r for r in comparison_rows if r["split_train"] == "train"],
        "val":   [r for r in comparison_rows if r["split_train"] == "val"],
        "test":  [r for r in comparison_rows if r["split_train"] == "test"],
    }

    rows_table = []
    for subset_name, subset in subsets.items():
        if not subset:
            continue
        # Cloud API
        tp, fp, fn, tn, n, p, rec, f1 = confusion(subset, pred_cloud)
        print(("{:<22} {:>7} {:>10} " + "{:>4} " * 5 +
               " {:>5.2f} {:>5.2f} {:>5.2f}").format(
                "cloud_api", "(--)", subset_name,
                tp, fp, fn, tn, n, p, rec, f1))
        rows_table.append({"method": "cloud_api", "threshold": "",
                           "subset": subset_name,
                           "tp": tp, "fp": fp, "fn": fn, "tn": tn,
                           "n": n, "precision": p, "recall": rec, "f1": f1})

        # v2 at multiple thresholds
        for thr in (0.30, 0.50, 0.70, 0.80, 0.90):
            tp, fp, fn, tn, n, p, rec, f1 = confusion(subset, pred_v2_at(thr))
            print(("{:<22} {:>7.2f} {:>10} " + "{:>4} " * 5 +
                   " {:>5.2f} {:>5.2f} {:>5.2f}").format(
                    "v2_max", thr, subset_name,
                    tp, fp, fn, tn, n, p, rec, f1))
            rows_table.append({"method": "v2_max", "threshold": thr,
                               "subset": subset_name,
                               "tp": tp, "fp": fp, "fn": fn, "tn": tn,
                               "n": n, "precision": p, "recall": rec, "f1": f1})

        for thr in (0.30, 0.50, 0.70, 0.80, 0.90):
            tp, fp, fn, tn, n, p, rec, f1 = confusion(subset, pred_v2_top3_at(thr))
            print(("{:<22} {:>7.2f} {:>10} " + "{:>4} " * 5 +
                   " {:>5.2f} {:>5.2f} {:>5.2f}").format(
                    "v2_top3mean", thr, subset_name,
                    tp, fp, fn, tn, n, p, rec, f1))
            rows_table.append({"method": "v2_top3mean", "threshold": thr,
                               "subset": subset_name,
                               "tp": tp, "fp": fp, "fn": fn, "tn": tn,
                               "n": n, "precision": p, "recall": rec, "f1": f1})
        print()

    # Save confusion table
    with open(out_dir / "confusion.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows_table[0].keys()))
        w.writeheader()
        w.writerows(rows_table)

    # Per-cat-id breakdown at the chosen threshold (0.5 on max)
    print("Per-cat breakdown (v2_max @ 0.5):")
    by_cat = defaultdict(list)
    for r in comparison_rows:
        by_cat[r["cat_id"] or "?"].append(r)
    print(("{:<10} " + "{:>4} " * 5 +
           " {:>5} {:>5} {:>5}").format(
            "cat", "TP", "FP", "FN", "TN", "N", "P", "R", "F1"))
    for cat in sorted(by_cat):
        tp, fp, fn, tn, n, p, rec, f1 = confusion(
            by_cat[cat], pred_v2_at(0.5))
        print(("{:<10} " + "{:>4} " * 5 +
               " {:>5.2f} {:>5.2f} {:>5.2f}").format(
                cat, tp, fp, fn, tn, n, p, rec, f1))
    print()

    # Disagreements: bursts where prey_v2 and cloud_api give different
    # answers (most useful for visual review).
    disagreements = []
    for r in comparison_rows:
        if r["cloud_api"] not in ("0", "1"):
            continue
        api = int(r["cloud_api"])
        v2 = 1 if r["v2_max_p"] >= 0.5 else 0
        if api != v2:
            r2 = dict(r)
            r2["api"] = api
            r2["v2"] = v2
            r2["who_is_right"] = (
                "api" if int(r["effective_human"]) == api else "v2"
            )
            disagreements.append(r2)
    print(f"Disagreements (cloud_api vs v2_max@0.5): {len(disagreements)}")
    print(f"  v2 correct, api wrong: "
          f"{sum(1 for r in disagreements if r['who_is_right']=='v2')}")
    print(f"  api correct, v2 wrong: "
          f"{sum(1 for r in disagreements if r['who_is_right']=='api')}")
    with open(out_dir / "disagreements.csv", "w", newline="") as f:
        if disagreements:
            w = csv.DictWriter(f, fieldnames=list(disagreements[0].keys()))
            w.writeheader()
            w.writerows(disagreements)

    print()
    print(f"Wrote: {out_dir}/per_burst.csv")
    print(f"       {out_dir}/confusion.csv")
    print(f"       {out_dir}/disagreements.csv ({len(disagreements)} rows)")


if __name__ == "__main__":
    main()
