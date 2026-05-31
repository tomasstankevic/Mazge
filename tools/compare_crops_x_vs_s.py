#!/usr/bin/env python3
"""Compare prey_v3 burst-level predictions when fed crops from yolo11x vs yolo11s.

Uses the same prey_v3/bodyA checkpoint (trained on yolo11x crops). Scores each
burst with the best rule from metric_sweep:
    mode=topk_mean, topk=3, w_prey=0.8, w_cat=0.0, w_count=0.0, threshold=0.3

Outputs:
  - aggregate confusion matrix for each detector
  - per-burst CSV with both predictions, ground truth, split
  - lists of bursts where predictions disagree (and why)
  - frame-coverage gap (which (burst, frame) yolo11s missed vs yolo11x)
"""
from __future__ import annotations

import csv
import json
import os
import sys
from collections import defaultdict
from pathlib import Path

os.environ.setdefault("SSL_CERT_FILE", __import__("certifi").where())

import torch
from PIL import Image
from torchvision import transforms

REPO = Path(__file__).resolve().parent.parent
DATASET = REPO / "dataset"
RUN_DIR = REPO / "models" / "prey_v3" / "bodyA"
CKPT = RUN_DIR / "best_burst_f1.pt"
OUT_DIR = RUN_DIR / "crops_x_vs_s"

# Best rule from metric_sweep summary.json
MODE = "topk_mean"
TOPK = 3
W_PREY = 0.8
W_CAT = 0.0
W_COUNT = 0.0
PREY_COUNT_THR = 0.3
THRESHOLD = 0.3
IMG_SIZE = 224

CROPS_X = DATASET / "crops_yolo11x_rotcrop"
CROPS_S = DATASET / "crops_yolo11s_rotcrop"


def load_model(device):
    sys.path.insert(0, str(REPO / "tools"))
    from train_prey_v3 import PreyV3Classifier
    model = PreyV3Classifier().to(device)
    ckpt = torch.load(CKPT, map_location=device, weights_only=False)
    model.load_state_dict(ckpt["model"])
    model.eval()
    return model


@torch.no_grad()
def predict_burst(model, device, tf, frame_rows):
    if not frame_rows:
        return []
    imgs = torch.stack([
        tf(Image.open(DATASET / r["body_path"]).convert("RGB"))
        for r in frame_rows
    ]).to(device)
    fidx = torch.tensor([int(r["frame_idx"]) for r in frame_rows],
                        dtype=torch.long, device=device)
    prey_logits, cat_logits = model(imgs, fidx)
    prey_probs = torch.sigmoid(prey_logits).cpu().numpy()
    cat_probs = torch.softmax(cat_logits, dim=1).cpu().numpy()
    return [
        {
            "frame_idx": int(r["frame_idx"]),
            "prey_prob": float(pp),
            "p_mazge": float(cp[0]),
        }
        for r, pp, cp in zip(frame_rows, prey_probs, cat_probs)
    ]


def burst_score(frames):
    if not frames:
        return 0.0, 0
    scores = [W_PREY * f["prey_prob"] + W_CAT * f["p_mazge"] for f in frames]
    k = min(TOPK, len(scores))
    agg = sum(sorted(scores, reverse=True)[:k]) / k
    prey_count = sum(1 for f in frames if f["prey_prob"] >= PREY_COUNT_THR)
    norm = prey_count / max(len(frames), 1)
    return agg + W_COUNT * norm, prey_count


def load_crop_index(path: Path):
    """Return {burst_id: [rows with non-empty body_path sorted by frame_idx]}."""
    by_burst = defaultdict(list)
    dedup = {}
    for r in csv.DictReader(open(path / "_index.csv")):
        dedup[(r["burst_id"], r["frame_idx"], r["image_id"])] = r
    for r in dedup.values():
        if r.get("body_path"):
            by_burst[r["burst_id"]].append(r)
    for b in by_burst.values():
        b.sort(key=lambda x: int(x["frame_idx"]))
    return by_burst


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    if torch.backends.mps.is_available():
        device = torch.device("mps")
    elif torch.cuda.is_available():
        device = torch.device("cuda")
    else:
        device = torch.device("cpu")

    model = load_model(device)
    tf = transforms.Compose([
        transforms.Resize((IMG_SIZE, IMG_SIZE)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],
                             std=[0.229, 0.224, 0.225]),
    ])

    bursts_meta = {r["burst_id"]: r
                   for r in csv.DictReader(open(DATASET / "bursts.csv"))}
    split = json.loads((RUN_DIR / "split_assignment.json").read_text())

    by_x = load_crop_index(CROPS_X)
    by_s = load_crop_index(CROPS_S)

    common = sorted(set(by_x) & set(by_s))
    print(f"Bursts in both crop sets: {len(common)} "
          f"(x={len(by_x)}, s={len(by_s)})")

    rows = []
    for bid in common:
        bm = bursts_meta.get(bid)
        if not bm or bm.get("human_prey") not in ("0", "1"):
            continue
        y = 0 if bm.get("human_direction") == "exiting" else int(bm["human_prey"])
        x_frames = by_x[bid]
        s_frames = by_s[bid]
        preds_x = predict_burst(model, device, tf, x_frames)
        preds_s = predict_burst(model, device, tf, s_frames)
        sx, cx = burst_score(preds_x)
        ss, cs = burst_score(preds_s)
        rows.append({
            "burst_id": bid,
            "split": split.get(bid, ""),
            "cat_id": bm.get("cat_id", ""),
            "direction": bm.get("human_direction", ""),
            "y": y,
            "x_score": round(sx, 4),
            "x_pred": int(sx >= THRESHOLD),
            "x_nframes": len(x_frames),
            "x_prey_count": cx,
            "s_score": round(ss, 4),
            "s_pred": int(ss >= THRESHOLD),
            "s_nframes": len(s_frames),
            "s_prey_count": cs,
        })

    def cm(rows, key):
        tp = fp = fn = tn = 0
        for r in rows:
            y, p = r["y"], r[key]
            if y == 1 and p == 1: tp += 1
            elif y == 0 and p == 1: fp += 1
            elif y == 1 and p == 0: fn += 1
            else: tn += 1
        return tp, fp, fn, tn

    def pretty(tag, m):
        tp, fp, fn, tn = m
        prec = tp / max(tp + fp, 1)
        rec = tp / max(tp + fn, 1)
        f1 = 2 * prec * rec / max(prec + rec, 1e-8)
        return (f"{tag:>10s}: TP={tp:>3d} FP={fp:>3d} FN={fn:>3d} TN={tn:>3d} "
                f"prec={prec:.3f} rec={rec:.3f} f1={f1:.3f}")

    mx = cm(rows, "x_pred")
    ms = cm(rows, "s_pred")
    print()
    print(f"Evaluated {len(rows)} bursts (y=1: {sum(1 for r in rows if r['y']==1)}, "
          f"y=0: {sum(1 for r in rows if r['y']==0)})")
    print(pretty("yolo11x", mx))
    print(pretty("yolo11s", ms))

    # By split
    for sp in ("train", "val", "test"):
        sr = [r for r in rows if r["split"] == sp]
        if not sr:
            continue
        print(f"  split={sp:5s} n={len(sr):>3d}  "
              f"x:{pretty('', cm(sr, 'x_pred'))[12:]}  "
              f"s:{pretty('', cm(sr, 's_pred'))[12:]}")

    # Write per-burst CSV
    with open(OUT_DIR / "per_burst.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    # Disagreements
    flipped = [r for r in rows if r["x_pred"] != r["s_pred"]]
    print(f"\nDisagreements (x_pred != s_pred): {len(flipped)}")
    flipped.sort(key=lambda r: (r["y"], r["x_pred"], r["burst_id"]))
    for r in flipped:
        verdict = ("S-FN gained" if r["y"] == 1 and r["s_pred"] == 0
                   else "S-FP gained" if r["y"] == 0 and r["s_pred"] == 1
                   else "S-recovered TP" if r["y"] == 1 and r["s_pred"] == 1
                   else "S-fixed FP")
        print(f"  {r['burst_id']:32s} split={r['split']:5s} y={r['y']} "
              f"x={r['x_pred']}({r['x_score']:.3f}, "
              f"{r['x_prey_count']}/{r['x_nframes']}f) "
              f"s={r['s_pred']}({r['s_score']:.3f}, "
              f"{r['s_prey_count']}/{r['s_nframes']}f) "
              f"dir={r['direction']:10s} cat={r['cat_id']:8s} -> {verdict}")

    # Frame coverage gap
    gap_rows = []
    for bid in common:
        x_frames = {int(r["frame_idx"]): r for r in by_x[bid]}
        s_frames = {int(r["frame_idx"]): r for r in by_s[bid]}
        for fi in sorted(set(x_frames) | set(s_frames)):
            in_x = fi in x_frames
            in_s = fi in s_frames
            if in_x != in_s:
                gap_rows.append({
                    "burst_id": bid,
                    "frame_idx": fi,
                    "in_x": int(in_x),
                    "in_s": int(in_s),
                })
    with open(OUT_DIR / "coverage_gap.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["burst_id", "frame_idx", "in_x", "in_s"])
        w.writeheader()
        w.writerows(gap_rows)
    only_x = sum(1 for r in gap_rows if r["in_x"] and not r["in_s"])
    only_s = sum(1 for r in gap_rows if r["in_s"] and not r["in_x"])
    print(f"\nFrame coverage gap: x-only={only_x}, s-only={only_s} "
          f"(see {OUT_DIR}/coverage_gap.csv)")
    print(f"Per-burst predictions: {OUT_DIR}/per_burst.csv")


if __name__ == "__main__":
    main()
