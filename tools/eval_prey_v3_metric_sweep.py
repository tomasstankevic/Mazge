#!/usr/bin/env python3
"""Evaluate prey_v3 and sweep combined burst metrics.

Combined burst score:
    score = agg_frame_score + w_count * prey_frame_count_norm

where:
    agg_frame_score is max or top-k mean of per-frame
        (w_prey * prey_prob + w_cat * p_mazge)
    prey_frame_count_norm is the fraction of frames in the burst whose
        prey_prob >= prey_count_threshold.
"""
from __future__ import annotations

import argparse
import csv
import itertools
import json
import logging
import os
from collections import defaultdict
from pathlib import Path

os.environ.setdefault("SSL_CERT_FILE", __import__("certifi").where())

import torch
from PIL import Image
from torchvision import transforms

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s %(levelname)-5s %(message)s",
                    datefmt="%H:%M:%S")
log = logging.getLogger("eval_v3")

REPO = Path(__file__).resolve().parent.parent
DATASET = REPO / "dataset"
DEFAULT_RUN = REPO / "models" / "prey_v3" / "bodyA"
DEFAULT_CROPS = DATASET / "crops_yolo11x_rotcrop"
IMG_SIZE = 224


def load_model(ckpt_path: Path, device):
    import sys
    sys.path.insert(0, str(REPO / "tools"))
    from train_prey_v3 import PreyV3Classifier

    model = PreyV3Classifier().to(device)
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
def predict_burst(model, device, frames, tf):
    if not frames:
        return []
    imgs = torch.stack([tf(Image.open(DATASET / r["body_path"]).convert("RGB"))
                        for r in frames]).to(device)
    frame_idx = torch.tensor([int(r["frame_idx"]) for r in frames],
                             dtype=torch.long, device=device)
    prey_logits, cat_logits = model(imgs, frame_idx)
    prey_probs = torch.sigmoid(prey_logits).cpu().numpy()
    cat_probs = torch.softmax(cat_logits, dim=1).cpu().numpy()
    out = []
    for r, prey_p, cp in zip(frames, prey_probs, cat_probs):
        out.append({
            "frame_idx": int(r["frame_idx"]),
            "prey_prob": float(prey_p),
            "p_mazge": float(cp[0]),
            "p_benis": float(cp[1]),
        })
    return out


def confusion(rows, pred_key="pred"):
    tp = fp = fn = tn = 0
    for r in rows:
        y = int(r["effective_human"])
        p = int(r[pred_key])
        if p == 1 and y == 1:
            tp += 1
        elif p == 1 and y == 0:
            fp += 1
        elif p == 0 and y == 1:
            fn += 1
        else:
            tn += 1
    n = tp + fp + fn + tn
    prec = tp / max(tp + fp, 1)
    rec = tp / max(tp + fn, 1)
    f1 = 2 * prec * rec / max(prec + rec, 1e-8)
    return {
        "tp": tp, "fp": fp, "fn": fn, "tn": tn, "n": n,
        "precision": prec, "recall": rec, "f1": f1,
    }


def frame_score(frame, w_prey: float, w_cat: float) -> float:
    return w_prey * frame["prey_prob"] + w_cat * frame["p_mazge"]


def burst_score(frames, mode: str, topk: int,
                w_prey: float, w_cat: float,
                w_count: float, prey_count_threshold: float) -> float:
    scores = [frame_score(f, w_prey, w_cat) for f in frames]
    if not scores:
        return 0.0
    if mode == "max":
        agg = max(scores)
    elif mode == "topk_mean":
        k = min(topk, len(scores))
        top = sorted(scores, reverse=True)[:k]
        agg = sum(top) / k
    else:
        raise ValueError(mode)

    prey_count = sum(1 for f in frames if f["prey_prob"] >= prey_count_threshold)
    prey_count_norm = prey_count / max(len(frames), 1)
    return agg + w_count * prey_count_norm


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run-dir", default=str(DEFAULT_RUN))
    ap.add_argument("--ckpt", default="best_burst_f1")
    ap.add_argument("--crops", default=str(DEFAULT_CROPS))
    ap.add_argument("--device", default="auto")
    ap.add_argument("--subset", default="all", choices=["all", "train", "val", "test"])
    ap.add_argument("--fn-weight", type=float, default=1.0,
                    help="Weight applied to false negatives in optimization objective")
    ap.add_argument("--fp-weight", type=float, default=1.0,
                    help="Weight applied to false positives in optimization objective")
    args = ap.parse_args()

    run_dir = Path(args.run_dir)
    out_dir = run_dir / "metric_sweep"
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.device == "auto":
        if torch.backends.mps.is_available():
            device = torch.device("mps")
        elif torch.cuda.is_available():
            device = torch.device("cuda")
        else:
            device = torch.device("cpu")
    else:
        device = torch.device(args.device)

    ckpt_path = run_dir / f"{args.ckpt}.pt"
    model, ckpt = load_model(ckpt_path, device)
    split_assign = json.loads((run_dir / "split_assignment.json").read_text())

    bursts_meta = {r["burst_id"]: r for r in csv.DictReader(open(DATASET / "bursts.csv"))}
    by_burst = defaultdict(list)
    dedup = {}
    for r in csv.DictReader(open(Path(args.crops) / "_index.csv")):
        dedup[(r["burst_id"], r["frame_idx"], r["image_id"])] = r
    for r in dedup.values():
        if r.get("body_path"):
            by_burst[r["burst_id"]].append(r)

    tf = eval_tf()
    per_burst = []
    for bid, frames in sorted(by_burst.items()):
        bm = bursts_meta.get(bid)
        if not bm:
            continue
        if bm.get("human_prey") not in ("0", "1"):
            continue
        eff = "0" if bm.get("human_direction") == "exiting" else bm["human_prey"]
        preds = predict_burst(model, device, sorted(frames, key=lambda x: int(x["frame_idx"])), tf)
        if not preds:
            continue
        per_burst.append({
            "burst_id": bid,
            "split": split_assign.get(bid, ""),
            "effective_human": eff,
            "cat_id": bm.get("cat_id", ""),
            "frames": preds,
        })

    if args.subset != "all":
        per_burst = [r for r in per_burst if r["split"] == args.subset]

    log.info("Evaluating %d bursts on subset=%s", len(per_burst), args.subset)

    # Baseline: prey only max with threshold sweep
    baseline_rows = []
    for thr in [i / 100 for i in range(10, 100, 5)]:
        rows = []
        for r in per_burst:
            max_prey = max(f["prey_prob"] for f in r["frames"])
            rows.append({
                "effective_human": r["effective_human"],
                "pred": 1 if max_prey >= thr else 0,
            })
        cm = confusion(rows)
        baseline_rows.append({"method": "prey_max", "threshold": thr, **cm})

    # Combined metric sweep
    w_prey_vals = [0.8, 1.0, 1.2]
    w_cat_vals = [-0.3, -0.2, -0.1, 0.0, 0.1, 0.2]
    w_count_vals = [0.0, 0.1, 0.2, 0.3, 0.4]
    prey_count_thr_vals = [0.3, 0.5, 0.7]
    modes = ["max", "topk_mean"]
    topk_vals = [2, 3]
    thresh_vals = [i / 100 for i in range(10, 160, 2)]

    best = None
    all_rows = []
    for w_prey, w_cat, w_count, prey_count_thr, mode in itertools.product(
            w_prey_vals, w_cat_vals, w_count_vals, prey_count_thr_vals, modes):
        ks = topk_vals if mode == "topk_mean" else [1]
        for k in ks:
            burst_scores = []
            for r in per_burst:
                s = burst_score(r["frames"], mode, k, w_prey, w_cat,
                                w_count, prey_count_thr)
                burst_scores.append((r["effective_human"], s))
            for thr in thresh_vals:
                rows = [{"effective_human": y, "pred": 1 if s >= thr else 0}
                        for y, s in burst_scores]
                cm = confusion(rows)
                rec = {
                    "mode": mode,
                    "topk": k,
                    "w_prey": w_prey,
                    "w_cat": w_cat,
                    "w_count": w_count,
                    "prey_count_threshold": prey_count_thr,
                    "threshold": thr,
                    **cm,
                    "objective": args.fn_weight * cm["fn"] + args.fp_weight * cm["fp"],
                }
                all_rows.append(rec)
                key = (rec["objective"], cm["fn"], cm["fp"], -cm["f1"])
                if best is None or key < best[0]:
                    best = (key, rec)

    best_rec = best[1]

    # Save outputs
    with open(out_dir / "baseline_sweep.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(baseline_rows[0].keys()))
        w.writeheader()
        w.writerows(baseline_rows)

    with open(out_dir / "combined_sweep.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(all_rows[0].keys()))
        w.writeheader()
        w.writerows(all_rows)

    top10 = sorted(all_rows, key=lambda r: (r["objective"], r["fn"], r["fp"], -r["f1"]))[:10]
    with open(out_dir / "top10.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(top10[0].keys()))
        w.writeheader()
        w.writerows(top10)

    summary = {
        "subset": args.subset,
        "run_dir": str(run_dir),
        "checkpoint": args.ckpt,
        "ckpt_epoch": ckpt.get("epoch"),
        "fn_weight": args.fn_weight,
        "fp_weight": args.fp_weight,
        "n_bursts": len(per_burst),
        "best_combined": best_rec,
        "best_baseline": sorted(baseline_rows,
                                key=lambda r: (args.fn_weight * r["fn"] + args.fp_weight * r["fp"],
                                               r["fn"], r["fp"], -r["f1"]))[0],
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2))

    print("Best combined metric:")
    print(json.dumps(best_rec, indent=2))
    print("Best baseline prey_max:")
    print(json.dumps(summary["best_baseline"], indent=2))
    print(f"Saved sweep outputs to {out_dir}")


if __name__ == "__main__":
    main()
