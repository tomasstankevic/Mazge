#!/usr/bin/env python3
"""Compare two prey_v3 models, each with its own (detector, crops) pipeline.

Default config:
  X: model=bodyA   (trained on yolo11x crops), crops=crops_yolo11x_rotcrop
  S: model=bodyA_s (trained on yolo11s crops), crops=crops_yolo11s_rotcrop

Each side uses its own best-rule from metric_sweep summary.json.
"""
from __future__ import annotations
import csv
import json
import os
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("SSL_CERT_FILE", __import__("certifi").where())

import torch
from PIL import Image
from torchvision import transforms

REPO = Path(__file__).resolve().parent.parent
DATASET = REPO / "dataset"
IMG_SIZE = 224

X_RUN = REPO / "models" / "prey_v3" / "bodyA"
S_RUN = REPO / "models" / "prey_v3" / "bodyA_s"
X_CROPS = DATASET / "crops_yolo11x_rotcrop"
S_CROPS = DATASET / "crops_yolo11s_rotcrop"

OUT_DIR = REPO / "models" / "prey_v3" / "x_vs_s_full"
OUT_DIR.mkdir(parents=True, exist_ok=True)


@dataclass
class Rule:
    mode: str
    topk: int
    w_prey: float
    w_cat: float
    w_count: float
    prey_count_threshold: float
    threshold: float

    @classmethod
    def from_sweep(cls, summary_path: Path) -> "Rule":
        s = json.loads(summary_path.read_text())["best_combined"]
        return cls(
            mode=s["mode"], topk=s["topk"], w_prey=s["w_prey"],
            w_cat=s["w_cat"], w_count=s["w_count"],
            prey_count_threshold=s["prey_count_threshold"],
            threshold=s["threshold"],
        )


def load_model(ckpt_path: Path, device):
    sys.path.insert(0, str(REPO / "tools"))
    from train_prey_v3 import PreyV3Classifier
    model = PreyV3Classifier().to(device)
    ckpt = torch.load(ckpt_path, map_location=device, weights_only=False)
    model.load_state_dict(ckpt["model"])
    model.eval()
    return model


@torch.no_grad()
def predict_burst(model, device, tf, rows):
    if not rows:
        return []
    imgs = torch.stack([
        tf(Image.open(DATASET / r["body_path"]).convert("RGB")) for r in rows
    ]).to(device)
    fidx = torch.tensor([int(r["frame_idx"]) for r in rows],
                        dtype=torch.long, device=device)
    prey_logits, cat_logits = model(imgs, fidx)
    pp = torch.sigmoid(prey_logits).cpu().numpy()
    cp = torch.softmax(cat_logits, dim=1).cpu().numpy()
    return [
        {"frame_idx": int(r["frame_idx"]),
         "prey_prob": float(p), "p_mazge": float(c[0])}
        for r, p, c in zip(rows, pp, cp)
    ]


def burst_score(frames, rule: Rule):
    if not frames:
        return 0.0, 0
    scores = [rule.w_prey * f["prey_prob"] + rule.w_cat * f["p_mazge"]
              for f in frames]
    if rule.mode == "max":
        agg = max(scores)
    else:
        k = min(rule.topk, len(scores))
        agg = sum(sorted(scores, reverse=True)[:k]) / k
    prey_count = sum(1 for f in frames
                     if f["prey_prob"] >= rule.prey_count_threshold)
    norm = prey_count / max(len(frames), 1)
    return agg + rule.w_count * norm, prey_count


def load_crop_index(path: Path):
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
    return (f"{tag:>20s}: TP={tp:>3d} FP={fp:>3d} FN={fn:>3d} TN={tn:>3d} "
            f"prec={prec:.3f} rec={rec:.3f} f1={f1:.3f}")


def main():
    device = (torch.device("mps") if torch.backends.mps.is_available()
              else torch.device("cuda") if torch.cuda.is_available()
              else torch.device("cpu"))
    print(f"Device: {device}")

    rule_x = Rule.from_sweep(X_RUN / "metric_sweep" / "summary.json")
    rule_s = Rule.from_sweep(S_RUN / "metric_sweep" / "summary.json")
    print(f"Rule X: {rule_x}")
    print(f"Rule S: {rule_s}")

    model_x = load_model(X_RUN / "best_burst_f1.pt", device)
    model_s = load_model(S_RUN / "best_burst_f1.pt", device)

    tf = transforms.Compose([
        transforms.Resize((IMG_SIZE, IMG_SIZE)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],
                             std=[0.229, 0.224, 0.225]),
    ])

    bursts_meta = {r["burst_id"]: r
                   for r in csv.DictReader(open(DATASET / "bursts.csv"))}
    # Use S split for X bursts too (split is per-burst, identical mapping)
    split_x = json.loads((X_RUN / "split_assignment.json").read_text())
    split_s = json.loads((S_RUN / "split_assignment.json").read_text())

    by_x = load_crop_index(X_CROPS)
    by_s = load_crop_index(S_CROPS)
    common = sorted(set(by_x) & set(by_s))
    print(f"Bursts in both crop sets: {len(common)} "
          f"(x={len(by_x)}, s={len(by_s)})")

    rows = []
    for bid in common:
        bm = bursts_meta.get(bid)
        if not bm or bm.get("human_prey") not in ("0", "1"):
            continue
        y = 0 if bm.get("human_direction") == "exiting" else int(bm["human_prey"])
        px = predict_burst(model_x, device, tf, by_x[bid])
        ps = predict_burst(model_s, device, tf, by_s[bid])
        sx, cx = burst_score(px, rule_x)
        ss, cs = burst_score(ps, rule_s)
        rows.append({
            "burst_id": bid,
            "split_x": split_x.get(bid, ""),
            "split_s": split_s.get(bid, ""),
            "cat_id": bm.get("cat_id", ""),
            "direction": bm.get("human_direction", ""),
            "y": y,
            "x_score": round(sx, 4), "x_pred": int(sx >= rule_x.threshold),
            "x_nframes": len(by_x[bid]), "x_prey_count": cx,
            "s_score": round(ss, 4), "s_pred": int(ss >= rule_s.threshold),
            "s_nframes": len(by_s[bid]), "s_prey_count": cs,
        })

    nrows = len(rows)
    npos = sum(1 for r in rows if r["y"] == 1)
    print(f"\nEvaluated {nrows} bursts (y=1: {npos}, y=0: {nrows - npos})")
    mx = cm(rows, "x_pred")
    ms = cm(rows, "s_pred")
    print(pretty("yolo11x+bodyA", mx))
    print(pretty("yolo11s+bodyA_s", ms))

    for sp in ("train", "val", "test"):
        # Use S split (matches each model's own split since same algorithm/seed)
        sr_s = [r for r in rows if r["split_s"] == sp]
        sr_x = [r for r in rows if r["split_x"] == sp]
        if sr_s:
            print(f"  split_s={sp:5s} n={len(sr_s):>3d}  "
                  f"s:{pretty('', cm(sr_s, 's_pred'))[22:]}")
        if sr_x:
            print(f"  split_x={sp:5s} n={len(sr_x):>3d}  "
                  f"x:{pretty('', cm(sr_x, 'x_pred'))[22:]}")

    with open(OUT_DIR / "per_burst.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    flipped = [r for r in rows if r["x_pred"] != r["s_pred"]]
    print(f"\nDisagreements (x_pred != s_pred): {len(flipped)}")
    flipped.sort(key=lambda r: (r["y"], r["x_pred"], r["burst_id"]))
    for r in flipped:
        verdict = ("S-FN gained" if r["y"] == 1 and r["s_pred"] == 0
                   else "S-FP gained" if r["y"] == 0 and r["s_pred"] == 1
                   else "S-recovered TP" if r["y"] == 1 and r["s_pred"] == 1
                   else "S-fixed FP")
        print(f"  {r['burst_id']:32s} sp_x={r['split_x']:5s} sp_s={r['split_s']:5s} "
              f"y={r['y']} "
              f"x={r['x_pred']}({r['x_score']:.3f}, "
              f"{r['x_prey_count']}/{r['x_nframes']}f) "
              f"s={r['s_pred']}({r['s_score']:.3f}, "
              f"{r['s_prey_count']}/{r['s_nframes']}f) "
              f"dir={r['direction']:10s} cat={r['cat_id']:8s} -> {verdict}")

    print(f"\nPer-burst predictions: {OUT_DIR}/per_burst.csv")


if __name__ == "__main__":
    main()
