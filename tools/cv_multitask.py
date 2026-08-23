#!/usr/bin/env python3
"""K-fold cross-validation for the multitask model — robust signal vs noise.

The single hash split has only ~4 prey / ~30 cat bursts in test, so per-run
metrics are dominated by noise. This runs stratified K-fold CV over bursts:
every burst is held out exactly once, and we POOL the out-of-fold predictions
so prey FP/FN and cat accuracy are measured over the FULL labeled set
(~33 prey, hundreds of cat bursts).

Run it twice to isolate the effect of the extra heads on the primary tasks:
  --mode all       full 4-head model (prey+cat+subject+direction)
  --mode prey_cat  aux weights zeroed (prey+cat only), same code/folds

Usage:
  uv run python tools/cv_multitask.py --folds 5 --epochs 15 --mode all
  uv run python tools/cv_multitask.py --folds 5 --epochs 15 --mode prey_cat
"""
from __future__ import annotations

import argparse
import random
import sys
from collections import defaultdict
from pathlib import Path

import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
from tools.train_multitask import (  # noqa: E402
    DATASET, DEFAULT_CROPS, AllFrames, BurstFrames, MazgeMultiTask,
    _masked_ce, _metrics, get_eval_tf, get_train_tf, load_rows,
)


def predict_rows(model, loader, device):
    model.eval()
    rows = []
    with torch.no_grad():
        for batch in loader:
            imgs = batch["image"].to(device)
            fidx = batch["frame_idx"].to(device)
            prey_l, cat_l, subj_l, dir_l = model(imgs, fidx)
            prey_p = torch.sigmoid(prey_l).cpu().numpy()
            cat_sm = torch.softmax(cat_l, 1).cpu().numpy()
            subj_pred = subj_l.argmax(1).cpu().numpy()
            dir_pred = dir_l.argmax(1).cpu().numpy()
            for i, bid in enumerate(batch["burst_id"]):
                rows.append({
                    "burst_id": bid,
                    "prey_prob": float(prey_p[i]), "prey_y": int(batch["prey_label"][i]),
                    "cat_p": cat_sm[i].tolist(), "cat_y": int(batch["cat_label"][i]),
                    "subj_pred": int(subj_pred[i]), "subj_y": int(batch["subject_label"][i]),
                    "dir_pred": int(dir_pred[i]), "dir_y": int(batch["dir_label"][i]),
                })
    return rows


def train_fold(frames_by_burst, train_b, test_b, args, device, weights):
    train_rows = [r for b in train_b for r in frames_by_burst[b]]
    npos = sum(r["effective_prey"] == "1" for r in train_rows)
    pw = min((len(train_rows) - npos) / max(npos, 1), 40.0)
    pos_weight = torch.tensor([pw], device=device)

    train_ds = BurstFrames(sorted(train_b), frames_by_burst, get_train_tf())
    test_ds = AllFrames({b: frames_by_burst[b] for b in test_b}, get_eval_tf())
    train_ld = DataLoader(train_ds, batch_size=args.bs, shuffle=True, num_workers=0)
    test_ld = DataLoader(test_ds, batch_size=64, shuffle=False, num_workers=0)

    model = MazgeMultiTask(direction_grad=False).to(device)
    for epoch in range(1, args.epochs + 1):
        frozen = epoch <= args.freeze_epochs
        model.freeze_backbone(frozen)
        lr = args.lr if frozen else args.lr_finetune
        opt = torch.optim.AdamW([p for p in model.parameters() if p.requires_grad],
                                lr=lr, weight_decay=1e-4)
        model.train()
        for batch in train_ld:
            imgs = batch["image"].to(device)
            prey_y = batch["prey_label"].to(device)
            prey_l, cat_l, subj_l, dir_l = model(imgs, batch["frame_idx"].to(device))
            prey_loss = F.binary_cross_entropy_with_logits(prey_l, prey_y, pos_weight=pos_weight)
            cat_loss, _, _ = _masked_ce(cat_l, batch["cat_label"].to(device))
            subj_loss, _, _ = _masked_ce(subj_l, batch["subject_label"].to(device))
            dir_loss, _, _ = _masked_ce(dir_l, batch["dir_label"].to(device))
            loss = (weights["prey"] * prey_loss + weights["cat"] * cat_loss
                    + weights["subject"] * subj_loss + weights["direction"] * dir_loss)
            opt.zero_grad(); loss.backward(); opt.step()
    return predict_rows(model, test_ld, device)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--crops", default=DEFAULT_CROPS)
    ap.add_argument("--folds", type=int, default=5)
    ap.add_argument("--epochs", type=int, default=15)
    ap.add_argument("--freeze-epochs", type=int, default=3)
    ap.add_argument("--bs", type=int, default=32)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--lr-finetune", type=float, default=1e-4)
    ap.add_argument("--mode", choices=["all", "prey_cat"], default="all")
    ap.add_argument("--seed", type=int, default=1337)
    ap.add_argument("--save-preds", default="", help="write per-burst pooled preds CSV")
    args = ap.parse_args()

    weights = {"all": {"prey": 1.0, "cat": 0.3, "subject": 0.3, "direction": 0.2},
               "prey_cat": {"prey": 1.0, "cat": 0.3, "subject": 0.0, "direction": 0.0}}[args.mode]

    random.seed(args.seed)
    torch.manual_seed(args.seed)
    device = ("mps" if torch.backends.mps.is_available()
              else "cuda" if torch.cuda.is_available() else "cpu")

    rows = load_rows(DATASET / args.crops)
    frames_by_burst = defaultdict(list)
    for r in rows:
        frames_by_burst[r["burst_id"]].append(r)

    # Stratified folds: prey and non-prey bursts distributed round-robin.
    prey_b, non_b = [], []
    for b, fs in frames_by_burst.items():
        (prey_b if fs[0]["effective_prey"] == "1" else non_b).append(b)
    rng = random.Random(args.seed)
    rng.shuffle(prey_b); rng.shuffle(non_b)
    fold_of = {}
    for i, b in enumerate(prey_b):
        fold_of[b] = i % args.folds
    for i, b in enumerate(non_b):
        fold_of[b] = i % args.folds

    print(f"CV mode={args.mode} folds={args.folds} epochs={args.epochs} device={device}")
    print(f"total bursts={len(frames_by_burst)} prey_bursts={len(prey_b)}")

    pooled = []
    for f in range(args.folds):
        test_b = [b for b in frames_by_burst if fold_of[b] == f]
        train_b = [b for b in frames_by_burst if fold_of[b] != f]
        fold_prey = sum(frames_by_burst[b][0]["effective_prey"] == "1" for b in test_b)
        print(f"  fold {f}: train={len(train_b)} test={len(test_b)} (test prey={fold_prey})", flush=True)
        pooled += train_fold(frames_by_burst, train_b, test_b, args, device, weights)

    m = _metrics(pooled, threshold=0.5)
    pf, pb = m["prey_frame"], m["prey_burst"]
    print("\n=== POOLED out-of-fold (every burst tested once) ===")
    print(f"mode={args.mode}")
    print(f"PREY frame:  TP={pf['tp']} FP={pf['fp']} FN={pf['fn']}  "
          f"P={pf['precision']:.3f} R={pf['recall']:.3f} F1={pf['f1']:.3f}")
    print(f"PREY burst:  TP={pb['tp']} FP={pb['fp']} FN={pb['fn']}  "
          f"P={pb['precision']:.3f} R={pb['recall']:.3f} F1={pb['f1']:.3f}")
    print(f"CAT   burst acc: {m['cat_burst_acc']:.3f}  (n={m['cat_burst_n']})")
    print(f"SUBJECT frame acc: {m['subject_frame_acc']:.3f}  (n={m['subject_frame_n']})")
    print(f"DIRECTION burst acc: {m['direction_burst_acc']:.3f}  (n={m['direction_burst_n']})")

    if args.save_preds:
        import csv as _csv
        # per-FRAME rows so we can study temporal indicators (frames-flagged, mean).
        with open(args.save_preds, "w", newline="") as f:
            w = _csv.writer(f)
            w.writerow(["burst_id", "prey_y", "prey_prob"])
            for r in pooled:
                w.writerow([r["burst_id"], r["prey_y"], round(r["prey_prob"], 4)])
        print(f"wrote per-frame preds ({len(pooled)} rows) -> {args.save_preds}")


if __name__ == "__main__":
    main()
