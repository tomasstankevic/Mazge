#!/usr/bin/env python3
"""Train a Benis-vs-Mazge classifier on YOLO11x body crops.

Architecture:
  EfficientNet-B0 (ImageNet pretrained, 5M params)
  -> 2-way softmax for cat identity: mazge vs benis

Training discipline:
  - Deduplicate crop-index rows in memory by (burst_id, frame_idx, image_id)
  - Use only frames with a body crop and a known cat_id in {mazge, benis}
  - Split per-burst, stratified by cat identity, so frames from one burst never
    leak across train/val/test
  - Sample one random frame per burst during training to reduce overfitting to
    near-duplicate frames from the same event

Outputs (models/cat_id_v2/<tag>/):
  best_val_acc.pt
  best_val_burst_acc.pt
  last.pt
  train_log.csv
  metrics_live.json
  split_assignment.json
  test_metrics.json
  test_predictions.csv

Usage:
  cd Mazge && uv run python tools/train_cat_id_v2.py
  uv run python tools/train_cat_id_v2.py --epochs 20 --tag bodyA
"""
from __future__ import annotations

import argparse
import csv
import json
import logging
import os
import random
import time
from collections import Counter, defaultdict
from pathlib import Path

os.environ.setdefault("SSL_CERT_FILE", __import__("certifi").where())

import torch
import torch.nn as nn
import torch.nn.functional as F
from PIL import Image
from torch.utils.data import DataLoader, Dataset
from torchvision import transforms
from torchvision.models import EfficientNet_B0_Weights, efficientnet_b0

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s %(levelname)-5s %(message)s",
                    datefmt="%H:%M:%S")
log = logging.getLogger("cat_id_v2")

REPO = Path(__file__).resolve().parent.parent
DATASET = REPO / "dataset"
DEFAULT_CROPS = "crops_yolo11x_rotcrop"
DEFAULT_OUT = REPO / "models" / "cat_id_v2"

IMG_SIZE = 224
TRAIN_FRAC = 0.60
VAL_FRAC = 0.20
CAT_TO_INT = {"mazge": 0, "benis": 1}
INT_TO_CAT = {0: "mazge", 1: "benis"}


class CatIdBurstFrames(Dataset):
    """Sample one random body crop per burst on each access."""

    def __init__(self, bursts: list[str], frames_by_burst: dict[str, list[dict]],
                 transform=None):
        self.bursts = bursts
        self.frames_by_burst = frames_by_burst
        self.transform = transform
        self.rng = random.Random(0xC0FFEE)

    def __len__(self):
        return len(self.bursts)

    def __getitem__(self, idx):
        bid = self.bursts[idx]
        row = self.rng.choice(self.frames_by_burst[bid])
        img = Image.open(DATASET / row["body_path"]).convert("RGB")
        if self.transform:
            img = self.transform(img)
        label = CAT_TO_INT[row["cat_id"]]
        return {
            "image": img,
            "label": torch.tensor(label, dtype=torch.long),
            "burst_id": bid,
            "frame_idx": int(row["frame_idx"]),
        }


class CatIdEvalFrames(Dataset):
    """Flat iterator over all deduplicated body crops."""

    def __init__(self, frames_by_burst: dict[str, list[dict]], transform=None):
        self.rows = [r for frames in frames_by_burst.values() for r in frames]
        self.transform = transform

    def __len__(self):
        return len(self.rows)

    def __getitem__(self, idx):
        row = self.rows[idx]
        img = Image.open(DATASET / row["body_path"]).convert("RGB")
        if self.transform:
            img = self.transform(img)
        label = CAT_TO_INT[row["cat_id"]]
        return {
            "image": img,
            "label": torch.tensor(label, dtype=torch.long),
            "burst_id": row["burst_id"],
            "frame_idx": int(row["frame_idx"]),
        }


def restratify_identity_splits(bursts_meta: dict[str, dict], seed: int) -> dict[str, str]:
    """Assign train/val/test per burst, stratified by cat identity."""

    out: dict[str, str] = {}
    rng = random.Random(seed)
    for cat_name in ("mazge", "benis"):
        bursts = sorted(bid for bid, meta in bursts_meta.items()
                        if meta["cat_id"] == cat_name)
        rng.shuffle(bursts)
        n_train = int(round(len(bursts) * TRAIN_FRAC))
        n_val = int(round(len(bursts) * VAL_FRAC))
        for i, bid in enumerate(bursts):
            if i < n_train:
                out[bid] = "train"
            elif i < n_train + n_val:
                out[bid] = "val"
            else:
                out[bid] = "test"
    return out


def get_train_tf():
    return transforms.Compose([
        transforms.Resize((IMG_SIZE, IMG_SIZE)),
        transforms.RandomHorizontalFlip(p=0.5),
        transforms.RandomAffine(degrees=5, translate=(0.05, 0.05),
                                 scale=(0.95, 1.05)),
        transforms.ColorJitter(brightness=0.15, contrast=0.15),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],
                              std=[0.229, 0.224, 0.225]),
    ])


def get_eval_tf():
    return transforms.Compose([
        transforms.Resize((IMG_SIZE, IMG_SIZE)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],
                              std=[0.229, 0.224, 0.225]),
    ])


class CatIdClassifier(nn.Module):
    def __init__(self):
        super().__init__()
        backbone = efficientnet_b0(weights=EfficientNet_B0_Weights.DEFAULT)
        in_features = backbone.classifier[1].in_features
        backbone.classifier = nn.Sequential(
            nn.Dropout(0.2),
            nn.Linear(in_features, 2),
        )
        self.model = backbone

    def freeze_backbone(self, freeze: bool = True):
        for name, param in self.model.named_parameters():
            if not name.startswith("classifier"):
                param.requires_grad = not freeze

    def forward(self, x):
        return self.model(x)


@torch.no_grad()
def evaluate(model, loader, device):
    model.eval()
    rows = []
    loss_sum = 0.0
    n = 0
    for batch in loader:
        imgs = batch["image"].to(device)
        labels = batch["label"].to(device)
        logits = model(imgs)
        loss = F.cross_entropy(logits, labels)
        probs = torch.softmax(logits, dim=1)[:, 1].cpu().numpy()
        preds = logits.argmax(dim=1).cpu().numpy()
        labels_np = labels.cpu().numpy()
        for bid, fidx, prob_benis, pred, label in zip(
                batch["burst_id"], batch["frame_idx"].numpy(), probs, preds, labels_np):
            rows.append({
                "burst_id": bid,
                "frame_idx": int(fidx),
                "prob_benis": float(prob_benis),
                "pred": int(pred),
                "label": int(label),
            })
        loss_sum += loss.item() * imgs.size(0)
        n += imgs.size(0)

    frame_correct = sum(1 for r in rows if r["pred"] == r["label"])
    frame_acc = frame_correct / max(len(rows), 1)
    frame_tp = sum(1 for r in rows if r["pred"] == 1 and r["label"] == 1)
    frame_tn = sum(1 for r in rows if r["pred"] == 0 and r["label"] == 0)
    frame_fp = sum(1 for r in rows if r["pred"] == 1 and r["label"] == 0)
    frame_fn = sum(1 for r in rows if r["pred"] == 0 and r["label"] == 1)
    benis_rec = frame_tp / max(frame_tp + frame_fn, 1)
    mazge_rec = frame_tn / max(frame_tn + frame_fp, 1)
    frame_bal_acc = 0.5 * (benis_rec + mazge_rec)

    by_burst: dict[str, dict[str, float | int]] = defaultdict(
        lambda: {"label": 0, "sum_prob": 0.0, "n": 0})
    for r in rows:
        agg = by_burst[r["burst_id"]]
        agg["label"] = r["label"]
        agg["sum_prob"] += r["prob_benis"]
        agg["n"] += 1
    burst_rows = []
    for bid, agg in by_burst.items():
        prob_benis = agg["sum_prob"] / max(int(agg["n"]), 1)
        pred = 1 if prob_benis >= 0.5 else 0
        burst_rows.append({
            "burst_id": bid,
            "prob_benis": float(prob_benis),
            "pred": pred,
            "label": int(agg["label"]),
        })

    burst_correct = sum(1 for r in burst_rows if r["pred"] == r["label"])
    burst_acc = burst_correct / max(len(burst_rows), 1)
    burst_tp = sum(1 for r in burst_rows if r["pred"] == 1 and r["label"] == 1)
    burst_tn = sum(1 for r in burst_rows if r["pred"] == 0 and r["label"] == 0)
    burst_fp = sum(1 for r in burst_rows if r["pred"] == 1 and r["label"] == 0)
    burst_fn = sum(1 for r in burst_rows if r["pred"] == 0 and r["label"] == 1)
    burst_benis_rec = burst_tp / max(burst_tp + burst_fn, 1)
    burst_mazge_rec = burst_tn / max(burst_tn + burst_fp, 1)
    burst_bal_acc = 0.5 * (burst_benis_rec + burst_mazge_rec)

    return {
        "loss": loss_sum / max(n, 1),
        "frame_acc": frame_acc,
        "frame_bal_acc": frame_bal_acc,
        "frame_tp": frame_tp,
        "frame_tn": frame_tn,
        "frame_fp": frame_fp,
        "frame_fn": frame_fn,
        "burst_acc": burst_acc,
        "burst_bal_acc": burst_bal_acc,
        "burst_tp": burst_tp,
        "burst_tn": burst_tn,
        "burst_fp": burst_fp,
        "burst_fn": burst_fn,
        "n_frames": len(rows),
        "n_bursts": len(burst_rows),
        "rows": rows,
        "burst_rows": burst_rows,
    }


def load_deduped_rows(crops_dir: Path) -> tuple[list[dict], int]:
    burst_meta = {
        row["burst_id"]: row
        for row in csv.DictReader(open(DATASET / "bursts.csv", newline=""))
    }
    deduped: dict[tuple[str, str, str], dict] = {}
    duplicate_count = 0
    for row in csv.DictReader(open(crops_dir / "_index.csv", newline="")):
        key = (row["burst_id"], row["frame_idx"], row["image_id"])
        prev = deduped.get(key)
        if prev is not None:
            duplicate_count += 1
            # Prefer the entry that actually has a body crop path.
            if prev.get("body_path") and not row.get("body_path"):
                continue
        deduped[key] = row

    rows = []
    for row in deduped.values():
        if not (row.get("body_path") or "").strip():
            continue
        meta = burst_meta.get(row["burst_id"], {})
        cat_id = (meta.get("cat_id") or "").strip().lower()
        if cat_id not in CAT_TO_INT:
            continue
        row = dict(row)
        row["cat_id"] = cat_id
        rows.append(row)
    return rows, duplicate_count


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--crops", default=DEFAULT_CROPS)
    ap.add_argument("--tag", default="bodyA")
    ap.add_argument("--epochs", type=int, default=20)
    ap.add_argument("--freeze-epochs", type=int, default=5)
    ap.add_argument("--bs", type=int, default=32)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--lr-finetune", type=float, default=1e-4)
    ap.add_argument("--weight-decay", type=float, default=1e-4)
    ap.add_argument("--seed", type=int, default=1337)
    ap.add_argument("--device", default="auto")
    args = ap.parse_args()

    random.seed(args.seed)
    torch.manual_seed(args.seed)

    out_dir = DEFAULT_OUT / args.tag
    out_dir.mkdir(parents=True, exist_ok=True)
    crops_dir = DATASET / args.crops
    if not (crops_dir / "_index.csv").exists():
        raise SystemExit(f"No _index.csv in {crops_dir}")

    log.info("Output: %s", out_dir)
    rows, duplicate_count = load_deduped_rows(crops_dir)
    log.info("Loaded %d deduplicated body crops (%d duplicate index rows ignored)",
             len(rows), duplicate_count)

    bursts_meta: dict[str, dict] = {}
    for row in rows:
        bid = row["burst_id"]
        if bid not in bursts_meta:
            bursts_meta[bid] = {"cat_id": row["cat_id"]}
    split_assign = restratify_identity_splits(bursts_meta, args.seed)
    (out_dir / "split_assignment.json").write_text(
        json.dumps(split_assign, indent=2, sort_keys=True))

    by_split: dict[str, dict[str, list[dict]]] = {
        "train": defaultdict(list),
        "val": defaultdict(list),
        "test": defaultdict(list),
    }
    for row in rows:
        by_split[split_assign[row["burst_id"]]][row["burst_id"]].append(row)

    split_summary = {}
    for sp in ("train", "val", "test"):
        bursts = list(by_split[sp].keys())
        counts = Counter(bursts_meta[bid]["cat_id"] for bid in bursts)
        frames = sum(len(by_split[sp][bid]) for bid in bursts)
        split_summary[sp] = {
            "total_bursts": len(bursts),
            "total_frames": frames,
            "mazge_bursts": counts.get("mazge", 0),
            "benis_bursts": counts.get("benis", 0),
        }
        log.info("%s: %d bursts (%d mazge, %d benis), %d frames",
                 sp, len(bursts), counts.get("mazge", 0),
                 counts.get("benis", 0), frames)

    train_ds = CatIdBurstFrames(list(by_split["train"].keys()), by_split["train"],
                                get_train_tf())
    val_ds = CatIdEvalFrames(by_split["val"], get_eval_tf())
    test_ds = CatIdEvalFrames(by_split["test"], get_eval_tf())

    train_loader = DataLoader(train_ds, batch_size=args.bs, shuffle=True,
                              num_workers=2, pin_memory=True,
                              persistent_workers=True)
    val_loader = DataLoader(val_ds, batch_size=args.bs, shuffle=False,
                            num_workers=2, pin_memory=True,
                            persistent_workers=True)
    test_loader = DataLoader(test_ds, batch_size=args.bs, shuffle=False,
                             num_workers=2, pin_memory=True)

    if args.device == "auto":
        if torch.backends.mps.is_available():
            device = torch.device("mps")
        elif torch.cuda.is_available():
            device = torch.device("cuda")
        else:
            device = torch.device("cpu")
    else:
        device = torch.device(args.device)
    log.info("Device: %s", device)

    class_counts = Counter(meta["cat_id"] for meta in bursts_meta.values()
                           if split_assign)
    train_class_counts = Counter(
        bursts_meta[bid]["cat_id"] for bid in by_split["train"].keys())
    class_weights = torch.tensor([
        1.0 / max(train_class_counts.get("mazge", 1), 1),
        1.0 / max(train_class_counts.get("benis", 1), 1),
    ], dtype=torch.float32)
    class_weights = class_weights / class_weights.mean()
    log.info("Train burst counts: mazge=%d benis=%d",
             train_class_counts.get("mazge", 0),
             train_class_counts.get("benis", 0))

    model = CatIdClassifier().to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr,
                                  weight_decay=args.weight_decay)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer, T_max=max(args.epochs - args.freeze_epochs, 1))

    log_f = open(out_dir / "train_log.csv", "w", newline="")
    log_w = csv.writer(log_f)
    log_w.writerow([
        "epoch", "train_loss", "val_loss", "val_frame_acc", "val_frame_bal_acc",
        "val_burst_acc", "val_burst_bal_acc", "val_burst_tp", "val_burst_fp",
        "val_burst_fn", "val_burst_tn", "lr", "secs", "backbone_frozen",
    ])

    state = {
        "current_epoch": 0,
        "total_epochs": args.epochs,
        "best_val_acc": 0.0,
        "best_val_acc_epoch": None,
        "best_val_burst_acc": 0.0,
        "best_val_burst_acc_epoch": None,
        "split_summary": split_summary,
        "history": [],
    }

    weights_dev = class_weights.to(device)
    for epoch in range(1, args.epochs + 1):
        backbone_frozen = epoch <= args.freeze_epochs
        if epoch == 1:
            model.freeze_backbone(True)
        elif epoch == args.freeze_epochs + 1:
            model.freeze_backbone(False)
            for group in optimizer.param_groups:
                group["lr"] = args.lr_finetune
            log.info("Unfreezing backbone, lr -> %g", args.lr_finetune)

        t0 = time.time()
        model.train()
        train_loss_sum = 0.0
        n_batches = 0
        for batch in train_loader:
            imgs = batch["image"].to(device)
            labels = batch["label"].to(device)
            logits = model(imgs)
            loss = F.cross_entropy(logits, labels, weight=weights_dev)
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            optimizer.step()
            train_loss_sum += loss.item()
            n_batches += 1
        avg_train_loss = train_loss_sum / max(n_batches, 1)

        if not backbone_frozen:
            scheduler.step()

        val_metrics = evaluate(model, val_loader, device)
        elapsed = time.time() - t0
        cur_lr = optimizer.param_groups[0]["lr"]
        log_w.writerow([
            epoch, f"{avg_train_loss:.4f}", f"{val_metrics['loss']:.4f}",
            f"{val_metrics['frame_acc']:.3f}", f"{val_metrics['frame_bal_acc']:.3f}",
            f"{val_metrics['burst_acc']:.3f}", f"{val_metrics['burst_bal_acc']:.3f}",
            val_metrics["burst_tp"], val_metrics["burst_fp"],
            val_metrics["burst_fn"], val_metrics["burst_tn"],
            f"{cur_lr:.6f}", f"{elapsed:.1f}", int(backbone_frozen),
        ])
        log_f.flush()

        state["current_epoch"] = epoch
        state["history"].append({
            "epoch": epoch,
            "train_loss": avg_train_loss,
            "val_loss": val_metrics["loss"],
            "val_frame_acc": val_metrics["frame_acc"],
            "val_frame_bal_acc": val_metrics["frame_bal_acc"],
            "val_burst_acc": val_metrics["burst_acc"],
            "val_burst_bal_acc": val_metrics["burst_bal_acc"],
            "lr": cur_lr,
            "secs": elapsed,
            "backbone_frozen": backbone_frozen,
        })

        log.info("[%3d/%d] train_loss=%.3f val_loss=%.3f frame_acc=%.3f "
                 "frame_bal=%.3f burst_acc=%.3f burst_bal=%.3f %s %.0fs",
                 epoch, args.epochs, avg_train_loss, val_metrics["loss"],
                 val_metrics["frame_acc"], val_metrics["frame_bal_acc"],
                 val_metrics["burst_acc"], val_metrics["burst_bal_acc"],
                 "frozen" if backbone_frozen else "tune", elapsed)

        ckpt = {
            "model": model.state_dict(),
            "epoch": epoch,
            "args": vars(args),
            "val_metrics": {k: v for k, v in val_metrics.items()
                            if k not in {"rows", "burst_rows"}},
            "split_summary": split_summary,
        }
        torch.save(ckpt, out_dir / "last.pt")
        if val_metrics["frame_acc"] > state["best_val_acc"]:
            state["best_val_acc"] = val_metrics["frame_acc"]
            state["best_val_acc_epoch"] = epoch
            torch.save(ckpt, out_dir / "best_val_acc.pt")
        if val_metrics["burst_acc"] > state["best_val_burst_acc"]:
            state["best_val_burst_acc"] = val_metrics["burst_acc"]
            state["best_val_burst_acc_epoch"] = epoch
            torch.save(ckpt, out_dir / "best_val_burst_acc.pt")

        (out_dir / "metrics_live.json").write_text(json.dumps({
            "current_epoch": epoch,
            "total_epochs": args.epochs,
            "best_val_acc": state["best_val_acc"],
            "best_val_acc_epoch": state["best_val_acc_epoch"],
            "best_val_burst_acc": state["best_val_burst_acc"],
            "best_val_burst_acc_epoch": state["best_val_burst_acc_epoch"],
            "duplicate_rows_ignored": duplicate_count,
            "split_summary": split_summary,
            "history": state["history"],
        }, indent=2))

    best_ckpt = torch.load(out_dir / "best_val_burst_acc.pt", map_location=device,
                           weights_only=False)
    model.load_state_dict(best_ckpt["model"])
    test_metrics = evaluate(model, test_loader, device)
    log.info("TEST burst-best checkpoint @ epoch %d: frame_acc=%.3f frame_bal=%.3f "
             "burst_acc=%.3f burst_bal=%.3f",
             best_ckpt["epoch"], test_metrics["frame_acc"],
             test_metrics["frame_bal_acc"], test_metrics["burst_acc"],
             test_metrics["burst_bal_acc"])

    (out_dir / "test_metrics.json").write_text(json.dumps({
        "checkpoint": "best_val_burst_acc",
        "ckpt_epoch": best_ckpt["epoch"],
        "duplicate_rows_ignored": duplicate_count,
        "frame": {
            "acc": test_metrics["frame_acc"],
            "balanced_acc": test_metrics["frame_bal_acc"],
            "tp": test_metrics["frame_tp"],
            "tn": test_metrics["frame_tn"],
            "fp": test_metrics["frame_fp"],
            "fn": test_metrics["frame_fn"],
        },
        "burst": {
            "acc": test_metrics["burst_acc"],
            "balanced_acc": test_metrics["burst_bal_acc"],
            "tp": test_metrics["burst_tp"],
            "tn": test_metrics["burst_tn"],
            "fp": test_metrics["burst_fp"],
            "fn": test_metrics["burst_fn"],
        },
        "n_frames": test_metrics["n_frames"],
        "n_bursts": test_metrics["n_bursts"],
        "split_summary": split_summary,
    }, indent=2))

    with open(out_dir / "test_predictions.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["burst_id", "frame_idx", "prob_benis", "pred", "label"])
        w.writeheader()
        w.writerows(test_metrics["rows"])

    log_f.close()
    log.info("Saved to %s", out_dir)


if __name__ == "__main__":
    main()