"""Train a MobileNetV3-Small prey detector on the Mazge catflap dataset.

Usage:
  uv run python tools/train_prey_model.py                     # default
  uv run python tools/train_prey_model.py --epochs 100 --bs 32
  uv run python tools/train_prey_model.py --resume models/prey_v1/best.pt

Outputs:
  models/prey_v1/best.pt       — best checkpoint (by val recall @ fixed FPR)
  models/prey_v1/last.pt       — last epoch checkpoint
  models/prey_v1/metrics.json  — final eval metrics
  models/prey_v1/train_log.csv — per-epoch training log
"""
from __future__ import annotations

import argparse
import csv
import json
import time
from collections import Counter
from pathlib import Path

import torch
import torch.nn as nn
import torch.nn.functional as F
from PIL import Image
from torch.utils.data import DataLoader, Dataset, WeightedRandomSampler
from torchvision import transforms
from torchvision.models import mobilenet_v3_small, MobileNet_V3_Small_Weights

REPO = Path(__file__).resolve().parent.parent
DATASET_DIR = REPO / "dataset"

# ── Preprocessing (matches firmware crop+rotate exactly) ──────────────
CROP_BOX = (64, 48, 64 + 384, 48 + 384)
IMG_SIZE = 384

CAT_ID_MAP = {"mazge": 0, "benis": 1, "unknown": 2, "": 2}


# ── Dataset ───────────────────────────────────────────────────────────
class PreyDataset(Dataset):
    """One row per frame from manifest.csv."""

    def __init__(self, manifest_path: Path, split: str, transform=None,
                 include_cat_id: bool = True):
        self.transform = transform
        self.include_cat_id = include_cat_id
        self.rows = []

        with open(manifest_path) as f:
            for row in csv.DictReader(f):
                if row["split"] != split:
                    continue
                if row["has_jpg"] != "1":
                    continue
                if row["weak_label"] == "":
                    continue
                self.rows.append(row)

        labels = Counter(int(r["weak_label"]) for r in self.rows)
        self.n_pos = labels.get(1, 0)
        self.n_neg = labels.get(0, 0)
        print(f"  {split}: {len(self.rows)} frames "
              f"(prey={self.n_pos}, no-prey={self.n_neg}, "
              f"ratio=1:{self.n_neg // max(self.n_pos, 1)})")

    def __len__(self):
        return len(self.rows)

    def __getitem__(self, idx):
        row = self.rows[idx]
        img_path = REPO / row["image_id"]
        img = Image.open(img_path)

        # Apply the standard crop+rotate
        img = img.crop(CROP_BOX).rotate(90, expand=True).convert("L")

        # Convert to 3-channel (pretrained models expect 3ch)
        img = img.convert("RGB")

        if self.transform:
            img = self.transform(img)

        label = float(row["weak_label"])
        weight = float(row["weak_confidence"])
        cat_id = CAT_ID_MAP.get(row.get("cat_id", ""), 2)
        frame_idx = int(row.get("frame_idx", 0))

        return {
            "image": img,
            "label": torch.tensor(label),
            "weight": torch.tensor(weight),
            "cat_id": torch.tensor(cat_id, dtype=torch.long),
            "frame_idx": torch.tensor(frame_idx, dtype=torch.long),
        }


# ── Augmentations ─────────────────────────────────────────────────────
def get_train_transform():
    return transforms.Compose([
        transforms.Resize((IMG_SIZE, IMG_SIZE)),
        transforms.RandomHorizontalFlip(p=0.5),
        transforms.RandomAffine(degrees=5, translate=(0.05, 0.05), scale=(0.9, 1.1)),
        transforms.ColorJitter(brightness=0.3, contrast=0.3),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],
                             std=[0.229, 0.224, 0.225]),
    ])


def get_val_transform():
    return transforms.Compose([
        transforms.Resize((IMG_SIZE, IMG_SIZE)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],
                             std=[0.229, 0.224, 0.225]),
    ])


# ── Model ─────────────────────────────────────────────────────────────
class PreyDetector(nn.Module):
    """MobileNetV3-Small backbone + prey head + optional cat-ID auxiliary head."""

    def __init__(self, num_cat_ids: int = 3):
        super().__init__()
        backbone = mobilenet_v3_small(weights=MobileNet_V3_Small_Weights.DEFAULT)

        # Backbone features (everything except the classifier)
        self.features = backbone.features
        self.avgpool = backbone.avgpool

        # MobileNetV3-Small last channel = 576
        feat_dim = 576

        # Frame index embedding (10 possible indices)
        self.frame_emb = nn.Embedding(10, 16)
        head_dim = feat_dim + 16  # 592

        # Prey head (binary)
        self.prey_head = nn.Sequential(
            nn.Linear(head_dim, 128),
            nn.Hardswish(),
            nn.Dropout(0.3),
            nn.Linear(128, 1),
        )

        # Cat-ID auxiliary head (3-class: mazge/benis/unknown)
        self.cat_id_head = nn.Sequential(
            nn.Linear(head_dim, 64),
            nn.Hardswish(),
            nn.Dropout(0.2),
            nn.Linear(64, num_cat_ids),
        )

    def forward(self, x, frame_idx=None):
        feat = self.features(x)
        feat = self.avgpool(feat)
        feat = feat.flatten(1)  # (B, 576)

        if frame_idx is not None:
            f_emb = self.frame_emb(frame_idx)  # (B, 16)
            feat = torch.cat([feat, f_emb], dim=1)  # (B, 592)
        else:
            # Pad with zeros if no frame_idx
            feat = torch.cat([feat, torch.zeros(feat.size(0), 16, device=feat.device)], dim=1)

        prey_logit = self.prey_head(feat).squeeze(-1)  # (B,)
        cat_id_logits = self.cat_id_head(feat)          # (B, 3)

        return prey_logit, cat_id_logits


# ── Training ──────────────────────────────────────────────────────────
def make_sampler(dataset: PreyDataset, oversample_prey: int = 8):
    """WeightedRandomSampler to oversample prey frames."""
    weights = []
    for row in dataset.rows:
        if int(row["weak_label"]) == 1:
            weights.append(float(oversample_prey))
        else:
            weights.append(1.0)
    return WeightedRandomSampler(weights, num_samples=len(weights), replacement=True)


def train_one_epoch(model, loader, optimizer, scaler, device, pos_weight, cat_id_weight=0.3):
    model.train()
    total_loss = 0.0
    total_prey_loss = 0.0
    total_cat_loss = 0.0
    n = 0

    for batch in loader:
        imgs = batch["image"].to(device)
        labels = batch["label"].to(device)
        weights = batch["weight"].to(device)
        cat_ids = batch["cat_id"].to(device)
        frame_idx = batch["frame_idx"].to(device)

        with torch.amp.autocast("cpu" if device.type == "cpu" else device.type):
            prey_logit, cat_logits = model(imgs, frame_idx)

            # Weighted BCE for prey
            prey_loss = F.binary_cross_entropy_with_logits(
                prey_logit, labels, weight=weights,
                pos_weight=pos_weight.to(device),
            )

            # Cat-ID cross entropy (only on frames with known cat_id, i.e. not "unknown"=2)
            cat_mask = cat_ids < 2  # mazge=0, benis=1
            if cat_mask.any():
                cat_loss = F.cross_entropy(cat_logits[cat_mask], cat_ids[cat_mask])
            else:
                cat_loss = torch.tensor(0.0, device=device)

            loss = prey_loss + cat_id_weight * cat_loss

        optimizer.zero_grad(set_to_none=True)
        if scaler is not None:
            scaler.scale(loss).backward()
            scaler.unscale_(optimizer)
            nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            scaler.step(optimizer)
            scaler.update()
        else:
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            optimizer.step()

        bs = imgs.size(0)
        total_loss += loss.item() * bs
        total_prey_loss += prey_loss.item() * bs
        total_cat_loss += cat_loss.item() * bs
        n += bs

    return total_loss / n, total_prey_loss / n, total_cat_loss / n


@torch.no_grad()
def evaluate(model, loader, device, pos_weight, threshold=0.5):
    model.eval()
    all_probs = []
    all_labels = []
    all_cat_preds = []
    all_cat_true = []
    total_loss = 0.0
    n = 0

    for batch in loader:
        imgs = batch["image"].to(device)
        labels = batch["label"].to(device)
        weights = batch["weight"].to(device)
        cat_ids = batch["cat_id"].to(device)
        frame_idx = batch["frame_idx"].to(device)

        prey_logit, cat_logits = model(imgs, frame_idx)

        loss = F.binary_cross_entropy_with_logits(
            prey_logit, labels, weight=weights,
            pos_weight=pos_weight.to(device),
        )

        probs = torch.sigmoid(prey_logit)
        all_probs.extend(probs.cpu().tolist())
        all_labels.extend(labels.cpu().int().tolist())
        all_cat_preds.extend(cat_logits.argmax(dim=1).cpu().tolist())
        all_cat_true.extend(cat_ids.cpu().tolist())

        total_loss += loss.item() * imgs.size(0)
        n += imgs.size(0)

    # Prey metrics
    preds = [1 if p >= threshold else 0 for p in all_probs]
    tp = sum(1 for p, l in zip(preds, all_labels) if p == 1 and l == 1)
    fp = sum(1 for p, l in zip(preds, all_labels) if p == 1 and l == 0)
    fn = sum(1 for p, l in zip(preds, all_labels) if p == 0 and l == 1)
    tn = sum(1 for p, l in zip(preds, all_labels) if p == 0 and l == 0)

    precision = tp / max(tp + fp, 1)
    recall = tp / max(tp + fn, 1)
    f1 = 2 * precision * recall / max(precision + recall, 1e-8)

    # Cat-ID accuracy (only on known labels)
    cat_correct = sum(1 for p, t in zip(all_cat_preds, all_cat_true)
                      if t < 2 and p == t)
    cat_total = sum(1 for t in all_cat_true if t < 2)
    cat_acc = cat_correct / max(cat_total, 1)

    return {
        "loss": total_loss / n,
        "precision": precision,
        "recall": recall,
        "f1": f1,
        "tp": tp, "fp": fp, "fn": fn, "tn": tn,
        "cat_id_acc": cat_acc,
        "cat_id_n": cat_total,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--bs", type=int, default=32)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--outdir", type=str, default="models/prey_v1")
    ap.add_argument("--resume", type=str, default=None)
    ap.add_argument("--oversample", type=int, default=8,
                    help="Oversample prey frames by this factor")
    ap.add_argument("--cat-id-weight", type=float, default=0.3,
                    help="Weight for cat-ID auxiliary loss")
    ap.add_argument("--threshold", type=float, default=0.5)
    args = ap.parse_args()

    outdir = REPO / args.outdir
    outdir.mkdir(parents=True, exist_ok=True)

    # Device
    if torch.backends.mps.is_available():
        device = torch.device("mps")
        print("Using MPS (Apple Silicon GPU)")
    elif torch.cuda.is_available():
        device = torch.device("cuda")
    else:
        device = torch.device("cpu")
        print("Using CPU")

    # Load datasets
    manifest = DATASET_DIR / "manifest.csv"
    print(f"\nLoading dataset from {manifest}")
    train_ds = PreyDataset(manifest, "train", get_train_transform())
    val_ds = PreyDataset(manifest, "val", get_val_transform())
    test_ds = PreyDataset(manifest, "test", get_val_transform())

    # DataLoaders
    train_sampler = make_sampler(train_ds, args.oversample)
    train_loader = DataLoader(train_ds, batch_size=args.bs, sampler=train_sampler,
                              num_workers=4, pin_memory=True, persistent_workers=True)
    val_loader = DataLoader(val_ds, batch_size=args.bs, shuffle=False,
                            num_workers=2, pin_memory=True, persistent_workers=True)
    test_loader = DataLoader(test_ds, batch_size=args.bs, shuffle=False,
                             num_workers=2, pin_memory=True)

    # Pos weight for BCE
    pos_weight = torch.tensor([train_ds.n_neg / max(train_ds.n_pos, 1)])
    print(f"\npos_weight: {pos_weight.item():.1f}")

    # Model
    model = PreyDetector().to(device)
    param_count = sum(p.numel() for p in model.parameters())
    print(f"Model parameters: {param_count:,}")

    if args.resume:
        ckpt = torch.load(args.resume, map_location=device, weights_only=True)
        model.load_state_dict(ckpt["model"])
        print(f"Resumed from {args.resume}")

    # Optimizer + scheduler
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=args.epochs)

    # AMP scaler (not used on MPS, only CUDA)
    scaler = torch.amp.GradScaler() if device.type == "cuda" else None

    # Training loop
    log_path = outdir / "train_log.csv"
    log_f = open(log_path, "w", newline="")
    log_w = csv.writer(log_f)
    log_w.writerow(["epoch", "train_loss", "prey_loss", "cat_loss",
                     "val_loss", "val_prec", "val_rec", "val_f1",
                     "val_cat_acc", "lr", "secs"])

    best_f1 = 0.0
    print(f"\n{'='*70}")
    print(f"Training for {args.epochs} epochs, batch_size={args.bs}, lr={args.lr}")
    print(f"{'='*70}\n")

    for epoch in range(1, args.epochs + 1):
        t0 = time.time()
        train_loss, prey_loss, cat_loss = train_one_epoch(
            model, train_loader, optimizer, scaler, device,
            pos_weight, args.cat_id_weight,
        )
        val_metrics = evaluate(model, val_loader, device, pos_weight, args.threshold)
        scheduler.step()

        elapsed = time.time() - t0
        lr = optimizer.param_groups[0]["lr"]

        log_w.writerow([epoch, f"{train_loss:.4f}", f"{prey_loss:.4f}",
                        f"{cat_loss:.4f}", f"{val_metrics['loss']:.4f}",
                        f"{val_metrics['precision']:.3f}",
                        f"{val_metrics['recall']:.3f}",
                        f"{val_metrics['f1']:.3f}",
                        f"{val_metrics['cat_id_acc']:.3f}",
                        f"{lr:.6f}", f"{elapsed:.1f}"])
        log_f.flush()

        prey_str = (f"P={val_metrics['precision']:.2f} R={val_metrics['recall']:.2f} "
                     f"F1={val_metrics['f1']:.2f}")
        cat_str = f"cat_acc={val_metrics['cat_id_acc']:.2f}"
        tp_fp = f"TP={val_metrics['tp']} FP={val_metrics['fp']} FN={val_metrics['fn']}"
        print(f"  [{epoch:3d}/{args.epochs}] loss={train_loss:.3f} "
              f"val_loss={val_metrics['loss']:.3f} | {prey_str} {tp_fp} | "
              f"{cat_str} | {elapsed:.0f}s")

        # Save checkpoints
        ckpt = {
            "model": model.state_dict(),
            "optimizer": optimizer.state_dict(),
            "epoch": epoch,
            "val_metrics": val_metrics,
            "args": vars(args),
        }
        torch.save(ckpt, outdir / "last.pt")

        if val_metrics["f1"] > best_f1:
            best_f1 = val_metrics["f1"]
            torch.save(ckpt, outdir / "best.pt")
            print(f"         ★ New best F1={best_f1:.3f}")

    log_f.close()

    # Final evaluation on test set
    print(f"\n{'='*70}")
    print("Final evaluation on TEST set:")
    print(f"{'='*70}")

    best_ckpt = torch.load(outdir / "best.pt", map_location=device, weights_only=True)
    model.load_state_dict(best_ckpt["model"])

    test_metrics = evaluate(model, test_loader, device, pos_weight, args.threshold)
    print(f"  Prey:    P={test_metrics['precision']:.3f} R={test_metrics['recall']:.3f} "
          f"F1={test_metrics['f1']:.3f}")
    print(f"  Counts:  TP={test_metrics['tp']} FP={test_metrics['fp']} "
          f"FN={test_metrics['fn']} TN={test_metrics['tn']}")
    print(f"  Cat-ID:  acc={test_metrics['cat_id_acc']:.3f} (n={test_metrics['cat_id_n']})")
    print(f"  Best epoch: {best_ckpt['epoch']}")

    # Save metrics
    metrics = {
        "test": test_metrics,
        "best_epoch": best_ckpt["epoch"],
        "best_val_f1": best_f1,
        "args": vars(args),
        "model_params": param_count,
    }
    with open(outdir / "metrics.json", "w") as f:
        json.dump(metrics, f, indent=2)
    print(f"\nSaved to {outdir}/")


if __name__ == "__main__":
    main()
