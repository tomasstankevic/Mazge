#!/usr/bin/env python3
"""Train prey_v3 multitask model on YOLO11x body crops.

Model outputs per frame:
  - prey_logit (binary prey)
  - cat_logits (2-way cat identity: mazge vs benis)

This keeps one shared backbone and optimizes both tasks jointly.
"""
from __future__ import annotations

import argparse
import csv
import json
import logging
import os
import random
import time
from collections import defaultdict
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
log = logging.getLogger("prey_v3")

REPO = Path(__file__).resolve().parent.parent
DATASET = REPO / "dataset"
DEFAULT_CROPS = "crops_yolo11x_rotcrop"
DEFAULT_OUT = REPO / "models" / "prey_v3"

IMG_SIZE = 224
PREY_TRAIN_FRAC = 0.60
PREY_VAL_FRAC = 0.20
CAT_MAP = {"mazge": 0, "benis": 1}


def get_train_tf():
    return transforms.Compose([
        transforms.Resize((IMG_SIZE, IMG_SIZE)),
        transforms.RandomHorizontalFlip(p=0.5),
        transforms.RandomAffine(degrees=5, translate=(0.05, 0.05),
                                scale=(0.95, 1.05)),
        transforms.ColorJitter(brightness=0.2, contrast=0.2),
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


def restratify_prey_splits(bursts_meta: dict[str, dict], seed: int = 1337) -> dict[str, str]:
    out: dict[str, str] = {}
    prey_bursts = sorted(b for b, m in bursts_meta.items()
                         if m["effective_prey"] == "1")
    rng = random.Random(seed)
    rng.shuffle(prey_bursts)
    n_train = int(round(len(prey_bursts) * PREY_TRAIN_FRAC))
    n_val = int(round(len(prey_bursts) * PREY_VAL_FRAC))
    for i, bid in enumerate(prey_bursts):
        if i < n_train:
            out[bid] = "train"
        elif i < n_train + n_val:
            out[bid] = "val"
        else:
            out[bid] = "test"
    for bid, m in bursts_meta.items():
        if bid not in out:
            out[bid] = m["original_split"] or "train"
    return out


class PreyBurstFrames(Dataset):
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
        cat_id = row.get("cat_id", "")
        cat_label = CAT_MAP.get(cat_id, -1)
        return {
            "image": img,
            "prey_label": torch.tensor(float(row["effective_prey"]), dtype=torch.float32),
            "cat_label": torch.tensor(cat_label, dtype=torch.long),
            "burst_id": bid,
            "frame_idx": int(row["frame_idx"]),
        }


class BurstEvalFrames(Dataset):
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
        cat_id = row.get("cat_id", "")
        cat_label = CAT_MAP.get(cat_id, -1)
        return {
            "image": img,
            "prey_label": torch.tensor(float(row["effective_prey"]), dtype=torch.float32),
            "cat_label": torch.tensor(cat_label, dtype=torch.long),
            "burst_id": row["burst_id"],
            "frame_idx": int(row["frame_idx"]),
        }


class PreyV3Classifier(nn.Module):
    def __init__(self):
        super().__init__()
        backbone = efficientnet_b0(weights=EfficientNet_B0_Weights.DEFAULT)
        self.features = backbone.features
        self.avgpool = backbone.avgpool
        feat_dim = backbone.classifier[1].in_features

        self.frame_emb = nn.Embedding(10, 16)
        head_in = feat_dim + 16

        self.prey_head = nn.Sequential(
            nn.Dropout(0.2),
            nn.Linear(head_in, 1),
        )
        self.cat_head = nn.Sequential(
            nn.Dropout(0.2),
            nn.Linear(head_in, 2),
        )

    def freeze_backbone(self, freeze: bool = True):
        for p in self.features.parameters():
            p.requires_grad = not freeze

    def forward(self, x, frame_idx):
        feat = self.features(x)
        feat = self.avgpool(feat).flatten(1)
        emb = self.frame_emb(frame_idx.clamp(min=0, max=9))
        feat = torch.cat([feat, emb], dim=1)
        prey_logit = self.prey_head(feat).squeeze(-1)
        cat_logits = self.cat_head(feat)
        return prey_logit, cat_logits


@torch.no_grad()
def evaluate(model, loader, device, threshold=0.5):
    model.eval()
    rows = []
    loss_sum = 0.0
    n = 0
    for batch in loader:
        imgs = batch["image"].to(device)
        prey_labels = batch["prey_label"].to(device)
        cat_labels = batch["cat_label"].to(device)
        frame_idx = batch["frame_idx"].to(device)

        prey_logits, cat_logits = model(imgs, frame_idx)
        prey_loss = F.binary_cross_entropy_with_logits(prey_logits, prey_labels)

        cat_mask = cat_labels >= 0
        if cat_mask.any():
            cat_loss = F.cross_entropy(cat_logits[cat_mask], cat_labels[cat_mask])
            cat_pred = cat_logits[cat_mask].argmax(dim=1)
            cat_acc = (cat_pred == cat_labels[cat_mask]).float().mean().item()
        else:
            cat_loss = torch.tensor(0.0, device=device)
            cat_acc = 0.0

        total_loss = prey_loss + cat_loss
        loss_sum += total_loss.item() * imgs.size(0)
        n += imgs.size(0)

        prey_probs = torch.sigmoid(prey_logits).cpu().numpy()
        cat_probs = torch.softmax(cat_logits, dim=1).cpu().numpy()
        for bid, fidx, prey_p, cprob, y in zip(batch["burst_id"],
                                               batch["frame_idx"].numpy(),
                                               prey_probs,
                                               cat_probs,
                                               prey_labels.cpu().numpy()):
            rows.append({
                "burst_id": bid,
                "frame_idx": int(fidx),
                "prey_prob": float(prey_p),
                "p_mazge": float(cprob[0]),
                "p_benis": float(cprob[1]),
                "label": int(y),
            })

    frame_preds = [1 if r["prey_prob"] >= threshold else 0 for r in rows]
    ftp = sum(1 for r, p in zip(rows, frame_preds) if p == 1 and r["label"] == 1)
    ffp = sum(1 for r, p in zip(rows, frame_preds) if p == 1 and r["label"] == 0)
    ffn = sum(1 for r, p in zip(rows, frame_preds) if p == 0 and r["label"] == 1)
    ftn = sum(1 for r, p in zip(rows, frame_preds) if p == 0 and r["label"] == 0)
    f_prec = ftp / max(ftp + ffp, 1)
    f_rec = ftp / max(ftp + ffn, 1)
    f_f1 = 2 * f_prec * f_rec / max(f_prec + f_rec, 1e-8)

    by_burst: dict[str, dict] = defaultdict(lambda: {
        "prey_prob": 0.0, "label": 0, "p_mazge": 0.0, "n": 0
    })
    for r in rows:
        agg = by_burst[r["burst_id"]]
        agg["prey_prob"] = max(agg["prey_prob"], r["prey_prob"])
        agg["label"] = max(agg["label"], r["label"])
        agg["p_mazge"] += r["p_mazge"]
        agg["n"] += 1

    btp = bfp = bfn = btn = 0
    for b in by_burst.values():
        pred = 1 if b["prey_prob"] >= threshold else 0
        y = b["label"]
        if pred == 1 and y == 1:
            btp += 1
        elif pred == 1 and y == 0:
            bfp += 1
        elif pred == 0 and y == 1:
            bfn += 1
        else:
            btn += 1
    b_prec = btp / max(btp + bfp, 1)
    b_rec = btp / max(btp + bfn, 1)
    b_f1 = 2 * b_prec * b_rec / max(b_prec + b_rec, 1e-8)

    return {
        "loss": loss_sum / max(n, 1),
        "frame_prec": f_prec,
        "frame_rec": f_rec,
        "frame_f1": f_f1,
        "frame_tp": ftp,
        "frame_fp": ffp,
        "frame_fn": ffn,
        "frame_tn": ftn,
        "burst_prec": b_prec,
        "burst_rec": b_rec,
        "burst_f1": b_f1,
        "burst_tp": btp,
        "burst_fp": bfp,
        "burst_fn": bfn,
        "burst_tn": btn,
        "rows": rows,
    }


def load_rows(crops_dir: Path) -> list[dict]:
    bursts_meta = {r["burst_id"]: r for r in csv.DictReader(open(DATASET / "bursts.csv"))}
    dedup: dict[tuple[str, str, str], dict] = {}
    for r in csv.DictReader(open(crops_dir / "_index.csv")):
        key = (r["burst_id"], r["frame_idx"], r["image_id"])
        dedup[key] = r

    rows = []
    for r in dedup.values():
        if not r.get("body_path"):
            continue
        if r.get("effective_prey") not in ("0", "1"):
            continue
        b = bursts_meta.get(r["burst_id"], {})
        cat_id = (b.get("cat_id") or "").strip().lower()
        out = dict(r)
        out["cat_id"] = cat_id
        rows.append(out)
    return rows


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--crops", default=DEFAULT_CROPS)
    ap.add_argument("--tag", default="bodyA")
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--freeze-epochs", type=int, default=5)
    ap.add_argument("--bs", type=int, default=32)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--lr-finetune", type=float, default=1e-4)
    ap.add_argument("--weight-decay", type=float, default=1e-4)
    ap.add_argument("--cat-loss-weight", type=float, default=0.30)
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

    rows = load_rows(crops_dir)
    bursts_meta: dict[str, dict] = {}
    for r in rows:
        b = r["burst_id"]
        if b not in bursts_meta:
            bursts_meta[b] = {
                "effective_prey": r["effective_prey"],
                "original_split": r["split"],
            }

    split_assign = restratify_prey_splits(bursts_meta, args.seed)
    (out_dir / "split_assignment.json").write_text(
        json.dumps(split_assign, indent=2, sort_keys=True)
    )

    by_split = {"train": defaultdict(list), "val": defaultdict(list), "test": defaultdict(list)}
    for r in rows:
        by_split[split_assign[r["burst_id"]]][r["burst_id"]].append(r)

    split_summary = {}
    for sp in ("train", "val", "test"):
        bursts = list(by_split[sp].keys())
        prey_bursts = [b for b in bursts if bursts_meta[b]["effective_prey"] == "1"]
        split_summary[sp] = {
            "total_bursts": len(bursts),
            "prey_bursts": len(prey_bursts),
            "total_frames": sum(len(by_split[sp][b]) for b in bursts),
        }
        log.info("%s: %d bursts (%d prey)", sp, len(bursts), len(prey_bursts))

    train_ds = PreyBurstFrames(list(by_split["train"].keys()), by_split["train"], get_train_tf())
    val_ds = BurstEvalFrames(by_split["val"], get_eval_tf())
    test_ds = BurstEvalFrames(by_split["test"], get_eval_tf())

    train_loader = DataLoader(train_ds, batch_size=args.bs, shuffle=True,
                              num_workers=2, pin_memory=True, persistent_workers=True)
    val_loader = DataLoader(val_ds, batch_size=args.bs, shuffle=False,
                            num_workers=2, pin_memory=True, persistent_workers=True)
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

    n_pos = split_summary["train"]["prey_bursts"]
    n_neg = split_summary["train"]["total_bursts"] - n_pos
    pos_weight = torch.tensor([max(n_neg / max(n_pos, 1), 1.0)], device=device)

    model = PreyV3Classifier().to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr,
                                  weight_decay=args.weight_decay)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimizer, T_max=max(args.epochs - args.freeze_epochs, 1)
    )

    log_f = open(out_dir / "train_log.csv", "w", newline="")
    log_w = csv.writer(log_f)
    log_w.writerow([
        "epoch", "train_loss", "val_loss", "val_frame_f1", "val_burst_f1",
        "val_burst_tp", "val_burst_fp", "val_burst_fn", "lr", "secs", "backbone_frozen"
    ])

    best_loss = float("inf")
    best_burst_f1 = 0.0
    history = []

    for epoch in range(1, args.epochs + 1):
        frozen = epoch <= args.freeze_epochs
        if epoch == 1:
            model.freeze_backbone(True)
        elif epoch == args.freeze_epochs + 1:
            model.freeze_backbone(False)
            for g in optimizer.param_groups:
                g["lr"] = args.lr_finetune
            log.info("Unfreezing backbone, lr -> %g", args.lr_finetune)

        t0 = time.time()
        model.train()
        train_loss_sum = 0.0
        n_batches = 0
        for batch in train_loader:
            imgs = batch["image"].to(device)
            prey_labels = batch["prey_label"].to(device)
            cat_labels = batch["cat_label"].to(device)
            frame_idx = batch["frame_idx"].to(device)

            prey_logits, cat_logits = model(imgs, frame_idx)
            prey_loss = F.binary_cross_entropy_with_logits(
                prey_logits, prey_labels, pos_weight=pos_weight
            )
            cat_mask = cat_labels >= 0
            if cat_mask.any():
                cat_loss = F.cross_entropy(cat_logits[cat_mask], cat_labels[cat_mask])
            else:
                cat_loss = torch.tensor(0.0, device=device)
            loss = prey_loss + args.cat_loss_weight * cat_loss

            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            optimizer.step()
            train_loss_sum += loss.item()
            n_batches += 1

        if not frozen:
            scheduler.step()

        val = evaluate(model, val_loader, device, threshold=0.5)
        elapsed = time.time() - t0
        lr = optimizer.param_groups[0]["lr"]
        avg_train_loss = train_loss_sum / max(n_batches, 1)

        log_w.writerow([
            epoch, f"{avg_train_loss:.4f}", f"{val['loss']:.4f}",
            f"{val['frame_f1']:.3f}", f"{val['burst_f1']:.3f}",
            val["burst_tp"], val["burst_fp"], val["burst_fn"],
            f"{lr:.6f}", f"{elapsed:.1f}", int(frozen)
        ])
        log_f.flush()

        history.append({
            "epoch": epoch,
            "train_loss": avg_train_loss,
            "val_loss": val["loss"],
            "val_frame_f1": val["frame_f1"],
            "val_burst_f1": val["burst_f1"],
            "val_burst_tp": val["burst_tp"],
            "val_burst_fp": val["burst_fp"],
            "val_burst_fn": val["burst_fn"],
            "lr": lr,
            "secs": elapsed,
            "backbone_frozen": frozen,
        })

        ckpt = {
            "model": model.state_dict(),
            "epoch": epoch,
            "args": vars(args),
            "val_metrics": {k: v for k, v in val.items() if k != "rows"},
            "split_summary": split_summary,
            "history": history,
        }
        torch.save(ckpt, out_dir / "last.pt")
        if val["loss"] < best_loss:
            best_loss = val["loss"]
            torch.save(ckpt, out_dir / "best_loss.pt")
        if val["burst_f1"] > best_burst_f1:
            best_burst_f1 = val["burst_f1"]
            torch.save(ckpt, out_dir / "best_burst_f1.pt")

        (out_dir / "metrics_live.json").write_text(json.dumps({
            "current_epoch": epoch,
            "total_epochs": args.epochs,
            "best_loss": best_loss,
            "best_burst_f1": best_burst_f1,
            "split_summary": split_summary,
            "history": history,
        }, indent=2))

        log.info("[%3d/%d] train_loss=%.3f val_loss=%.3f burstF1=%.3f tp/fp/fn=%d/%d/%d %s %.0fs",
                 epoch, args.epochs, avg_train_loss, val["loss"], val["burst_f1"],
                 val["burst_tp"], val["burst_fp"], val["burst_fn"],
                 "frozen" if frozen else "tune", elapsed)

    best_ckpt = torch.load(out_dir / "best_burst_f1.pt", map_location=device,
                           weights_only=False)
    model.load_state_dict(best_ckpt["model"])
    test = evaluate(model, test_loader, device, threshold=0.5)

    (out_dir / "test_metrics.json").write_text(json.dumps({
        "checkpoint": "best_burst_f1",
        "ckpt_epoch": best_ckpt["epoch"],
        "frame": {
            "precision": test["frame_prec"],
            "recall": test["frame_rec"],
            "f1": test["frame_f1"],
            "tp": test["frame_tp"],
            "fp": test["frame_fp"],
            "fn": test["frame_fn"],
            "tn": test["frame_tn"],
        },
        "burst": {
            "precision": test["burst_prec"],
            "recall": test["burst_rec"],
            "f1": test["burst_f1"],
            "tp": test["burst_tp"],
            "fp": test["burst_fp"],
            "fn": test["burst_fn"],
            "tn": test["burst_tn"],
        },
        "split_summary": split_summary,
    }, indent=2))

    with open(out_dir / "test_predictions.csv", "w", newline="") as f:
        w = csv.DictWriter(
            f,
            fieldnames=["burst_id", "frame_idx", "prey_prob", "p_mazge", "p_benis", "label"],
        )
        w.writeheader()
        w.writerows(test["rows"])

    log_f.close()
    log.info("Done. Saved to %s", out_dir)


if __name__ == "__main__":
    main()
