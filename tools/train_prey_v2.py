#!/usr/bin/env python3
"""Train the v2 prey classifier on YOLO11x body crops.

Architecture:
  EfficientNet-B0 (ImageNet pretrained, 5M params)
  → single sigmoid head for P(prey)

Training discipline (lessons from v1 failure):
  - Burst-level sampler — sample a burst then a frame, not a frame.
    Removes "16 events × 10 frames treated as 160 independent samples"
    overfitting.
  - One imbalance lever: pos_weight in BCE. No oversampling, no
    confidence weighting.
  - Re-stratified split: ensure val + test have enough prey bursts
    for early-stopping to be meaningful.
  - Conservative augmentations (HFlip + small affine + ColorJitter
    only). No RandomErasing — could blank out the prey.
  - 5 frozen-backbone epochs → 25 unfrozen epochs at cosine lr.
  - Eval on per-burst metric: max P(prey) over the burst's frames
    is the burst score (matches product behaviour).

Outputs (models/prey_v2/<run_tag>/):
  best_loss.pt              checkpoint with lowest val_loss
  best_burst_recall.pt      checkpoint with highest val burst recall
  last.pt                   final epoch
  train_log.csv             per-epoch metrics
  metrics_live.json         consumed by the dashboard
  dashboard.html            live training progress
  split_assignment.json     which burst -> which split (reproducible)

Usage:
  cd Mazge && uv run python tools/train_prey_v2.py
  uv run python tools/train_prey_v2.py --epochs 40 --tag bodyA
"""
from __future__ import annotations

import argparse
import csv
import hashlib
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
from torchvision.models import efficientnet_b0, EfficientNet_B0_Weights

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s %(levelname)-5s %(message)s",
                    datefmt="%H:%M:%S")
log = logging.getLogger("prey_v2")

REPO = Path(__file__).resolve().parent.parent
DATASET = REPO / "dataset"
DEFAULT_CROPS = "crops_yolo11x_rotcrop"
DEFAULT_OUT = REPO / "models" / "prey_v2"

IMG_SIZE = 224
PREY_TRAIN_FRAC = 0.60
PREY_VAL_FRAC = 0.20  # remaining 0.20 -> test
# We keep non-prey bursts on their existing split (deterministic
# SHA1(burst_id) hash) so the train/val/test distribution of the
# negative class is unchanged from v1. We only re-stratify the
# rare prey bursts so val + test have enough positives to be
# meaningful.


# ── Dataset & sampling ──────────────────────────────────────────────

class PreyBurstFrames(Dataset):
    """Group frames by burst. Sampling is at burst level.

    For each __getitem__(burst_idx):
      - sample a random frame from that burst (uniform)
      - return its body crop + the burst's label
    """

    def __init__(self, bursts: list[str], frames_by_burst: dict[str, list[dict]],
                 transform=None):
        self.bursts = bursts
        self.frames_by_burst = frames_by_burst
        self.transform = transform
        self.rng = random.Random(0xc0ffee)

    def __len__(self):
        return len(self.bursts)

    def __getitem__(self, idx):
        bid = self.bursts[idx]
        frames = self.frames_by_burst[bid]
        row = self.rng.choice(frames)
        img_path = DATASET / row["body_path"]
        img = Image.open(img_path).convert("RGB")
        if self.transform:
            img = self.transform(img)
        label = float(row["effective_prey"])
        return {
            "image": img,
            "label": torch.tensor(label),
            "burst_id": bid,
            "frame_idx": int(row["frame_idx"]),
        }


class BurstEvalFrames(Dataset):
    """Flat iterator over ALL body crops (for per-frame & per-burst eval)."""

    def __init__(self, frames_by_burst: dict[str, list[dict]], transform=None):
        self.rows = [r for frames in frames_by_burst.values() for r in frames]
        self.transform = transform

    def __len__(self):
        return len(self.rows)

    def __getitem__(self, idx):
        row = self.rows[idx]
        img_path = DATASET / row["body_path"]
        img = Image.open(img_path).convert("RGB")
        if self.transform:
            img = self.transform(img)
        return {
            "image": img,
            "label": torch.tensor(float(row["effective_prey"])),
            "burst_id": row["burst_id"],
            "frame_idx": int(row["frame_idx"]),
        }


# ── Split assignment ────────────────────────────────────────────────

def restratify_prey_splits(bursts_meta: dict[str, dict],
                           seed: int = 1337) -> dict[str, str]:
    """Build a deterministic per-burst split assignment.

    - Non-prey bursts keep their original split from bursts.csv.
    - Prey bursts are re-shuffled (deterministic) into a 60/20/20
      train/val/test split so val/test get a meaningful number of
      positives.
    """
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


# ── Augmentations ──────────────────────────────────────────────────

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


# ── Model ──────────────────────────────────────────────────────────

class PreyClassifier(nn.Module):
    def __init__(self):
        super().__init__()
        backbone = efficientnet_b0(weights=EfficientNet_B0_Weights.DEFAULT)
        # Replace classifier head with single-logit sigmoid head
        in_features = backbone.classifier[1].in_features
        backbone.classifier = nn.Sequential(
            nn.Dropout(0.2),
            nn.Linear(in_features, 1),
        )
        self.model = backbone

    def freeze_backbone(self, freeze: bool = True):
        for n, p in self.model.named_parameters():
            if not n.startswith("classifier"):
                p.requires_grad = not freeze

    def forward(self, x):
        return self.model(x).squeeze(-1)  # (B,)


# ── Eval ────────────────────────────────────────────────────────────

@torch.no_grad()
def evaluate(model, loader, device, threshold=0.5):
    model.eval()
    rows = []
    for batch in loader:
        imgs = batch["image"].to(device)
        logits = model(imgs)
        probs = torch.sigmoid(logits).cpu().numpy()
        labels = batch["label"].numpy()
        for bid, fidx, p, y in zip(batch["burst_id"],
                                    batch["frame_idx"].numpy(),
                                    probs, labels):
            rows.append({"burst_id": bid, "frame_idx": int(fidx),
                         "prob": float(p), "label": int(y)})

    # Per-frame
    preds = [1 if r["prob"] >= threshold else 0 for r in rows]
    tp = sum(1 for r, p in zip(rows, preds) if p == 1 and r["label"] == 1)
    fp = sum(1 for r, p in zip(rows, preds) if p == 1 and r["label"] == 0)
    fn = sum(1 for r, p in zip(rows, preds) if p == 0 and r["label"] == 1)
    tn = sum(1 for r, p in zip(rows, preds) if p == 0 and r["label"] == 0)
    f_prec = tp / max(tp + fp, 1)
    f_rec = tp / max(tp + fn, 1)
    f_f1 = 2 * f_prec * f_rec / max(f_prec + f_rec, 1e-8)

    # Per-burst (max prob)
    by_burst: dict[str, dict] = defaultdict(lambda: {"prob": 0.0, "label": 0})
    for r in rows:
        by_burst[r["burst_id"]]["prob"] = max(
            by_burst[r["burst_id"]]["prob"], r["prob"])
        by_burst[r["burst_id"]]["label"] = max(
            by_burst[r["burst_id"]]["label"], r["label"])

    btp = sum(1 for b in by_burst.values()
              if b["prob"] >= threshold and b["label"] == 1)
    bfp = sum(1 for b in by_burst.values()
              if b["prob"] >= threshold and b["label"] == 0)
    bfn = sum(1 for b in by_burst.values()
              if b["prob"] < threshold and b["label"] == 1)
    btn = sum(1 for b in by_burst.values()
              if b["prob"] < threshold and b["label"] == 0)
    b_prec = btp / max(btp + bfp, 1)
    b_rec = btp / max(btp + bfn, 1)
    b_f1 = 2 * b_prec * b_rec / max(b_prec + b_rec, 1e-8)

    # Loss (use raw)
    import math
    bce = 0.0
    for r in rows:
        p = max(min(r["prob"], 1 - 1e-7), 1e-7)
        y = r["label"]
        bce += -(y * math.log(p) + (1 - y) * math.log(1 - p))
    bce /= max(len(rows), 1)

    return {
        "frame_prec": f_prec, "frame_rec": f_rec, "frame_f1": f_f1,
        "frame_tp": tp, "frame_fp": fp, "frame_fn": fn, "frame_tn": tn,
        "burst_prec": b_prec, "burst_rec": b_rec, "burst_f1": b_f1,
        "burst_tp": btp, "burst_fp": bfp, "burst_fn": bfn, "burst_tn": btn,
        "n_frames": len(rows), "n_bursts": len(by_burst),
        "loss": bce,
        "rows": rows,
    }


# ── Dashboard ──────────────────────────────────────────────────────

DASH_CSS = """
body{background:#1a1a1a;color:#eee;font-family:-apple-system,monospace;
margin:0;padding:16px;}
h1{margin:0 0 8px 0;font-size:18px;}
h2{font-size:14px;margin:16px 0 6px 0;border-bottom:1px solid #444;
padding-bottom:4px;}
.bar{background:#333;height:18px;border-radius:9px;overflow:hidden;margin:8px 0;}
.fill{background:linear-gradient(90deg,#0c8,#0fa);height:100%;}
.stat{display:inline-block;background:#262626;padding:6px 12px;
border-radius:6px;margin:4px 6px 4px 0;}
.stat b{color:#0fa;}
.stat.warn b{color:#fa0;}
.stat.bad b{color:#f55;}
.updated{color:#888;font-size:11px;float:right;}
table{border-collapse:collapse;margin-top:8px;font-size:12px;}
th,td{padding:2px 8px;border-bottom:1px solid #333;text-align:right;}
th{background:#222;}
canvas{background:#222;border-radius:6px;margin:8px 0;}
"""


def write_dashboard(out_dir: Path, state: dict, args, tag: str) -> None:
    history = state["history"]
    epoch = state["current_epoch"]
    total = state["total_epochs"]
    pct = (epoch / total * 100) if total else 0
    pieces = [
        "<!doctype html><html><head><meta charset='utf-8'>",
        "<meta http-equiv='refresh' content='10'>",
        f"<title>prey_v2/{tag}</title>",
        f"<style>{DASH_CSS}</style></head><body>",
        f"<h1>prey_v2 / {tag} — EfficientNet-B0 on YOLO11x body crops</h1>",
        f"<span class='updated'>refresh 10 s · {time.strftime('%H:%M:%S')}</span>",
        f"<div class='bar'><div class='fill' style='width:{pct:.1f}%'></div></div>",
        f"<div class='stat'>Epoch <b>{epoch}/{total}</b></div>",
        f"<div class='stat'>Best val_loss <b>{state.get('best_loss', float('inf')):.3f}</b> @ epoch <b>{state.get('best_loss_epoch', '-')}</b></div>",
        f"<div class='stat'>Best burst F1 <b>{state.get('best_burst_f1', 0):.3f}</b> @ epoch <b>{state.get('best_burst_f1_epoch', '-')}</b></div>",
        f"<div class='stat'>Status: <b>{state.get('status', 'training')}</b></div>",
    ]

    # Dataset summary
    s = state.get("split_summary", {})
    if s:
        pieces.append("<h2>Dataset (re-stratified)</h2>")
        for k in ("train", "val", "test"):
            vv = s.get(k, {})
            cls = ("warn" if vv.get('prey_bursts', 0) < 3 else "")
            pieces.append(
                f"<div class='stat {cls}'>{k}: prey "
                f"<b>{vv.get('prey_bursts', 0)}</b>/{vv.get('total_bursts', 0)} "
                f"bursts, {vv.get('total_frames', 0)} frames</div>")

    if history:
        last = history[-1]
        pieces.append("<h2>Latest epoch metrics</h2>")
        pieces.append(f"<div class='stat'>train_loss "
                      f"<b>{last['train_loss']:.3f}</b></div>")
        pieces.append(f"<div class='stat'>val_loss "
                      f"<b>{last['val_loss']:.3f}</b></div>")
        pieces.append(f"<div class='stat'>val burst <b>"
                      f"P={last['val_burst_prec']:.2f} "
                      f"R={last['val_burst_rec']:.2f} "
                      f"F1={last['val_burst_f1']:.2f}</b></div>")
        pieces.append(f"<div class='stat'>val frame "
                      f"<b>F1={last['val_frame_f1']:.2f}</b></div>")
        pieces.append(f"<div class='stat'>val TP/FP/FN bursts "
                      f"<b>{last['val_burst_tp']}/"
                      f"{last['val_burst_fp']}/"
                      f"{last['val_burst_fn']}</b></div>")
        pieces.append(f"<div class='stat'>lr "
                      f"<b>{last['lr']:.2e}</b></div>")
        pieces.append(f"<div class='stat'>{last['secs']:.0f}s/epoch</div>")

    if history:
        pieces.append("<h2>Per-epoch log</h2>")
        pieces.append("<table><tr><th>ep</th><th>train_loss</th>"
                      "<th>val_loss</th>"
                      "<th>frame F1</th><th>burst F1</th>"
                      "<th>burst P/R</th>"
                      "<th>burst TP/FP/FN</th>"
                      "<th>frozen</th><th>lr</th><th>s</th></tr>")
        for h in history[-60:]:
            pieces.append(
                f"<tr><td>{h['epoch']}</td>"
                f"<td>{h['train_loss']:.3f}</td>"
                f"<td>{h['val_loss']:.3f}</td>"
                f"<td>{h['val_frame_f1']:.2f}</td>"
                f"<td>{h['val_burst_f1']:.2f}</td>"
                f"<td>{h['val_burst_prec']:.2f} / {h['val_burst_rec']:.2f}</td>"
                f"<td>{h['val_burst_tp']}/{h['val_burst_fp']}/{h['val_burst_fn']}</td>"
                f"<td>{'frozen' if h.get('backbone_frozen') else 'tune'}</td>"
                f"<td>{h['lr']:.1e}</td>"
                f"<td>{h['secs']:.0f}</td></tr>")
        pieces.append("</table>")

    pieces.append("</body></html>")
    dash = out_dir / "dashboard.html"
    tmp = dash.with_suffix(".html.tmp")
    tmp.write_text("".join(pieces))
    tmp.replace(dash)


# ── Main ───────────────────────────────────────────────────────────

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
    ap.add_argument("--seed", type=int, default=1337)
    ap.add_argument("--device", default="auto")
    args = ap.parse_args()

    out_dir = DEFAULT_OUT / args.tag
    out_dir.mkdir(parents=True, exist_ok=True)
    log.info("Output: %s", out_dir)
    log.info("Dashboard: file://%s/dashboard.html", out_dir)

    crops_dir = DATASET / args.crops
    if not (crops_dir / "_index.csv").exists():
        raise SystemExit(f"No _index.csv in {crops_dir}")

    # Load crops index + restratify prey splits
    log.info("Loading crops index...")
    rows = [r for r in csv.DictReader(open(crops_dir / "_index.csv"))
            if r["body_path"]]
    log.info("  %d body crops across %d bursts",
             len(rows), len({r["burst_id"] for r in rows}))

    bursts_meta: dict[str, dict] = {}
    for r in rows:
        b = r["burst_id"]
        if b not in bursts_meta:
            bursts_meta[b] = {
                "effective_prey": r["effective_prey"],
                "human_subject": r["human_subject"],
                "human_direction": r["human_direction"],
                "original_split": r["split"],
            }
    split_assign = restratify_prey_splits(bursts_meta, args.seed)
    (out_dir / "split_assignment.json").write_text(
        json.dumps(split_assign, indent=2, sort_keys=True))

    # Build frames_by_burst per split
    by_split: dict[str, dict[str, list[dict]]] = {
        "train": defaultdict(list),
        "val": defaultdict(list),
        "test": defaultdict(list),
    }
    for r in rows:
        sp = split_assign[r["burst_id"]]
        by_split[sp][r["burst_id"]].append(r)

    # Summary
    split_summary = {}
    for sp in ("train", "val", "test"):
        bursts = list(by_split[sp].keys())
        prey_bursts = [b for b in bursts
                       if bursts_meta[b]["effective_prey"] == "1"]
        frames = sum(len(by_split[sp][b]) for b in bursts)
        prey_frames = sum(len(by_split[sp][b]) for b in prey_bursts)
        split_summary[sp] = {
            "total_bursts": len(bursts),
            "prey_bursts": len(prey_bursts),
            "total_frames": frames,
            "prey_frames": prey_frames,
        }
        log.info("  %s: %d bursts (%d prey), %d frames (%d prey)",
                 sp, len(bursts), len(prey_bursts), frames, prey_frames)

    # Datasets
    train_ds = PreyBurstFrames(list(by_split["train"].keys()),
                                by_split["train"], get_train_tf())
    val_ds = BurstEvalFrames(by_split["val"], get_eval_tf())
    test_ds = BurstEvalFrames(by_split["test"], get_eval_tf())

    train_loader = DataLoader(train_ds, batch_size=args.bs, shuffle=True,
                              num_workers=2, pin_memory=True,
                              persistent_workers=True)
    val_loader = DataLoader(val_ds, batch_size=args.bs, shuffle=False,
                            num_workers=2, pin_memory=True,
                            persistent_workers=True)
    test_loader = DataLoader(test_ds, batch_size=args.bs, shuffle=False,
                             num_workers=2, pin_memory=True)

    # Device
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

    # Pos weight (per-burst basis — sampler returns one frame per burst,
    # so the imbalance the model sees per epoch matches per-burst rates).
    n_pos = split_summary["train"]["prey_bursts"]
    n_neg = split_summary["train"]["total_bursts"] - n_pos
    pos_weight = torch.tensor([max(n_neg / max(n_pos, 1), 1.0)])
    log.info("pos_weight = %.2f  (train bursts: %d prey / %d total)",
             pos_weight.item(), n_pos, n_pos + n_neg)

    # Model
    model = PreyClassifier().to(device)
    optimiser = torch.optim.AdamW(model.parameters(), lr=args.lr,
                                   weight_decay=args.weight_decay)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
        optimiser, T_max=args.epochs - args.freeze_epochs)

    # Train log csv
    log_path = out_dir / "train_log.csv"
    log_f = open(log_path, "w", newline="")
    log_w = csv.writer(log_f)
    log_w.writerow(["epoch", "train_loss", "val_loss",
                    "val_frame_f1", "val_burst_f1",
                    "val_burst_prec", "val_burst_rec",
                    "val_burst_tp", "val_burst_fp", "val_burst_fn",
                    "lr", "secs", "backbone_frozen"])

    state = {
        "tag": args.tag,
        "current_epoch": 0,
        "total_epochs": args.epochs,
        "split_summary": split_summary,
        "history": [],
        "best_loss": float("inf"),
        "best_loss_epoch": None,
        "best_burst_f1": 0.0,
        "best_burst_f1_epoch": None,
        "status": "starting",
    }
    write_dashboard(out_dir, state, args, args.tag)

    log.info("Training for %d epochs, %d frozen", args.epochs, args.freeze_epochs)
    state["status"] = "training"
    write_dashboard(out_dir, state, args, args.tag)

    pos_weight_dev = pos_weight.to(device)
    for epoch in range(1, args.epochs + 1):
        # Backbone freeze/unfreeze
        backbone_frozen = epoch <= args.freeze_epochs
        if epoch == 1:
            model.freeze_backbone(True)
        elif epoch == args.freeze_epochs + 1:
            model.freeze_backbone(False)
            # Switch to fine-tune lr
            for g in optimiser.param_groups:
                g["lr"] = args.lr_finetune
            log.info("  unfreezing backbone, lr -> %g", args.lr_finetune)

        model.train()
        t0 = time.time()
        train_loss_sum = 0.0
        n_batches = 0
        for batch in train_loader:
            imgs = batch["image"].to(device)
            labels = batch["label"].to(device)
            logits = model(imgs)
            loss = F.binary_cross_entropy_with_logits(
                logits, labels, pos_weight=pos_weight_dev)
            optimiser.zero_grad(set_to_none=True)
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            optimiser.step()
            train_loss_sum += loss.item()
            n_batches += 1
        avg_train_loss = train_loss_sum / max(n_batches, 1)

        if not backbone_frozen:
            scheduler.step()

        # Eval
        v = evaluate(model, val_loader, device, threshold=0.5)
        elapsed = time.time() - t0
        cur_lr = optimiser.param_groups[0]["lr"]

        log_w.writerow([
            epoch, f"{avg_train_loss:.4f}", f"{v['loss']:.4f}",
            f"{v['frame_f1']:.3f}", f"{v['burst_f1']:.3f}",
            f"{v['burst_prec']:.3f}", f"{v['burst_rec']:.3f}",
            v['burst_tp'], v['burst_fp'], v['burst_fn'],
            f"{cur_lr:.6f}", f"{elapsed:.1f}", int(backbone_frozen),
        ])
        log_f.flush()

        log.info("[%3d/%d] train_loss=%.3f  val_loss=%.3f  "
                 "burst F1=%.2f P=%.2f R=%.2f  tp/fp/fn=%d/%d/%d  "
                 "%s %.0fs",
                 epoch, args.epochs, avg_train_loss, v["loss"],
                 v["burst_f1"], v["burst_prec"], v["burst_rec"],
                 v["burst_tp"], v["burst_fp"], v["burst_fn"],
                 "frozen" if backbone_frozen else "tune ",
                 elapsed)

        state["current_epoch"] = epoch
        state["history"].append({
            "epoch": epoch,
            "train_loss": avg_train_loss,
            "val_loss": v["loss"],
            "val_frame_f1": v["frame_f1"],
            "val_burst_f1": v["burst_f1"],
            "val_burst_prec": v["burst_prec"],
            "val_burst_rec": v["burst_rec"],
            "val_burst_tp": v["burst_tp"],
            "val_burst_fp": v["burst_fp"],
            "val_burst_fn": v["burst_fn"],
            "lr": cur_lr,
            "secs": elapsed,
            "backbone_frozen": backbone_frozen,
        })

        # Save checkpoints
        ckpt = {
            "model": model.state_dict(),
            "epoch": epoch,
            "args": vars(args),
            "val_metrics": {k: vv for k, vv in v.items() if k != "rows"},
            "split_summary": split_summary,
            "pos_weight": pos_weight.item(),
        }
        torch.save(ckpt, out_dir / "last.pt")
        if v["loss"] < state["best_loss"]:
            state["best_loss"] = v["loss"]
            state["best_loss_epoch"] = epoch
            torch.save(ckpt, out_dir / "best_loss.pt")
        if v["burst_f1"] > state["best_burst_f1"]:
            state["best_burst_f1"] = v["burst_f1"]
            state["best_burst_f1_epoch"] = epoch
            torch.save(ckpt, out_dir / "best_burst_f1.pt")

        write_dashboard(out_dir, state, args, args.tag)
        (out_dir / "metrics_live.json").write_text(json.dumps({
            "current_epoch": epoch,
            "total_epochs": args.epochs,
            "history": state["history"],
            "best_loss": state["best_loss"],
            "best_burst_f1": state["best_burst_f1"],
        }))

    state["status"] = "evaluating test"
    write_dashboard(out_dir, state, args, args.tag)

    # Final test eval (use best by burst F1)
    log.info("Loading best_burst_f1 checkpoint for test eval...")
    best_ckpt = torch.load(out_dir / "best_burst_f1.pt", map_location=device,
                           weights_only=False)
    model.load_state_dict(best_ckpt["model"])
    t = evaluate(model, test_loader, device, threshold=0.5)
    log.info("TEST best_burst_f1 ckpt @ ep %d:", best_ckpt["epoch"])
    log.info("  frame F1=%.2f P=%.2f R=%.2f tp/fp/fn=%d/%d/%d",
             t["frame_f1"], t["frame_prec"], t["frame_rec"],
             t["frame_tp"], t["frame_fp"], t["frame_fn"])
    log.info("  burst F1=%.2f P=%.2f R=%.2f tp/fp/fn=%d/%d/%d",
             t["burst_f1"], t["burst_prec"], t["burst_rec"],
             t["burst_tp"], t["burst_fp"], t["burst_fn"])

    (out_dir / "test_metrics.json").write_text(json.dumps({
        "checkpoint": "best_burst_f1",
        "ckpt_epoch": best_ckpt["epoch"],
        "threshold": 0.5,
        "frame": {k: t[k] for k in ("frame_prec", "frame_rec", "frame_f1",
                                     "frame_tp", "frame_fp",
                                     "frame_fn", "frame_tn")},
        "burst": {k: t[k] for k in ("burst_prec", "burst_rec", "burst_f1",
                                     "burst_tp", "burst_fp",
                                     "burst_fn", "burst_tn")},
        "n_frames": t["n_frames"],
        "n_bursts": t["n_bursts"],
    }, indent=2))

    # Save per-frame test predictions for downstream analysis
    with open(out_dir / "test_predictions.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["burst_id", "frame_idx", "prob", "label"])
        w.writeheader()
        w.writerows(t["rows"])

    state["status"] = "done"
    write_dashboard(out_dir, state, args, args.tag)
    log_f.close()
    log.info("Done. Dashboard: file://%s/dashboard.html", out_dir)


if __name__ == "__main__":
    main()
