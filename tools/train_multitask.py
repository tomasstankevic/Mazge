#!/usr/bin/env python3
"""Train the combined Mazge multitask model (prey + cat-id + subject + direction).

One EfficientNet-B0 backbone, four heads:

  prey_head       1  logit   binary prey            (PRIMARY task)
  cat_head        2  logits  cat identity mazge/benis
  subject_head    4  logits  empty / cat / human / other   ("no-cat" categories)
  direction_head  2  logits  entering / exiting

Design constraint (owner): the direction head must NOT compromise prey
detection. So the direction head is fed **stop-gradient (detached)** backbone
features by default — it still produces a direction output from the shared
representation, but its gradients never flow into the backbone that prey/cat/
subject shape. Pass --direction-grad to let direction co-train the backbone.

Data: dataset/<crops>/_index.csv (body crops) joined with dataset/bursts.csv for
the burst-level human labels. Split is the stable per-burst hash split from
bursts.csv (NOT restratified) so results are directly comparable across runs and
against a baseline checkpoint evaluated on the same split.

Usage:
  uv run python tools/train_multitask.py --epochs 25 --tag combined_v1
  uv run python tools/train_multitask.py --smoke            # 2 epochs, quick
  uv run python tools/train_multitask.py --baseline models/prey_v3/bodyA/best_burst_f1.pt
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
log = logging.getLogger("multitask")

REPO = Path(__file__).resolve().parent.parent
DATASET = REPO / "dataset"
DEFAULT_CROPS = "crops_yolo11x_rotcrop"
DEFAULT_OUT = REPO / "models" / "multitask"

IMG_SIZE = 224
CAT_MAP = {"mazge": 0, "benis": 1}
SUBJ_MAP = {"empty": 0, "cat": 1, "human": 2, "other": 3}
DIR_MAP = {"entering": 0, "exiting": 1}
INT_TO_CAT = {0: "mazge", 1: "benis"}
INT_TO_SUBJ = {0: "empty", 1: "cat", 2: "human", 3: "other"}
INT_TO_DIR = {0: "entering", 1: "exiting"}


def get_train_tf():
    return transforms.Compose([
        transforms.Resize((IMG_SIZE, IMG_SIZE)),
        transforms.RandomHorizontalFlip(p=0.5),
        transforms.RandomAffine(degrees=5, translate=(0.05, 0.05), scale=(0.95, 1.05)),
        transforms.ColorJitter(brightness=0.2, contrast=0.2),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ])


def get_eval_tf():
    return transforms.Compose([
        transforms.Resize((IMG_SIZE, IMG_SIZE)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
    ])


def load_rows(crops_dir: Path) -> list[dict]:
    """Join body-crop index with burst-level human labels."""
    bursts_meta = {r["burst_id"]: r for r in csv.DictReader(open(DATASET / "bursts.csv"))}
    dedup: dict[tuple[str, str, str], dict] = {}
    for r in csv.DictReader(open(crops_dir / "_index.csv")):
        dedup[(r["burst_id"], r["frame_idx"], r["image_id"])] = r

    rows = []
    for r in dedup.values():
        if not r.get("body_path"):
            continue
        if r.get("effective_prey") not in ("0", "1"):
            continue
        b = bursts_meta.get(r["burst_id"], {})
        out = dict(r)
        out["cat_label"] = CAT_MAP.get((b.get("cat_id") or "").strip().lower(), -1)
        out["subject_label"] = SUBJ_MAP.get((b.get("human_subject") or "").strip().lower(), -1)
        out["dir_label"] = DIR_MAP.get((b.get("human_direction") or "").strip().lower(), -1)
        out["split"] = b.get("split") or r.get("split") or "train"
        rows.append(out)
    return rows


class BurstFrames(Dataset):
    """Train: sample one random frame per burst each epoch (reduces near-dup overfit)."""

    def __init__(self, bursts: list[str], frames_by_burst: dict[str, list[dict]], tf):
        self.bursts = bursts
        self.frames_by_burst = frames_by_burst
        self.tf = tf
        self.rng = random.Random(0xC0FFEE)

    def __len__(self):
        return len(self.bursts)

    def __getitem__(self, idx):
        row = self.rng.choice(self.frames_by_burst[self.bursts[idx]])
        return self._make(row)

    def _make(self, row):
        img = Image.open(DATASET / row["body_path"]).convert("RGB")
        return {
            "image": self.tf(img),
            "prey_label": torch.tensor(float(row["effective_prey"]), dtype=torch.float32),
            "cat_label": torch.tensor(row["cat_label"], dtype=torch.long),
            "subject_label": torch.tensor(row["subject_label"], dtype=torch.long),
            "dir_label": torch.tensor(row["dir_label"], dtype=torch.long),
            "burst_id": row["burst_id"],
            "frame_idx": int(row["frame_idx"]),
        }


class AllFrames(BurstFrames):
    """Eval: every frame."""

    def __init__(self, frames_by_burst: dict[str, list[dict]], tf):
        self.rows = [r for fs in frames_by_burst.values() for r in fs]
        self.tf = tf

    def __len__(self):
        return len(self.rows)

    def __getitem__(self, idx):
        return self._make(self.rows[idx])


class MazgeMultiTask(nn.Module):
    def __init__(self, direction_grad: bool = False):
        super().__init__()
        backbone = efficientnet_b0(weights=EfficientNet_B0_Weights.DEFAULT)
        self.features = backbone.features
        self.avgpool = backbone.avgpool
        feat_dim = backbone.classifier[1].in_features
        self.frame_emb = nn.Embedding(10, 16)
        head_in = feat_dim + 16
        self.direction_grad = direction_grad

        self.prey_head = nn.Sequential(nn.Dropout(0.2), nn.Linear(head_in, 1))
        self.cat_head = nn.Sequential(nn.Dropout(0.2), nn.Linear(head_in, 2))
        self.subject_head = nn.Sequential(nn.Dropout(0.2), nn.Linear(head_in, 4))
        self.direction_head = nn.Sequential(nn.Dropout(0.2), nn.Linear(head_in, 2))

    def freeze_backbone(self, freeze: bool = True):
        for p in self.features.parameters():
            p.requires_grad = not freeze

    def forward(self, x, frame_idx):
        feat = self.avgpool(self.features(x)).flatten(1)
        emb = self.frame_emb(frame_idx.clamp(min=0, max=9))
        feat = torch.cat([feat, emb], dim=1)
        # Direction gets stop-gradient features unless explicitly allowed to
        # co-train the backbone (protects the primary prey task by default).
        dir_feat = feat if self.direction_grad else feat.detach()
        return (
            self.prey_head(feat).squeeze(-1),
            self.cat_head(feat),
            self.subject_head(feat),
            self.direction_head(dir_feat),
        )


def _masked_ce(logits, labels):
    mask = labels >= 0
    if not mask.any():
        return torch.tensor(0.0, device=logits.device), 0.0, 0
    loss = F.cross_entropy(logits[mask], labels[mask])
    acc = (logits[mask].argmax(1) == labels[mask]).float().mean().item()
    return loss, acc, int(mask.sum())


@torch.no_grad()
def evaluate(model, loader, device, pos_weight, weights, threshold=0.5):
    model.eval()
    rows = []
    loss_sum = n = 0.0
    for batch in loader:
        imgs = batch["image"].to(device)
        prey_y = batch["prey_label"].to(device)
        cat_y = batch["cat_label"].to(device)
        subj_y = batch["subject_label"].to(device)
        dir_y = batch["dir_label"].to(device)
        fidx = batch["frame_idx"].to(device)

        prey_l, cat_l, subj_l, dir_l = model(imgs, fidx)
        prey_loss = F.binary_cross_entropy_with_logits(prey_l, prey_y, pos_weight=pos_weight)
        cat_loss, _, _ = _masked_ce(cat_l, cat_y)
        subj_loss, _, _ = _masked_ce(subj_l, subj_y)
        dir_loss, _, _ = _masked_ce(dir_l, dir_y)
        total = (weights["prey"] * prey_loss + weights["cat"] * cat_loss
                 + weights["subject"] * subj_loss + weights["direction"] * dir_loss)
        loss_sum += total.item() * imgs.size(0)
        n += imgs.size(0)

        prey_p = torch.sigmoid(prey_l).cpu().numpy()
        cat_sm = torch.softmax(cat_l, 1).cpu().numpy()
        subj_pred = subj_l.argmax(1).cpu().numpy()
        dir_pred = dir_l.argmax(1).cpu().numpy()
        for i, bid in enumerate(batch["burst_id"]):
            rows.append({
                "burst_id": bid,
                "prey_prob": float(prey_p[i]), "prey_y": int(prey_y[i].item()),
                "cat_p": cat_sm[i].tolist(), "cat_y": int(cat_y[i].item()),
                "subj_pred": int(subj_pred[i]), "subj_y": int(subj_y[i].item()),
                "dir_pred": int(dir_pred[i]), "dir_y": int(dir_y[i].item()),
            })
    return {"loss": loss_sum / max(n, 1), **_metrics(rows, threshold)}


def _prf(tp, fp, fn):
    p = tp / max(tp + fp, 1)
    r = tp / max(tp + fn, 1)
    f = 2 * p * r / max(p + r, 1e-8)
    return round(p, 4), round(r, 4), round(f, 4)


def _metrics(rows, threshold):
    # --- prey: per-frame ---
    ftp = sum(r["prey_prob"] >= threshold and r["prey_y"] == 1 for r in rows)
    ffp = sum(r["prey_prob"] >= threshold and r["prey_y"] == 0 for r in rows)
    ffn = sum(r["prey_prob"] < threshold and r["prey_y"] == 1 for r in rows)
    fp_, fr_, ff_ = _prf(ftp, ffp, ffn)

    # --- prey: per-burst (max over frames) ---
    bagg = defaultdict(lambda: {"p": 0.0, "y": 0})
    catagg = defaultdict(lambda: {"sm": [0.0, 0.0], "y": -1, "n": 0})
    diragg = defaultdict(lambda: {"pred": Counter(), "y": -1})
    subj_correct = subj_tot = 0
    dir_frame_correct = dir_frame_tot = 0
    cat_frame_correct = cat_frame_tot = 0
    for r in rows:
        b = bagg[r["burst_id"]]
        b["p"] = max(b["p"], r["prey_prob"]); b["y"] = max(b["y"], r["prey_y"])
        if r["subj_y"] >= 0:
            subj_tot += 1; subj_correct += (r["subj_pred"] == r["subj_y"])
        if r["dir_y"] >= 0:
            dir_frame_tot += 1; dir_frame_correct += (r["dir_pred"] == r["dir_y"])
            d = diragg[r["burst_id"]]; d["y"] = r["dir_y"]; d["pred"][r["dir_pred"]] += 1
        if r["cat_y"] >= 0:
            cat_frame_tot += 1; cat_frame_correct += (int(r["cat_p"][1] > r["cat_p"][0]) == r["cat_y"])
            c = catagg[r["burst_id"]]; c["y"] = r["cat_y"]; c["n"] += 1
            c["sm"][0] += r["cat_p"][0]; c["sm"][1] += r["cat_p"][1]

    btp = sum(v["p"] >= threshold and v["y"] == 1 for v in bagg.values())
    bfp = sum(v["p"] >= threshold and v["y"] == 0 for v in bagg.values())
    bfn = sum(v["p"] < threshold and v["y"] == 1 for v in bagg.values())
    bp_, br_, bf_ = _prf(btp, bfp, bfn)

    cat_burst_correct = sum(
        (int(v["sm"][1] > v["sm"][0]) == v["y"]) for v in catagg.values())
    dir_burst_correct = sum(
        (v["pred"].most_common(1)[0][0] == v["y"]) for v in diragg.values())

    return {
        "prey_frame": {"precision": fp_, "recall": fr_, "f1": ff_,
                       "tp": ftp, "fp": ffp, "fn": ffn},
        "prey_burst": {"precision": bp_, "recall": br_, "f1": bf_,
                       "tp": btp, "fp": bfp, "fn": bfn},
        "cat_frame_acc": round(cat_frame_correct / max(cat_frame_tot, 1), 4),
        "cat_burst_acc": round(cat_burst_correct / max(len(catagg), 1), 4),
        "cat_burst_n": len(catagg),
        "subject_frame_acc": round(subj_correct / max(subj_tot, 1), 4),
        "subject_frame_n": subj_tot,
        "direction_frame_acc": round(dir_frame_correct / max(dir_frame_tot, 1), 4),
        "direction_burst_acc": round(dir_burst_correct / max(len(diragg), 1), 4),
        "direction_burst_n": len(diragg),
    }


def build_splits(rows):
    frames_by_burst = defaultdict(list)
    for r in rows:
        frames_by_burst[r["burst_id"]].append(r)
    split_of = {b: fs[0]["split"] for b, fs in frames_by_burst.items()}
    splits = {s: {b: frames_by_burst[b] for b in frames_by_burst if split_of[b] == s}
              for s in ("train", "val", "test")}
    return frames_by_burst, splits


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--crops", default=DEFAULT_CROPS)
    ap.add_argument("--tag", default="combined_v1")
    ap.add_argument("--epochs", type=int, default=25)
    ap.add_argument("--freeze-epochs", type=int, default=4)
    ap.add_argument("--bs", type=int, default=32)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--lr-finetune", type=float, default=1e-4)
    ap.add_argument("--weight-decay", type=float, default=1e-4)
    ap.add_argument("--w-cat", type=float, default=0.30)
    ap.add_argument("--w-subject", type=float, default=0.30)
    ap.add_argument("--w-direction", type=float, default=0.20)
    ap.add_argument("--direction-grad", action="store_true",
                    help="let direction head co-train the backbone (default: detached)")
    ap.add_argument("--pos-weight", type=float, default=0.0, help="0 = auto from data")
    ap.add_argument("--seed", type=int, default=1337)
    ap.add_argument("--device", default="auto")
    ap.add_argument("--workers", type=int, default=0,
                    help="DataLoader workers; 0 avoids multiprocessing (robust under nohup)")
    ap.add_argument("--baseline", default="", help="prey_v3 .pt to eval on same test split")
    ap.add_argument("--smoke", action="store_true", help="2 epochs, tiny, sanity check")
    args = ap.parse_args()

    if args.smoke:
        args.epochs, args.freeze_epochs, args.tag = 2, 1, args.tag + "_smoke"

    random.seed(args.seed)
    torch.manual_seed(args.seed)
    device = ("mps" if torch.backends.mps.is_available() else
              "cuda" if torch.cuda.is_available() else "cpu") if args.device == "auto" else args.device
    log.info("device=%s", device)

    crops_dir = DATASET / args.crops
    if not (crops_dir / "_index.csv").exists():
        raise SystemExit(f"No _index.csv in {crops_dir} — run build_crops.py first")

    rows = load_rows(crops_dir)
    frames_by_burst, splits = build_splits(rows)
    for s in ("train", "val", "test"):
        prey_b = sum(fs[0]["effective_prey"] == "1" for fs in splits[s].values())
        log.info("%-5s bursts=%d prey_bursts=%d frames=%d", s, len(splits[s]),
                 prey_b, sum(len(fs) for fs in splits[s].values()))

    # auto pos_weight = neg_frames / pos_frames (train)
    train_rows = [r for fs in splits["train"].values() for r in fs]
    npos = sum(r["effective_prey"] == "1" for r in train_rows)
    nneg = len(train_rows) - npos
    pw = args.pos_weight or (nneg / max(npos, 1))
    pw = min(pw, 40.0)
    log.info("prey pos_weight=%.1f (train pos=%d neg=%d)", pw, npos, nneg)
    pos_weight = torch.tensor([pw], device=device)
    weights = {"prey": 1.0, "cat": args.w_cat, "subject": args.w_subject,
               "direction": args.w_direction}

    train_ds = BurstFrames(sorted(splits["train"]), frames_by_burst, get_train_tf())
    val_ds = AllFrames(splits["val"], get_eval_tf())
    test_ds = AllFrames(splits["test"], get_eval_tf())
    train_ld = DataLoader(train_ds, batch_size=args.bs, shuffle=True, num_workers=args.workers, drop_last=False)
    val_ld = DataLoader(val_ds, batch_size=64, shuffle=False, num_workers=args.workers)
    test_ld = DataLoader(test_ds, batch_size=64, shuffle=False, num_workers=args.workers)

    model = MazgeMultiTask(direction_grad=args.direction_grad).to(device)
    out_dir = DEFAULT_OUT / args.tag
    out_dir.mkdir(parents=True, exist_ok=True)

    best = {"prey_burst_f1": -1.0, "epoch": -1}
    log_csv = open(out_dir / "train_log.csv", "w", newline="")
    writer = csv.writer(log_csv)
    writer.writerow(["epoch", "phase", "train_loss", "val_loss", "val_prey_burst_f1",
                     "val_cat_burst_acc", "val_subject_acc", "val_dir_burst_acc"])

    for epoch in range(1, args.epochs + 1):
        frozen = epoch <= args.freeze_epochs
        model.freeze_backbone(frozen)
        lr = args.lr if frozen else args.lr_finetune
        opt = torch.optim.AdamW([p for p in model.parameters() if p.requires_grad],
                                lr=lr, weight_decay=args.weight_decay)
        model.train()
        t0 = time.time()
        tl = tn = 0.0
        for batch in train_ld:
            imgs = batch["image"].to(device)
            prey_y = batch["prey_label"].to(device)
            cat_y = batch["cat_label"].to(device)
            subj_y = batch["subject_label"].to(device)
            dir_y = batch["dir_label"].to(device)
            fidx = batch["frame_idx"].to(device)

            prey_l, cat_l, subj_l, dir_l = model(imgs, fidx)
            prey_loss = F.binary_cross_entropy_with_logits(prey_l, prey_y, pos_weight=pos_weight)
            cat_loss, _, _ = _masked_ce(cat_l, cat_y)
            subj_loss, _, _ = _masked_ce(subj_l, subj_y)
            dir_loss, _, _ = _masked_ce(dir_l, dir_y)
            loss = (weights["prey"] * prey_loss + weights["cat"] * cat_loss
                    + weights["subject"] * subj_loss + weights["direction"] * dir_loss)
            opt.zero_grad()
            loss.backward()
            opt.step()
            tl += loss.item() * imgs.size(0); tn += imgs.size(0)

        val = evaluate(model, val_ld, device, pos_weight, weights)
        train_loss = tl / max(tn, 1)
        log.info("epoch %2d [%s] train=%.4f val=%.4f | prey_burst_f1=%.3f "
                 "cat_burst=%.3f subj=%.3f dir_burst=%.3f (%.0fs)",
                 epoch, "frozen" if frozen else "ft", train_loss, val["loss"],
                 val["prey_burst"]["f1"], val["cat_burst_acc"],
                 val["subject_frame_acc"], val["direction_burst_acc"], time.time() - t0)
        writer.writerow([epoch, "frozen" if frozen else "ft", round(train_loss, 4),
                         round(val["loss"], 4), val["prey_burst"]["f1"],
                         val["cat_burst_acc"], val["subject_frame_acc"],
                         val["direction_burst_acc"]])
        log_csv.flush()

        # Select on prey burst F1 (product priority); tiebreak on cat burst acc.
        score = (val["prey_burst"]["f1"], val["cat_burst_acc"])
        if score > (best["prey_burst_f1"], best.get("cat_burst_acc", -1)):
            best = {"prey_burst_f1": val["prey_burst"]["f1"],
                    "cat_burst_acc": val["cat_burst_acc"], "epoch": epoch}
            torch.save({"model": model.state_dict(), "epoch": epoch,
                        "direction_grad": args.direction_grad}, out_dir / "best.pt")
    log_csv.close()

    # Final test with best checkpoint
    ckpt = torch.load(out_dir / "best.pt", map_location=device)
    model.load_state_dict(ckpt["model"])
    test = evaluate(model, test_ld, device, pos_weight, weights)
    log.info("TEST (best epoch %d): prey_burst_f1=%.3f prey_frame_f1=%.3f "
             "cat_burst=%.3f subj=%.3f dir_burst=%.3f",
             best["epoch"], test["prey_burst"]["f1"], test["prey_frame"]["f1"],
             test["cat_burst_acc"], test["subject_frame_acc"], test["direction_burst_acc"])

    result = {"tag": args.tag, "best_epoch": best["epoch"],
              "direction_grad": args.direction_grad, "pos_weight": pw,
              "weights": weights, "test": test}

    # Optional: evaluate a baseline prey_v3 checkpoint on the SAME test split.
    if args.baseline and Path(args.baseline).exists():
        result["baseline"] = eval_baseline(args.baseline, test_ld, device, pos_weight, weights)
        b = result["baseline"]["test"]
        log.info("BASELINE prey_v3 on same test split: prey_burst_f1=%.3f "
                 "prey_frame_f1=%.3f cat_burst=%.3f",
                 b["prey_burst"]["f1"], b["prey_frame"]["f1"], b["cat_burst_acc"])

    (out_dir / "test_metrics.json").write_text(json.dumps(result, indent=2))
    log.info("wrote %s", out_dir / "test_metrics.json")


def eval_baseline(ckpt_path, test_ld, device, pos_weight, weights):
    """Load the 2-head prey_v3 model and eval prey+cat on the same test loader."""
    import sys
    sys.path.insert(0, str(REPO))
    from tools.train_prey_v3 import PreyV3Classifier

    net = PreyV3Classifier().to(device)
    sd = torch.load(ckpt_path, map_location=device)
    net.load_state_dict(sd["model"] if "model" in sd else sd)
    net.eval()

    # Wrap the 2-head net so evaluate() can consume it (subject/dir absent -> dummy).
    class _Wrap(nn.Module):
        def __init__(self, m): super().__init__(); self.m = m
        def forward(self, x, fidx):
            prey_l, cat_l = self.m(x, fidx)
            b = x.size(0)
            subj = torch.zeros(b, 4, device=x.device)
            dr = torch.zeros(b, 2, device=x.device)
            return prey_l, cat_l, subj, dr

    m = _Wrap(net).to(device)
    return {"ckpt": str(ckpt_path), "test": evaluate(m, test_ld, device, pos_weight, weights)}


if __name__ == "__main__":
    main()
