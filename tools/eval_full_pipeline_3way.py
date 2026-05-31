#!/usr/bin/env python3
"""3-way end-to-end pipeline comparison: x vs s vs s480.

Same door-gate semantics as eval_full_pipeline.py but compares three pipelines
side-by-side over all 655 labelled bursts. Each pipeline uses its own
(detector + matching classifier + own metric-sweep rule).

Outputs:
  models/prey_v3/full_eval_3way/{summary.txt, summary.json, per_burst.csv}
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
OUT_DIR = REPO / "models" / "prey_v3" / "full_eval_3way"
OUT_DIR.mkdir(parents=True, exist_ok=True)

PIPELINES = [
    {"tag": "x",    "run": REPO / "models/prey_v3/bodyA",
     "crops": DATASET / "crops_yolo11x_rotcrop", "label": "yolo11x+bodyA"},
    {"tag": "s",    "run": REPO / "models/prey_v3/bodyA_s",
     "crops": DATASET / "crops_yolo11s_rotcrop", "label": "yolo11s640+bodyA_s"},
    {"tag": "s480", "run": REPO / "models/prey_v3/bodyA_s480",
     "crops": DATASET / "crops_yolo11s480_rotcrop", "label": "yolo11s480+bodyA_s480"},
]


@dataclass
class Rule:
    mode: str; topk: int
    w_prey: float; w_cat: float; w_count: float
    prey_count_threshold: float; threshold: float

    @classmethod
    def from_sweep(cls, p: Path) -> "Rule":
        s = json.loads(p.read_text())["best_combined"]
        return cls(s["mode"], s["topk"], s["w_prey"], s["w_cat"],
                   s["w_count"], s["prey_count_threshold"], s["threshold"])


def load_model(ckpt: Path, device):
    sys.path.insert(0, str(REPO / "tools"))
    from train_prey_v3 import PreyV3Classifier
    m = PreyV3Classifier().to(device)
    m.load_state_dict(torch.load(ckpt, map_location=device,
                                  weights_only=False)["model"])
    m.eval()
    return m


@torch.no_grad()
def predict_burst(model, device, tf, rows):
    if not rows:
        return []
    imgs = torch.stack([tf(Image.open(DATASET / r["body_path"]).convert("RGB"))
                        for r in rows]).to(device)
    fidx = torch.tensor([int(r["frame_idx"]) for r in rows],
                        dtype=torch.long, device=device)
    pl, cl = model(imgs, fidx)
    pp = torch.sigmoid(pl).cpu().numpy()
    cp = torch.softmax(cl, dim=1).cpu().numpy()
    return [{"frame_idx": int(r["frame_idx"]),
             "prey_prob": float(p), "p_mazge": float(c[0])}
            for r, p, c in zip(rows, pp, cp)]


def burst_score(frames, rule: Rule):
    if not frames:
        return 0.0, 0
    s = [rule.w_prey * f["prey_prob"] + rule.w_cat * f["p_mazge"] for f in frames]
    agg = max(s) if rule.mode == "max" else \
        sum(sorted(s, reverse=True)[:min(rule.topk, len(s))]) / min(rule.topk, len(s))
    pc = sum(1 for f in frames if f["prey_prob"] >= rule.prey_count_threshold)
    return agg + rule.w_count * pc / max(len(frames), 1), pc


def load_crop_index(p: Path):
    by_burst = defaultdict(list)
    dedup = {}
    for r in csv.DictReader(open(p / "_index.csv")):
        dedup[(r["burst_id"], r["frame_idx"], r["image_id"])] = r
    for r in dedup.values():
        if r.get("body_path"):
            by_burst[r["burst_id"]].append(r)
    for b in by_burst.values():
        b.sort(key=lambda x: int(x["frame_idx"]))
    return by_burst


def door_outcome(cat_detected, prey_pred, y, direction):
    if not cat_detected:
        return "no_action"
    if direction == "exiting":
        return "skip_exit"
    if prey_pred == 0:
        return "open_correct" if y == 0 else "open_prey"
    else:
        return "block_fp" if y == 0 else "block_correct"


def door_metrics(rows):
    door_rows = [r for r in rows if r["outcome"] in
                 ("open_correct", "open_prey", "block_fp", "block_correct")]
    fn = sum(1 for r in door_rows if r["outcome"] == "open_prey")
    fp = sum(1 for r in door_rows if r["outcome"] == "block_fp")
    tp = sum(1 for r in door_rows if r["outcome"] == "open_correct")
    tn = sum(1 for r in door_rows if r["outcome"] == "block_correct")
    return {"n": len(door_rows), "tp": tp, "tn": tn, "fp": fp, "fn": fn,
            "open_correct": tp / max(tp + fp, 1),
            "block_correct": tn / max(tn + fn, 1)}


def coverage(rows):
    ent = [r for r in rows if r["direction"] == "entering"]
    prey = [r for r in ent if r["y"] == 1]
    return {
        "n_entering": len(ent),
        "n_entering_detected": sum(1 for r in ent if r["cat_detected"]),
        "ent_det_rate": sum(1 for r in ent if r["cat_detected"]) / max(len(ent), 1),
        "n_prey": len(prey),
        "n_prey_detected": sum(1 for r in prey if r["cat_detected"]),
        "prey_det_rate": sum(1 for r in prey if r["cat_detected"]) / max(len(prey), 1),
    }


def evaluate(name, model, rule, by_burst, bursts_meta, manifest_night,
             device, tf, split_map):
    rows = []
    for bid, bm in sorted(bursts_meta.items()):
        if bm.get("human_prey") not in ("0", "1"):
            continue
        y = 0 if bm.get("human_direction") == "exiting" else int(bm["human_prey"])
        direction = bm.get("human_direction", "")
        cat_id = bm.get("cat_id", "")
        nights = [manifest_night.get((bid, i), 0) for i in range(10)]
        is_night = any(nights)
        frames = by_burst.get(bid, [])
        cat_detected = len(frames) > 0
        if cat_detected:
            preds = predict_burst(model, device, tf, frames)
            score, pc = burst_score(preds, rule)
            prey_pred = 1 if score >= rule.threshold else 0
        else:
            score, pc, prey_pred = 0.0, 0, 0
        outcome = door_outcome(cat_detected, prey_pred, y, direction)
        rows.append({
            "burst_id": bid, "split": split_map.get(bid, ""),
            "cat_id": cat_id, "direction": direction,
            "y": y, "is_night": int(is_night),
            "nframes": len(frames),
            "cat_detected": int(cat_detected),
            "score": round(score, 4),
            "prey_pred": prey_pred, "outcome": outcome,
        })
    return rows


def main():
    device = (torch.device("mps") if torch.backends.mps.is_available()
              else torch.device("cuda") if torch.cuda.is_available()
              else torch.device("cpu"))
    print(f"Device: {device}")

    bursts_meta = {r["burst_id"]: r
                   for r in csv.DictReader(open(DATASET / "bursts.csv"))}
    manifest_night = {}
    for r in csv.DictReader(open(DATASET / "manifest.csv")):
        try:
            manifest_night[(r["burst_id"], int(r["frame_idx"]))] = int(r["night"])
        except (KeyError, ValueError):
            pass

    tf = transforms.Compose([
        transforms.Resize((IMG_SIZE, IMG_SIZE)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406],
                             std=[0.229, 0.224, 0.225]),
    ])

    results = {}
    for p in PIPELINES:
        print(f"\n--- {p['label']} ---")
        rule = Rule.from_sweep(p["run"] / "metric_sweep" / "summary.json")
        print(f"Rule: {rule}")
        model = load_model(p["run"] / "best_burst_f1.pt", device)
        by_burst = load_crop_index(p["crops"])
        split_map = json.loads((p["run"] / "split_assignment.json").read_text())
        rows = evaluate(p["tag"], model, rule, by_burst, bursts_meta,
                        manifest_night, device, tf, split_map)
        results[p["tag"]] = {"rule": rule, "rows": rows, "label": p["label"]}

    lines = []
    def out(s=""):
        lines.append(s); print(s)

    out()
    out("=" * 100)
    out("3-WAY FULL-DATA EVALUATION (all labelled bursts, door-gate semantics)")
    out("=" * 100)
    for tag, r in results.items():
        d = door_metrics(r["rows"])
        c = coverage(r["rows"])
        out()
        out(f"### {r['label']} (rule threshold={r['rule'].threshold})")
        out(f"  Door decisions: n={d['n']}  TP_open={d['tp']}  TN_block={d['tn']}  "
            f"FP_block={d['fp']}  FN_safety={d['fn']}")
        out(f"  Open correctness: {d['open_correct']:.3f}  "
            f"Block correctness: {d['block_correct']:.3f}")
        out(f"  Coverage: cat_det={c['n_entering_detected']}/{c['n_entering']} "
            f"({100*c['ent_det_rate']:.1f}%)  prey_det={c['n_prey_detected']}/{c['n_prey']} "
            f"({100*c['prey_det_rate']:.1f}%)")

    out()
    out("### Side-by-side door metrics")
    out(f"  {'pipeline':24s}  {'TP_open':>7s} {'TN_block':>8s} {'FP_block':>8s} "
        f"{'FN_safety':>9s} {'open_corr':>9s} {'block_corr':>10s}")
    for tag, r in results.items():
        d = door_metrics(r["rows"])
        out(f"  {r['label']:24s}  {d['tp']:>7d} {d['tn']:>8d} {d['fp']:>8d} "
            f"{d['fn']:>9d} {d['open_correct']:>9.3f} {d['block_correct']:>10.3f}")

    out()
    out("### By split (TP_open / TN_block / FP_block / FN_safety)")
    for sp in ("train", "val", "test"):
        out(f"  split={sp}")
        for tag, r in results.items():
            sr = [x for x in r["rows"] if x["split"] == sp]
            if not sr:
                continue
            d = door_metrics(sr)
            c = coverage(sr)
            out(f"    {r['label']:24s} n={len(sr):>3d}  "
                f"tp={d['tp']:>3d} tn={d['tn']:>2d} fp={d['fp']:>2d} fn={d['fn']:>2d}  "
                f"cat_det={100*c['ent_det_rate']:>5.1f}%  prey_det={100*c['prey_det_rate']:>5.1f}%")

    out()
    out("### Safety FNs (prey entered with door open)")
    for tag, r in results.items():
        leaks = [x for x in r["rows"] if x["outcome"] == "open_prey"]
        out(f"  {r['label']}: {len(leaks)} prey leaks")
        for x in leaks:
            out(f"    {x['burst_id']}  split={x['split']}  cat={x['cat_id']}  "
                f"score={x['score']}  nframes={x['nframes']}")

    (OUT_DIR / "summary.txt").write_text("\n".join(lines) + "\n")

    # JSON summary
    summary = {}
    for tag, r in results.items():
        summary[tag] = {
            "label": r["label"],
            "rule": r["rule"].__dict__,
            "door_metrics": door_metrics(r["rows"]),
            "coverage": coverage(r["rows"]),
            "by_split": {sp: door_metrics([x for x in r["rows"] if x["split"] == sp])
                         | coverage([x for x in r["rows"] if x["split"] == sp])
                         for sp in ("train", "val", "test")},
        }
    (OUT_DIR / "summary.json").write_text(json.dumps(summary, indent=2, default=str))

    # joined per-burst CSV
    by_bid = {}
    for tag, r in results.items():
        for x in r["rows"]:
            d = by_bid.setdefault(x["burst_id"], {
                "burst_id": x["burst_id"], "y": x["y"],
                "direction": x["direction"], "cat_id": x["cat_id"],
                "is_night": x["is_night"]})
            d[f"{tag}_split"] = x["split"]
            d[f"{tag}_nframes"] = x["nframes"]
            d[f"{tag}_score"] = x["score"]
            d[f"{tag}_outcome"] = x["outcome"]
    fields = ["burst_id", "y", "direction", "cat_id", "is_night"]
    for tag in ["x", "s", "s480"]:
        fields += [f"{tag}_split", f"{tag}_nframes", f"{tag}_score", f"{tag}_outcome"]
    with open(OUT_DIR / "per_burst.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in sorted(by_bid.values(), key=lambda x: x["burst_id"]):
            w.writerow({k: r.get(k, "") for k in fields})

    print(f"\nWrote {OUT_DIR}/summary.{{txt,json}}, per_burst.csv")


if __name__ == "__main__":
    main()
