#!/usr/bin/env python3
"""End-to-end pipeline evaluation on ALL labelled bursts.

For each detector pipeline (yolo11x + bodyA, yolo11s + bodyA_s), evaluates
every burst with human_prey in {0,1} including bursts where the detector
produced NO crops. Under the door-gate policy:

  door opens iff cat_detected AND prey_score < threshold

a no-crop burst means "no cat detected" → "no door action" → outcome is
correct iff y=0 (since the safe default of keeping door closed is right
when there's no prey to let in, and a retry on the next burst will catch
genuine cat visits within seconds).

Two views are reported:
  1. Safety/correctness: did the right door action happen?
     - open ⟺ cat detected AND prey_pred=0 AND direction != exiting
     - FN_safety: y=1 (prey) AND door opened
     - FP_open : y=0 AND cat detected but door blocked
     - TN_block: y=1 AND door blocked (prey kept out) — GOOD
     - TP_open : y=0 AND cat detected AND door opened
     - skip    : no cat detected (no door action; retry handles)
  2. Per-burst classification (open subset only): same TP/FP/FN/TN as
     the earlier 399-burst comparison, restricted to bursts the
     detector cropped.

Breakdowns by split, cat_id, direction, day/night.

Outputs:
  models/prey_v3/full_eval/{per_burst.csv, summary.json, summary.txt}
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

OUT_DIR = REPO / "models" / "prey_v3" / "full_eval"
OUT_DIR.mkdir(parents=True, exist_ok=True)


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
    s = [rule.w_prey * f["prey_prob"] + rule.w_cat * f["p_mazge"]
         for f in frames]
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
    """Return one of:
       open_correct  (y=0, cat detected, door opened) — TP_open
       open_prey     (y=1, cat detected, door opened) — FN_safety (BAD)
       block_fp      (y=0, cat detected, door blocked) — FP_open
       block_correct (y=1, cat detected, door blocked) — TN_block
       no_action     (no cat detected; retry handles)
    """
    if not cat_detected:
        return "no_action"
    # Exiting bursts: door doesn't need to open (cat is leaving)
    if direction == "exiting":
        return "skip_exit"
    if prey_pred == 0:
        return "open_correct" if y == 0 else "open_prey"
    else:
        return "block_fp" if y == 0 else "block_correct"


def evaluate_pipeline(name, model, rule, by_burst, bursts_meta, manifest_night,
                      device, tf, split_map):
    rows = []
    cats = sorted(set(b["cat_id"] for b in bursts_meta.values()) - {""})
    for bid, bm in sorted(bursts_meta.items()):
        if bm.get("human_prey") not in ("0", "1"):
            continue
        y = 0 if bm.get("human_direction") == "exiting" else int(bm["human_prey"])
        direction = bm.get("human_direction", "")
        cat_id = bm.get("cat_id", "")
        # Night = any frame in burst was night (most bursts are uniform)
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
            "score": round(score, 4), "prey_count": pc,
            "prey_pred": prey_pred, "outcome": outcome,
        })
    return rows


def confusion_from_outcomes(rows):
    c = defaultdict(int)
    for r in rows:
        c[r["outcome"]] += 1
    return dict(c)


def door_metrics(rows):
    """Restrict to bursts that triggered any door consideration
    (cat detected, entering — so opens are possible).
    """
    door_rows = [r for r in rows if r["outcome"] in
                 ("open_correct", "open_prey", "block_fp", "block_correct")]
    n = len(door_rows)
    fn_safety = sum(1 for r in door_rows if r["outcome"] == "open_prey")
    fp_block  = sum(1 for r in door_rows if r["outcome"] == "block_fp")
    tp_open   = sum(1 for r in door_rows if r["outcome"] == "open_correct")
    tn_block  = sum(1 for r in door_rows if r["outcome"] == "block_correct")
    return {
        "n_decisions": n,
        "tp_open": tp_open, "tn_block": tn_block,
        "fp_block": fp_block, "fn_safety": fn_safety,
        "open_correctness": tp_open / max(tp_open + fp_block, 1),
        "block_correctness": tn_block / max(tn_block + fn_safety, 1),
    }


def coverage_metrics(rows):
    # All bursts (y in {0,1}, direction entering or exiting)
    n_all = len(rows)
    n_entering = [r for r in rows if r["direction"] == "entering"]
    n_cat_detected = sum(1 for r in n_entering if r["cat_detected"])
    n_prey = [r for r in n_entering if r["y"] == 1]
    n_prey_detected = sum(1 for r in n_prey if r["cat_detected"])
    return {
        "n_all": n_all,
        "n_entering": len(n_entering),
        "n_entering_cat_detected": n_cat_detected,
        "entering_detection_rate": n_cat_detected / max(len(n_entering), 1),
        "n_prey_entering": len(n_prey),
        "n_prey_entering_cat_detected": n_prey_detected,
        "prey_detection_rate": n_prey_detected / max(len(n_prey), 1),
    }


def breakdown(rows, key):
    out = {}
    for r in rows:
        v = r.get(key)
        k = v if v not in (None, "") else "(empty)"
        out.setdefault(k, []).append(r)
    return {k: door_metrics(v) | coverage_metrics(v) for k, v in out.items()}


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
    # Night flag per (burst, frame_idx) from manifest
    manifest_night = {}
    for r in csv.DictReader(open(DATASET / "manifest.csv")):
        try:
            manifest_night[(r["burst_id"], int(r["frame_idx"]))] = int(r["night"])
        except (KeyError, ValueError):
            pass

    by_x = load_crop_index(X_CROPS)
    by_s = load_crop_index(S_CROPS)
    split_x = json.loads((X_RUN / "split_assignment.json").read_text())
    split_s = json.loads((S_RUN / "split_assignment.json").read_text())

    print(f"\nEvaluating yolo11x + bodyA on all labelled bursts ...")
    rows_x = evaluate_pipeline("x", model_x, rule_x, by_x, bursts_meta,
                                manifest_night, device, tf, split_x)
    print(f"Evaluating yolo11s + bodyA_s on all labelled bursts ...")
    rows_s = evaluate_pipeline("s", model_s, rule_s, by_s, bursts_meta,
                                manifest_night, device, tf, split_s)

    def report(name, rows):
        out = {
            "n_bursts": len(rows),
            "outcomes": confusion_from_outcomes(rows),
            "door_metrics": door_metrics(rows),
            "coverage": coverage_metrics(rows),
            "by_split": breakdown(rows, "split"),
            "by_cat": breakdown(rows, "cat_id"),
            "by_direction": breakdown(rows, "direction"),
            "by_night": breakdown(rows, "is_night"),
        }
        return out

    summary = {
        "rules": {"x": rule_x.__dict__, "s": rule_s.__dict__},
        "x": report("x", rows_x),
        "s": report("s", rows_s),
    }
    (OUT_DIR / "summary.json").write_text(json.dumps(summary, indent=2, default=str))

    # Pretty text summary
    lines = []
    def out(s=""):
        lines.append(s); print(s)
    out("=" * 100)
    out("FULL-DATA EVALUATION (all labelled bursts, door-gate semantics)")
    out("=" * 100)

    for tag, rep in [("yolo11x+bodyA  ", summary["x"]),
                     ("yolo11s+bodyA_s", summary["s"])]:
        d = rep["door_metrics"]; c = rep["coverage"]; o = rep["outcomes"]
        out()
        out(f"### {tag}")
        out(f"  Total labelled bursts: {rep['n_bursts']}")
        out(f"  Outcomes: {o}")
        out(f"  Door decisions (cat detected & entering): n={d['n_decisions']}")
        out(f"    TP open (good cat, door opens) : {d['tp_open']}")
        out(f"    TN block (prey, door blocked)  : {d['tn_block']}  ✓ SAFE")
        out(f"    FP block (good cat, door blocked, annoying): {d['fp_block']}")
        out(f"    FN safety (prey, door OPENS, BAD)         : {d['fn_safety']}")
        out(f"    Open correctness  : {d['open_correctness']:.3f}")
        out(f"    Block correctness : {d['block_correctness']:.3f}")
        out(f"  Coverage:")
        out(f"    Entering bursts cat-detection rate: "
            f"{c['n_entering_cat_detected']}/{c['n_entering']} "
            f"({100*c['entering_detection_rate']:.1f}%)")
        out(f"    Prey-entering cat-detection rate  : "
            f"{c['n_prey_entering_cat_detected']}/{c['n_prey_entering']} "
            f"({100*c['prey_detection_rate']:.1f}%)")

    out()
    out("### By split")
    for sp in ("train", "val", "test"):
        out(f"  split={sp}")
        for tag, rep in [("x", summary["x"]), ("s", summary["s"])]:
            d = rep["by_split"].get(sp)
            if not d:
                continue
            out(f"    {tag}: n_all={d['n_all']:>3d}  "
                f"tp_open={d['tp_open']:>3d}  tn_block={d['tn_block']:>2d}  "
                f"fp_block={d['fp_block']:>3d}  fn_safety={d['fn_safety']:>2d}  "
                f"cat_det={100*d['entering_detection_rate']:>5.1f}%  "
                f"prey_det={100*d['prey_detection_rate']:>5.1f}%")

    out()
    out("### By cat")
    cats = sorted(set(summary["x"]["by_cat"]) | set(summary["s"]["by_cat"]))
    for cat in cats:
        out(f"  cat={cat or '(none)'}")
        for tag, rep in [("x", summary["x"]), ("s", summary["s"])]:
            d = rep["by_cat"].get(cat)
            if not d:
                continue
            out(f"    {tag}: n_all={d['n_all']:>3d}  "
                f"tp_open={d['tp_open']:>3d}  tn_block={d['tn_block']:>2d}  "
                f"fp_block={d['fp_block']:>3d}  fn_safety={d['fn_safety']:>2d}  "
                f"cat_det={100*d['entering_detection_rate']:>5.1f}%  "
                f"prey_det={100*d['prey_detection_rate']:>5.1f}%")

    out()
    out("### By night/day")
    for is_night in (0, 1):
        out(f"  is_night={is_night}")
        for tag, rep in [("x", summary["x"]), ("s", summary["s"])]:
            d = rep["by_night"].get(is_night)
            if not d:
                continue
            out(f"    {tag}: n_all={d['n_all']:>3d}  "
                f"tp_open={d['tp_open']:>3d}  tn_block={d['tn_block']:>2d}  "
                f"fp_block={d['fp_block']:>3d}  fn_safety={d['fn_safety']:>2d}  "
                f"cat_det={100*d['entering_detection_rate']:>5.1f}%  "
                f"prey_det={100*d['prey_detection_rate']:>5.1f}%")

    # FN safety listing (genuine prey leaks if any)
    out()
    out("### Safety FNs (prey entered with door open)")
    for tag, rows in [("x", rows_x), ("s", rows_s)]:
        leaks = [r for r in rows if r["outcome"] == "open_prey"]
        out(f"  {tag}: {len(leaks)} prey leaks")
        for r in leaks:
            out(f"    {r['burst_id']}  split={r['split']}  cat={r['cat_id']}  "
                f"score={r['score']}  nframes={r['nframes']}")

    (OUT_DIR / "summary.txt").write_text("\n".join(lines) + "\n")

    # per-burst joined CSV
    by_bid = {}
    for r in rows_x:
        by_bid[r["burst_id"]] = {"burst_id": r["burst_id"], "y": r["y"],
                                  "direction": r["direction"],
                                  "cat_id": r["cat_id"],
                                  "is_night": r["is_night"],
                                  "split_x": r["split"],
                                  "x_nframes": r["nframes"],
                                  "x_score": r["score"],
                                  "x_prey_pred": r["prey_pred"],
                                  "x_outcome": r["outcome"]}
    for r in rows_s:
        d = by_bid.setdefault(r["burst_id"], {"burst_id": r["burst_id"],
                                                "y": r["y"],
                                                "direction": r["direction"],
                                                "cat_id": r["cat_id"],
                                                "is_night": r["is_night"]})
        d["split_s"] = r["split"]
        d["s_nframes"] = r["nframes"]
        d["s_score"] = r["score"]
        d["s_prey_pred"] = r["prey_pred"]
        d["s_outcome"] = r["outcome"]
    fields = ["burst_id", "y", "direction", "cat_id", "is_night",
              "split_x", "x_nframes", "x_score", "x_prey_pred", "x_outcome",
              "split_s", "s_nframes", "s_score", "s_prey_pred", "s_outcome"]
    with open(OUT_DIR / "per_burst.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in sorted(by_bid.values(), key=lambda x: x["burst_id"]):
            w.writerow({k: r.get(k, "") for k in fields})

    print(f"\nWrote {OUT_DIR}/summary.json, summary.txt, per_burst.csv")


if __name__ == "__main__":
    main()
