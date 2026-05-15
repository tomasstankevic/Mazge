"""Correlate human direction labels with ToF distance progression across burst frames.

For each burst with a confident human direction label, compute:
  - distance trajectory: dist at f0, f1, ... f9 (or as many as available)
  - linear-fit slope (mm per 100ms frame)
  - overall trend: dist[end] - dist[start]
  - n_valid: count of frames with valid distance (>= 0)

Then aggregate: mean/median/distribution of slope by direction class.
"""
import csv
import json
from collections import defaultdict
from pathlib import Path
from statistics import mean, median, stdev

REPO = Path(__file__).resolve().parent.parent
SD = REPO / "captures" / "sd"

# Load human direction labels per burst from labels.jsonl
direction_by_burst: dict[str, str] = {}
DIR = {0: "entering", 1: "exiting"}
for line in (REPO / "dataset" / "labels.jsonl").open():
    try:
        rec = json.loads(line)
    except Exception:
        continue
    src = rec.get("source", "")
    if not src.startswith("human:") or not src.endswith(":burst_direction"):
        continue
    parts = rec.get("image_id", "").split("/")
    if len(parts) < 4:
        continue
    burst_id = parts[2]
    conf = rec.get("confidence", 0.0)
    if conf == 0.0:
        # unclear
        direction_by_burst[burst_id] = "unclear"
    else:
        direction_by_burst[burst_id] = DIR.get(rec.get("label"), "?")

# Walk burst meta.jsons and pull distance trajectories
by_dir: dict[str, list[dict]] = defaultdict(list)
for d in sorted(SD.iterdir()):
    if not d.is_dir():
        continue
    direction = direction_by_burst.get(d.name)
    if direction not in ("entering", "exiting"):
        continue  # skip unclear/unlabeled
    meta_p = d / "meta.json"
    if not meta_p.exists():
        continue
    try:
        meta = json.loads(meta_p.read_text())
    except Exception:
        continue
    images = meta.get("images", [])
    dists = []
    for img in images:
        d_mm = img.get("dist", img.get("distance", -1))
        dists.append(int(d_mm) if d_mm is not None else -1)
    valid = [(i, v) for i, v in enumerate(dists) if v >= 0]
    if len(valid) < 2:
        continue
    # Linear fit slope (mm per frame index)
    xs = [i for i, _ in valid]
    ys = [v for _, v in valid]
    n = len(xs)
    sx, sy = sum(xs), sum(ys)
    sxx = sum(x * x for x in xs)
    sxy = sum(x * y for x, y in zip(xs, ys))
    denom = n * sxx - sx * sx
    slope = (n * sxy - sx * sy) / denom if denom else 0
    # Frame index of the min reading (within the burst's full 10-frame timeline)
    min_idx_in_valid = min(range(n), key=lambda i: ys[i])
    min_frame = xs[min_idx_in_valid]
    min_val = ys[min_idx_in_valid]
    # First valid distance vs last valid distance (timing-aware)
    first_dist = ys[0]
    last_dist = ys[-1]
    # Where is the min within the valid samples? early/middle/late
    rel_pos = min_idx_in_valid / max(1, n - 1)  # 0=earliest valid, 1=latest valid
    by_dir[direction].append({
        "burst": d.name,
        "n_valid": n,
        "first_dist": first_dist,
        "last_dist": last_dist,
        "delta": last_dist - first_dist,
        "slope_mm_per_frame": slope,
        "min_dist": min_val,
        "min_frame": min_frame,            # absolute frame index 0..9
        "min_rel_pos": rel_pos,            # 0..1 within valid samples
        "trajectory": dists,
    })

print(f"Bursts analyzed:")
for direction, rows in by_dir.items():
    print(f"  {direction}: {len(rows)}")
print()

for direction in ("entering", "exiting"):
    rows = by_dir.get(direction, [])
    if not rows:
        continue
    slopes = [r["slope_mm_per_frame"] for r in rows]
    deltas = [r["delta"] for r in rows]
    mins = [r["min_dist"] for r in rows]
    n_valid = [r["n_valid"] for r in rows]
    min_frames = [r["min_frame"] for r in rows]
    min_rels = [r["min_rel_pos"] for r in rows]
    firsts = [r["first_dist"] for r in rows]
    print(f"=== {direction.upper()} (n={len(rows)}) ===")
    print(f"  slope mm/frame:  mean={mean(slopes):+7.1f}  "
          f"median={median(slopes):+7.1f}  "
          f"std={stdev(slopes):.1f}" if len(slopes) > 1 else "")
    print(f"  delta (last-first): mean={mean(deltas):+7.1f}  "
          f"median={median(deltas):+7.1f}")
    print(f"  min distance:    mean={mean(mins):7.1f}  median={median(mins):7.1f}")
    print(f"  min frame idx:   mean={mean(min_frames):4.1f}  median={median(min_frames):4.1f}")
    print(f"  min rel pos:     mean={mean(min_rels):.2f}  median={median(min_rels):.2f}  (0=first valid, 1=last valid)")
    print(f"  first valid dist: mean={mean(firsts):7.1f}  median={median(firsts):7.1f}")
    print(f"  valid frames:    mean={mean(n_valid):4.1f}/10")
    decreasing = sum(1 for s in slopes if s < -5)
    increasing = sum(1 for s in slopes if s > 5)
    flat = len(slopes) - decreasing - increasing
    print(f"  trend (slope <-5/[-5,+5]/>+5):  decr={decreasing}  flat={flat}  incr={increasing}")
    print()

# Decision rule simulation: predict direction from slope
print("=== Simple slope-based classifier ===")
print("Rule: slope < threshold => entering, else => exiting")
for thr in (-30, -20, -10, -5, 0, 5, 10, 20):
    tp = sum(1 for r in by_dir.get("entering", []) if r["slope_mm_per_frame"] < thr)
    fn = sum(1 for r in by_dir.get("entering", []) if r["slope_mm_per_frame"] >= thr)
    fp = sum(1 for r in by_dir.get("exiting", []) if r["slope_mm_per_frame"] < thr)
    tn = sum(1 for r in by_dir.get("exiting", []) if r["slope_mm_per_frame"] >= thr)
    total = tp + fn + fp + tn
    if total == 0:
        continue
    acc = (tp + tn) / total
    prec = tp / (tp + fp) if (tp + fp) else 0
    rec = tp / (tp + fn) if (tp + fn) else 0
    print(f"  thr={thr:>+4}: acc={acc:.2%}  precision(entering)={prec:.2%}  "
          f"recall(entering)={rec:.2%}  TP={tp} FN={fn} FP={fp} TN={tn}")

# Min-distance classifier: cat exiting pokes head right up to sensor
print()
print("=== Min-distance classifier ===")
print("Rule: min_dist < threshold => exiting (cat very close to sensor)")
for thr in (80, 100, 120, 150, 180, 200, 220, 250):
    tp = sum(1 for r in by_dir.get("exiting", []) if r["min_dist"] < thr)
    fn = sum(1 for r in by_dir.get("exiting", []) if r["min_dist"] >= thr)
    fp = sum(1 for r in by_dir.get("entering", []) if r["min_dist"] < thr)
    tn = sum(1 for r in by_dir.get("entering", []) if r["min_dist"] >= thr)
    total = tp + fn + fp + tn
    if total == 0:
        continue
    acc = (tp + tn) / total
    prec = tp / (tp + fp) if (tp + fp) else 0
    rec = tp / (tp + fn) if (tp + fn) else 0
    print(f"  thr={thr:>3}mm: acc={acc:.2%}  precision(exiting)={prec:.2%}  "
          f"recall(exiting)={rec:.2%}  TP={tp} FN={fn} FP={fp} TN={tn}")

# Combined: min_dist + slope
print()
print("=== Combined min-dist + slope classifier ===")
print("Rule: exit if min_dist<150 AND slope >= -10 (came close, not moving away fast)")
def predict(r):
    return "exiting" if (r["min_dist"] < 150 and r["slope_mm_per_frame"] >= -10) else "entering"
ok = miss = 0
confmat = {("entering", "entering"): 0, ("entering", "exiting"): 0,
           ("exiting", "entering"): 0, ("exiting", "exiting"): 0}
for direction in ("entering", "exiting"):
    for r in by_dir.get(direction, []):
        pred = predict(r)
        confmat[(direction, pred)] += 1
        if pred == direction: ok += 1
        else: miss += 1
print(f"  accuracy = {ok / (ok + miss):.2%}")
header = "true\\pred"
print(f"  {header:>12}  entering  exiting")
print(f"  {'entering':>12}  {confmat[('entering','entering')]:>8}  {confmat[('entering','exiting')]:>8}")
print(f"  {'exiting':>12}  {confmat[('exiting','entering')]:>8}  {confmat[('exiting','exiting')]:>8}")

# === Min-frame-index classifier ===
# Hypothesis: exiting cats poke head close EARLY (cat already at sensor, then walks past).
# Entering cats reach minimum distance LATE (approaching over time).
print()
print("=== Min-frame-index classifier (when does the closest reading happen?) ===")
print("NOTE: most bursts only have valid ToF for last 2-3 frames (cat blocks beam),")
print("      so this feature alone is weak. Kept for reference.")
for thr in (0, 1, 2, 3, 4, 5, 6):
    tp = sum(1 for r in by_dir.get("exiting", []) if r["min_frame"] <= thr)
    fn = sum(1 for r in by_dir.get("exiting", []) if r["min_frame"] > thr)
    fp = sum(1 for r in by_dir.get("entering", []) if r["min_frame"] <= thr)
    tn = sum(1 for r in by_dir.get("entering", []) if r["min_frame"] > thr)
    total = tp + fn + fp + tn
    if total == 0:
        continue
    acc = (tp + tn) / total
    prec = tp / (tp + fp) if (tp + fp) else 0
    rec = tp / (tp + fn) if (tp + fn) else 0
    print(f"  thr={thr}: acc={acc:.2%}  precision(exiting)={prec:.2%}  "
          f"recall(exiting)={rec:.2%}  TP={tp} FN={fn} FP={fp} TN={tn}")

# === First-valid-distance classifier ===
# For exiting, cat is already at sensor when first valid ToF arrives.
# For entering, first valid reading is the cat approaching the flap (further).
print()
print("=== First-valid-distance classifier ===")
print("Rule: first_dist < threshold => exiting (cat already close on first reading)")
for thr in (80, 100, 120, 150, 180, 200, 220, 250, 300):
    tp = sum(1 for r in by_dir.get("exiting", []) if r["first_dist"] < thr)
    fn = sum(1 for r in by_dir.get("exiting", []) if r["first_dist"] >= thr)
    fp = sum(1 for r in by_dir.get("entering", []) if r["first_dist"] < thr)
    tn = sum(1 for r in by_dir.get("entering", []) if r["first_dist"] >= thr)
    total = tp + fn + fp + tn
    if total == 0:
        continue
    acc = (tp + tn) / total
    prec = tp / (tp + fp) if (tp + fp) else 0
    rec = tp / (tp + fn) if (tp + fn) else 0
    f1 = 2 * prec * rec / (prec + rec) if (prec + rec) else 0
    print(f"  thr={thr:>3}mm: acc={acc:.2%}  prec={prec:.2%}  rec={rec:.2%}  "
          f"F1={f1:.3f}  TP={tp} FN={fn} FP={fp} TN={tn}")

# === Combined min_dist + min_frame ===
print()
print("=== Combined min_dist + min_frame classifier ===")
print("Sweep both thresholds, report best F1 for exiting class.")
best = (0, None)
for d_thr in range(80, 261, 10):
    for f_thr in range(0, 7):
        tp = sum(1 for r in by_dir.get("exiting", [])
                 if r["min_dist"] < d_thr and r["min_frame"] <= f_thr)
        fn = sum(1 for r in by_dir.get("exiting", [])
                 if not (r["min_dist"] < d_thr and r["min_frame"] <= f_thr))
        fp = sum(1 for r in by_dir.get("entering", [])
                 if r["min_dist"] < d_thr and r["min_frame"] <= f_thr)
        tn = sum(1 for r in by_dir.get("entering", [])
                 if not (r["min_dist"] < d_thr and r["min_frame"] <= f_thr))
        prec = tp / (tp + fp) if (tp + fp) else 0
        rec = tp / (tp + fn) if (tp + fn) else 0
        f1 = (2 * prec * rec / (prec + rec)) if (prec + rec) else 0
        if f1 > best[0]:
            best = (f1, (d_thr, f_thr, tp, fn, fp, tn, prec, rec))
print(f"  best F1={best[0]:.3f} at min_dist<{best[1][0]} AND min_frame<={best[1][1]}")
print(f"  TP={best[1][2]} FN={best[1][3]} FP={best[1][4]} TN={best[1][5]}")
print(f"  precision={best[1][6]:.2%}  recall={best[1][7]:.2%}")
total = sum(best[1][2:6])
print(f"  accuracy={(best[1][2] + best[1][5]) / total:.2%}")

# === Combined min_dist + first_dist classifier ===
print()
print("=== Combined min_dist + first_dist classifier ===")
print("Sweep both thresholds independently.")
best = (0, None)
for min_thr in range(80, 261, 10):
    for first_thr in range(80, 301, 10):
        tp = sum(1 for r in by_dir.get("exiting", [])
                 if r["min_dist"] < min_thr and r["first_dist"] < first_thr)
        fn = sum(1 for r in by_dir.get("exiting", [])
                 if not (r["min_dist"] < min_thr and r["first_dist"] < first_thr))
        fp = sum(1 for r in by_dir.get("entering", [])
                 if r["min_dist"] < min_thr and r["first_dist"] < first_thr)
        tn = sum(1 for r in by_dir.get("entering", [])
                 if not (r["min_dist"] < min_thr and r["first_dist"] < first_thr))
        prec = tp / (tp + fp) if (tp + fp) else 0
        rec = tp / (tp + fn) if (tp + fn) else 0
        f1 = (2 * prec * rec / (prec + rec)) if (prec + rec) else 0
        if f1 > best[0]:
            best = (f1, (min_thr, first_thr, tp, fn, fp, tn, prec, rec))
print(f"  best F1={best[0]:.3f} at min_dist<{best[1][0]} AND first_dist<{best[1][1]}")
print(f"  TP={best[1][2]} FN={best[1][3]} FP={best[1][4]} TN={best[1][5]}")
print(f"  precision={best[1][6]:.2%}  recall={best[1][7]:.2%}")
total = sum(best[1][2:6])
print(f"  accuracy={(best[1][2] + best[1][5]) / total:.2%}")

# === 3-feature: min_dist + first_dist + n_close (frames < 200mm) ===
# Exiting bursts have multiple close readings (cat body fills FOV).
# Entering bursts often have just one close-ish reading at the end.
print()
print("=== 3-feature: min_dist + first_dist + n_close ===")
print("(n_close = number of valid readings < 200mm)")
def feats(r):
    n_close = sum(1 for v in r["trajectory"] if 0 <= v < 200)
    return r["min_dist"], r["first_dist"], n_close

best3 = (0, None)
for min_thr in range(100, 261, 20):
    for first_thr in range(100, 301, 20):
        for n_close_thr in range(1, 5):
            tp = fn = fp = tn = 0
            for r in by_dir.get("exiting", []):
                m, f, nc = feats(r)
                pred = (m < min_thr and f < first_thr and nc >= n_close_thr)
                if pred: tp += 1
                else: fn += 1
            for r in by_dir.get("entering", []):
                m, f, nc = feats(r)
                pred = (m < min_thr and f < first_thr and nc >= n_close_thr)
                if pred: fp += 1
                else: tn += 1
            prec = tp / (tp + fp) if (tp + fp) else 0
            rec = tp / (tp + fn) if (tp + fn) else 0
            f1 = (2 * prec * rec / (prec + rec)) if (prec + rec) else 0
            if f1 > best3[0]:
                best3 = (f1, (min_thr, first_thr, n_close_thr, tp, fn, fp, tn, prec, rec))
print(f"  best F1={best3[0]:.3f} at min<{best3[1][0]} AND first<{best3[1][1]} "
      f"AND n_close>={best3[1][2]}")
print(f"  TP={best3[1][3]} FN={best3[1][4]} FP={best3[1][5]} TN={best3[1][6]}")
print(f"  precision={best3[1][7]:.2%}  recall={best3[1][8]:.2%}")
total = sum(best3[1][3:7])
print(f"  accuracy={(best3[1][3] + best3[1][6]) / total:.2%}")

# === Slope on top of min+first: does it add anything? ===
# Try requiring slope >= s_thr (i.e., NOT moving away fast) in addition to
# min<180 and first<230.
print()
print("=== Does adding slope help? (on top of min<180 AND first<230) ===")
for s_thr in (-100, -50, -30, -20, -10, 0, 10):
    tp = sum(1 for r in by_dir.get("exiting", [])
             if r["min_dist"] < 180 and r["first_dist"] < 230
             and r["slope_mm_per_frame"] >= s_thr)
    fn = sum(1 for r in by_dir.get("exiting", [])
             if not (r["min_dist"] < 180 and r["first_dist"] < 230
                     and r["slope_mm_per_frame"] >= s_thr))
    fp = sum(1 for r in by_dir.get("entering", [])
             if r["min_dist"] < 180 and r["first_dist"] < 230
             and r["slope_mm_per_frame"] >= s_thr)
    tn = sum(1 for r in by_dir.get("entering", [])
             if not (r["min_dist"] < 180 and r["first_dist"] < 230
                     and r["slope_mm_per_frame"] >= s_thr))
    total = tp + fn + fp + tn
    prec = tp / (tp + fp) if (tp + fp) else 0
    rec = tp / (tp + fn) if (tp + fn) else 0
    f1 = 2 * prec * rec / (prec + rec) if (prec + rec) else 0
    acc = (tp + tn) / total if total else 0
    print(f"  slope>={s_thr:>+4}: acc={acc:.2%} prec={prec:.2%} rec={rec:.2%} "
          f"F1={f1:.3f}  TP={tp} FN={fn} FP={fp} TN={tn}")

# Show some confused cases (using best min_dist + first_dist rule)
print()
print("=== Confused cases (using best-F1 rule from min_dist + first_dist) ===")
md_thr, fd_thr = best[1][0], best[1][1]
def is_exit_pred(r):
    return r["min_dist"] < md_thr and r["first_dist"] < fd_thr

n_missed = sum(1 for r in by_dir.get("exiting", []) if not is_exit_pred(r))
n_false = sum(1 for r in by_dir.get("entering", []) if is_exit_pred(r))
print(f"  Missed exits ({n_missed}, true exit but rule says enter):")
for r in by_dir.get("exiting", [])[:10]:
    if not is_exit_pred(r):
        print(f"    {r['burst']}: min={r['min_dist']}mm first={r['first_dist']}mm "
              f"slope={r['slope_mm_per_frame']:+.0f} traj={r['trajectory']}")
print(f"\n  False-alarm exits ({n_false}, true enter but rule says exit):")
for r in by_dir.get("entering", []):
    if is_exit_pred(r):
        print(f"    {r['burst']}: min={r['min_dist']}mm first={r['first_dist']}mm "
              f"slope={r['slope_mm_per_frame']:+.0f} traj={r['trajectory']}")
