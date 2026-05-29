#!/usr/bin/env python3

import csv
from collections import Counter


rows = list(csv.DictReader(open("dataset/bursts.csv")))
labeled = [r for r in rows if r.get("human_prey") in ("0", "1")]


def cm(pred_fn):
    tp = fp = fn = tn = 0
    for r in labeled:
        y = int(r["human_prey"])
        p = 1 if pred_fn(r) else 0
        if p == 1 and y == 1:
            tp += 1
        elif p == 1 and y == 0:
            fp += 1
        elif p == 0 and y == 1:
            fn += 1
        else:
            tn += 1
    return tp, fp, fn, tn


def fw_count(r):
    return int(r.get("fw_prey_count") or 0)


fw_current = cm(lambda r: r.get("fw_burst_label") == "1")      # >=2
fw_tiered = cm(lambda r: fw_count(r) >= 1)                        # >=1 trigger

_, fp_cur, fn_cur, _ = fw_current
_, fp_tier, fn_tier, _ = fw_tiered

fps_tiered = [r for r in labeled if int(r["human_prey"]) == 0 and fw_count(r) >= 1]
fp_minutes_tiered = sum(3 if fw_count(r) == 1 else 15 for r in fps_tiered)
fp_minutes_current = 15 * fp_cur

fp_by_count = Counter(fw_count(r) for r in fps_tiered)

print(f"labeled_bursts={len(labeled)}")
print(f"fw_current_tp_fp_fn_tn={fw_current}")
print(f"fw_tiered_tp_fp_fn_tn={fw_tiered}")
print(f"delta_fp={fp_tier - fp_cur}")
print(f"delta_fn={fn_tier - fn_cur}")
print(f"fp_minutes_current={fp_minutes_current}")
print(f"fp_minutes_tiered={fp_minutes_tiered}")
print(f"delta_fp_minutes={fp_minutes_tiered - fp_minutes_current}")
print(f"fp_by_fw_prey_count={dict(sorted(fp_by_count.items()))}")
