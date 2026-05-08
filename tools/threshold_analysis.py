"""Threshold analysis: precision/recall at N=1..10 required prey-flagged frames."""
import json
from pathlib import Path

ROOT = Path("captures/prey_review")
reproc_b = json.loads((ROOT / "reprocess.json").read_text())

TRUE_BURSTS = {
    "20260501_015148_gen1",
    "20260504_021353_gen10",
    "20260505_005540_gen1",
    "20260505_220846_gen8",
    "20260505_221411_gen11",
    "20260505_230846_gen22",
    "20260505_231157_gen23",
    "20260507_032001_gen3",
    "20260507_050927_gen5",
    "20260507_212120_gen9",
    "20260508_035852_gen1",
}

bursts = sorted(reproc_b.keys())
true_set = TRUE_BURSTS & set(bursts)
false_set = set(bursts) - true_set

flagged = {}
for b in bursts:
    flagged[b] = sum(1 for v in reproc_b[b]["verdicts"] if v == 1)

print(f"Dataset: {len(bursts)} bursts ({len(true_set)} true prey, {len(false_set)} false-pos)\n")
print("Per-burst flagged frame counts:")
print(f"  {'Burst':<32} {'flags':>5}  truth")
for b in sorted(bursts):
    truth = "TRUE PREY" if b in true_set else "false"
    print(f"  {b:<32} {flagged[b]:>5}  {truth}")

print(f"\nThreshold sweep (require N flagged frames to trigger lockout):\n")
print(f"{'N':>3} {'TP':>4} {'FP':>4} {'FN':>4} {'TN':>4} "
      f"{'precision':>10} {'recall':>8} {'F1':>6}")
for N in range(1, 11):
    tp = sum(1 for b in true_set if flagged[b] >= N)
    fp = sum(1 for b in false_set if flagged[b] >= N)
    fn = len(true_set) - tp
    tn = len(false_set) - fp
    p = tp / (tp + fp) if (tp + fp) else 0
    r = tp / (tp + fn) if (tp + fn) else 0
    f1 = (2 * p * r / (p + r)) if (p + r) else 0
    print(f"{N:>3} {tp:>4} {fp:>4} {fn:>4} {tn:>4} "
          f"{p*100:>9.0f}% {r*100:>7.0f}% {f1*100:>5.0f}%")
