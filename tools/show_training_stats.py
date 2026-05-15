"""Show full training-ready label stats."""
import csv
from collections import Counter
from pathlib import Path

rows = list(csv.DictReader(open(Path(__file__).parent.parent / "dataset" / "manifest.csv")))
print(f"Total frames: {len(rows)}")
print()
print("=== weak_label x weak_confidence (training distribution) ===")
buckets = Counter((r["weak_label"], r["weak_confidence"]) for r in rows)
for (lab, conf), n in sorted(buckets.items()):
    print(f"  label={lab:>2}  conf={conf:>4}  n={n:>5}")
print()
print("=== Per-split distribution ===")
print(f"{'split':>6}  {'prey_hard':>10}  {'prey_weak':>10}  {'no_prey':>10}")
for split in ["train", "val", "test"]:
    s = [r for r in rows if r["split"] == split and r["weak_label"] != ""]
    ph = sum(1 for r in s if r["weak_label"] == "1" and r["weak_confidence"] == "1.0")
    pw = sum(1 for r in s if r["weak_label"] == "1" and r["weak_confidence"] != "1.0")
    nn = sum(1 for r in s if r["weak_label"] == "0")
    print(f"{split:>6}  {ph:>10}  {pw:>10}  {nn:>10}")
print()
print("=== Subject (human-confirmed > YOLO) ===")
print(dict(Counter(r["subject"] for r in rows if r["subject"] != "")))
print()
print("=== Human direction (per-frame) ===")
print(dict(Counter(r["human_direction"] for r in rows if r["human_direction"] != "")))
