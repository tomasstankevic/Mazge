"""Scan all burst folders, find prey-detected events, and report which frame
indices most often contained prey.

For each burst:
  - Read meta.json
  - apiResult == 1 means at least one frame triggered prey
  - apiResults[i] is the per-frame verdict: -1=skipped, 0=no prey, 1=prey
"""
import json
from collections import Counter
from pathlib import Path

ROOT = Path("captures/sd")

prey_bursts = []
all_per_frame = Counter()        # how often each frame index gave prey across ALL bursts
prey_burst_per_frame = Counter() # per-frame prey within prey-positive bursts only
checked_per_frame = Counter()    # how often each frame was actually checked (not -1)

for folder in sorted(ROOT.iterdir()):
    if not folder.is_dir():
        continue
    meta = folder / "meta.json"
    if not meta.exists():
        continue
    try:
        m = json.loads(meta.read_text())
    except Exception:
        continue
    api = m.get("apiResult", -1)
    per_frame = m.get("apiResults") or []
    for i, r in enumerate(per_frame):
        if r != -1:
            checked_per_frame[i] += 1
        if r == 1:
            all_per_frame[i] += 1
    if api == 1:
        prey_bursts.append(folder.name)
        for i, r in enumerate(per_frame):
            if r == 1:
                prey_burst_per_frame[i] += 1

print(f"Total burst folders: {sum(1 for f in ROOT.iterdir() if f.is_dir())}")
print(f"Prey-positive bursts: {len(prey_bursts)}")
print()

if not prey_bursts:
    print("No prey detections in any burst.")
    raise SystemExit(0)

print("Frame index | times-prey (all bursts) | times-checked | hit-rate (when checked)")
for i in range(10):
    hit = all_per_frame.get(i, 0)
    chk = checked_per_frame.get(i, 0)
    rate = (hit * 100 / chk) if chk else 0
    bar = "#" * (hit * 30 // max(all_per_frame.values() or [1]))
    print(f"  f{i:02d}       |  {hit:5d}                  |  {chk:5d}        |  {rate:5.1f}%  {bar}")

print()
print("Frame index | prey count WITHIN prey-positive bursts")
for i in range(10):
    cnt = prey_burst_per_frame.get(i, 0)
    pct = (cnt * 100 / len(prey_bursts)) if prey_bursts else 0
    bar = "#" * (cnt * 30 // max(prey_burst_per_frame.values() or [1]))
    print(f"  f{i:02d}       |  {cnt:5d}  ({pct:5.1f}%)  {bar}")

print()
print("Prey-positive bursts (chronological):")
for b in prey_bursts:
    m = json.loads((ROOT / b / "meta.json").read_text())
    per = m.get("apiResults") or []
    flags = "".join("P" if r == 1 else ("." if r == 0 else "-") for r in per)
    print(f"  {b}  [{flags}]   sent={m.get('apiFramesSent')}")
