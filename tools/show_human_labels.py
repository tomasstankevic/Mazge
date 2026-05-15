"""Print a summary of human labels in dataset/labels.jsonl."""
import json
from collections import Counter, defaultdict
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
LABELS = REPO / "dataset" / "labels.jsonl"

human_by_burst: dict[str, dict] = defaultdict(dict)
for line in LABELS.open():
    try:
        rec = json.loads(line)
    except Exception:
        continue
    src = rec.get("source", "")
    if not src.startswith("human:"):
        continue
    parts = rec.get("image_id", "").split("/")
    if len(parts) < 4:
        continue
    burst_id = parts[2]
    kind = src.split(":")[-1]
    # Keep last value per (burst, kind) — file is append-only
    human_by_burst[burst_id][kind] = (rec.get("label"), rec.get("confidence"))

print(f"Bursts with at least one human label: {len(human_by_burst)}")
print()

prey = Counter()
direction = Counter()
subject = Counter()
SUBJ = {0: "empty", 1: "cat", 2: "human", 3: "other"}
DIR = {0: "entering", 1: "exiting"}
PREY = {0: "no", 1: "yes"}

for axes in human_by_burst.values():
    if "burst_prey" in axes:
        lab, conf = axes["burst_prey"]
        prey["unclear" if conf == 0.0 else PREY.get(lab, lab)] += 1
    if "burst_direction" in axes:
        lab, conf = axes["burst_direction"]
        direction["unclear" if conf == 0.0 else DIR.get(lab, lab)] += 1
    if "burst_subject" in axes:
        lab, conf = axes["burst_subject"]
        subject["unclear" if conf == 0.0 else SUBJ.get(lab, lab)] += 1

print(f"Prey:      {dict(prey)}")
print(f"Direction: {dict(direction)}")
print(f"Subject:   {dict(subject)}")

# Cross-tab: subject x prey (only confidently labelled bursts)
print()
print("Subject x Prey crosstab:")
cross = Counter()
for axes in human_by_burst.values():
    if "burst_prey" not in axes or "burst_subject" not in axes:
        continue
    p_lab, p_conf = axes["burst_prey"]
    s_lab, s_conf = axes["burst_subject"]
    if p_conf == 0.0 or s_conf == 0.0:
        continue
    cross[(SUBJ.get(s_lab, "?"), PREY.get(p_lab, "?"))] += 1
print(f"  {'subject':>8} {'prey=no':>10} {'prey=yes':>10}")
for sub in ["cat", "human", "other", "empty"]:
    no = cross.get((sub, "no"), 0)
    yes = cross.get((sub, "yes"), 0)
    if no or yes:
        print(f"  {sub:>8} {no:>10} {yes:>10}")
