"""Check if any false-alarm exits (entering bursts predicted as exit) carry prey."""
import json
from collections import defaultdict
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
human = defaultdict(dict)
for line in (REPO / "dataset" / "labels.jsonl").open():
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
    human[parts[2]][src.split(":")[-1]] = (rec.get("label"), rec.get("confidence"))

# Run the same exit-rule on ALL bursts and find any prey-positive that would
# be misclassified as exit.
SD = REPO / "captures" / "sd"
SUBJ = {0: "empty", 1: "cat", 2: "human", 3: "other"}

def burst_features(burst_dir):
    meta_p = burst_dir / "meta.json"
    if not meta_p.exists():
        return None
    try:
        meta = json.loads(meta_p.read_text())
    except Exception:
        return None
    images = meta.get("images", [])
    dists = [img.get("dist", img.get("distance", -1)) for img in images]
    valid = [(i, v) for i, v in enumerate(dists) if v >= 0]
    if not valid:
        return None
    return {
        "min_dist": min(v for _, v in valid),
        "first_dist": valid[0][1],
    }

def predict_exit(f):
    return f["min_dist"] < 180 and f["first_dist"] < 230

danger = []
ok_exits_filtered = 0
total_prey = 0
for d in sorted(SD.iterdir()):
    if not d.is_dir():
        continue
    h = human.get(d.name, {})
    p_lab, p_conf = h.get("burst_prey", (None, None))
    is_prey = (p_lab == 1 and p_conf == 1.0)
    if is_prey:
        total_prey += 1
    f = burst_features(d)
    if not f:
        continue
    if predict_exit(f):
        if is_prey:
            danger.append((d.name, f["min_dist"], f["first_dist"]))
        else:
            ok_exits_filtered += 1

print(f"Total bursts with confirmed prey: {total_prey}")
print(f"Bursts the exit-rule would skip API for: {ok_exits_filtered + len(danger)}")
print()
if danger:
    print(f"!!! {len(danger)} PREY bursts would be misclassified as exit (let through):")
    for b, m, f in danger:
        print(f"  {b}: min_dist={m}mm  first_dist={f}mm")
else:
    print("No prey bursts would be misclassified as exit. Rule is SAFE.")
