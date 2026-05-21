"""Find bursts where firmware said no-prey but human said yes (false negatives)."""
import json
from collections import defaultdict
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SD = REPO / "captures" / "sd"

# Build per-burst index of latest human prey verdict
human_prey: dict[str, tuple[int, float, str]] = {}
for line in (REPO / "dataset" / "labels.jsonl").open():
    try: rec = json.loads(line)
    except: continue
    src = rec.get("source", "")
    if not (src.startswith("human:") and src.endswith(":burst_prey")):
        continue
    parts = rec.get("image_id", "").split("/")
    if len(parts) < 4: continue
    burst_id = parts[2]
    human_prey[burst_id] = (rec.get("label"), rec.get("confidence"), rec.get("ts", ""))

# Walk burst folders, find mismatches
false_neg = []  # human=yes, fw=no
false_pos = []  # human=no,  fw=yes
for d in sorted(SD.iterdir()):
    if not d.is_dir(): continue
    mp = d / "meta.json"
    if not mp.exists(): continue
    try: m = json.loads(mp.read_text())
    except: continue
    fw = m.get("apiResult", -1)
    hp = human_prey.get(d.name)
    if hp is None: continue
    h_lab, h_conf, h_ts = hp
    if h_conf == 0.0: continue  # unclear
    if h_lab == 1 and fw == 0:
        false_neg.append((d.name, m, h_ts))
    elif h_lab == 0 and fw == 1:
        false_pos.append((d.name, m, h_ts))

print(f"False negatives (human=PREY, firmware=clear): {len(false_neg)}")
for name, m, ts in false_neg:
    direction = m.get("direction", "?")
    DIR = {0:"?", 1:"ENTER", 2:"EXIT"}
    sent = m.get("apiFramesSent", "?")
    results = m.get("apiResults", [])
    fw_prey_count = sum(1 for r in results if r == 1)
    print(f"  {name}")
    print(f"    direction={DIR.get(direction, direction)} minDist={m.get('directionMinDist', '?')}mm firstDist={m.get('directionFirstDist', '?')}mm")
    print(f"    apiFramesSent={sent}  apiResults={results}  fw_prey_count={fw_prey_count}")
    print(f"    labeled_at={ts}")

print()
print(f"False positives (firmware=PREY, human=clear): {len(false_pos)}")
for name, m, ts in false_pos:
    print(f"  {name}  fw_prey_count={sum(1 for r in m.get('apiResults', []) if r == 1)}/{m.get('apiFramesSent','?')}")
