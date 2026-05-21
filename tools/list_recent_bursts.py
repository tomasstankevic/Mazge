"""List tonight's bursts (2026-05-20 evening + 2026-05-21)."""
import json
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SD = REPO / "captures" / "sd"

labeled = set()
for line in (REPO / "dataset" / "labels.jsonl").open():
    try: rec = json.loads(line)
    except: continue
    src = rec.get("source", "")
    if not (src.startswith("human:") and src.endswith(":burst_prey")):
        continue
    parts = rec.get("image_id", "").split("/")
    if len(parts) >= 4:
        labeled.add(parts[2])

recent = []
for d in sorted(SD.iterdir()):
    if not d.is_dir():
        continue
    n = d.name
    if not ("20260520_1" in n or "20260520_2" in n or n.startswith("20260521_")):
        continue
    if (d / "meta.json").exists():
        try:
            m = json.loads((d / "meta.json").read_text())
            recent.append((n, m.get("apiResult", "?"), m.get("direction", "?"),
                          m.get("directionMinDist", "?"), m.get("directionFirstDist", "?"),
                          n in labeled))
        except Exception:
            pass

print(f"Bursts from last ~24h: {len(recent)}")
print(f"  labeled:   {sum(1 for *_, lab in recent if lab)}")
print(f"  unlabeled: {sum(1 for *_, lab in recent if not lab)}")
print()
DIR = {0: "?", 1: "ENTER", 2: "EXIT"}
print(f"{'burst':<28} {'api':<5} {'dir':<6} {'minDist':<8} {'first':<8} labeled")
for n, p, d, mind, firstd, lab in recent:
    pmark = "PREY" if p == 1 else ("clear" if p == 0 else "?")
    print(f"  {n:<26} {pmark:<5} {DIR.get(d, d):<6} {mind:<8} {firstd:<8} {'X' if lab else ''}")
