"""Pull meta.json for all missing recent SD folders and report prey detections."""
import json
import sys
import urllib.request
from pathlib import Path

ROOT = Path("captures/sd")
HOST = "192.168.0.41"

folders_json = json.loads(Path("/tmp/sdfolders.json").read_text())
all_folders = sorted(folders_json["folders"])
local = {p.name for p in ROOT.iterdir() if p.is_dir()}
missing = [f for f in all_folders if f.startswith("2026051") and f not in local]
print(f"Missing recent folders: {len(missing)}", flush=True)

prey_found = []
errors = 0
for i, folder in enumerate(missing):
    dst = ROOT / folder
    dst.mkdir(parents=True, exist_ok=True)
    url = f"http://{HOST}/sdget?f={folder}/meta.json"
    try:
        with urllib.request.urlopen(url, timeout=15) as r:
            data = r.read()
        (dst / "meta.json").write_bytes(data)
        meta = json.loads(data)
        prey = meta.get("apiResult", -1)
        if prey == 1:
            prey_found.append(folder)
            print(f"  PREY: {folder}", flush=True)
    except Exception as e:
        errors += 1
        if errors <= 5:
            print(f"  ERR: {folder} - {e}", flush=True)
    if (i + 1) % 50 == 0:
        print(f"  ... {i+1}/{len(missing)} done", flush=True)

print(f"\nDone: {len(missing)} folders checked, {errors} errors")
print(f"New prey-positive bursts: {len(prey_found)}")
for f in prey_found:
    print(f"  {f}")
