"""Pull metadata for missing recent burst folders.
Sequential, with delay between fetches to avoid crashing the board.
"""
import json
import sys
import time
import urllib.request
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SD = REPO / "captures" / "sd"
HOST = "192.168.0.41"

# Get list of folders on remote
print("Fetching /sdfolders...")
try:
    with urllib.request.urlopen(f"http://{HOST}/sdfolders", timeout=60) as r:
        remote = json.loads(r.read())["folders"]
except Exception as e:
    sys.exit(f"sdfolders failed: {e}")

local = {p.name for p in SD.iterdir() if p.is_dir()}
missing = sorted(f for f in remote if f not in local)
print(f"Remote: {len(remote)}  Local: {len(local)}  Missing: {len(missing)}")
if not missing:
    sys.exit(0)

prey_found = []
errors = 0
for i, folder in enumerate(missing, 1):
    dst = SD / folder
    dst.mkdir(parents=True, exist_ok=True)
    url = f"http://{HOST}/sdget?f={folder}/meta.json"
    success = False
    for attempt in range(3):
        try:
            with urllib.request.urlopen(url, timeout=15) as r:
                data = r.read()
            (dst / "meta.json").write_bytes(data)
            meta = json.loads(data)
            success = True
            prey = meta.get("apiResult")
            direction = meta.get("direction", -1)
            DIR = {0: "?", 1: "ENTER", 2: "EXIT"}
            print(f"  [{i}/{len(missing)}] {folder}  prey={prey}  dir={DIR.get(direction, direction)}  "
                  f"min={meta.get('directionMinDist', '?')}mm  first={meta.get('directionFirstDist', '?')}mm")
            if prey == 1:
                prey_found.append(folder)
            break
        except Exception as e:
            if attempt < 2:
                time.sleep(2.0 * (attempt + 1))
            else:
                errors += 1
                print(f"  [{i}/{len(missing)}] {folder}  ERR: {e}", flush=True)
    if success:
        time.sleep(0.5)  # be gentle to the board

print(f"\nDone: {len(missing) - errors}/{len(missing)} fetched, {errors} errors")
print(f"New prey-positive bursts: {len(prey_found)}")
for f in prey_found:
    print(f"  {f}")
