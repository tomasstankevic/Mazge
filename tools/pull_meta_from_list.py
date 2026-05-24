"""Pull meta.json for folder names listed in /tmp/new_folders.json.

Gentle: one folder at a time with retries and short sleep, never re-hits
/sdfolders or /sdlist (those tend to wedge the HTTP task).
"""
import json
import sys
import time
import urllib.request
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SD = REPO / "captures" / "sd"
HOST = "192.168.0.41"

list_path = Path("/tmp/new_folders.json")
folders = json.loads(list_path.read_text())
print(f"Pulling meta.json for {len(folders)} folders")

prey_found, no_meta, errors = [], [], 0
DIR = {0: "?", 1: "ENTER", 2: "EXIT"}

for i, folder in enumerate(folders, 1):
    dst = SD / folder
    dst.mkdir(parents=True, exist_ok=True)
    if (dst / "meta.json").exists():
        continue  # idempotent
    url = f"http://{HOST}/sdget?f={folder}/meta.json"
    success = False
    for attempt in range(4):
        try:
            with urllib.request.urlopen(url, timeout=20) as r:
                data = r.read()
            (dst / "meta.json").write_bytes(data)
            try:
                meta = json.loads(data)
            except Exception:
                meta = {}
            success = True
            prey = meta.get("apiResult")
            direction = meta.get("direction", -1)
            print(f"  [{i}/{len(folders)}] {folder}  prey={prey}  "
                  f"dir={DIR.get(direction, direction)}  "
                  f"min={meta.get('directionMinDist', '?')}mm  "
                  f"first={meta.get('directionFirstDist', '?')}mm")
            if prey == 1:
                prey_found.append(folder)
            break
        except Exception as e:
            if attempt < 3:
                time.sleep(1.5 * (attempt + 1))
            else:
                errors += 1
                print(f"  [{i}/{len(folders)}] {folder}  ERR: {e}", flush=True)
    if success:
        time.sleep(0.35)

print(f"\nDone: {len(folders) - errors}/{len(folders)} fetched, {errors} errors")
print(f"Prey-positive bursts: {len(prey_found)}")
for f in prey_found:
    print(f"  {f}")
Path("/tmp/new_prey_folders.json").write_text(json.dumps(prey_found))
