"""Download JPGs for all bursts from a given day (or set).
Used to backfill so the labeler can show all frames."""
import json
import sys
import time
import urllib.request
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SD = REPO / "captures" / "sd"
HOST = "192.168.0.41"

# Day prefixes to fetch (multiple allowed)
DAYS = sys.argv[1:] or ["20260520", "20260521"]

def fetch(url, timeout=30):
    for tr in range(4):
        try:
            with urllib.request.urlopen(url, timeout=timeout) as r:
                return r.read()
        except Exception as e:
            if tr < 3:
                time.sleep(2.0 * (tr + 1))
            else:
                raise

burst_dirs = sorted(d for d in SD.iterdir() if d.is_dir()
                    and any(d.name.startswith(day) for day in DAYS))
print(f"Checking {len(burst_dirs)} bursts for missing JPGs (days={DAYS})")

total_files = 0
total_bytes = 0
for d in burst_dirs:
    mp = d / "meta.json"
    if not mp.exists():
        continue
    try:
        meta = json.loads(mp.read_text())
    except Exception:
        continue
    images = meta.get("images", [])
    if not images:
        continue
    # what's already on disk?
    present = {f.name for f in d.iterdir() if f.suffix == ".jpg"}
    missing = [img["f"] for img in images if img.get("f") and img["f"] not in present]
    if not missing:
        continue
    print(f"  {d.name}: {len(missing)} files", flush=True)
    for name in missing:
        try:
            data = fetch(f"http://{HOST}/sdget?f={d.name}/{name}")
            if len(data) < 1000:
                print(f"    short response for {name}: {len(data)}B")
                continue
            (d / name).write_bytes(data)
            total_files += 1
            total_bytes += len(data)
        except Exception as e:
            print(f"    ERR {name}: {e}")
        time.sleep(0.3)

print(f"\nDone: {total_files} files, {total_bytes//1024}KB total")
