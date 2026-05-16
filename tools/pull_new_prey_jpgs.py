"""Download full JPGs for new prey-positive bursts (2026-05-16 batch)."""
import json
import time
import urllib.request
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SD = REPO / "captures" / "sd"
HOST = "192.168.0.41"

prey = ["20260516_023522_gen1", "20260516_024006_gen2"]

for folder in prey:
    url = f"http://{HOST}/sdlist?dir={folder}"
    with urllib.request.urlopen(url, timeout=30) as r:
        d = json.loads(r.read())
    files = [f.split("/")[-1] for f in d.get("files", [])]
    local_dir = SD / folder
    local_dir.mkdir(parents=True, exist_ok=True)
    local_files = {f.name for f in local_dir.iterdir()}
    to_get = [f for f in files if f not in local_files]
    if not to_get:
        print(f"{folder}: already complete")
        continue
    print(f"{folder}: downloading {len(to_get)} files...", flush=True)
    for name in to_get:
        for attempt in range(3):
            try:
                get_url = f"http://{HOST}/sdget?f={folder}/{name}"
                with urllib.request.urlopen(get_url, timeout=60) as r:
                    data = r.read()
                (local_dir / name).write_bytes(data)
                break
            except Exception as e:
                if attempt < 2:
                    time.sleep(2.0 * (attempt + 1))
                else:
                    print(f"  ERR {name}: {e}")
        time.sleep(0.3)
    print(f"  done ({len(to_get)} files)")

print("All prey bursts downloaded.")
