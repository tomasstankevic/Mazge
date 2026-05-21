"""Download full JPGs for new prey-positive bursts (2026-05-21 sync)."""
import json
import time
import urllib.request
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SD = REPO / "captures" / "sd"
HOST = "192.168.0.41"

prey = ["20260517_010914_gen1", "20260521_013251_gen4"]

for folder in prey:
    url = f"http://{HOST}/sdlist?dir={folder}"
    for tr in range(3):
        try:
            with urllib.request.urlopen(url, timeout=30) as r:
                d = json.loads(r.read())
            break
        except Exception as e:
            print(f"  list {folder} try {tr}: {e}")
            time.sleep(3)
    else:
        continue
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
        for attempt in range(5):
            try:
                with urllib.request.urlopen(
                    f"http://{HOST}/sdget?f={folder}/{name}", timeout=30) as r:
                    data = r.read()
                if len(data) > 1000:
                    (local_dir / name).write_bytes(data)
                    print(f"  {name}: {len(data)//1024}KB", flush=True)
                    break
                else:
                    raise RuntimeError(f"short response: {len(data)}B")
            except Exception as e:
                if attempt < 4:
                    time.sleep(3.0 * (attempt + 1))
                else:
                    print(f"  ERR {name}: {e}")
        time.sleep(0.5)

print("Done.")
