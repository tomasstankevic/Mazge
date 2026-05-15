"""Download full JPGs for prey-positive bursts."""
import json
import urllib.request
from pathlib import Path

ROOT = Path("captures/sd")
HOST = "192.168.0.41"
prey_folders = [
    "20260512_013740_gen1",
    "20260512_014807_gen6",
    "20260514_000754_gen5",
    "20260514_233130_gen2",
    "20260515_031942_gen14",
    "20260515_032008_gen16",
    "20260515_032343_gen17",
    "20260515_032406_gen18",
]

for folder in prey_folders:
    url = f"http://{HOST}/sdlist?dir={folder}"
    with urllib.request.urlopen(url, timeout=30) as r:
        d = json.loads(r.read())
    files = [f.split("/")[-1] for f in d.get("files", [])]
    local_dir = ROOT / folder
    local_dir.mkdir(parents=True, exist_ok=True)
    local_files = {f.name for f in local_dir.iterdir()}
    to_get = [f for f in files if f not in local_files]
    if not to_get:
        print(f"{folder}: already complete")
        continue
    print(f"{folder}: downloading {len(to_get)} files...", flush=True)
    for name in to_get:
        get_url = f"http://{HOST}/sdget?f={folder}/{name}"
        with urllib.request.urlopen(get_url, timeout=60) as r:
            data = r.read()
        (local_dir / name).write_bytes(data)
    print(f"  done ({len(to_get)} files)", flush=True)

print("All prey bursts downloaded.")
