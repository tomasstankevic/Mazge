"""List missing burst folders (on device but not local) and download them all."""
import json
import os
import subprocess
import sys
from pathlib import Path

import urllib.request

HOST = "192.168.0.41"
LOCAL = Path("captures/sd")


def remote_list():
    url = f"http://{HOST}/sdlist"
    with urllib.request.urlopen(url, timeout=120) as r:
        d = json.loads(r.read())
    files = d.get("files", [])
    folders = sorted({f.split("/")[0] for f in files if "/" in f})
    return folders


def main():
    folders = remote_list()
    print(f"Remote folders: {len(folders)}")
    local = {p.name for p in LOCAL.iterdir() if p.is_dir()}
    missing = [f for f in folders if f not in local]
    print(f"Missing locally: {len(missing)}")
    if not missing:
        return
    for i, f in enumerate(missing, 1):
        print(f"[{i}/{len(missing)}] downloading {f}")
        subprocess.run(["python3", "tools/sync_sd.py", f], check=False)


if __name__ == "__main__":
    main()
