"""Download all images for bursts that have only meta.json locally.

Uses /sdlist to discover actual filenames (handles both old fNN.jpg and
new fNN_NNNNms.jpg formats).
"""
import json
import os
import re
import sys
import urllib.parse
import urllib.request
from pathlib import Path

HOST = sys.argv[1] if len(sys.argv) > 1 else "192.168.0.41"
LOCAL = Path("captures/sd")


def remote_files():
    """Get all files from device /sdlist as list of strings."""
    url = f"http://{HOST}/sdlist"
    with urllib.request.urlopen(url, timeout=120) as r:
        text = r.read().decode("utf-8", errors="ignore")
    # Robust extraction (sdlist may have malformed JSON for very long lists)
    return re.findall(r'"([^"]+\.(?:jpg|json))"', text)


def download(remote_path, local_path):
    url = f"http://{HOST}/sdget?f={urllib.parse.quote(remote_path)}"
    try:
        with urllib.request.urlopen(url, timeout=60) as r:
            data = r.read()
        local_path.parent.mkdir(parents=True, exist_ok=True)
        local_path.write_bytes(data)
        return len(data)
    except Exception as e:
        print(f"  ERR {remote_path}: {e}")
        return 0


def main():
    files = remote_files()
    print(f"Remote total files: {len(files)}")

    # Group by burst folder
    by_folder = {}
    for f in files:
        if "/" not in f:
            continue
        folder, name = f.split("/", 1)
        by_folder.setdefault(folder, []).append(name)

    target_only_meta = []  # folders with only meta locally
    for d in sorted(LOCAL.iterdir()):
        if not d.is_dir():
            continue
        local_files = {f.name for f in d.iterdir() if f.is_file()}
        # If only meta.json (or fewer than expected jpgs)
        local_jpgs = sum(1 for f in local_files if f.endswith(".jpg"))
        if local_jpgs == 0 and "meta.json" in local_files:
            target_only_meta.append(d.name)

    print(f"Bursts missing JPGs: {len(target_only_meta)}")

    total_downloaded = 0
    for i, folder in enumerate(target_only_meta, 1):
        remote = by_folder.get(folder, [])
        if not remote:
            print(f"[{i}/{len(target_only_meta)}] {folder}: no remote files")
            continue
        local_dir = LOCAL / folder
        local_set = {f.name for f in local_dir.iterdir() if f.is_file()}
        to_get = [n for n in remote if n not in local_set]
        if not to_get:
            continue
        print(f"[{i}/{len(target_only_meta)}] {folder}: downloading {len(to_get)} files")
        for name in to_get:
            n = download(f"{folder}/{name}", local_dir / name)
            if n > 0:
                total_downloaded += 1

    print(f"\nDone. Downloaded {total_downloaded} files.")


if __name__ == "__main__":
    main()
