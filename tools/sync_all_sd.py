#!/usr/bin/env python3
"""Sync all SD card files to local captures/sd/, skipping existing ones.

Usage:
  python3 tools/sync_all_sd.py [--ip 192.168.0.41] [--dir captures/sd]
"""
import json
import sys
import time
import urllib.request
import urllib.parse
from pathlib import Path


def fetch_sdlist(base_url: str, timeout: float = 60.0) -> list[str]:
    """Fetch full file listing from device. Returns list of paths like 'folder/file'."""
    url = f"{base_url}/sdlist"
    print(f"Fetching file list from {url} (timeout {timeout}s)...")
    with urllib.request.urlopen(url, timeout=timeout) as r:
        data = json.loads(r.read())
    if not data.get("ok"):
        print(f"ERROR: {data.get('error', 'unknown')}")
        sys.exit(1)
    return data.get("files", [])


def download_file(base_url: str, remote_path: str, local_path: Path) -> bool:
    """Download a single file."""
    url = f"{base_url}/sdget?f={urllib.parse.quote(remote_path)}"
    try:
        with urllib.request.urlopen(url, timeout=30) as r:
            data = r.read()
        local_path.write_bytes(data)
        return True
    except Exception as e:
        print(f"    FAILED {remote_path}: {e}")
        return False


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Sync all SD card files to local folder")
    parser.add_argument("--ip", default="192.168.0.41", help="ESP32 IP")
    parser.add_argument("--dir", default="captures/sd", help="Local output directory")
    parser.add_argument("--timeout", type=float, default=60.0, help="Timeout for sdlist fetch")
    args = parser.parse_args()

    base_url = f"http://{args.ip}"
    output_dir = Path(args.dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Fetch remote listing
    remote_files = fetch_sdlist(base_url, timeout=args.timeout)
    print(f"Device has {len(remote_files)} files")

    # Group by folder
    folders = {}
    for f in remote_files:
        parts = f.split("/")
        if len(parts) == 2:
            folder, filename = parts
        else:
            folder, filename = "(root)", f
        folders.setdefault(folder, []).append(f)

    print(f"In {len(folders)} folders")

    # Check local state and download missing
    total_downloaded = 0
    total_skipped = 0
    total_failed = 0

    for folder in sorted(folders.keys()):
        local_folder = output_dir / folder
        local_existing = set()
        if local_folder.exists():
            local_existing = {f.name for f in local_folder.iterdir() if f.is_file()}

        remote_in_folder = folders[folder]
        to_download = []
        for remote_path in remote_in_folder:
            filename = remote_path.split("/")[-1]
            if filename not in local_existing:
                to_download.append((remote_path, filename))

        if not to_download:
            total_skipped += len(remote_in_folder)
            continue

        # Download missing files
        local_folder.mkdir(parents=True, exist_ok=True)
        print(f"  {folder}: {len(to_download)} new / {len(remote_in_folder)} total")

        for remote_path, filename in to_download:
            local_path = local_folder / filename
            if download_file(base_url, remote_path, local_path):
                total_downloaded += 1
            else:
                total_failed += 1

        total_skipped += len(remote_in_folder) - len(to_download)

    print(f"\nDone: {total_downloaded} downloaded, {total_skipped} skipped, {total_failed} failed")


if __name__ == "__main__":
    main()
