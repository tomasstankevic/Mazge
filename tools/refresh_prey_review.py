"""Refresh the prey-review HTML report with all prey-positive bursts.

Steps:
  1. Pull /sdlist from device, find missing burst folders locally
  2. Download missing folders (using actual filenames from sdlist)
  3. Scan all local burst folders, find prey-positive ones (apiResult==1)
  4. Copy/sync them into captures/prey_review/
  5. Regenerate captures/prey_review/index.html

Re-runnable any time. Idempotent.

Usage: python tools/refresh_prey_review.py [--host 192.168.0.41]
"""
import argparse
import json
import re
import shutil
import subprocess
import sys
import urllib.parse
import urllib.request
from pathlib import Path

ROOT = Path("captures/sd")
REVIEW = Path("captures/prey_review")


def remote_files(host):
    """Get all files from /sdlist (handles truncation by extracting matches)."""
    url = f"http://{host}/sdlist"
    try:
        with urllib.request.urlopen(url, timeout=180) as r:
            text = r.read().decode("utf-8", errors="ignore")
    except Exception as e:
        print(f"ERROR fetching sdlist: {e}")
        return []
    # Robust extraction (sdlist may be truncated)
    return re.findall(r'"([^"]+\.(?:jpg|json))"', text)


def download(host, remote_path, local_path):
    url = f"http://{host}/sdget?f={urllib.parse.quote(remote_path)}"
    try:
        with urllib.request.urlopen(url, timeout=60) as r:
            data = r.read()
    except Exception as e:
        return 0, str(e)
    local_path.parent.mkdir(parents=True, exist_ok=True)
    local_path.write_bytes(data)
    return len(data), ""


def sync_folder(host, folder, remote_files_in_folder):
    """Download any files for `folder` that are not already local."""
    local_dir = ROOT / folder
    local_set = (
        {f.name for f in local_dir.iterdir() if f.is_file()}
        if local_dir.exists() else set()
    )
    to_get = [n for n in remote_files_in_folder if n not in local_set]
    if not to_get:
        return 0
    print(f"  syncing {folder}: {len(to_get)} files...")
    downloaded = 0
    for name in to_get:
        n, err = download(host, f"{folder}/{name}", local_dir / name)
        if n > 0:
            downloaded += 1
        else:
            print(f"    ERR {name}: {err}")
    return downloaded


def is_prey_positive(meta_path):
    try:
        m = json.loads(meta_path.read_text())
    except Exception:
        return False
    return m.get("apiResult") == 1


def find_local_prey_bursts():
    return sorted(
        d.name for d in ROOT.iterdir()
        if d.is_dir() and (d / "meta.json").exists()
        and is_prey_positive(d / "meta.json")
    )


def copy_to_review(burst):
    """Copy a prey burst folder into captures/prey_review/, skip files
    that are already there with the same size."""
    src = ROOT / burst
    dst = REVIEW / burst
    dst.mkdir(parents=True, exist_ok=True)
    copied = 0
    for f in src.iterdir():
        if not f.is_file():
            continue
        d = dst / f.name
        if d.exists() and d.stat().st_size == f.stat().st_size:
            continue
        shutil.copy2(f, d)
        copied += 1
    return copied


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.0.41")
    ap.add_argument("--skip-sync", action="store_true",
                    help="Skip downloading from device (use local data only)")
    args = ap.parse_args()

    if not args.skip_sync:
        print("=== Step 1: Discover device files ===")
        files = remote_files(args.host)
        print(f"Got {len(files)} entries from /sdlist")

        by_folder = {}
        for f in files:
            if "/" not in f:
                continue
            folder, name = f.split("/", 1)
            by_folder.setdefault(folder, []).append(name)
        print(f"  -> {len(by_folder)} unique folders on device\n")

        print("=== Step 2: Sync missing folders / files ===")
        local = {p.name for p in ROOT.iterdir() if p.is_dir()}
        new_folders = [f for f in by_folder if f not in local]
        if new_folders:
            print(f"New folders to fetch: {len(new_folders)}")
        # Sync new folders
        for folder in sorted(by_folder):
            if folder in local and (ROOT / folder / "meta.json").exists():
                continue  # already have meta, skip (we'll fill in jpgs only if prey)
            sync_folder(args.host, folder, by_folder[folder])

    print("\n=== Step 3: Find prey-positive bursts locally ===")
    prey_bursts = find_local_prey_bursts()
    print(f"Found {len(prey_bursts)} prey-positive bursts")

    if not args.skip_sync:
        print("\n=== Step 4: Ensure all JPGs are downloaded for prey bursts ===")
        for burst in prey_bursts:
            local_dir = ROOT / burst
            jpgs_local = sum(1 for f in local_dir.iterdir() if f.suffix == ".jpg")
            if jpgs_local < 8 and burst in by_folder:
                sync_folder(args.host, burst, by_folder[burst])

    print("\n=== Step 5: Copy prey bursts into captures/prey_review/ ===")
    REVIEW.mkdir(parents=True, exist_ok=True)
    total_copied = 0
    for burst in prey_bursts:
        n = copy_to_review(burst)
        if n > 0:
            print(f"  + {burst}: {n} files")
            total_copied += n
    print(f"Total files copied: {total_copied}")

    print("\n=== Step 6: Regenerate review HTML ===")
    subprocess.run(["python3", "tools/prey_review_html.py"], check=False)

    print("\n=== Step 7: Regenerate review PDF ===")
    subprocess.run(["uv", "run", "python", "tools/prey_review_pdf.py"], check=False)

    print("\nDone. Open captures/prey_review/index.html or report.pdf to view.")


if __name__ == "__main__":
    main()
