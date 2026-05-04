#!/usr/bin/env python3
"""Sync SD card with local captures/sd folder - download new files."""
import os
import sys
import urllib.request
import urllib.parse
from pathlib import Path

def scan_local_sd(sd_dir: Path) -> dict:
    """Scan local SD folder structure."""
    folders = {}
    for item in sd_dir.iterdir():
        if item.is_dir() and not item.name.startswith('.'):
            files = [f.name for f in item.iterdir() if f.is_file()]
            folders[item.name] = files
    return folders

def get_remote_files(base_url: str, folder: str) -> list:
    """Get file list for a folder from device."""
    files = []
    for i in range(20):  # Try fetching files f00.jpg through f19.jpg
        filename = f"f{i:02d}.jpg"
        url = f"{base_url}/sdget?f={urllib.parse.quote(folder + '/' + filename)}"
        try:
            with urllib.request.urlopen(url, timeout=5) as r:
                r.read()  # Just check if it exists
                files.append(filename)
        except urllib.error.HTTPError as e:
            if e.code == 404:
                break  # File doesn't exist, stop trying
        except Exception:
            break  # Connection error, stop
    # Metadata file is optional but usually present.
    meta_url = f"{base_url}/sdget?f={urllib.parse.quote(folder + '/meta.json')}"
    try:
        with urllib.request.urlopen(meta_url, timeout=5) as r:
            r.read()
            files.append("meta.json")
    except Exception:
        pass
    return files

def download_file(base_url: str, folder: str, filename: str, output_path: Path) -> bool:
    """Download a single file."""
    url = f"{base_url}/sdget?f={urllib.parse.quote(folder + '/' + filename)}"
    try:
        with urllib.request.urlopen(url, timeout=30) as r:
            data = r.read()
        output_path.write_bytes(data)
        print(f"  ✓ {filename} ({len(data)} bytes)")
        return True
    except Exception as e:
        print(f"  ✗ {filename}: {e}")
        return False

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 sync_sd.py <folder_name> [--ip 192.168.0.41]")
        print("Example: python3 sync_sd.py 20260504_051349_gen2")
        sys.exit(1)
    
    folder = sys.argv[1]
    ip = "192.168.0.41"
    if "--ip" in sys.argv:
        ip = sys.argv[sys.argv.index("--ip") + 1]
    
    base_url = f"http://{ip}"
    sd_dir = Path("captures/sd")
    local_folder = sd_dir / folder
    
    # Check what's already downloaded
    local_files = set()
    if local_folder.exists():
        local_files = {f.name for f in local_folder.iterdir() if f.is_file()}
    
    print(f"Local files in {folder}: {len(local_files)}")
    
    # Get what's on the device
    print(f"Checking device for {folder}...")
    remote_files = get_remote_files(base_url, folder)
    print(f"Device has: {len(remote_files)} files")
    
    # Download missing ones
    to_download = set(remote_files) - local_files
    if not to_download:
        print("✓ All files already downloaded")
        sys.exit(0)
    
    print(f"\nDownloading {len(to_download)} new files:")
    local_folder.mkdir(parents=True, exist_ok=True)
    
    success = 0
    for filename in sorted(to_download):
        if download_file(base_url, folder, filename, local_folder / filename):
            success += 1
    
    print(f"\nResults: {success}/{len(to_download)} downloaded")
