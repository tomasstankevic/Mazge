#!/usr/bin/env python3
"""Simple SD card downloader - fetch files via HTTP."""
import sys
import urllib.request
import urllib.parse
from pathlib import Path

def download_file(base_url: str, remote_path: str, output_dir: Path):
    """Download a single file from the SD card."""
    output_dir.mkdir(parents=True, exist_ok=True)
    local_path = output_dir / remote_path.replace("/", "_")
    
    if local_path.exists():
        print(f"  ⊘ {remote_path} (already exists)")
        return None
    
    url = f"{base_url}/sdget?f={urllib.parse.quote(remote_path)}"
    print(f"Downloading: {remote_path}")
    try:
        with urllib.request.urlopen(url, timeout=30) as r:
            data = r.read()
        local_path.write_bytes(data)
        print(f"  ✓ {len(data)} bytes → {local_path}")
        return True
    except Exception as e:
        print(f"  ✗ Failed: {e}")
        return False

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 simple_sd_download.py <file1> [file2] ... [--ip 192.168.0.41] [--dir captures/sd]")
        print("Example: python3 simple_sd_download.py burst_000061_gen1/f00.jpg burst_000020_gen1/f01.jpg")
        sys.exit(1)
    
    # Parse args
    files = []
    ip = "192.168.0.41"
    output_dir = Path("captures/sd")
    
    for arg in sys.argv[1:]:
        if arg.startswith("--ip"):
            ip = arg.split("=")[1] if "=" in arg else sys.argv[sys.argv.index(arg) + 1]
        elif arg.startswith("--dir"):
            output_dir = Path(arg.split("=")[1] if "=" in arg else sys.argv[sys.argv.index(arg) + 1])
        else:
            files.append(arg)
    
    base_url = f"http://{ip}"
    success = 0
    skipped = 0
    failed = 0
    for f in files:
        result = download_file(base_url, f, output_dir)
        if result is True:
            success += 1
        elif result is None:
            skipped += 1
        else:
            failed += 1
    
    print(f"\nResults: {success} downloaded, {skipped} skipped, {failed} failed")
