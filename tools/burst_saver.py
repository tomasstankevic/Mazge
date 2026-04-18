#!/usr/bin/env python3
"""Auto-download burst captures from ESP32-CAM.

Polls the /stats endpoint, detects new bursts via burstGen counter,
and downloads all images to a local folder organized by timestamp.

Usage: python3 tools/burst_saver.py [--ip 192.168.0.42] [--dir captures]
"""
import argparse
import io
import json
import os
import time
import urllib.request
from datetime import datetime
from PIL import Image


def main():
    parser = argparse.ArgumentParser(description="Auto-save ESP32-CAM burst captures")
    parser.add_argument("--ip", default="192.168.0.42", help="ESP32 IP address")
    parser.add_argument("--dir", default="captures", help="Output directory")
    parser.add_argument("--poll", type=float, default=1.0, help="Poll interval (seconds)")
    args = parser.parse_args()

    base_url = f"http://{args.ip}"
    os.makedirs(args.dir, exist_ok=True)

    last_gen = 0
    print(f"Watching {base_url} for new bursts... (saving to {args.dir}/)")

    while True:
        try:
            with urllib.request.urlopen(f"{base_url}/stats", timeout=3) as r:
                stats = json.loads(r.read())

            gen = stats.get("burstGen", 0)
            archives = stats.get("burstArchives", 0)
            counts = stats.get("burstCounts", [])
            dist = stats.get("distance", -1)

            if gen != last_gen and gen > 0 and archives > 0:
                # New burst detected — download the latest archive
                a = archives - 1
                count = counts[a] if a < len(counts) else 0
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                burst_dir = os.path.join(args.dir, f"burst_{ts}_gen{gen}")
                os.makedirs(burst_dir, exist_ok=True)

                print(f"\n[{ts}] New burst! gen={gen} archive={a} frames={count} dist={dist}mm")
                for i in range(count):
                    url = f"{base_url}/burst?a={a}&i={i}"
                    path = os.path.join(burst_dir, f"frame_{i:02d}.jpg")
                    try:
                        with urllib.request.urlopen(url, timeout=5) as r:
                            data = r.read()
                        img = Image.open(io.BytesIO(data))
                        img = img.rotate(90, expand=True)
                        img.save(path, 'JPEG', quality=90)
                        sz = os.path.getsize(path)
                        print(f"  saved {path} ({sz} bytes)")
                    except Exception as e:
                        print(f"  FAILED frame {i}: {e}")

                print(f"  Done: {count} frames -> {burst_dir}/")
                last_gen = gen
            else:
                # Status line
                print(f"\r  dist={dist:>4}mm  archives={archives}  gen={gen}", end="", flush=True)

        except Exception as e:
            print(f"\r  Connection error: {e}", end="", flush=True)

        time.sleep(args.poll)


if __name__ == "__main__":
    main()
