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
from PIL import Image, ImageOps


def main():
    parser = argparse.ArgumentParser(description="Auto-save ESP32-CAM burst captures")
    parser.add_argument("--ip", default="192.168.0.42", help="ESP32 IP address")
    parser.add_argument("--dir", default="captures", help="Output directory")
    parser.add_argument("--poll", type=float, default=1.0, help="Poll interval (seconds)")
    args = parser.parse_args()

    base_url = f"http://{args.ip}"
    os.makedirs(args.dir, exist_ok=True)

    last_gen = 0
    last_archive_count = 0
    print(f"Watching {base_url} for new bursts... (saving to {args.dir}/)")

    while True:
        try:
            with urllib.request.urlopen(f"{base_url}/stats", timeout=3) as r:
                stats = json.loads(r.read())

            gen = stats.get("burstGen", 0)
            archives = stats.get("burstArchives", 0)
            counts = stats.get("burstCounts", [])
            dist = stats.get("distance", -1)

            # Detect new bursts: gen changed, or archive count grew
            new_bursts = 0
            if gen != last_gen and gen > 0 and archives > 0:
                if gen < last_gen:
                    # Board rebooted (gen reset), download all archives
                    new_bursts = archives
                else:
                    new_bursts = gen - last_gen
                    new_bursts = min(new_bursts, archives)  # can't exceed what's stored

            if new_bursts > 0:
                # Download the newest `new_bursts` archives (oldest first)
                start_a = archives - new_bursts
                for a in range(start_a, archives):
                    count = counts[a] if a < len(counts) else 0
                    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                    burst_gen = gen - (archives - 1 - a)
                    burst_dir = os.path.join(args.dir, f"burst_{ts}_gen{burst_gen}")
                    os.makedirs(burst_dir, exist_ok=True)

                    print(f"\n[{ts}] New burst! gen={burst_gen} archive={a} frames={count} dist={dist}mm")
                    for i in range(count):
                        url = f"{base_url}/burst?a={a}&i={i}"
                        path = os.path.join(burst_dir, f"frame_{i:02d}.jpg")
                        saved = False
                        for attempt in range(3):
                            try:
                                with urllib.request.urlopen(url, timeout=10) as r:
                                    data = r.read()
                                img = Image.open(io.BytesIO(data))
                                img = img.rotate(90, expand=True)
                                # Adaptive contrast stretch: lifts dark foreground
                                # without needing ROI, handles bright sky + dark cat
                                img = ImageOps.autocontrast(img, cutoff=1)
                                img.save(path, 'JPEG', quality=92)
                                sz = os.path.getsize(path)
                                print(f"  saved {path} ({sz} bytes)")
                                saved = True
                                break
                            except Exception as e:
                                if attempt < 2:
                                    time.sleep(0.5)
                                else:
                                    print(f"  FAILED frame {i}: {e}")

                    print(f"  Done: {count} frames -> {burst_dir}/")

                last_gen = gen
                last_archive_count = archives
            else:
                # Status line
                print(f"\r  dist={dist:>4}mm  archives={archives}  gen={gen}", end="", flush=True)

        except Exception as e:
            print(f"\r  Connection error: {e}", end="", flush=True)

        time.sleep(args.poll)


if __name__ == "__main__":
    main()
