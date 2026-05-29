#!/usr/bin/env python3
"""Gently pull missing JPGs for recent non-prey bursts.

Scans captures/sd/*/meta.json, filters to recent folders and apiResult==0,
and downloads only missing JPGs listed in meta.json.
"""

from __future__ import annotations

import argparse
import json
import time
import urllib.request
from datetime import datetime, timedelta
from pathlib import Path


def is_timestamp_folder(name: str) -> bool:
    return len(name) >= 15 and name[8] == "_" and name[:8].isdigit() and name[9:15].isdigit()


def fetch(url: str, timeout: float = 20.0, retries: int = 4) -> bytes:
    last_exc: Exception | None = None
    for attempt in range(1, retries + 1):
        try:
            with urllib.request.urlopen(url, timeout=timeout) as response:
                return response.read()
        except Exception as exc:  # noqa: BLE001
            last_exc = exc
            if attempt < retries:
                time.sleep(1.2 * attempt)
    raise RuntimeError(str(last_exc))


def main() -> None:
    parser = argparse.ArgumentParser(description="Pull missing JPGs for recent non-prey bursts")
    parser.add_argument("--host", default="192.168.0.41")
    parser.add_argument("--hours", type=int, default=48)
    parser.add_argument("--sleep", type=float, default=0.35, help="Delay between JPG fetches")
    args = parser.parse_args()

    sd = Path("captures/sd")
    cutoff = (datetime.now() - timedelta(hours=args.hours)).strftime("%Y%m%d_%H%M%S")

    recent = []
    for d in sorted(sd.iterdir()):
        if not d.is_dir() or not is_timestamp_folder(d.name) or d.name[:15] < cutoff:
            continue
        meta_path = d / "meta.json"
        if not meta_path.exists():
            continue
        try:
            meta = json.loads(meta_path.read_text())
        except Exception:
            continue
        if meta.get("apiResult") == 0:
            recent.append((d, meta))

    burst_count = 0
    file_count = 0
    byte_count = 0
    error_count = 0

    for d, meta in recent:
        image_names = [x.get("f") for x in meta.get("images", []) if x.get("f")]
        existing = {p.name for p in d.glob("*.jpg")}
        missing = [name for name in image_names if name not in existing]
        if not missing:
            continue
        burst_count += 1
        print(f"{d.name}: pulling {len(missing)} jpgs")

        for name in missing:
            url = f"http://{args.host}/sdget?f={d.name}/{name}"
            try:
                data = fetch(url)
                if len(data) < 1000:
                    raise RuntimeError(f"short_response:{len(data)}")
                (d / name).write_bytes(data)
                file_count += 1
                byte_count += len(data)
            except Exception as exc:  # noqa: BLE001
                error_count += 1
                print(f"  ERR {name}: {exc}")
            time.sleep(args.sleep)

    print(
        f"recent_non_prey_bursts={len(recent)} bursts_with_missing={burst_count} "
        f"files_downloaded={file_count} bytes={byte_count} errors={error_count}"
    )


if __name__ == "__main__":
    main()
