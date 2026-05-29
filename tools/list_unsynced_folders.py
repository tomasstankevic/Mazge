#!/usr/bin/env python3
"""List SD folders present on device but missing locally."""

from __future__ import annotations

import argparse
import json
import time
import urllib.request
from datetime import datetime, timedelta
from pathlib import Path

HOST = "192.168.0.41"


def fetch_remote_folders(host: str, retries: int, timeout: float) -> list[str]:
    url = f"http://{host}/sdfolders"
    last_exc: Exception | None = None
    for attempt in range(1, retries + 1):
        try:
            with urllib.request.urlopen(url, timeout=timeout) as response:
                payload = response.read()
            data = json.loads(payload)
            folders = data.get("folders", [])
            if not isinstance(folders, list):
                raise ValueError("Invalid /sdfolders payload")
            return [str(x) for x in folders]
        except Exception as exc:  # noqa: BLE001
            last_exc = exc
            print(f"attempt {attempt}/{retries} failed: {exc}")
            time.sleep(2)
    raise RuntimeError(f"Failed to fetch /sdfolders from {host}: {last_exc}")


def is_timestamp_folder(name: str) -> bool:
    return len(name) >= 15 and name[8] == "_" and name[:8].isdigit() and name[9:15].isdigit()


def main() -> None:
    parser = argparse.ArgumentParser(description="List unsynced SD folders")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--hours", type=int, default=48, help="Filter window for recent missing folders")
    parser.add_argument("--retries", type=int, default=8)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--all", action="store_true", help="Print all missing folders, not just summary")
    args = parser.parse_args()

    remote = fetch_remote_folders(args.host, args.retries, args.timeout)
    local = {d.name for d in Path("captures/sd").iterdir() if d.is_dir()}
    missing = sorted(x for x in remote if x not in local)

    cutoff = (datetime.now() - timedelta(hours=args.hours)).strftime("%Y%m%d_%H%M%S")
    missing_recent = [x for x in missing if is_timestamp_folder(x) and x[:15] >= cutoff]

    print(f"remote_total={len(remote)}")
    print(f"local_total={len(local)}")
    print(f"missing_total={len(missing)}")
    print(f"cutoff_{args.hours}h={cutoff}")
    print(f"missing_{args.hours}h={len(missing_recent)}")

    print(f"MISSING_{args.hours}H_START")
    for folder in missing_recent:
        print(folder)
    print(f"MISSING_{args.hours}H_END")

    if args.all:
        print("MISSING_ALL_START")
        for folder in missing:
            print(folder)
        print("MISSING_ALL_END")


if __name__ == "__main__":
    main()
