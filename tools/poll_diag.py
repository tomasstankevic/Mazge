"""Poll the ESP32 /diag endpoint every minute and log to JSONL.

Use this to capture the next crash: leave it running, then when the board
reboots the bootCount will increment and the persistedRR will record the
reason. JSONL makes it trivial to grep/awk later.

    uv run python tools/poll_diag.py
    uv run python tools/poll_diag.py --host 192.168.0.41 --interval 60

The output file (`tools/diag_log.jsonl`) is gitignored. One line per poll:
    {"ts": "...", "ok": true, "bootCount": 3, "resetReasonName": "SW", ...}

When `bootCount` increases between samples, the firmware rebooted between
them. The line where it changed is the post-crash snapshot — look at its
`resetReasonName` to see why.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import signal
import socket
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

DEFAULT_HOST = "192.168.0.41"
DEFAULT_INTERVAL = 60
DEFAULT_OUT = Path(__file__).resolve().parent / "diag_log.jsonl"


def poll_once(host: str, timeout: float = 5.0) -> dict:
    url = f"http://{host}/diag"
    ts = dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds")
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            body = r.read().decode("utf-8", errors="replace")
        d = json.loads(body)
        d["ts"] = ts
        d["ok"] = True
        return d
    except (urllib.error.URLError, socket.timeout, ConnectionError, OSError) as e:
        return {"ts": ts, "ok": False, "err": str(e)}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--interval", type=int, default=DEFAULT_INTERVAL,
                    help="seconds between polls")
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    args = ap.parse_args()

    args.out.parent.mkdir(parents=True, exist_ok=True)
    print(f"Polling http://{args.host}/diag every {args.interval}s "
          f"-> {args.out}", flush=True)
    print("Ctrl-C to stop. Lines where bootCount jumps = crash.", flush=True)

    last_boot = None
    stop = False

    def handle_sig(_a, _b):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, handle_sig)
    signal.signal(signal.SIGTERM, handle_sig)

    with args.out.open("a") as f:
        while not stop:
            rec = poll_once(args.host)
            f.write(json.dumps(rec, sort_keys=True) + "\n")
            f.flush()
            if rec.get("ok"):
                bc = rec.get("bootCount")
                rr = rec.get("resetReasonName", "?")
                heap = rec.get("freeHeap")
                minheap = rec.get("minFreeHeap")
                wHW0 = rec.get("workerStackHW0", 0)
                wHW1 = rec.get("workerStackHW1", 0)
                marker = ""
                if last_boot is not None and bc != last_boot:
                    marker = f"  ** REBOOT detected (was {last_boot}, now {bc}, rr={rr}) **"
                last_boot = bc
                print(f"{rec['ts']}  boot#{bc:<3} rr={rr:<8} "
                      f"heap={heap}/min={minheap} wHW={wHW0}/{wHW1}"
                      f"{marker}", flush=True)
            else:
                marker = ""
                if last_boot is not None:
                    marker = "  ** unreachable (possible crash in progress) **"
                print(f"{rec['ts']}  ERR: {rec.get('err')}{marker}", flush=True)
            # Sleep in 1-second slices so Ctrl-C is responsive
            for _ in range(args.interval):
                if stop:
                    break
                time.sleep(1)

    print("\nStopped.", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
