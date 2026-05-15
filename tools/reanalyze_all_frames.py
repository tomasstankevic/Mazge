"""Re-analyze ALL frames (not just the 5 the firmware sent) of recent bursts.

For each burst folder in captures/sd/ (filtered by date), this script:
  1. Loads all 10 frame JPEGs.
  2. Applies the same crop+rotate+grayscale the firmware does on-device:
     - Crop 384x384 region from (64,48) of the 640x480 frame.
     - Rotate 90° CCW.
     - Convert to grayscale.
  3. Sends each frame to the prey API in parallel.
  4. Writes results to <burst>/full_analysis.json

Then prints a summary: how many "extra" prey hits we'd have caught by
sending all 10 frames instead of 5.

Usage:
  uv run python tools/reanalyze_all_frames.py             # last 7 days
  uv run python tools/reanalyze_all_frames.py --days 14
  uv run python tools/reanalyze_all_frames.py --since 20260510
  uv run python tools/reanalyze_all_frames.py --only-prey   # only prey-positive bursts
"""
from __future__ import annotations

import argparse
import asyncio
import base64
import datetime as dt
import io
import json
import os
import ssl
import sys
import time
from pathlib import Path

import aiohttp
import certifi
from PIL import Image

ROOT = Path(__file__).resolve().parent.parent
SD = ROOT / "captures" / "sd"
PREY_API_URL = os.environ.get(
    "PREY_API_URL",
    "https://prey-detection.florian-mutel.workers.dev",
)
PREY_API_KEY = os.environ.get("PREY_DETECTOR_API_KEY", "")
if not PREY_API_KEY:
    p = ROOT / "API_key_prey_detector.txt"
    if p.exists():
        PREY_API_KEY = p.read_text().strip()

CROP_X, CROP_Y, CROP_SZ = 64, 48, 384
CONCURRENCY = 3


def preprocess_frame(jpg_bytes: bytes) -> bytes:
    """Match firmware: crop 384x384 from (64,48), rotate 90° CCW, grayscale, JPEG."""
    img = Image.open(io.BytesIO(jpg_bytes))
    img = img.crop((CROP_X, CROP_Y, CROP_X + CROP_SZ, CROP_Y + CROP_SZ))
    img = img.rotate(90, expand=True)  # CCW
    img = img.convert("L")
    out = io.BytesIO()
    img.save(out, format="JPEG", quality=85)
    return out.getvalue()


async def call_api(sess: aiohttp.ClientSession, ssl_ctx, jpg_bytes: bytes,
                   retries: int = 5) -> dict:
    b64 = base64.b64encode(jpg_bytes).decode()
    last = None
    for attempt in range(retries):
        try:
            t0 = time.perf_counter()
            async with sess.post(
                PREY_API_URL,
                headers={
                    "Content-Type": "application/json",
                    "Authorization": f"Bearer {PREY_API_KEY}",
                },
                json={"image_base64": b64},
                timeout=aiohttp.ClientTimeout(total=20),
                ssl=ssl_ctx,
            ) as resp:
                if resp.status == 429:
                    # Rate limit: long exponential backoff
                    wait = 5.0 * (2 ** attempt)
                    last = f"429 (waiting {wait:.0f}s)"
                    await asyncio.sleep(wait)
                    continue
                resp.raise_for_status()
                data = await resp.json()
            return {
                "detected": bool(data.get("detected", False)),
                "latency_ms": (time.perf_counter() - t0) * 1000,
            }
        except Exception as exc:
            last = exc
            if attempt < retries - 1:
                await asyncio.sleep(1.0 * (attempt + 1))
    return {"detected": None, "error": str(last)}


def burst_date(name: str) -> dt.date | None:
    try:
        return dt.datetime.strptime(name[:8], "%Y%m%d").date()
    except ValueError:
        return None


def load_meta(folder: Path) -> dict:
    p = folder / "meta.json"
    if not p.exists():
        return {}
    try:
        return json.loads(p.read_text())
    except Exception:
        return {}


def select_bursts(since: dt.date, only_prey: bool) -> list[Path]:
    out = []
    for d in sorted(SD.iterdir()):
        if not d.is_dir():
            continue
        bd = burst_date(d.name)
        if bd is None or bd < since:
            continue
        meta = load_meta(d)
        if only_prey and meta.get("apiResult") != 1:
            continue
        # need actual jpgs
        jpgs = sorted(d.glob("f*.jpg"))
        if len(jpgs) < 5:
            continue
        out.append(d)
    return out


async def analyze_burst(sem: asyncio.Semaphore, sess, ssl_ctx, folder: Path,
                        force: bool) -> dict:
    out_path = folder / "full_analysis.json"
    if out_path.exists() and not force:
        try:
            cached = json.loads(out_path.read_text())
            # Don't trust cached results where every frame errored out
            if any(r is not None for r in cached.get("full_results", [])):
                return cached
        except Exception:
            pass

    jpgs = sorted(folder.glob("f*.jpg"))
    meta = load_meta(folder)
    fw_results = meta.get("apiResults", [-1] * len(jpgs))

    async def one(idx: int, jp: Path):
        async with sem:
            try:
                cropped = preprocess_frame(jp.read_bytes())
            except Exception as exc:
                return idx, {"detected": None, "error": f"preprocess: {exc}"}
            r = await call_api(sess, ssl_ctx, cropped)
            return idx, r

    tasks = [one(i, jp) for i, jp in enumerate(jpgs)]
    results: list[dict] = [{} for _ in jpgs]
    for coro in asyncio.as_completed(tasks):
        i, r = await coro
        results[i] = r

    summary = {
        "burst": folder.name,
        "frames_total": len(jpgs),
        "fw_apiResults": fw_results,
        "fw_prey_count": sum(1 for x in fw_results if x == 1),
        "fw_sent_count": sum(1 for x in fw_results if x != -1),
        "full_results": [r.get("detected") for r in results],
        "full_prey_count": sum(1 for r in results if r.get("detected") is True),
        "errors": [r for r in results if "error" in r],
        "frame_files": [jp.name for jp in jpgs],
    }
    # Only cache if we got at least one real response
    if any(r is not None for r in summary["full_results"]):
        out_path.write_text(json.dumps(summary, indent=2))
    return summary


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--days", type=int, default=7)
    ap.add_argument("--since", help="YYYYMMDD")
    ap.add_argument("--only-prey", action="store_true",
                    help="Only re-analyze bursts already flagged as prey")
    ap.add_argument("--force", action="store_true",
                    help="Re-run even if full_analysis.json exists")
    ap.add_argument("--concurrency", type=int, default=CONCURRENCY)
    args = ap.parse_args()

    if not PREY_API_KEY:
        sys.exit("ERROR: PREY_DETECTOR_API_KEY not set and API_key_prey_detector.txt missing")

    if args.since:
        since = dt.datetime.strptime(args.since, "%Y%m%d").date()
    else:
        since = dt.date.today() - dt.timedelta(days=args.days)

    bursts = select_bursts(since, args.only_prey)
    print(f"Re-analyzing {len(bursts)} bursts since {since.isoformat()}"
          f"{' (only-prey)' if args.only_prey else ''}", flush=True)

    sem = asyncio.Semaphore(args.concurrency)
    ssl_ctx = ssl.create_default_context(cafile=certifi.where())
    timeout = aiohttp.ClientTimeout(total=30)
    summaries: list[dict] = []
    async with aiohttp.ClientSession(timeout=timeout) as sess:
        for i, folder in enumerate(bursts, 1):
            t0 = time.time()
            s = await analyze_burst(sem, sess, ssl_ctx, folder, args.force)
            dur = time.time() - t0
            extra = s["full_prey_count"] - s["fw_prey_count"]
            mark = ""
            if s["full_prey_count"] > 0 and s["fw_prey_count"] == 0:
                mark = "  *** NEW PREY ***"
            elif extra > 0:
                mark = f"  (+{extra} extra hits)"
            print(f"  [{i}/{len(bursts)}] {folder.name}  "
                  f"fw {s['fw_prey_count']}/{s['fw_sent_count']} "
                  f"-> full {s['full_prey_count']}/{s['frames_total']}  "
                  f"({dur:.1f}s){mark}", flush=True)
            summaries.append(s)

    # ── aggregate report ─────────────────────────────────────────────
    print("\n" + "=" * 72)
    print("SUMMARY")
    print("=" * 72)
    total_bursts = len(summaries)
    fw_prey_bursts = sum(1 for s in summaries if s["fw_prey_count"] >= 2)
    full_prey_bursts = sum(1 for s in summaries if s["full_prey_count"] >= 2)
    new_prey_bursts = [s for s in summaries
                       if s["full_prey_count"] >= 2 and s["fw_prey_count"] < 2]
    extra_hits_total = sum(
        max(0, s["full_prey_count"] - s["fw_prey_count"]) for s in summaries
    )
    total_frames_sent_fw = sum(s["fw_sent_count"] for s in summaries)
    total_frames_full = sum(s["frames_total"] for s in summaries)
    fw_total_hits = sum(s["fw_prey_count"] for s in summaries)
    full_total_hits = sum(s["full_prey_count"] for s in summaries)

    print(f"Bursts analyzed:           {total_bursts}")
    print(f"Frames sent (firmware):    {total_frames_sent_fw}")
    print(f"Frames sent (full):        {total_frames_full}")
    print(f"Prey-flagged frames fw:    {fw_total_hits}")
    print(f"Prey-flagged frames full:  {full_total_hits}  (+{full_total_hits - fw_total_hits})")
    print(f"Bursts >=2 prey (fw):      {fw_prey_bursts}")
    print(f"Bursts >=2 prey (full):    {full_prey_bursts}  (+{full_prey_bursts - fw_prey_bursts})")
    print(f"Extra hits across bursts:  {extra_hits_total}")
    if new_prey_bursts:
        print("\nBursts that WOULD have been flagged as prey if all frames analyzed:")
        for s in new_prey_bursts:
            print(f"  {s['burst']}: full {s['full_prey_count']}/{s['frames_total']} "
                  f"(fw {s['fw_prey_count']}/{s['fw_sent_count']})")

    # write aggregate JSON
    out = ROOT / "captures" / "full_reanalysis_summary.json"
    out.write_text(json.dumps({
        "since": since.isoformat(),
        "generated": dt.datetime.now().isoformat(timespec="seconds"),
        "totals": {
            "bursts": total_bursts,
            "frames_fw": total_frames_sent_fw,
            "frames_full": total_frames_full,
            "hits_fw": fw_total_hits,
            "hits_full": full_total_hits,
            "prey_bursts_fw": fw_prey_bursts,
            "prey_bursts_full": full_prey_bursts,
        },
        "new_prey_bursts": [s["burst"] for s in new_prey_bursts],
        "bursts": summaries,
    }, indent=2))
    print(f"\nAggregate written to {out.relative_to(ROOT)}")


if __name__ == "__main__":
    asyncio.run(main())
