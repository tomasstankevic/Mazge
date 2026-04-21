#!/usr/bin/env python3
"""Auto-download burst captures from ESP32-CAM.

Uses long-polling via /burst_wait?gen=N endpoint: the ESP32 blocks until
a new burst is available, eliminating polling latency. Falls back to
/stats polling if /burst_wait is unavailable (old firmware).

Usage: python3 tools/burst_saver.py [--ip 192.168.0.41] [--dir captures]
"""
import argparse
import io
import json
import os
import time
import urllib.request
from datetime import datetime
from pathlib import Path
from PIL import Image, ImageOps

MJPEG_BOUNDARY = b"--123456789000000000000987654321"


def download_burst_mjpeg(base_url: str, archive_idx: int, timeout: float = 15.0) -> list[bytes]:
    """Download all burst frames as a single MJPEG stream, return list of JPEG bytes."""
    url = f"{base_url}/burststream?a={archive_idx}"
    with urllib.request.urlopen(url, timeout=timeout) as r:
        data = r.read()
    # Split on boundary, extract JPEG payloads
    parts = data.split(MJPEG_BOUNDARY)
    frames = []
    for part in parts:
        # Each part: \r\nContent-Type: image/jpeg\r\nContent-Length: NNN\r\n\r\n<jpeg data>
        idx = part.find(b"\r\n\r\n")
        if idx < 0:
            continue
        jpeg = part[idx + 4:]
        # Strip trailing whitespace/boundary remnants
        if jpeg and jpeg[:2] == b"\xff\xd8":
            frames.append(jpeg)
    return frames


def fetch_json(url: str, timeout: float = 3.0):
    """Fetch JSON and capture local timing around the request."""
    started_ms = time.time() * 1000
    with urllib.request.urlopen(url, timeout=timeout) as r:
        payload = json.loads(r.read())
    finished_ms = time.time() * 1000
    return payload, {
        "started_ms": round(started_ms, 1),
        "finished_ms": round(finished_ms, 1),
        "midpoint_ms": round((started_ms + finished_ms) / 2, 1),
        "rtt_ms": round(finished_ms - started_ms, 1),
    }


def build_timing_manifest(base_url: str, stats: dict, stats_req: dict, meta: dict | None, meta_req: dict | None, download: dict) -> dict:
    """Combine ESP32 and laptop timing into a single manifest."""
    meta = meta or {}
    meta_req = meta_req or {}
    uptime_ms = meta.get("uptimeMs", stats.get("uptimeMs"))
    device_to_local_offset_ms = None
    if uptime_ms is not None and meta_req.get("midpoint_ms") is not None:
        device_to_local_offset_ms = round(meta_req["midpoint_ms"] - uptime_ms, 1)

    def approx_local(remote_ms):
        if remote_ms is None or device_to_local_offset_ms is None:
            return None
        return round(device_to_local_offset_ms + remote_ms, 1)

    trigger_ms = meta.get("triggerMs")
    archive_ms = meta.get("archiveMs")
    first_frame_ms = meta.get("firstFrameMs")
    last_frame_ms = meta.get("lastFrameMs")

    approx_trigger_ms = approx_local(trigger_ms)
    approx_archive_ms = approx_local(archive_ms)
    approx_first_frame_ms = approx_local(first_frame_ms)
    approx_last_frame_ms = approx_local(last_frame_ms)

    latency = {
        "trigger_to_archive_ms": round(archive_ms - trigger_ms, 1) if trigger_ms and archive_ms else None,
        "capture_span_ms": round(last_frame_ms - first_frame_ms, 1) if first_frame_ms and last_frame_ms else None,
        "archive_to_download_start_ms": round(download["started_ms"] - approx_archive_ms, 1) if approx_archive_ms else None,
        "trigger_to_download_end_ms": round(download["finished_ms"] - approx_trigger_ms, 1) if approx_trigger_ms else None,
    }

    return {
        "base_url": base_url,
        "generation": meta.get("generation"),
        "archive_index": meta.get("archive"),
        "frame_count": meta.get("count"),
        "distance_mm": stats.get("distance"),
        "frame_distance_mm": meta.get("distanceMm", []),
        "stats_request": stats_req,
        "burstmeta_request": meta_req,
        "esp32_clock": {
            "uptime_ms": uptime_ms,
            "device_to_local_offset_ms": device_to_local_offset_ms,
        },
        "esp32_capture": {
            "trigger_ms": trigger_ms,
            "archive_ms": archive_ms,
            "first_frame_ms": first_frame_ms,
            "last_frame_ms": last_frame_ms,
            "frame_capture_ms": meta.get("frameCaptureMs", []),
            "approx_local_trigger_ms": approx_trigger_ms,
            "approx_local_archive_ms": approx_archive_ms,
            "approx_local_first_frame_ms": approx_first_frame_ms,
            "approx_local_last_frame_ms": approx_last_frame_ms,
        },
        "download": download,
        "latency_ms": latency,
    }


def main():
    parser = argparse.ArgumentParser(description="Auto-save ESP32-CAM burst captures")
    parser.add_argument("--ip", default="192.168.0.41", help="ESP32 IP address")
    parser.add_argument("--dir", default="captures", help="Output directory")
    parser.add_argument("--poll", type=float, default=1.0, help="Fallback poll interval (seconds)")
    args = parser.parse_args()

    base_url = f"http://{args.ip}"
    stream_url = f"http://{args.ip}:81"  # /burst_wait is on stream server
    output_dir = Path(args.dir)
    output_dir.mkdir(exist_ok=True)

    last_gen = 0
    last_archive_count = 0
    estimated_boot_epoch = None  # estimated ESP32 boot time (wall-clock seconds)
    use_long_poll = True  # try /burst_wait first, fall back to /stats
    print(f"Watching {base_url} for new bursts... (saving to {args.dir}/)")

    # Don't skip existing archives — leave last_gen=0 so first poll downloads them

    while True:
        try:
            if use_long_poll:
                try:
                    stats, stats_req = fetch_json(
                        f"{stream_url}/burst_wait?gen={last_gen}", timeout=35)
                except urllib.error.HTTPError as e:
                    if e.code == 404:
                        print("  /burst_wait not available, falling back to polling")
                        use_long_poll = False
                        continue
                    raise
                except urllib.error.URLError:
                    # Connection refused / timeout — board may have rebooted
                    time.sleep(args.poll)
                    continue
            else:
                stats, stats_req = fetch_json(f"{base_url}/stats", timeout=3)

            gen = stats.get("burstGen", 0)
            archives = stats.get("burstArchives", 0)
            counts = stats.get("burstCounts", [])
            dist = stats.get("distance", -1)
            uptime = stats.get("uptimeMs", 0)

            # Detect reboot by checking if estimated boot time shifted.
            # Boot epoch = wall_clock - uptime. Stays ~constant during normal
            # operation, but jumps when the board reboots.
            rebooted = False
            if uptime > 0:
                new_boot_epoch = time.time() - uptime / 1000
                if estimated_boot_epoch is not None and abs(new_boot_epoch - estimated_boot_epoch) > 30:
                    rebooted = True
                    print(f"\n  Reboot detected (boot time shifted by {abs(new_boot_epoch - estimated_boot_epoch):.0f}s)")
                estimated_boot_epoch = new_boot_epoch

            # Detect new bursts: gen changed, or archive count grew
            new_bursts = 0
            if gen != last_gen and gen > 0 and archives > 0:
                if rebooted or gen < last_gen:
                    # Board rebooted — download all archives
                    print(f"  Downloading all {archives} archives")
                    new_bursts = archives
                else:
                    new_bursts = gen - last_gen
                    new_bursts = min(new_bursts, archives)  # can't exceed what's stored
            elif rebooted and archives > 0:
                # gen == last_gen but board rebooted — these are new archives
                print(f"  Downloading all {archives} archives (gen unchanged but rebooted)")
                new_bursts = archives

            if new_bursts > 0:
                # Download the newest `new_bursts` archives (oldest first)
                start_a = archives - new_bursts
                for a in range(start_a, archives):
                    count = counts[a] if a < len(counts) else 0
                    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                    burst_gen = gen - (archives - 1 - a)
                    burst_dir = output_dir / f"burst_{ts}_gen{burst_gen}"
                    burst_dir.mkdir(exist_ok=True)
                    try:
                        burst_meta, burst_meta_req = fetch_json(f"{base_url}/burstmeta?a={a}", timeout=3)
                    except Exception as exc:
                        burst_meta, burst_meta_req = None, None
                        print(f"  note: burstmeta unavailable ({exc})")
                    download_started_ms = round(time.time() * 1000, 1)
                    frame_timings = []

                    print(f"\n[{ts}] New burst! gen={burst_gen} archive={a} frames={count} dist={dist}mm")
                    try:
                        mjpeg_started_ms = time.time() * 1000
                        jpeg_frames = download_burst_mjpeg(base_url, a)
                        mjpeg_finished_ms = time.time() * 1000
                        print(f"  MJPEG stream: {len(jpeg_frames)} frames in {mjpeg_finished_ms - mjpeg_started_ms:.0f}ms")
                    except Exception as e:
                        print(f"  MJPEG download failed: {e}, falling back to per-frame")
                        jpeg_frames = None

                    for i in range(count):
                        path = burst_dir / f"frame_{i:02d}.jpg"
                        frame_started_ms = time.time() * 1000

                        if jpeg_frames and i < len(jpeg_frames):
                            data = jpeg_frames[i]
                        else:
                            # Fallback: individual frame download
                            url = f"{base_url}/burst?a={a}&i={i}"
                            with urllib.request.urlopen(url, timeout=10) as r:
                                data = r.read()

                        frame_finished_ms = time.time() * 1000
                        img = Image.open(io.BytesIO(data))
                        img = img.transpose(Image.Transpose.ROTATE_90)
                        # Crop 20% from top (after rotation) — occluded by cat flap frame
                        w_img, h_img = img.size
                        crop_top = int(h_img * 0.20)
                        img = img.crop((0, crop_top, w_img, h_img))
                        img.save(path, 'JPEG', quality=97)
                        sz = path.stat().st_size
                        frame_timings.append({
                            "frame": i,
                            "started_ms": round(frame_started_ms, 1),
                            "finished_ms": round(frame_finished_ms, 1),
                            "download_ms": round(frame_finished_ms - frame_started_ms, 1),
                            "bytes": len(data),
                            "saved_bytes": sz,
                        })
                        print(f"  saved {path} ({sz} bytes)")

                    download_finished_ms = round(time.time() * 1000, 1)
                    download_info = {
                        "started_ms": download_started_ms,
                        "finished_ms": download_finished_ms,
                        "duration_ms": round(download_finished_ms - download_started_ms, 1),
                        "frame_timings": frame_timings,
                    }
                    manifest = build_timing_manifest(base_url, stats, stats_req, burst_meta, burst_meta_req, download_info)
                    (burst_dir / "burst_meta.json").write_text(json.dumps(manifest, indent=2))

                    print(
                        f"  Done: {count} frames -> {burst_dir}/ "
                        f"download={download_info['duration_ms']:.0f}ms "
                        f"trigger->download_end={manifest['latency_ms'].get('trigger_to_download_end_ms')}ms"
                    )

                last_gen = gen
                last_archive_count = archives
            else:
                # Status line
                print(f"\r  dist={dist:>4}mm  archives={archives}  gen={gen}", end="", flush=True)

        except Exception as e:
            print(f"\r  Connection error: {e}", end="", flush=True)

        if not use_long_poll:
            time.sleep(args.poll)


if __name__ == "__main__":
    main()
