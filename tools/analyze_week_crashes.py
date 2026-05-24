"""Analyze meta.json files across new bursts to detect device reboots/crashes.

A reboot is detected when uptimeMs in a later-epoch burst is LESS than uptimeMs
in an earlier-epoch burst. The gen counter also resets on reboot, but only
after the first trigger after boot.
"""
import json
from pathlib import Path
from datetime import datetime, timezone, timedelta

REPO = Path(__file__).resolve().parent.parent
SD = REPO / "captures" / "sd"

# Load all bursts since 2026-05-21
all_meta = []
for d in sorted(SD.iterdir()):
    if not d.is_dir():
        continue
    n = d.name
    if not n.startswith("20"):
        continue
    if n < "20260521":
        continue
    mp = d / "meta.json"
    if not mp.exists():
        continue
    try:
        m = json.loads(mp.read_text())
    except Exception:
        continue
    epoch = m.get("epoch", 0)
    uptime = m.get("uptimeMs", 0)
    gen = m.get("gen", 0)
    api = m.get("apiResult", -1)
    direction = m.get("direction", -1)
    all_meta.append((n, epoch, uptime, gen, api, direction))

all_meta.sort(key=lambda x: x[1] or 0)

CET = timezone(timedelta(hours=2))
def fmt(epoch):
    return datetime.fromtimestamp(epoch, CET).strftime("%Y-%m-%d %H:%M:%S")

print(f"Analyzing {len(all_meta)} bursts since 2026-05-21\n")

# Detect reboots: when uptimeMs drops, OR when gen resets to 1 between consecutive bursts
reboots = []
prev = None
for entry in all_meta:
    name, epoch, uptime, gen, api, direction = entry
    if prev is None:
        prev = entry
        continue
    pname, pepoch, puptime, pgen, _, _ = prev
    # uptime decreased = reboot
    if uptime < puptime - 2000:  # 2s tolerance
        gap_s = (epoch - pepoch) if epoch and pepoch else 0
        new_uptime_s = uptime / 1000
        reboots.append({
            "after": pname,
            "after_t": fmt(pepoch),
            "after_uptime_s": puptime / 1000,
            "before": name,
            "before_t": fmt(epoch),
            "first_burst_uptime_s": new_uptime_s,
            "gap_between_bursts_s": gap_s,
            "downtime_estimate_s": max(0, gap_s - new_uptime_s),
        })
    prev = entry

print(f"Detected reboots (between bursts): {len(reboots)}\n")
print(f"{'last burst before':<22} {'time (CET)':<20} {'next burst':<22} "
      f"{'time':<20} {'gap':>8} {'fresh_up':>10} {'down_est':>10}")
for r in reboots:
    print(f"  {r['after']:<22} {r['after_t']:<20} {r['before']:<22} {r['before_t']:<20} "
          f"{int(r['gap_between_bursts_s']):>7}s {int(r['first_burst_uptime_s']):>9}s "
          f"{int(r['downtime_estimate_s']):>9}s")

# Burst stats per day
from collections import Counter, defaultdict
per_day = defaultdict(lambda: {"total": 0, "prey": 0, "enter": 0, "exit": 0, "unknown": 0})
for name, epoch, uptime, gen, api, direction in all_meta:
    day = name.split("_")[0]
    per_day[day]["total"] += 1
    if api == 1:
        per_day[day]["prey"] += 1
    if direction == 1:
        per_day[day]["enter"] += 1
    elif direction == 2:
        per_day[day]["exit"] += 1
    else:
        per_day[day]["unknown"] += 1

print("\nPer-day burst counts:")
print(f"  {'day':<10} {'total':>6} {'prey':>5} {'enter':>6} {'exit':>5} {'?':>5}")
for day in sorted(per_day):
    s = per_day[day]
    print(f"  {day:<10} {s['total']:>6} {s['prey']:>5} {s['enter']:>6} {s['exit']:>5} {s['unknown']:>5}")

# Long gaps between bursts (>4h overnight excluded)
print("\nLong gaps between bursts (>2h, daytime):")
prev = None
for name, epoch, uptime, gen, api, direction in all_meta:
    if prev and epoch and prev[1]:
        gap = epoch - prev[1]
        hour = datetime.fromtimestamp(epoch, CET).hour
        # Skip overnight gaps (likely cat is asleep)
        if gap > 7200 and not (1 <= hour <= 5):
            print(f"  {fmt(prev[1])} -> {fmt(epoch)}  gap={gap/3600:.1f}h  "
                  f"(prev={prev[0]}, next={name})")
    prev = (name, epoch, uptime, gen, api, direction)
