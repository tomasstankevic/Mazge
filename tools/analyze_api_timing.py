"""Analyze API timing across bursts: apiDoneMs - apiCallMs per burst.

Also flag possible crash patterns (very long api times that may have hit WDT).
"""
import json
from pathlib import Path
from datetime import datetime, timezone, timedelta
from collections import defaultdict

REPO = Path(__file__).resolve().parent.parent
SD = REPO / "captures" / "sd"
CET = timezone(timedelta(hours=2))

rows = []
for d in sorted(SD.iterdir()):
    if not d.is_dir() or not d.name.startswith("20"):
        continue
    mp = d / "meta.json"
    if not mp.exists():
        continue
    try:
        m = json.loads(mp.read_text())
    except Exception:
        continue
    epoch = m.get("epoch", 0)
    if not epoch:
        continue
    call = m.get("apiCallMs", 0)
    done = m.get("apiDoneMs", 0)
    api_dur = (done - call) if (call and done and done >= call) else None
    frames = m.get("apiFramesSent", 0)
    res = m.get("apiResult", -1)
    night = (m.get("images", [{}])[0].get("aec", 0) >= 150 and
             m.get("images", [{}])[0].get("gain", 0) >= 8)
    rows.append((epoch, d.name, api_dur, frames, res, night))

rows.sort()

# Per-day stats
by_day = defaultdict(list)
for ep, n, dur, fr, res, night in rows:
    if dur is None or fr == 0:
        continue
    day = datetime.fromtimestamp(ep, CET).strftime("%Y-%m-%d")
    by_day[day].append((dur, fr, res, night, n))

print("API duration per day (apiDoneMs - apiCallMs, only bursts with apiFramesSent>0):")
print(f"  {'day':<11} {'n':>5} {'mean_ms':>9} {'p50':>7} {'p90':>7} {'max':>7} "
      f"{'ms/frame':>10}")
for day in sorted(by_day):
    durs = [d for d, *_ in by_day[day]]
    fr_total = sum(f for _, f, *_ in by_day[day])
    ms_per_frame = sum(durs) / fr_total if fr_total else 0
    durs.sort()
    mean = sum(durs) / len(durs)
    p50 = durs[len(durs)//2]
    p90 = durs[int(len(durs)*0.9)]
    print(f"  {day:<11} {len(durs):>5} {int(mean):>9} {int(p50):>7} {int(p90):>7} "
          f"{int(durs[-1]):>7} {int(ms_per_frame):>10}")

# Same split by night vs day
print("\nApi duration day vs night since 2026-05-18:")
print(f"  {'day/night':<11} {'n':>5} {'mean_ms':>9} {'p50':>7} {'p90':>7} {'ms/frame':>10}")
for label, predicate in [("day", lambda r: not r[3]), ("night", lambda r: r[3])]:
    samp = [(d, f) for ep, _n, d, f, _r, ni in rows
            if d and f and ep > 1779000000 and ((not ni) if label == "day" else ni)]
    if not samp:
        continue
    durs = sorted(d for d, _ in samp)
    fr_total = sum(f for _, f in samp)
    ms_per_frame = sum(durs) / fr_total
    print(f"  {label:<11} {len(durs):>5} {int(sum(durs)/len(durs)):>9} "
          f"{int(durs[len(durs)//2]):>7} {int(durs[int(len(durs)*0.9)]):>7} "
          f"{int(ms_per_frame):>10}")

# Trend: median ms/frame per day, last 14 days
print("\nMedian API ms per FRAME per day (last 14 days):")
recent = sorted({datetime.fromtimestamp(ep, CET).strftime("%Y-%m-%d")
                 for ep, *_ in rows if ep > 1778500000})[-14:]
for day in recent:
    samp = [(d, f) for dy in [day] for d, f, *_ in by_day.get(dy, [])]
    if not samp:
        continue
    per_frame = sorted(d / f for d, f in samp if f)
    if not per_frame:
        continue
    median = per_frame[len(per_frame)//2]
    p90 = per_frame[int(len(per_frame)*0.9)]
    print(f"  {day}: n={len(per_frame):>3}  median={int(median):>5}ms/frame  "
          f"p90={int(p90):>5}ms/frame")

# Suspicious outliers (>25s)
print("\nVery slow API calls (>20s):")
for ep, n, dur, fr, res, night in rows:
    if dur and dur > 20000:
        t = datetime.fromtimestamp(ep, CET).strftime("%Y-%m-%d %H:%M:%S")
        print(f"  {t}  {n:<28} dur={dur/1000:.1f}s frames={fr} res={res} night={night}")
