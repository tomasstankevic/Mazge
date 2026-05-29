"""Comprehensive cat activity analysis.

Combines bursts.csv (human labels: prey, direction, subject, cat_id) with
meta.json (firmware ToF direction + prey API verdict). Falls back to
firmware verdict when no human label exists (mostly newer bursts).
"""
import csv
import json
import math
from collections import Counter, defaultdict
from datetime import datetime, timezone, timedelta
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SD = REPO / "captures" / "sd"
BURSTS = REPO / "dataset" / "bursts.csv"
CET = timezone(timedelta(hours=2))


# -------- Load bursts.csv (one row per burst) --------
rows = []
with BURSTS.open() as f:
    for r in csv.DictReader(f):
        burst_id = r["burst_id"]
        epoch = int(r["epoch"]) if r["epoch"] else 0
        # Direction: prefer human, fall back to firmware meta
        human_dir = r.get("human_direction", "")
        # Subject: prefer human, fall back to YOLO
        human_subj = r.get("human_subject", "")
        yolo_subj  = r.get("yolo_subject_majority", "")
        subject = human_subj if human_subj and human_subj != "unclear" else yolo_subj
        cat_id = r.get("cat_id", "")
        # Prey: consolidated_burst_label is the best truth
        prey = r.get("consolidated_burst_label", "")
        # Fallback to firmware label
        if prey in ("", None):
            prey = r.get("fw_burst_label", "")
        try:
            prey_n = int(prey)
        except Exception:
            prey_n = None

        rows.append({
            "burst_id": burst_id,
            "epoch": epoch,
            "dt": datetime.fromtimestamp(epoch, CET) if epoch else None,
            "direction": human_dir if human_dir else "",
            "subject": subject if subject else "",
            "cat_id": cat_id if cat_id else "",
            "prey": prey_n,
        })

# Backfill direction + prey from meta.json for newer bursts that have no
# human labels yet
DIR_MAP = {1: "entering", 2: "exiting", 0: "unclear", -1: "unclear"}
for r in rows:
    if r["direction"] and r["direction"] != "unclear":
        continue
    meta_path = SD / r["burst_id"] / "meta.json"
    if not meta_path.exists():
        continue
    try:
        m = json.loads(meta_path.read_text())
    except Exception:
        continue
    if not r["direction"]:
        r["direction"] = DIR_MAP.get(m.get("direction", -1), "unclear")
    if r["prey"] is None:
        api = m.get("apiResult", -1)
        if api in (0, 1):
            r["prey"] = api

rows.sort(key=lambda r: r["epoch"])

# Date range
dts = [r["dt"] for r in rows if r["dt"]]
date_min = min(dts).date() if dts else None
date_max = max(dts).date() if dts else None
days = (date_max - date_min).days + 1 if date_min else 0

print(f"=" * 70)
print(f"Cat activity report — {date_min} to {date_max} ({days} days, {len(rows)} bursts)")
print(f"=" * 70)


# -------- Cat ID breakdown --------
print(f"\nCAT IDENTITY (from human labels on entering bursts)")
cat_counter = Counter(r["cat_id"] for r in rows if r["cat_id"])
total_labeled = sum(cat_counter.values())
for cat, n in cat_counter.most_common():
    pct = n / total_labeled * 100 if total_labeled else 0
    print(f"  {cat:<12} {n:>4} bursts  ({pct:.0f}%)")
print(f"  {'unlabeled':<12} {len(rows) - total_labeled:>4} bursts")


# -------- Direction breakdown --------
print(f"\nDIRECTION")
dir_counter = Counter(r["direction"] or "missing" for r in rows)
for d, n in dir_counter.most_common():
    print(f"  {d:<10} {n:>4}")


# -------- Subject breakdown --------
print(f"\nSUBJECT (who/what triggered the burst)")
subj_counter = Counter(r["subject"] or "missing" for r in rows)
for s, n in subj_counter.most_common():
    print(f"  {s:<10} {n:>4}")


# -------- Prey breakdown --------
print(f"\nPREY VERDICT")
prey_counter = Counter()
for r in rows:
    if r["prey"] is None:
        prey_counter["unknown"] += 1
    else:
        prey_counter["prey" if r["prey"] == 1 else "no-prey"] += 1
for k, v in prey_counter.most_common():
    print(f"  {k:<10} {v:>4}")


# -------- Prey events: when does she actually bring prey? --------
print(f"\nPREY EVENTS (apiResult==1)")
prey_events = [r for r in rows if r["prey"] == 1 and r["dt"]]
print(f"  Total prey-positive bursts: {len(prey_events)}")
print(f"  By cat ID:")
prey_by_cat = Counter(r["cat_id"] or "unlabeled" for r in prey_events)
for c, n in prey_by_cat.most_common():
    print(f"    {c:<12} {n}")
print(f"  By direction:")
prey_by_dir = Counter(r["direction"] or "missing" for r in prey_events)
for d, n in prey_by_dir.most_common():
    print(f"    {d:<12} {n}")
print(f"  Hour of day:")
prey_by_hour = Counter(r["dt"].hour for r in prey_events)
for h in range(24):
    if prey_by_hour[h]:
        bar = "█" * prey_by_hour[h]
        print(f"    {h:02d}:00  {bar} {prey_by_hour[h]}")


# -------- Hour-of-day activity (all bursts) --------
print(f"\nACTIVITY BY HOUR OF DAY (all bursts, normalised to per-day)")
hour_counter = Counter(r["dt"].hour for r in rows if r["dt"])
hour_enter   = Counter(r["dt"].hour for r in rows if r["dt"] and r["direction"] == "entering")
hour_exit    = Counter(r["dt"].hour for r in rows if r["dt"] and r["direction"] == "exiting")
max_count = max(hour_counter.values()) if hour_counter else 1
print(f"  {'hour':<5} {'all':>4} {'enter':>6} {'exit':>5} {'per_day':>8}  bar")
for h in range(24):
    n = hour_counter[h]
    ne = hour_enter[h]; nx = hour_exit[h]
    pd_ = n / max(days, 1)
    bar = "█" * int(40 * n / max_count) if max_count else ""
    print(f"  {h:02d}:00 {n:>4} {ne:>6} {nx:>5} {pd_:>6.2f}/d  {bar}")


# -------- Day-by-day activity --------
print(f"\nDAY-BY-DAY ACTIVITY (last 14 days)")
by_day = defaultdict(lambda: {"total": 0, "enter": 0, "exit": 0, "prey": 0,
                              "mazge": 0, "benis": 0, "unknown_cat": 0})
for r in rows:
    if not r["dt"]:
        continue
    d = r["dt"].date().isoformat()
    by_day[d]["total"] += 1
    if r["direction"] == "entering": by_day[d]["enter"] += 1
    if r["direction"] == "exiting":  by_day[d]["exit"]  += 1
    if r["prey"] == 1:                by_day[d]["prey"]  += 1
    if r["cat_id"] == "mazge":        by_day[d]["mazge"] += 1
    elif r["cat_id"] == "benis":      by_day[d]["benis"] += 1
    elif r["cat_id"] == "unknown":    by_day[d]["unknown_cat"] += 1
last_days = sorted(by_day)[-14:]
print(f"  {'date':<11} {'total':>6} {'enter':>6} {'exit':>5} {'prey':>5} "
      f"{'mazge':>6} {'benis':>6} {'?':>4}")
for d in last_days:
    s = by_day[d]
    prey_marker = " *" if s["prey"] else ""
    print(f"  {d:<11} {s['total']:>6} {s['enter']:>6} {s['exit']:>5} {s['prey']:>5} "
          f"{s['mazge']:>6} {s['benis']:>6} {s['unknown_cat']:>4}{prey_marker}")


# -------- Day-of-week --------
print(f"\nDAY OF WEEK")
WEEKDAYS = ["Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"]
dow_total = Counter(r["dt"].weekday() for r in rows if r["dt"])
dow_prey  = Counter(r["dt"].weekday() for r in rows if r["dt"] and r["prey"] == 1)
print(f"  {'day':<5} {'bursts':>7} {'prey':>5}")
for i, name in enumerate(WEEKDAYS):
    print(f"  {name:<5} {dow_total[i]:>7} {dow_prey[i]:>5}")


# -------- Trip duration (exit → next entry of same cat) --------
print(f"\nTRIP DURATIONS (time between an EXIT and the next ENTRY)")
exits_then_entries = []
last_exit_t = None
for r in rows:
    if not r["dt"]: continue
    if r["direction"] == "exiting":
        last_exit_t = r["dt"]
    elif r["direction"] == "entering" and last_exit_t:
        delta = (r["dt"] - last_exit_t).total_seconds() / 60  # minutes
        if 0 < delta < 60 * 24 * 2:  # cap at 48h to ignore boundary effects
            exits_then_entries.append(delta)
        last_exit_t = None
if exits_then_entries:
    exits_then_entries.sort()
    n = len(exits_then_entries)
    median = exits_then_entries[n // 2]
    p25 = exits_then_entries[n // 4]
    p75 = exits_then_entries[3 * n // 4]
    p90 = exits_then_entries[int(n * 0.9)]
    print(f"  Trips analysed: {n}")
    print(f"  Median:  {median:6.0f} min  ({median/60:.1f}h)")
    print(f"  p25:     {p25:6.0f} min  ({p25/60:.1f}h)")
    print(f"  p75:     {p75:6.0f} min  ({p75/60:.1f}h)")
    print(f"  p90:     {p90:6.0f} min  ({p90/60:.1f}h)")
    print(f"  Buckets:")
    buckets = [("<5 min",   lambda x: x < 5),
               ("5-30 min", lambda x: 5 <= x < 30),
               ("30-60 min",lambda x: 30 <= x < 60),
               ("1-3 h",    lambda x: 60 <= x < 180),
               ("3-6 h",    lambda x: 180 <= x < 360),
               ("6-12 h",   lambda x: 360 <= x < 720),
               (">12 h",    lambda x: x >= 720)]
    for name, pred in buckets:
        cnt = sum(1 for x in exits_then_entries if pred(x))
        pct = cnt / n * 100
        print(f"    {name:<10} {cnt:>4}  ({pct:.0f}%)")


# -------- Prey by trip length (do longer trips bring more prey?) --------
print(f"\nDOES LONGER HUNT = MORE PREY? (prey-entry, prior-exit pairs)")
prey_trips = []
last_exit_t = None
for r in rows:
    if not r["dt"]: continue
    if r["direction"] == "exiting":
        last_exit_t = r["dt"]
    elif r["direction"] == "entering" and last_exit_t:
        delta_min = (r["dt"] - last_exit_t).total_seconds() / 60
        if 0 < delta_min < 60 * 24:
            prey_trips.append((delta_min, r["prey"] == 1))
        last_exit_t = None
if prey_trips:
    buckets = [("<30 min",  lambda x: x < 30),
               ("30-60 min",lambda x: 30 <= x < 60),
               ("1-3 h",    lambda x: 60 <= x < 180),
               ("3-6 h",    lambda x: 180 <= x < 360),
               ("6-12 h",   lambda x: 360 <= x < 720),
               (">12 h",    lambda x: x >= 720)]
    print(f"  {'duration':<10} {'trips':>5} {'prey':>5} {'rate':>6}")
    for name, pred in buckets:
        sub = [p for t, p in prey_trips if pred(t)]
        if not sub:
            continue
        prey_count = sum(sub)
        print(f"  {name:<10} {len(sub):>5} {prey_count:>5} {prey_count/len(sub)*100:>5.1f}%")


# -------- Mazge vs Benis side-by-side --------
print(f"\nMAZGE vs BENIS")
for cat in ["mazge", "benis", "unknown"]:
    sub = [r for r in rows if r["cat_id"] == cat]
    if not sub:
        continue
    enter = sum(1 for r in sub if r["direction"] == "entering")
    exit_ = sum(1 for r in sub if r["direction"] == "exiting")
    prey  = sum(1 for r in sub if r["prey"] == 1)
    print(f"  {cat}:")
    print(f"    bursts: {len(sub)}   entering: {enter}   exiting: {exit_}   prey: {prey}")
    if sub:
        hours = Counter(r["dt"].hour for r in sub if r["dt"])
        # Most active hour
        top_hr = hours.most_common(1)[0]
        print(f"    most active hour: {top_hr[0]:02d}:00 ({top_hr[1]} bursts)")


# -------- Summary one-liner --------
print(f"\n" + "=" * 70)
total = len(rows)
prey = sum(1 for r in rows if r["prey"] == 1)
mazge = sum(1 for r in rows if r["cat_id"] == "mazge")
benis = sum(1 for r in rows if r["cat_id"] == "benis")
enter = sum(1 for r in rows if r["direction"] == "entering")
exit_ = sum(1 for r in rows if r["direction"] == "exiting")
print(f"TL;DR: {days} days, {total} bursts, {prey} prey events ({prey/total*100:.1f}%), "
      f"{enter} entries / {exit_} exits, mazge={mazge} benis={benis}")
print(f"=" * 70)
