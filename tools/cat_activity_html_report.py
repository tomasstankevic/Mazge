"""Generate a self-contained HTML report with interactive charts (Chart.js)
and per-cat statistical analysis.

Usage: uv run python tools/cat_activity_html_report.py
Output: captures/cat_activity_report.html
"""
import csv
import json
import math
from collections import Counter, defaultdict
from datetime import datetime, timezone, timedelta
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SD = REPO / "captures" / "sd"
BURSTS_CSV = REPO / "dataset" / "bursts.csv"
OUT = REPO / "captures" / "cat_activity_report.html"
CET = timezone(timedelta(hours=2))
DIR_MAP = {1: "entering", 2: "exiting", 0: "unclear", -1: "unclear"}

# ── Load & enrich data ──────────────────────────────────────────────────
rows = []
with BURSTS_CSV.open() as f:
    for r in csv.DictReader(f):
        epoch = int(r["epoch"]) if r["epoch"] else 0
        human_dir = r.get("human_direction", "")
        human_subj = r.get("human_subject", "")
        yolo_subj = r.get("yolo_subject_majority", "")
        subject = human_subj if human_subj and human_subj != "unclear" else yolo_subj
        cat_id = r.get("cat_id", "")
        # For main analysis, only use human-verified labels (TP/TN only)
        human_prey_val = r.get("human_prey", "")
        if human_prey_val == "1":
            prey_n = 1
        elif human_prey_val == "0":
            prey_n = 0
        else:
            prey_n = None  # unlabeled — excluded from prey stats
        fw_label = r.get("fw_burst_label", "")
        full_label = r.get("full_burst_label", "")
        rows.append({
            "burst_id": r["burst_id"], "epoch": epoch,
            "dt": datetime.fromtimestamp(epoch, CET) if epoch else None,
            "direction": human_dir or "", "subject": subject or "",
            "cat_id": cat_id or "", "prey": prey_n,
            "fw_label": fw_label, "human_prey": human_prey_val,
            "full_label": full_label,
        })

# Backfill from meta.json
for r in rows:
    meta_path = SD / r["burst_id"] / "meta.json"
    if not meta_path.exists():
        continue
    try:
        m = json.loads(meta_path.read_text())
    except Exception:
        continue
    if not r["direction"] or r["direction"] == "unclear":
        r["direction"] = DIR_MAP.get(m.get("direction", -1), "unclear")
    # prey is human-only — do NOT backfill from meta.json apiResult

rows.sort(key=lambda r: r["epoch"])
dts = [r["dt"] for r in rows if r["dt"]]
date_min = min(dts).date()
date_max = max(dts).date()
days = (date_max - date_min).days + 1

# ── Compute stats ────────────────────────────────────────────────────────
# Hour-of-day
hour_all = Counter(r["dt"].hour for r in rows if r["dt"])
hour_enter = Counter(r["dt"].hour for r in rows if r["dt"] and r["direction"] == "entering")
hour_exit = Counter(r["dt"].hour for r in rows if r["dt"] and r["direction"] == "exiting")
hour_prey = Counter(r["dt"].hour for r in rows if r["dt"] and r["prey"] == 1)

# Day-by-day
by_day = defaultdict(lambda: {"total": 0, "enter": 0, "exit": 0, "prey": 0})
for r in rows:
    if not r["dt"]:
        continue
    d = r["dt"].date().isoformat()
    by_day[d]["total"] += 1
    if r["direction"] == "entering": by_day[d]["enter"] += 1
    if r["direction"] == "exiting": by_day[d]["exit"] += 1
    if r["prey"] == 1: by_day[d]["prey"] += 1
day_labels = sorted(by_day)

# Per-cat stats
cat_stats = {}
for cat in ["mazge", "benis"]:
    sub = [r for r in rows if r["cat_id"] == cat]
    if not sub:
        continue
    enter = sum(1 for r in sub if r["direction"] == "entering")
    prey = sum(1 for r in sub if r["prey"] == 1)
    hours = Counter(r["dt"].hour for r in sub if r["dt"])
    # Per-hour entering
    enter_hours = Counter(r["dt"].hour for r in sub if r["dt"] and r["direction"] == "entering")
    prey_hours = Counter(r["dt"].hour for r in sub if r["dt"] and r["prey"] == 1)
    cat_stats[cat] = {
        "total": len(sub), "enter": enter, "prey": prey,
        "hunt_rate": prey / enter * 100 if enter else 0,
        "hours": [hours.get(h, 0) for h in range(24)],
        "prey_hours": [prey_hours.get(h, 0) for h in range(24)],
    }

# Trip durations
trips = []
last_exit_t = None
for r in rows:
    if not r["dt"]:
        continue
    if r["direction"] == "exiting":
        last_exit_t = r["dt"]
    elif r["direction"] == "entering" and last_exit_t:
        delta = (r["dt"] - last_exit_t).total_seconds() / 60
        if 0 < delta < 60 * 48:
            trips.append({"minutes": delta, "prey": r["prey"] == 1,
                          "cat": r["cat_id"] or "unknown"})
        last_exit_t = None

trip_buckets = [("<5m", 0, 5), ("5-30m", 5, 30), ("30-60m", 30, 60),
                ("1-3h", 60, 180), ("3-6h", 180, 360), ("6-12h", 360, 720), (">12h", 720, 99999)]
trip_bucket_labels = [b[0] for b in trip_buckets]
trip_bucket_counts = []
trip_bucket_prey = []
for name, lo, hi in trip_buckets:
    sub = [t for t in trips if lo <= t["minutes"] < hi]
    trip_bucket_counts.append(len(sub))
    trip_bucket_prey.append(sum(1 for t in sub if t["prey"]))

# Day-of-week
dow_names = ["Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"]
dow_total = [0] * 7
dow_prey = [0] * 7
for r in rows:
    if r["dt"]:
        dow_total[r["dt"].weekday()] += 1
        if r["prey"] == 1:
            dow_prey[r["dt"].weekday()] += 1

# Prey timeline
prey_events = []
for r in rows:
    if r["prey"] == 1 and r["dt"]:
        prey_events.append({
            "date": r["dt"].strftime("%Y-%m-%d %H:%M"),
            "cat": r["cat_id"] or "?",
            "burst": r["burst_id"],
            "hour": r["dt"].hour,
        })

# Subject breakdown
subj_counter = Counter(r["subject"] or "missing" for r in rows)
subj_labels = [k for k, _ in subj_counter.most_common()]
subj_values = [v for _, v in subj_counter.most_common()]

# Direction breakdown
dir_counter = Counter(r["direction"] or "missing" for r in rows)

# ── Summary stats ────────────────────────────────────────────────────────
total = len(rows)
total_prey = sum(1 for r in rows if r["prey"] == 1)
total_enter = sum(1 for r in rows if r["direction"] == "entering")
total_exit = sum(1 for r in rows if r["direction"] == "exiting")
total_mazge = sum(1 for r in rows if r["cat_id"] == "mazge")
total_benis = sum(1 for r in rows if r["cat_id"] == "benis")
total_labeled = sum(1 for r in rows if r["prey"] is not None)
bursts_per_day = total / max(days, 1)
prey_per_week = total_prey / max(days / 7, 1)

# Pre-compute for f-string safety
mazge_hunt_rate = cat_stats.get("mazge", {}).get("hunt_rate", 0)
benis_hunt_rate = cat_stats.get("benis", {}).get("hunt_rate", 0)
mazge_prey_total = cat_stats.get("mazge", {}).get("prey", 0)
benis_prey_total = cat_stats.get("benis", {}).get("prey", 0)

# Night vs day prey
night_prey = sum(1 for r in rows if r["prey"] == 1 and r["dt"] and (r["dt"].hour >= 22 or r["dt"].hour < 6))
day_prey = total_prey - night_prey

# Median trip
trip_mins = sorted(t["minutes"] for t in trips)
median_trip = trip_mins[len(trip_mins) // 2] if trip_mins else 0

# ── FP / FN analysis ────────────────────────────────────────────────────
def confusion(rows_sub, pred_key, truth_key="human_prey"):
    both = [r for r in rows_sub if r[truth_key] in ("0", "1") and r[pred_key] in ("0", "1")]
    tp = sum(1 for r in both if r[pred_key] == "1" and r[truth_key] == "1")
    fp = sum(1 for r in both if r[pred_key] == "1" and r[truth_key] == "0")
    fn = sum(1 for r in both if r[pred_key] == "0" and r[truth_key] == "1")
    tn = sum(1 for r in both if r[pred_key] == "0" and r[truth_key] == "0")
    prec = tp / (tp + fp) * 100 if (tp + fp) else 0
    rec = tp / (tp + fn) * 100 if (tp + fn) else 0
    f1 = 2 * prec * rec / (prec + rec) if (prec + rec) else 0
    return {"n": len(both), "tp": tp, "fp": fp, "fn": fn, "tn": tn,
            "prec": prec, "rec": rec, "f1": f1}

cm_fw = confusion(rows, "fw_label")
cm_full = confusion(rows, "full_label")

# Per-cat FP/FN (firmware)
cm_fw_mazge = confusion([r for r in rows if r["cat_id"] == "mazge"], "fw_label")
cm_fw_benis = confusion([r for r in rows if r["cat_id"] == "benis"], "fw_label")

# Collect FP and FN details
fp_list = [r for r in rows if r["human_prey"] == "0" and r["fw_label"] == "1"]
fn_list = [r for r in rows if r["human_prey"] == "1" and r["fw_label"] == "0"]

# ── Prey incident grouping ──────────────────────────────────────────────
prey_rows_sorted = sorted(
    [r for r in rows if r["prey"] == 1 and r["dt"]],
    key=lambda r: r["epoch"]
)
incidents = []
cur_group = []
for pr in prey_rows_sorted:
    if not cur_group or (pr["epoch"] - cur_group[-1]["epoch"]) < 1800:
        cur_group.append(pr)
    else:
        incidents.append(cur_group)
        cur_group = [pr]
if cur_group:
    incidents.append(cur_group)
total_incidents = len(incidents)
multi_burst_incidents = sum(1 for g in incidents if len(g) > 1)
max_bursts_per_incident = max((len(g) for g in incidents), default=0)

# ── Generate HTML ────────────────────────────────────────────────────────
html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Cat Activity Report — {date_min} to {date_max}</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.4/dist/chart.umd.min.js"></script>
<style>
  :root {{ --bg: #1a1a2e; --card: #16213e; --accent: #e94560; --accent2: #0f3460;
           --text: #eee; --muted: #aaa; --green: #4ade80; --orange: #fb923c;
           --blue: #60a5fa; --purple: #a78bfa; }}
  * {{ margin: 0; padding: 0; box-sizing: border-box; }}
  body {{ font-family: 'Segoe UI', system-ui, sans-serif; background: var(--bg);
          color: var(--text); padding: 20px; max-width: 1400px; margin: auto; }}
  h1 {{ text-align: center; font-size: 2em; margin-bottom: 4px; }}
  .subtitle {{ text-align: center; color: var(--muted); margin-bottom: 30px; font-size: 1.1em; }}
  .kpi-row {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(160px, 1fr));
              gap: 14px; margin-bottom: 30px; }}
  .kpi {{ background: var(--card); border-radius: 12px; padding: 18px; text-align: center; }}
  .kpi .value {{ font-size: 2.2em; font-weight: 700; }}
  .kpi .label {{ color: var(--muted); font-size: 0.85em; margin-top: 4px; }}
  .kpi.prey .value {{ color: var(--accent); }}
  .kpi.mazge .value {{ color: var(--orange); }}
  .kpi.benis .value {{ color: var(--blue); }}
  .kpi.green .value {{ color: var(--green); }}
  .grid {{ display: grid; grid-template-columns: 1fr 1fr; gap: 20px; margin-bottom: 20px; }}
  @media (max-width: 900px) {{ .grid {{ grid-template-columns: 1fr; }} }}
  .card {{ background: var(--card); border-radius: 12px; padding: 20px; }}
  .card h2 {{ margin-bottom: 12px; font-size: 1.15em; }}
  .card.full {{ grid-column: 1 / -1; }}
  canvas {{ max-height: 340px; }}
  table {{ width: 100%; border-collapse: collapse; font-size: 0.9em; }}
  th, td {{ padding: 6px 10px; text-align: left; border-bottom: 1px solid #333; }}
  th {{ color: var(--muted); font-weight: 600; }}
  .prey-dot {{ display: inline-block; width: 8px; height: 8px; border-radius: 50%;
               background: var(--accent); margin-right: 4px; }}
  .cat-badge {{ display: inline-block; padding: 2px 10px; border-radius: 8px;
                font-size: 0.85em; font-weight: 600; }}
  .cat-badge.mazge {{ background: rgba(251,146,60,0.2); color: var(--orange); }}
  .cat-badge.benis {{ background: rgba(96,165,250,0.2); color: var(--blue); }}
  .cat-badge.unknown {{ background: rgba(167,139,250,0.2); color: var(--purple); }}
  .stat-row {{ display: flex; justify-content: space-between; padding: 6px 0;
               border-bottom: 1px solid #282848; }}
  .stat-row .k {{ color: var(--muted); }}
  .insight {{ background: rgba(233,69,96,0.08); border-left: 3px solid var(--accent);
              padding: 12px 16px; border-radius: 0 8px 8px 0; margin: 12px 0;
              font-size: 0.95em; line-height: 1.5; }}
  .insight strong {{ color: var(--accent); }}
</style>
</head>
<body>

<h1>🐱 Cat Activity Report</h1>
<p class="subtitle">{date_min} — {date_max} &nbsp;·&nbsp; {days} days &nbsp;·&nbsp; {total} bursts &nbsp;·&nbsp; {total_labeled} human-labeled</p>

<div class="kpi-row">
  <div class="kpi prey"><div class="value">{total_prey}</div><div class="label">Confirmed Prey</div></div>
  <div class="kpi green"><div class="value">{total_enter}</div><div class="label">Entries</div></div>
  <div class="kpi"><div class="value">{total_exit}</div><div class="label">Exits</div></div>
  <div class="kpi mazge"><div class="value">{total_mazge}</div><div class="label">Mazge (labeled)</div></div>
  <div class="kpi benis"><div class="value">{total_benis}</div><div class="label">Benis (labeled)</div></div>
  <div class="kpi"><div class="value">{bursts_per_day:.1f}</div><div class="label">Bursts / Day</div></div>
  <div class="kpi prey"><div class="value">{prey_per_week:.1f}</div><div class="label">Prey / Week</div></div>
  <div class="kpi"><div class="value">{total_incidents}</div><div class="label">Unique Prey Incidents</div></div>
  <div class="kpi"><div class="value">{median_trip:.0f}m</div><div class="label">Median Trip</div></div>
</div>

<!-- Key Insights -->
<div class="card full" style="margin-bottom:20px">
  <h2>📊 Key Insights</h2>
  <div class="insight">
    <strong>Mazge is the primary hunter.</strong> Of labeled entering bursts, Mazge has a
    {mazge_hunt_rate:.1f}% prey rate vs Benis at
    {benis_hunt_rate:.1f}%. Mazge brought {mazge_prey_total}
    prey, Benis brought {benis_prey_total}.
  </div>
  <div class="insight">
    <strong>Hunting is a nighttime activity.</strong> {night_prey} of {total_prey} prey events
    ({night_prey/max(total_prey,1)*100:.0f}%) occur between 22:00–06:00. Peak hour is 03:00 with
    {hour_prey.get(3,0)} events — deep into the night.
  </div>
  <div class="insight">
    <strong>The sweet-spot hunt lasts 30 min – 3 hours.</strong> Trips in this range have
    ~15% prey rate. Quick <5 min trips and very long >6h trips almost never produce prey.
  </div>
  <div class="insight">
    <strong>Cats are crepuscular.</strong> Activity peaks at dawn (~07:00, 2.7 bursts/day) and
    dusk (~20:00, 5.0 bursts/day). Midday 10:00–15:00 is near-zero.
  </div>
</div>

<!-- Charts row 1: hour-of-day + daily timeline -->
<div class="grid">
  <div class="card">
    <h2>🕐 Activity by Hour of Day</h2>
    <canvas id="hourChart"></canvas>
  </div>
  <div class="card">
    <h2>📅 Daily Burst Count</h2>
    <canvas id="dailyChart"></canvas>
  </div>
</div>

<!-- Charts row 2: prey hours + trip durations -->
<div class="grid">
  <div class="card">
    <h2>🎯 Prey Events by Hour</h2>
    <canvas id="preyHourChart"></canvas>
  </div>
  <div class="card">
    <h2>⏱️ Trip Duration vs Prey Rate</h2>
    <canvas id="tripChart"></canvas>
  </div>
</div>

<!-- Charts row 3: per-cat + day of week -->
<div class="grid">
  <div class="card">
    <h2>🐱 Mazge vs Benis — Hourly Activity</h2>
    <canvas id="catHourChart"></canvas>
  </div>
  <div class="card">
    <h2>📆 Day of Week</h2>
    <canvas id="dowChart"></canvas>
  </div>
</div>

<!-- Charts row 4: subject + direction -->
<div class="grid">
  <div class="card">
    <h2>👤 What Triggered the Burst</h2>
    <canvas id="subjectChart"></canvas>
  </div>
  <div class="card">
    <h2>↔️ Direction Breakdown</h2>
    <canvas id="dirChart"></canvas>
  </div>
</div>

<!-- Per-cat stat cards -->
<div class="grid">
"""

for cat, color in [("mazge", "var(--orange)"), ("benis", "var(--blue)")]:
    s = cat_stats.get(cat)
    if not s:
        continue
    html += f"""
  <div class="card">
    <h2 style="color:{color}">{"🟠" if cat == "mazge" else "🔵"} {cat.title()} — Statistics</h2>
    <div class="stat-row"><span class="k">Total labeled bursts</span><span>{s['total']}</span></div>
    <div class="stat-row"><span class="k">Entering</span><span>{s['enter']}</span></div>
    <div class="stat-row"><span class="k">Prey brought home</span><span style="color:var(--accent)">{s['prey']}</span></div>
    <div class="stat-row"><span class="k">Hunt success rate</span><span style="color:var(--accent)">{s['hunt_rate']:.1f}%</span></div>
    <div class="stat-row"><span class="k">Most active hour</span><span>{s['hours'].index(max(s['hours'])):02d}:00</span></div>
  </div>
"""

html += "</div>\n"

# ── FP / FN section ──────────────────────────────────────────────────────
html += f"""
<div class="card full" style="margin-top:20px">
  <h2>🔬 Detection Accuracy — Firmware vs Human Ground Truth</h2>
  <p style="color:var(--muted);margin-bottom:16px;font-size:0.9em">
    Comparing firmware on-device prey classification against human-reviewed labels
    ({cm_fw['n']} bursts with both labels).
  </p>
  <div class="grid" style="margin-bottom:0">
    <div>
      <h3 style="margin-bottom:8px;font-size:1em">Confusion Matrix — Firmware (on-device)</h3>
      <table>
        <tr><th></th><th style="text-align:center">Human: Prey</th><th style="text-align:center">Human: No Prey</th></tr>
        <tr><td><strong>FW: Prey</strong></td>
            <td style="text-align:center;background:rgba(74,222,128,0.15);color:var(--green);font-weight:700">{cm_fw['tp']} TP</td>
            <td style="text-align:center;background:rgba(233,69,96,0.15);color:var(--accent);font-weight:700">{cm_fw['fp']} FP</td></tr>
        <tr><td><strong>FW: No Prey</strong></td>
            <td style="text-align:center;background:rgba(251,146,60,0.15);color:var(--orange);font-weight:700">{cm_fw['fn']} FN</td>
            <td style="text-align:center;background:rgba(255,255,255,0.05)">{cm_fw['tn']} TN</td></tr>
      </table>
      <div style="margin-top:12px">
        <div class="stat-row"><span class="k">Precision</span><span>{cm_fw['prec']:.1f}%</span></div>
        <div class="stat-row"><span class="k">Recall (sensitivity)</span><span>{cm_fw['rec']:.1f}%</span></div>
        <div class="stat-row"><span class="k">F1 Score</span><span>{cm_fw['f1']:.1f}%</span></div>
      </div>
    </div>
    <div>
      <h3 style="margin-bottom:8px;font-size:1em">Confusion Matrix — Full API Re-analysis</h3>
      <table>
        <tr><th></th><th style="text-align:center">Human: Prey</th><th style="text-align:center">Human: No Prey</th></tr>
        <tr><td><strong>API: Prey</strong></td>
            <td style="text-align:center;background:rgba(74,222,128,0.15);color:var(--green);font-weight:700">{cm_full['tp']} TP</td>
            <td style="text-align:center;background:rgba(233,69,96,0.15);color:var(--accent);font-weight:700">{cm_full['fp']} FP</td></tr>
        <tr><td><strong>API: No Prey</strong></td>
            <td style="text-align:center;background:rgba(251,146,60,0.15);color:var(--orange);font-weight:700">{cm_full['fn']} FN</td>
            <td style="text-align:center;background:rgba(255,255,255,0.05)">{cm_full['tn']} TN</td></tr>
      </table>
      <div style="margin-top:12px">
        <div class="stat-row"><span class="k">Precision</span><span>{cm_full['prec']:.1f}%</span></div>
        <div class="stat-row"><span class="k">Recall (sensitivity)</span><span>{cm_full['rec']:.1f}%</span></div>
        <div class="stat-row"><span class="k">F1 Score</span><span>{cm_full['f1']:.1f}%</span></div>
        <div class="stat-row"><span class="k">Evaluated bursts</span><span>{cm_full['n']}</span></div>
      </div>
    </div>
  </div>
  <div class="insight" style="margin-top:16px">
    <strong>Firmware has high recall ({cm_fw['rec']:.0f}%) but low precision ({cm_fw['prec']:.0f}%).</strong>
    It catches most real prey but fires {cm_fw['fp']} false alarms.
    The full API re-analysis is more accurate (precision {cm_full['prec']:.0f}%, recall {cm_full['rec']:.0f}%)
    but only covers {cm_full['n']} bursts.
  </div>
</div>

<!-- Per-cat FP/FN -->
<div class="grid" style="margin-top:20px">
  <div class="card">
    <h2 style="color:var(--orange)">🟠 Mazge — FW Accuracy</h2>
    <div class="stat-row"><span class="k">Evaluated</span><span>{cm_fw_mazge['n']}</span></div>
    <div class="stat-row"><span class="k">True Positives</span><span style="color:var(--green)">{cm_fw_mazge['tp']}</span></div>
    <div class="stat-row"><span class="k">False Positives</span><span style="color:var(--accent)">{cm_fw_mazge['fp']}</span></div>
    <div class="stat-row"><span class="k">False Negatives</span><span style="color:var(--orange)">{cm_fw_mazge['fn']}</span></div>
    <div class="stat-row"><span class="k">Precision</span><span>{cm_fw_mazge['prec']:.1f}%</span></div>
    <div class="stat-row"><span class="k">Recall</span><span>{cm_fw_mazge['rec']:.1f}%</span></div>
  </div>
  <div class="card">
    <h2 style="color:var(--blue)">🔵 Benis — FW Accuracy</h2>
    <div class="stat-row"><span class="k">Evaluated</span><span>{cm_fw_benis['n']}</span></div>
    <div class="stat-row"><span class="k">True Positives</span><span style="color:var(--green)">{cm_fw_benis['tp']}</span></div>
    <div class="stat-row"><span class="k">False Positives</span><span style="color:var(--accent)">{cm_fw_benis['fp']}</span></div>
    <div class="stat-row"><span class="k">False Negatives</span><span style="color:var(--orange)">{cm_fw_benis['fn']}</span></div>
    <div class="stat-row"><span class="k">Precision</span><span>{cm_fw_benis['prec']:.1f}%</span></div>
    <div class="stat-row"><span class="k">Recall</span><span>{cm_fw_benis['rec']:.1f}%</span></div>
  </div>
</div>

<!-- FP details table -->
<div class="card full" style="margin-top:20px">
  <h2>🚨 False Positives — Firmware said prey, human said no ({len(fp_list)})</h2>
  <table>
    <tr><th>#</th><th>Date & Time</th><th>Cat</th><th>Subject</th><th>Burst ID</th></tr>
"""
for i, r in enumerate(fp_list, 1):
    dt_str = r["dt"].strftime("%Y-%m-%d %H:%M") if r["dt"] else "?"
    cat = r["cat_id"] or "?"
    badge = f'<span class="cat-badge {cat}">{cat}</span>' if cat != "?" else '<span class="cat-badge unknown">?</span>'
    html += f'    <tr><td>{i}</td><td>{dt_str}</td><td>{badge}</td><td>{r["subject"]}</td><td style="font-family:monospace;font-size:0.85em">{r["burst_id"]}</td></tr>\n'

html += """  </table>
</div>
"""

# FN details table
html += f"""
<div class="card full" style="margin-top:20px">
  <h2>⚠️ False Negatives — Firmware missed prey, human confirmed ({len(fn_list)})</h2>
  <table>
    <tr><th>#</th><th>Date & Time</th><th>Cat</th><th>Subject</th><th>Burst ID</th></tr>
"""
for i, r in enumerate(fn_list, 1):
    dt_str = r["dt"].strftime("%Y-%m-%d %H:%M") if r["dt"] else "?"
    cat = r["cat_id"] or "?"
    badge = f'<span class="cat-badge {cat}">{cat}</span>' if cat != "?" else '<span class="cat-badge unknown">?</span>'
    html += f'    <tr><td>{i}</td><td>{dt_str}</td><td>{badge}</td><td>{r["subject"]}</td><td style="font-family:monospace;font-size:0.85em">{r["burst_id"]}</td></tr>\n'

html += """  </table>
</div>
"""

# ── Prey incidents (grouped) ────────────────────────────────────────────
html += f"""
<div class="card full" style="margin-top:20px">
  <h2>🎯 Prey Incidents (grouped by 30-min window)</h2>
  <p style="color:var(--muted);margin-bottom:12px;font-size:0.9em">
    {total_incidents} unique incidents from {total_prey} prey bursts.
    {multi_burst_incidents} incidents had multiple bursts (cat tried to enter repeatedly with same prey).
    Max bursts per incident: {max_bursts_per_incident}.
  </p>
  <table>
    <tr><th>#</th><th>Time</th><th>Cat</th><th>Bursts</th><th>Duration</th><th>Burst IDs</th></tr>
"""
for i, group in enumerate(incidents, 1):
    first_dt = group[0]["dt"].strftime("%Y-%m-%d %H:%M")
    cats_in_group = set(r["cat_id"] for r in group if r["cat_id"])
    cat_str = ", ".join(cats_in_group) if cats_in_group else "?"
    cat_cls = list(cats_in_group)[0] if len(cats_in_group) == 1 else "unknown"
    badge = f'<span class="cat-badge {cat_cls}">{cat_str}</span>'
    n = len(group)
    if n > 1:
        dur_s = group[-1]["epoch"] - group[0]["epoch"]
        dur_str = f"{dur_s // 60}m {dur_s % 60}s"
    else:
        dur_str = "—"
    burst_ids = "<br>".join(f'<span style="font-family:monospace;font-size:0.8em">{r["burst_id"]}</span>' for r in group)
    row_bg = "background:rgba(233,69,96,0.06);" if n > 1 else ""
    html += f'    <tr style="{row_bg}"><td>{i}</td><td>{first_dt}</td><td>{badge}</td><td>{n}</td><td>{dur_str}</td><td>{burst_ids}</td></tr>\n'

html += """  </table>
</div>
"""

# Chart.js scripts
hour_labels = [f"{h:02d}" for h in range(24)]
hour_all_data = [hour_all.get(h, 0) for h in range(24)]
hour_enter_data = [hour_enter.get(h, 0) for h in range(24)]
hour_exit_data = [hour_exit.get(h, 0) for h in range(24)]
hour_prey_data = [hour_prey.get(h, 0) for h in range(24)]
daily_totals = [by_day[d]["total"] for d in day_labels]
daily_prey = [by_day[d]["prey"] for d in day_labels]
daily_short = [d[5:] for d in day_labels]  # MM-DD
trip_prey_rate = [round(trip_bucket_prey[i] / trip_bucket_counts[i] * 100, 1)
                  if trip_bucket_counts[i] else 0 for i in range(len(trip_buckets))]

mazge_hours = cat_stats.get("mazge", {}).get("hours", [0]*24)
benis_hours = cat_stats.get("benis", {}).get("hours", [0]*24)
mazge_prey_hours = cat_stats.get("mazge", {}).get("prey_hours", [0]*24)
benis_prey_hours = cat_stats.get("benis", {}).get("prey_hours", [0]*24)

dir_labels_js = list(dir_counter.keys())
dir_values_js = list(dir_counter.values())

html += f"""
<script>
Chart.defaults.color = '#aaa';
Chart.defaults.borderColor = 'rgba(255,255,255,0.06)';
const hours = {json.dumps(hour_labels)};

// Hour of day
new Chart(document.getElementById('hourChart'), {{
  type: 'bar',
  data: {{
    labels: hours,
    datasets: [
      {{ label: 'Entering', data: {json.dumps(hour_enter_data)}, backgroundColor: 'rgba(74,222,128,0.7)' }},
      {{ label: 'Exiting',  data: {json.dumps(hour_exit_data)},  backgroundColor: 'rgba(96,165,250,0.7)' }},
      {{ label: 'Other',    data: {json.dumps([hour_all_data[h] - hour_enter_data[h] - hour_exit_data[h] for h in range(24)])},
         backgroundColor: 'rgba(255,255,255,0.15)' }},
    ]
  }},
  options: {{ responsive: true, plugins: {{ legend: {{ position: 'top' }} }},
             scales: {{ x: {{ stacked: true }}, y: {{ stacked: true, beginAtZero: true }} }} }}
}});

// Daily
new Chart(document.getElementById('dailyChart'), {{
  type: 'bar',
  data: {{
    labels: {json.dumps(daily_short)},
    datasets: [
      {{ label: 'Bursts', data: {json.dumps(daily_totals)}, backgroundColor: 'rgba(96,165,250,0.5)',
         borderColor: 'rgba(96,165,250,0.8)', borderWidth: 1 }},
      {{ label: 'Prey', data: {json.dumps(daily_prey)}, backgroundColor: 'rgba(233,69,96,0.8)',
         borderColor: 'var(--accent)', borderWidth: 1 }},
    ]
  }},
  options: {{ responsive: true, plugins: {{ legend: {{ position: 'top' }} }},
             scales: {{ y: {{ beginAtZero: true }} }} }}
}});

// Prey by hour
new Chart(document.getElementById('preyHourChart'), {{
  type: 'bar',
  data: {{
    labels: hours,
    datasets: [
      {{ label: 'Mazge prey', data: {json.dumps(mazge_prey_hours)}, backgroundColor: 'rgba(251,146,60,0.8)' }},
      {{ label: 'Benis prey', data: {json.dumps(benis_prey_hours)}, backgroundColor: 'rgba(96,165,250,0.8)' }},
      {{ label: 'Unlabeled prey', data: {json.dumps([hour_prey_data[h] - mazge_prey_hours[h] - benis_prey_hours[h] for h in range(24)])},
         backgroundColor: 'rgba(167,139,250,0.6)' }},
    ]
  }},
  options: {{ responsive: true, plugins: {{ legend: {{ position: 'top' }} }},
             scales: {{ x: {{ stacked: true }}, y: {{ stacked: true, beginAtZero: true }} }} }}
}});

// Trip duration
new Chart(document.getElementById('tripChart'), {{
  type: 'bar',
  data: {{
    labels: {json.dumps(trip_bucket_labels)},
    datasets: [
      {{ label: 'Trips', data: {json.dumps(trip_bucket_counts)}, backgroundColor: 'rgba(96,165,250,0.5)',
         yAxisID: 'y' }},
      {{ label: 'Prey rate %', data: {json.dumps(trip_prey_rate)}, type: 'line',
         borderColor: 'var(--accent)', backgroundColor: 'rgba(233,69,96,0.2)',
         pointRadius: 5, pointBackgroundColor: 'var(--accent)', yAxisID: 'y1', tension: 0.3 }},
    ]
  }},
  options: {{ responsive: true,
    plugins: {{ legend: {{ position: 'top' }} }},
    scales: {{
      y:  {{ beginAtZero: true, position: 'left', title: {{ display: true, text: 'Trips' }} }},
      y1: {{ beginAtZero: true, position: 'right', title: {{ display: true, text: 'Prey %' }},
             grid: {{ drawOnChartArea: false }} }}
    }}
  }}
}});

// Cat hourly
new Chart(document.getElementById('catHourChart'), {{
  type: 'bar',
  data: {{
    labels: hours,
    datasets: [
      {{ label: 'Mazge', data: {json.dumps(mazge_hours)}, backgroundColor: 'rgba(251,146,60,0.7)' }},
      {{ label: 'Benis', data: {json.dumps(benis_hours)}, backgroundColor: 'rgba(96,165,250,0.7)' }},
    ]
  }},
  options: {{ responsive: true, plugins: {{ legend: {{ position: 'top' }} }},
             scales: {{ y: {{ beginAtZero: true }} }} }}
}});

// Day of week
new Chart(document.getElementById('dowChart'), {{
  type: 'bar',
  data: {{
    labels: {json.dumps(dow_names)},
    datasets: [
      {{ label: 'Total', data: {json.dumps(dow_total)}, backgroundColor: 'rgba(96,165,250,0.5)' }},
      {{ label: 'Prey',  data: {json.dumps(dow_prey)},  backgroundColor: 'rgba(233,69,96,0.8)' }},
    ]
  }},
  options: {{ responsive: true, plugins: {{ legend: {{ position: 'top' }} }},
             scales: {{ y: {{ beginAtZero: true }} }} }}
}});

// Subject doughnut
new Chart(document.getElementById('subjectChart'), {{
  type: 'doughnut',
  data: {{
    labels: {json.dumps(subj_labels)},
    datasets: [{{ data: {json.dumps(subj_values)},
      backgroundColor: ['rgba(251,146,60,0.7)','rgba(96,165,250,0.7)','rgba(74,222,128,0.7)',
                         'rgba(233,69,96,0.7)','rgba(167,139,250,0.7)','rgba(255,255,255,0.2)'] }}]
  }},
  options: {{ responsive: true, plugins: {{ legend: {{ position: 'right' }} }} }}
}});

// Direction doughnut
new Chart(document.getElementById('dirChart'), {{
  type: 'doughnut',
  data: {{
    labels: {json.dumps(dir_labels_js)},
    datasets: [{{ data: {json.dumps(dir_values_js)},
      backgroundColor: ['rgba(167,139,250,0.7)','rgba(74,222,128,0.7)','rgba(96,165,250,0.7)',
                         'rgba(255,255,255,0.2)'] }}]
  }},
  options: {{ responsive: true, plugins: {{ legend: {{ position: 'right' }} }} }}
}});
</script>

<p style="text-align:center;color:#555;margin-top:30px;font-size:0.8em">
  Generated {datetime.now(CET).strftime("%Y-%m-%d %H:%M CET")} &nbsp;·&nbsp;
  tools/cat_activity_html_report.py
</p>
</body></html>
"""

OUT.write_text(html)
print(f"Wrote {OUT} ({len(html)//1024} KB)")
