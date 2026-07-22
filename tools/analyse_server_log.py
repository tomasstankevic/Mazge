"""Analyse mazge server JSONL logs (last 24h-ish).

Reads logs/server/server.jsonl* and reports:
  - per-frame call distribution (endpoints, status codes, JPEG size, decision_ms)
  - cat_id distribution (frames + bursts)
  - cat-recognition rate
  - prey-score distribution
  - door_action / severity distribution
  - per-burst aggregate (which cat won, did door open, was it prey)
  - suspicious "no benis" diagnosis: how often benis softmax was non-trivial
"""

from __future__ import annotations

import json
import statistics
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LOG_DIR = ROOT / "logs" / "server"


def load_lines() -> list[dict]:
    rows = []
    for p in sorted(LOG_DIR.glob("server.jsonl*")):
        with p.open() as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                try:
                    rows.append(json.loads(line))
                except json.JSONDecodeError:
                    continue
    return rows


def fmt_pct(num: int, denom: int) -> str:
    return f"{100.0 * num / denom:5.1f}%" if denom else "  n/a "


def pctile(xs, p):
    if not xs:
        return 0
    xs2 = sorted(xs)
    k = int(round((len(xs2) - 1) * p))
    return xs2[k]


def main() -> None:
    rows = load_lines()
    print(f"=== loaded {len(rows)} log lines from {LOG_DIR}")
    if not rows:
        return

    eps = Counter(r.get("ep") for r in rows)
    print(f"\nendpoint mix: {dict(eps)}")

    v2 = [r for r in rows if r.get("ep") == "/v2/frame"]
    print(f"v2 calls: {len(v2)}")
    statuses = Counter(r.get("status_code") for r in v2)
    print(f"  status codes: {dict(statuses)}")

    ok = [r for r in v2 if r.get("status_code") == 200]
    if not ok:
        print("no successful frames")
        return

    # Per-frame distributions
    sizes = [r["jpeg_bytes"] for r in ok if "jpeg_bytes" in r]
    decisions = [r["decision_ms"] for r in ok if "decision_ms" in r]
    preys = [r["prey_score"] for r in ok if "prey_score" in r]
    cat_ids = Counter(r.get("cat_id") for r in ok)
    cat_recd = Counter(bool(r.get("cat_recognized")) for r in ok)
    door = Counter(r.get("door_action") for r in ok)
    severity = Counter(r.get("severity") for r in ok)
    reason = Counter(r.get("reason") for r in ok)

    print(f"\n--- per-frame ({len(ok)} frames) ---")
    print(f"  jpeg bytes      p50={pctile(sizes,.5):>7} p95={pctile(sizes,.95):>7}")
    print(f"  decision ms     p50={pctile(decisions,.5):>4} p95={pctile(decisions,.95):>4}")
    print(f"  prey_score      p50={pctile(preys,.5):.3f} p90={pctile(preys,.9):.3f} max={max(preys):.3f}")
    print(f"  cat_id          {dict(cat_ids)}")
    print(f"  cat_recognized  true={cat_recd[True]} ({fmt_pct(cat_recd[True], len(ok))})  false={cat_recd[False]}")
    print(f"  door_action     {dict(door)}")
    print(f"  severity        {dict(severity)}")
    print(f"  reason          {dict(reason)}")

    # Cat softmax distributions (benis is index 1)
    softmax_rows = [r.get("cat_softmax") for r in ok if r.get("cat_softmax")]
    if softmax_rows:
        mazge_p = [sm[0] for sm in softmax_rows if len(sm) >= 2]
        benis_p = [sm[1] for sm in softmax_rows if len(sm) >= 2]
        # How often was benis the WINNER (even if below threshold)?
        benis_won = sum(1 for sm in softmax_rows if len(sm) >= 2 and sm[1] > sm[0])
        # How often did benis cross the threshold?
        benis_conf = sum(1 for sm in softmax_rows if len(sm) >= 2 and sm[1] >= 0.70)
        # Distribution of benis probability
        print(f"\n--- cat softmax (over {len(softmax_rows)} frames) ---")
        print(f"  benis probability p50={pctile(benis_p,.5):.3f} p90={pctile(benis_p,.9):.3f} max={max(benis_p):.3f}")
        print(f"  benis won softmax (>mazge): {benis_won}  ({fmt_pct(benis_won, len(softmax_rows))})")
        print(f"  benis above 0.70 threshold: {benis_conf}  ({fmt_pct(benis_conf, len(softmax_rows))})")
        # Buckets for benis prob
        buckets = [0, 0.1, 0.3, 0.5, 0.7, 0.9, 1.01]
        labels = ["<0.10", "0.10-0.30", "0.30-0.50", "0.50-0.70", "0.70-0.90", ">=0.90"]
        bucket_counts = [0] * len(labels)
        for bp in benis_p:
            for i, hi in enumerate(buckets[1:]):
                if bp < hi:
                    bucket_counts[i] += 1
                    break
        print(f"  benis prob buckets:")
        for lbl, c in zip(labels, bucket_counts):
            print(f"    {lbl:>10} : {c:>4} ({fmt_pct(c, len(softmax_rows))})")

    # Per-burst aggregation: group by (device_id, burst_id)
    bursts: dict[tuple, list[dict]] = defaultdict(list)
    for r in ok:
        bursts[(r.get("device_id"), r.get("burst_id"))].append(r)
    print(f"\n--- per-burst ({len(bursts)} bursts) ---")
    burst_winners = Counter()
    burst_recd = Counter()
    burst_prey_detected = 0
    burst_door = Counter()
    benis_dominant_bursts = []
    for key, frames in bursts.items():
        # Modal cat across frames where cat_recognized=true
        winners = Counter(f.get("cat_id") for f in frames if f.get("cat_recognized"))
        if winners:
            top = winners.most_common(1)[0][0]
            burst_winners[top] += 1
            burst_recd[True] += 1
        else:
            burst_winners["NONE"] += 1
            burst_recd[False] += 1
        if any(f.get("detected") for f in frames):
            burst_prey_detected += 1
        # Sample door action for the burst (any deny -> deny)
        if any(f.get("door_action") == "deny" for f in frames):
            burst_door["deny"] += 1
        else:
            burst_door["allow_only"] += 1
        # Check: even if not recognized, was benis the majority winner?
        sm_winners = Counter()
        for f in frames:
            sm = f.get("cat_softmax")
            if sm and len(sm) >= 2:
                sm_winners["benis" if sm[1] > sm[0] else "mazge"] += 1
        if sm_winners.get("benis", 0) > sm_winners.get("mazge", 0):
            benis_dominant_bursts.append((key, sm_winners, frames[0].get("ts_ms")))

    print(f"  modal recognized cat per burst: {dict(burst_winners)}")
    print(f"  bursts with any cat recognized: {burst_recd[True]} / {len(bursts)} ({fmt_pct(burst_recd[True], len(bursts))})")
    print(f"  bursts with prey detected:      {burst_prey_detected} / {len(bursts)}")
    print(f"  door per-burst:                 {dict(burst_door)}")
    print(f"  bursts where benis dominated softmax (raw, ignoring threshold): {len(benis_dominant_bursts)}")
    for (key, sm_w, ts) in benis_dominant_bursts[:10]:
        import datetime
        when = datetime.datetime.utcfromtimestamp(ts / 1000).strftime('%Y-%m-%d %H:%M:%S') if ts else "?"
        print(f"    {when}  device={key[0]} burst={key[1]}  softmax winners={dict(sm_w)}")


if __name__ == "__main__":
    main()
