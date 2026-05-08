"""Early-stop heuristic analysis: combine threshold N + check only first K frames
of the new optimal order [8,7,9,5,6,3,4,2,1,0].

For each combination (K=4..10, N=1..3), compute:
  TP, FP, FN, TN, precision, recall, F1
  expected latency saved vs full-burst N=2 baseline
"""
import json
from pathlib import Path

reproc_b = json.loads(Path("captures/prey_review/reprocess.json").read_text())

TRUE_BURSTS = {
    "20260501_015148_gen1",
    "20260504_021353_gen10",
    "20260505_005540_gen1",
    "20260505_220846_gen8",
    "20260505_221411_gen11",
    "20260505_230846_gen22",
    "20260505_231157_gen23",
    "20260507_032001_gen3",
    "20260507_050927_gen5",
    "20260507_212120_gen9",
    "20260508_035852_gen1",
}

NEW_ORDER = [8, 7, 9, 5, 6, 3, 4, 2, 1, 0]
PER_FRAME_API_MS = 1100  # estimated ~1.1s per frame including network

bursts = sorted(reproc_b.keys())
true_set = TRUE_BURSTS & set(bursts)
false_set = set(bursts) - true_set


def burst_outcome(verdicts, K, N):
    """Return (decided_prey, frames_checked, frames_until_decision).

    decided_prey: True if at least N flagged prey within first K frames in NEW_ORDER.
    frames_until_decision:
      - if decided_prey: index of K-th-from-bottom prey hit (the threshold-trigger)
      - else: K (we always check all K to be sure)
    """
    flagged_count = 0
    threshold_hit_at = None
    for pos, idx in enumerate(NEW_ORDER[:K]):
        v = verdicts[idx]
        if v == 1:
            flagged_count += 1
            if flagged_count >= N and threshold_hit_at is None:
                threshold_hit_at = pos + 1  # 1-based
    if threshold_hit_at is not None:
        return True, threshold_hit_at, threshold_hit_at
    return False, K, K


print(f"Dataset: {len(bursts)} bursts ({len(true_set)} true, {len(false_set)} false-pos)")
print(f"Frame priority: {NEW_ORDER}")
print(f"Per-frame API time estimate: {PER_FRAME_API_MS} ms")
print()

print(f"{'K':>3} {'N':>3} {'TP':>4} {'FP':>4} {'FN':>4} {'TN':>4} "
      f"{'prec':>5} {'rec':>5} {'F1':>5}  "
      f"{'mean_lat_TP':>11} {'mean_lat_clear':>14}")

for N in [1, 2, 3]:
    for K in range(2, 11):
        if K < N:
            continue
        tp = fp = fn = tn = 0
        latencies_tp = []
        latencies_clear = []
        for b in bursts:
            decided, _, frames = burst_outcome(reproc_b[b]["verdicts"], K, N)
            truth = b in true_set
            if decided and truth:
                tp += 1
                latencies_tp.append(frames * PER_FRAME_API_MS)
            elif decided and not truth:
                fp += 1
            elif not decided and truth:
                fn += 1
                latencies_clear.append(frames * PER_FRAME_API_MS)  # bad: declared clear
            else:
                tn += 1
                latencies_clear.append(frames * PER_FRAME_API_MS)
        prec = tp / (tp + fp) if (tp + fp) else 0
        rec = tp / (tp + fn) if (tp + fn) else 0
        f1 = 2 * prec * rec / (prec + rec) if (prec + rec) else 0
        m_tp = (sum(latencies_tp) / len(latencies_tp)) if latencies_tp else 0
        m_cl = (sum(latencies_clear) / len(latencies_clear)) if latencies_clear else 0
        marker = " ← chosen" if (K == 5 and N == 2) else ""
        print(f"{K:>3} {N:>3} {tp:>4} {fp:>4} {fn:>4} {tn:>4} "
              f"{prec*100:>4.0f}% {rec*100:>4.0f}% {f1*100:>4.0f}%  "
              f"{m_tp:>10.0f}ms {m_cl:>13.0f}ms{marker}")
    print()
