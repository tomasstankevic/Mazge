"""Analyze trigger-to-first-positive timing across all bursts.

Uses meta.json fields:
  triggerMs - when ToF/fake trigger fired
  apiCallMs - when first API request started
  apiDoneMs - when API analysis ended
  totalMs[i] - per-frame API call time (in order of API order, not frame index)
  apiResults[i] - per-frame verdict by frame index

For trigger->first-positive we need to know in what ORDER frames were sent.
The old order was: 3,4,5,6,7,8,9,2,1,0.
"""
import json
import statistics
from pathlib import Path

ROOT = Path("captures/sd")

# Old API order (before our reorder commit)
OLD_ORDER = [3, 4, 5, 6, 7, 8, 9, 2, 1, 0]


def first_positive_offset_ms(meta):
    """Return ms from triggerMs to API verdict on the first positive frame.

    The totalMs array has one entry per API call (in API order).
    We sum totalMs entries until we find the one whose frame_idx == 1 in apiResults.
    """
    api = meta.get("apiResults", [])
    totals = meta.get("totalMs", [])
    if not api or not totals:
        return None, None

    # Determine API order: the firmware always followed OLD_ORDER, but skipped
    # frames where i >= archive.count (post-trigger frames not ready).
    # totals is filled in API call order — index by sent_count.
    sent = 0
    cum = 0
    for frame_idx in OLD_ORDER:
        if frame_idx >= len(api):
            continue
        verdict = api[frame_idx]
        if verdict == -1:
            continue  # not checked
        if sent >= len(totals):
            break
        cum += totals[sent]
        sent += 1
        if verdict == 1:
            return cum, sent
    return None, None


def trigger_to_done_ms(meta):
    return meta.get("apiDoneMs", 0) - meta.get("triggerMs", 0)


def trigger_to_first_api(meta):
    return meta.get("apiCallMs", 0) - meta.get("triggerMs", 0)


prey_first_pos = []   # ms from trigger to first prey verdict (only prey bursts)
no_prey_total = []    # ms from trigger to API done (no prey, all 10 sent)
prey_total = []       # ms from trigger to API done (prey bursts; uses early-exit)

for d in sorted(ROOT.iterdir()):
    if not d.is_dir():
        continue
    meta_path = d / "meta.json"
    if not meta_path.exists():
        continue
    try:
        m = json.loads(meta_path.read_text())
    except Exception:
        continue
    api_result = m.get("apiResult", -1)
    done_ms = trigger_to_done_ms(m)
    if done_ms <= 0 or done_ms > 60000:
        continue
    if api_result == 1:
        prey_total.append(done_ms)
        fp_ms, sent = first_positive_offset_ms(m)
        if fp_ms is not None:
            prey_first_pos.append((d.name, fp_ms, sent))
    elif api_result == 0:
        no_prey_total.append(done_ms)


def stats(label, vals):
    if not vals:
        print(f"{label}: no data")
        return
    vals_sorted = sorted(vals)
    n = len(vals_sorted)
    print(f"{label}: n={n} min={vals_sorted[0]:.0f}ms median={statistics.median(vals_sorted):.0f}ms "
          f"mean={statistics.mean(vals_sorted):.0f}ms p95={vals_sorted[int(n*0.95)]:.0f}ms "
          f"max={vals_sorted[-1]:.0f}ms")


print("=== Trigger -> API done (full burst processed) ===")
stats("  No prey   (all 10 frames sent)  ", no_prey_total)
stats("  Prey      (early-exit on first) ", prey_total)

print(f"\n=== Trigger -> first POSITIVE verdict (prey bursts only) ===")
prey_ms_only = [t for _, t, _ in prey_first_pos]
stats("  All prey bursts                 ", prey_ms_only)

print(f"\n=== Per-prey-burst details ===")
print(f"{'Burst':<32} {'first-pos ms':>13} {'frame-pos':>10}")
for name, ms, sent in sorted(prey_first_pos, key=lambda x: x[1]):
    print(f"{name:<32} {ms:>13.0f} {sent:>10}")
