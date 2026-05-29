---
name: mazge-update-cycle
description: "Run the recurring Mazge update cycle safely: list new board folders first, gently sync recent meta and JPGs, refresh prey reports/statistics/dashboard files, then launch prey/no-prey/direction labeling and Benis/Mazge review without crashing the board. Use when updating every few days, pulling recent captures, regenerating reports, or starting the labeling workflow."
---

# Mazge Update Cycle

Use this workflow when you want the regular every-few-days maintenance pass in Mazge.

## Safe command

Run from the Mazge repo root:

```bash
cd /Users/ruta/Tomas/repos/Mazge
uv run python tools/update_cycle.py --hours 72
```

For a local-only refresh that does not touch the board:

```bash
cd /Users/ruta/Tomas/repos/Mazge
uv run python tools/update_cycle.py --skip-sync --hours 72
```

For a reports-only refresh with no interactive labeling:

```bash
cd /Users/ruta/Tomas/repos/Mazge
uv run python tools/update_cycle.py --skip-labeling --hours 72
```

## What the script does

In order, it:

1. Calls `/sdfolders` and lists recent missing folders before downloading anything.
2. Pulls only `meta.json` for those missing recent folders, sequentially with retries and sleeps.
3. Pulls only missing JPGs for recent folders, sequentially with small delays.
4. Rebuilds dataset files.
5. Refreshes prey review HTML/PDF.
6. Refreshes cat activity HTML/text reports.
7. Refreshes human label and training statistics text outputs.
8. Runs the prey-positive labeling server.
9. Runs the unlabelled labeling server for no-prey / entry-exit / subject cleanup.
10. Generates fresh cat-ID review input and serves the Benis/Mazge review page.

## Why this is board-safe

Do not use the old broad sync/report commands for routine updates when the board is live.

This workflow avoids crashing the board because it:

- lists folder names first instead of immediately walking `/sdlist`
- syncs only recent missing folders
- fetches `meta.json` before JPGs
- fetches JPGs sequentially with delays instead of aggressive parallel pulls
- keeps report generation local after sync

## Important operational rules

- Run only one board-touching workflow at a time.
- Do not run OTA, `sync_all_sd.py`, `refresh_prey_review.py` without `--skip-sync`, or ad-hoc SD pull loops at the same time.
- If `/sdfolders` or `/sdget` starts timing out, reboot the board once and rerun the updater.
- During the labeling phases, stop the server with `Ctrl+C` once you are done with that phase so the script can continue.

## Labeling phases

The script opens these phases in sequence:

1. `prey-positive`
2. `unlabelled`
3. cat-ID review for `mazge` / `benis`

The cat-ID phase serves:

```text
http://127.0.0.1:8766/tools/cat_id_review.html?data=tools/catid_pending.json
```

That page exports `catid_labels.jsonl`.
Append that export to `dataset/labels.jsonl`, then rerun:

```bash
cd /Users/ruta/Tomas/repos/Mazge
uv run python tools/update_cycle.py --skip-sync --skip-labeling --hours 72
```

## Output files refreshed by the script

- `captures/prey_review/index.html`
- `captures/prey_review/report.pdf`
- `captures/cat_activity_report.html`
- `captures/cat_activity_report.txt`
- `captures/human_label_summary.txt`
- `captures/training_stats.txt`
- `captures/false_negative_last48h.txt` when available
- `captures/non_prey_last48h_suspects.txt` when available

## Recommended routine

Every few days:

```bash
cd /Users/ruta/Tomas/repos/Mazge
uv run python tools/update_cycle.py --hours 72
```

If the board was already synced recently and you only want fresh reports:

```bash
cd /Users/ruta/Tomas/repos/Mazge
uv run python tools/update_cycle.py --skip-sync --skip-labeling --hours 72
```
