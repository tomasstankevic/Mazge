# Deploy runbook — combined_v1 multitask model (for the server-side agent)

You are the agent on the **server Mac** (`mazge` / `mazge.local`), which runs the
Mazge inference server 24/7 under launchd. A new model was trained + validated on
the training laptop and pushed to git. Your job: **deploy it and verify it**, or
roll back if health fails.

Read [../../doc/retraining_2026-08_combined_model.md](../../doc/retraining_2026-08_combined_model.md)
for the full benchmark + rationale. Summary of what you're shipping:

- One EfficientNet-B0 with 4 heads (prey + cat-id + subject + direction),
  exported to ONNX. **Drop-in**: same inputs as prey_v3, the server reads only
  outputs 0 (prey) and 1 (cat), so **no `server/` code change is needed**.
- Validated (5-fold CV): prey recall 1.000 (**0 false negatives**), cat-id 96.1%.
- Ships with a retuned per-frame prey threshold **0.76** (was 0.50) via env var.

## Preconditions (a human does these — verify before running)

1. `cd ~/Mazge && git pull` — brings the code (env-overridable thresholds) + this
   runbook. Confirm HEAD includes commit `d4c4b99` or later.
2. The model ONNX has been AirDropped to the server (weights are gitignored, so
   `git pull` does NOT bring them):
   ```
   ~/Mazge/_bench_weights/multitask_combined_v1_224.onnx
   ```
   If only `best.pt` arrived, export it first:
   ```bash
   cd ~/Mazge
   uv run python tools/export_multitask_onnx.py \
     --ckpt models/multitask/combined_v1/best.pt \
     --out  _bench_weights/multitask_combined_v1_224.onnx
   ```
   Sanity: the ONNX must have 4 outputs (`prey_logit, cat_logits, subject_logits,
   direction_logits`) and inputs `image` [1,3,224,224] + `frame_idx` [1].

## Deploy

```bash
cd ~/Mazge
pwsh ops/deploy/deploy_combined_v1.ps1            # add -RunTests to also run pytest
```

The script (idempotent, backs up `~/.config/mazge/server.env` first):
1. sets `MAZGE_PREY_ONNX` to the new ONNX and `MAZGE_PREY_DETECT=0.76`,
2. `launchctl kickstart -k gui/$(id -u)/com.mazge.server`,
3. polls `http://mazge.local:8080/healthz` (expects `{"ok":true,"backend":"onnx"}`).

If the health check fails it tells you to roll back and exits non-zero.

## Verify (do NOT consider it done until these pass)

1. Health: `curl -s http://mazge.local:8080/healthz` → `ok=true`, `backend=onnx`.
2. Tests: `uv run python -m pytest server/tests/ -q` → 18 passed.
3. **Behavioural** — the reason for this change: watch the next few real bursts in
   `~/Mazge/logs/server/server.jsonl` (or the HA dashboard). A genuine cat should
   now get `cat_recognized=true` / `door_action=allow` (the old model rejected
   borderline cats at 0.64–0.70). Prey should still lock the door.
4. Expect a modestly higher prey false-positive rate than before is NOT the case —
   it is no worse; but if you see a real cat repeatedly prey-locked, note the
   burst id for the `_fp_review.txt` follow-up.

## Rollback

```bash
pwsh ops/deploy/deploy_combined_v1.ps1 -Rollback
```
Reverts `MAZGE_PREY_ONNX` to `prey_v3_bodyA_s480_224.onnx` and removes
`MAZGE_PREY_DETECT` (threshold back to 0.50), then restarts. Backups of
`server.env` are kept as `server.env.bak.<timestamp>`.

## Notes

- **Firmware (`src/main.cpp`) needs no change** — the v2 wire contract is
  unchanged and the tiered lockout (1 frame → 3 min, ≥2 frames → 15 min) still
  works off the server's per-frame `detected`.
- The subject/direction ONNX outputs are currently latent (server ignores them).
  Wiring them into `server/decisions.py` (e.g. a real no-cat gate) is optional
  future work, documented in the retraining doc.
