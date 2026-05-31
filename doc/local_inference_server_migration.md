# local inference server migration (cloud -> mac)

last updated: 2026-05-31

## goal

Replace the external cloud prey API with a local HTTP service running on another Mac, while keeping firmware behavior unchanged.

## current repository state

The repo now includes full model and evaluation artifacts needed for server-side inference:

- `models/cat_id_v2/bodyA/`
  - checkpoints: `best_val_acc.pt`, `best_val_burst_acc.pt`, `last.pt`
  - metadata: `split_assignment.json`, `metrics_live.json`, `test_metrics.json`, `test_predictions.csv`, `train_log.csv`
- `models/prey_v3/bodyA/`
  - checkpoints: `best_loss.pt`, `best_burst_f1.pt`, `last.pt`
  - metadata: `split_assignment.json`, `metrics_live.json`, `test_metrics.json`, `test_predictions.csv`, `train_log.csv`
  - sweep outputs: `metric_sweep/summary.json`, `metric_sweep/combined_sweep.csv`, `metric_sweep/baseline_sweep.csv`, `metric_sweep/top10.csv`
- `models/snout_loc_eval/`
  - `dashboard.html`, `per_image.csv`, `summary.csv`

## what was completed in this cycle

1. crop/index pipeline hardening
- dedupe-safe crop indexing and dedupe utility for `dataset/crops_yolo11x_rotcrop/_index.csv`.

2. cat identity model
- trained `cat_id_v2` body-crop classifier and saved reproducible train/test artifacts.

3. multitask prey model (v3)
- implemented and trained a shared-backbone model that predicts:
  - prey probability
  - cat identity logits/probabilities
- fixed label filtering for unclear labels.

4. burst decision optimization
- implemented combined metric sweep using:
  - prey score
  - cat-id contribution
  - prey-frame-count term (not frame index)
- added weighted objective support (FN and FP weighting), including FN-heavy tuning.

5. artifact versioning
- committed the full trained model/eval artifact trees listed above.

## firmware compatibility contract (do not break)

Firmware currently posts one frame per request as JSON:

```json
{"image_base64":"<base64_jpeg>"}
```

Firmware expects response JSON containing a boolean-like prey verdict key:

```json
{"detected": true}
```

Compatibility rules:

1. keep endpoint behavior equivalent to existing `PREY_API_URL` service.
2. keep response key `detected` so current parser in firmware works unchanged.
3. keep per-request latency low enough for multi-frame burst checking.

## recommended local server design

Use a small Python service (FastAPI or Flask) that:

1. accepts `image_base64`.
2. decodes JPEG.
3. runs stage pipeline:
- body detection (same preprocessing used in v3 experiments)
- prey_v3 forward pass (and optional cat_id_v2 cross-check)
- frame-level prey decision
4. returns `{"detected": <bool>, "score": <float>, "cat_id": <string optional>}`.

Important: you can add extra fields, but must keep `detected`.

## migration plan to another mac

1. clone repository and checkout latest main.
2. install Python + dependencies (`uv sync`).
3. verify checkpoints exist under `models/cat_id_v2/bodyA` and `models/prey_v3/bodyA`.
4. run a local server exposing cloud-compatible endpoint.
5. smoke test with one known prey frame and one known non-prey frame.
6. update firmware `PREY_API_URL` to local server URL.
7. run live burst test and verify lockout logic still behaves correctly.

## acceptance checklist

- [ ] local endpoint returns valid `{"detected": ...}` for frame POSTs
- [ ] prey-positive bursts still trigger lockout
- [ ] non-prey bursts do not increase false lockouts materially
- [ ] latency acceptable for burst processing
- [ ] server restart/reconnect behavior handled cleanly

## notes for next implementation step

When implementing the server, preserve the existing cloud contract first, then iterate internally on model logic. This minimizes firmware-side risk during migration.
