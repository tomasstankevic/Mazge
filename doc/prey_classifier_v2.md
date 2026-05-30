# Prey Classifier v2 — Design, Experiments, Open Questions

> Living document covering the rebuild of the on-device prey classifier
> after `prey_v1` failed to converge. Last update: 2026-05-31.

## TL;DR

- `prey_v1` (MobileNetV3-Small on whole frames) **overfit catastrophically**
  — val recall locked at 0.500 for 50 epochs while train loss collapsed
  to 0.05. Root cause: prey is ~30×30 px in a 384×384 frame and only
  ~16–28 unique prey **events** were treated as ~280 independent
  per-frame samples.
- The original [`niciBume/Cat_Prey_Analyzer`](https://github.com/niciBume/Cat_Prey_Analyzer)
  sidestepped the data scarcity problem with a **cascade**: cat
  detector → face detector → snout crop → VGG16 binary classifier.
  We adopted the same architectural pattern.
- We benchmarked **8 cat detectors** with 3 preprocessing variants
  (raw, rotated CCW, rotated + door-occlusion crop). The clear winner
  for our 640×480 ESP32-CAM data was **Ultralytics YOLO11x on rotated
  + door-cropped 480×480 frames**: 100 % per-burst recall, 2 % false
  positives on empty frames, 104 ms/frame on a Mac M4 CPU.
- We considered switching to MegaDetector V6 (specifically trained on
  camera-trap imagery) but it gave very poor results on rotated frames
  (95 % → 80 % per-frame recall, 0 % → 48 % FP on empty bursts).
  MegaDetector wins on RAW frames, but those keep the cat sideways
  which breaks the downstream snout heuristic.
- We **rejected the firmware-rotation idea** for the on-device nano
  YOLO: it tanks recall from 32 % to 17 % per-frame because the model
  was trained on the stretched 640×384 layout. Rotation only happens
  in the offline crop pipeline.
- We generated **224×224 body and snout crops** for 595 labeled bursts
  × 5 911 frames using `tools/build_crops.py`. Body bboxes look
  excellent. Snout heuristic ("top 45 % of body bbox") is reasonable
  for frontal entry but degrades for sideways / fisheye-distorted cats.
- Open question: **how to detect a cat snout reliably on body crops?**
  Options under evaluation: Haar cat-face cascade, animal pose
  estimation (ViTPose-Animal / DeepLabCut SuperAnimal-Quadruped),
  DeepFaune detector.

## What we already had

- **Labeled dataset** (per `mazge_dataset.md`):
  - 970 burst events, of which 654 have JPGs and 595 have full human
    labels (`human_subject`, `human_prey`, `human_direction`, `cat_id`).
  - Train / val / test 80 / 10 / 10 split by `SHA1(burst_id)`.
  - 28 prey-positive bursts, ~210 prey-positive frames (very rare class).
- **Cloud API** baseline at `https://prey-detection.florian-mutel.workers.dev/`.
  Per `mazge_reliability.md` we know it achieves ~85 % TPR @ ~5 % FPR
  on our domain. Our v2 target is to match or beat this with a model
  we can run on a cheap cloud server (no per-call vendor cost).
- **Failed `prey_v1`** under `models/prey_v1/`. See `train_log.csv` —
  val_loss climbs from 0.92 → 4.3 while train_loss falls 2.4 → 0.05.

## Why v1 failed (with receipts)

1. **Whole-frame classification.** Input was 384×384 = 147 456 pixels;
   prey occupied ~900 pixels (0.6 %). The CNN had to find a needle in
   a haystack at every iteration.
2. **Per-frame sampler on burst-level events.** A
   `WeightedRandomSampler(weights=…, oversample_prey=8)` treated each
   of 280 prey frames as independent, but they came from only ~16 cat
   events. The network memorised those 16 scenes.
3. **Triple-stacked positive weighting.** Sampler × 8 + `pos_weight ≈
   26` + `weak_confidence` weights → effective positive emphasis
   ≈ 200×. Train loss collapsed; val loss exploded.
4. **Aggressive augmentation hid the prey.** `RandomErasing(p=0.3,
   scale up to 0.15)` could blank out the entire mouse patch.
   `ColorJitter(0.4)` washed out cues. `RandomAffine ±10°` distorted
   the only signal.
5. **Pretrained 3-ch RGB weights on grayscale full frames.** ImageNet
   features look for natural-image textures; our IR-night greyscale
   frames look nothing like ImageNet.
6. **Auxiliary heads competed for capacity** with the main task on a
   ~16-event problem.
7. **Eval was per-frame, not per-burst.** Product behaviour aggregates
   10 frames into one decision; per-frame F1 with 20 val positives has
   a resolution of 0.05 in recall and we were inside the noise band
   for 50 epochs.

## The cascade design (after niciBume)

```
ESP32-CAM 640×480 RGB JPG
        │
        ▼
[ optional ] rotate CCW + crop top 160 px (server-side only)
        │
        ▼
Stage 1: Cat-body detector  ──►  bbox or skip
        │
        ▼
Stage 2: Snout localiser   ──►  refined head box or fallback to body
        │
        ▼
Stage 3: Prey classifier   ──►  P(prey)
        │
        ▼
Burst aggregator (max / top-3 mean / cumuli) ─►  decision @ τ
```

This decouples the data-hungry **localisation** task (where lots of
public cat data exists) from the data-poor **prey** task (where we
have only 28 events).

## Stage 1 — Cat-body detector benchmark

`tools/test_yolo_variants.py` evaluates 8 detector variants on the
same stratified sample of 82 bursts / 818 frames pulled from
`models/subject_detection_eval/summary.csv`.

### Variants

| name | weights | input | notes |
|---|---|---|---|
| `nano_ncnn` | shipped firmware NCNN | 640×384 stretched | what the ESP32 currently runs |
| `yolo11n_640` | Ultralytics YOLO11n | 640×640 letterbox | same nano, proper resize |
| `yolo11s_640` | YOLO11s | 640×640 | sleeper hit when rotated |
| `yolo11m_640` | YOLO11m | 640×640 | sweet spot |
| `yolo11x_640` | YOLO11x | 640×640 | **chosen for v2 crops** |
| `yolo11x_960` | YOLO11x | 960×960 | best raw recall, slow |
| `mdv6_yolov9c` | MegaDetector V6 yolov9-c | 1280×1280 | camera-trap pretrain, slow |
| `mdv6_yolov10c` | MegaDetector V6 yolov10-c | 1280×1280 | best on raw frames |

### Preprocessing benchmark (one full run per pre-processing)

#### Raw 640×480 (what we tested first)

| variant | ms/frame | cat_frame% | cat_burst% | empty FP% |
|---|--:|--:|--:|--:|
| nano_ncnn | 8 | 32 % | 76 % | 0 % |
| yolo11x_640 | 100 | 61 % | 97 % | 5 % |
| **mdv6_yolov10c** | 158 | **95 %** | **100 %** | **0 %** |

#### Rotated CCW 480×640

| variant | ms/frame | cat_frame% | cat_burst% | empty FP% |
|---|--:|--:|--:|--:|
| **nano_ncnn** | 8 | **17 %** ↓ | 47 % ↓ | 0 % |
| yolo11s_640 | 20 | 53 % ↑↑ | 93 % ↑↑ | 0 % |
| yolo11x_640 | 104 | 78 % ↑ | 100 % | 15 % |
| mdv6_yolov10c | 158 | 80 % ↓ | 100 % | **48 %** ↑↑ |

#### Rotated CCW + top-160 cropped 480×480 (door occlusion removed)

| variant | ms/frame | cat_frame% | cat_burst% | empty FP% |
|---|--:|--:|--:|--:|
| **yolo11x_640** | 134 | 73 % | **100 %** | **2 %** |
| yolo11s_640 | 23 | 61 % | 96 % | 0 % |
| mdv6_yolov10c | 157 | 80 % | 96 % | **65 %** ↑↑↑ |

### Key insights

1. **MegaDetector dies on rotated/cropped frames.** Vertical
   architectural lines (cat-flap rails) look like animal silhouettes
   when oriented vertically — empty-frame FPs go 0 → 48 → 65 %.
   MegaDetector was trained on naturally-oriented landscape camera-trap
   images.
2. **Nano NCNN is sensitive to the exact aspect ratio it was trained
   on.** Stretching 640×480 → 640×384 (what the firmware does today)
   keeps 32 % per-frame recall. Letterboxing or rotation kills it.
3. **YOLO11x_640 on rotated+cropped frames is the practical winner.**
   100 % per-burst recall, 2 % FP, 104 ms/frame. Upright crops feed
   directly into the snout heuristic.
4. **Rotation should happen in the offline pipeline, not in firmware.**
   On-device we keep nano_ncnn on raw frames for the trigger filter;
   server runs YOLO11x on rotated frames for the high-quality crops.

### Why `bursts.csv:human_direction` matters

Burst-level survey:

```
cat bursts (with JPGs):       455
  by direction:  entering 257  exiting 142  unclear 56
  by prey:       0:422  1:27  unclear:6
```

- 142 cat **exit** bursts are guaranteed no-prey (a cat physically
  can't carry prey out through the catflap going forwards).
- This adds 142 free hard negatives where the cat is in the unusual
  exiting pose — useful for teaching the classifier to ignore butt
  views.
- For exits we also flip the snout heuristic (snout at bottom of
  bbox).

## Stage 2 — Snout localisation

The original niciBume project ran a Haar cat-face detector on the
body crop, then **extended the face box 40 % downward** to capture
chin + mouth + paws (where prey is held).

### What `tools/build_crops.py` currently does

Heuristic: top 45 % of the body bbox (along the entering axis),
extended 10 % past the bbox edge, full body width.

```python
SNOUT_HEAD_FRAC = 0.45
SNOUT_EXTEND_FRAC = 0.10
# entering: snout = top portion of bbox
# exiting:  snout = bottom portion of bbox
```

### Where this heuristic fails

The cat is shaped like a vertical column when it's standing on its
hind legs to enter the cat-flap, so "top of bbox = head" works there.
But for:

- **Sideways approaches** the head is on the left or right, not the
  top, and our "snout box" is full-width so it does capture the head
  but with a lot of irrelevant body around it.
- **Fisheye distortion** at the very front of the lens warps the
  proportions, sometimes making the head the same height as the body.
- **Cat lying down** (in exit bursts especially) — the head is at one
  end of a long horizontal bbox.

### Open question: better snout localiser

Three options to compare next:

| option | how | speed (CPU) | reliability expected |
|---|---|--:|---|
| **B. Haar cat-face inside body crop** | `cv2.CascadeClassifier('haarcascade_frontalcatface_extended.xml')` on the upright body crop | < 5 ms | low–medium — Haar needs a frontal cat face, fails on sideways or distorted views |
| **C. Animal pose** | ViTPose-Animal / DeepLabCut SuperAnimal-Quadruped; 17 keypoints incl. nose + eyes + ears | ~100 ms | high — pose models handle arbitrary orientation and report per-keypoint confidence |
| **D. DeepFaune** | Already in `PytorchWildlife`; outputs body + head boxes natively | ~50 ms | medium-high — European wildlife focused, ought to know cats |

All three operate on the **YOLO11x body crop** (Stage 1 output)
not on the raw frame, so they get a clean centred input.

Also worth a baseline: **Stage 2 = identity** (use body crop
directly as classifier input, no snout). Easy to test, gives a
floor performance number.

## Stage 3 — Prey classifier (not yet built)

Plan (frozen after benchmark comparison of Stage 2 options):

- **Backbone**: EfficientNet-B0, ImageNet pretrained. ~5 M params,
  ~6 ms / frame on M4 CPU.
- **Input**: 224×224 RGB (so we keep colour cues — a vole is brown
  on a tabby cat).
- **Loss**: `BCEWithLogitsLoss(pos_weight = N_neg / N_pos)`. One
  imbalance lever, not three.
- **Sampler**: burst-level. Sample a burst, then a random frame from
  it. Removes the v1 "10× repetition treated as independent" bug.
- **Augmentations**: HFlip + small affine (±5°, scale 0.95–1.05).
  No RandomErasing, no MixUp, no ColorJitter (those hide the prey).
- **Schedule**: freeze backbone 5 epochs (lr 1e-3) → unfreeze 20
  epochs (lr 1e-4 cosine). Early stop on val loss.
- **Eval**: burst-level. Aggregate per-frame `P(prey)` over f5–f9
  via `max`, top-3 mean, and cumuli; pick whichever maximises
  precision at 90 % recall on val; freeze the threshold; **then**
  report on test.

## Code & artefacts created

- `tools/test_subject_detection.py` — first benchmark, YOLO nano +
  Haar on a stratified 82-burst sample.
- `tools/test_yolo_variants.py` — multi-variant detector bench with
  `--rotate` and `--crop-top` preprocessing flags. Outputs
  `comparison{,_rot,_rot_crop160}.txt` in
  `models/subject_detection_eval/variants/`.
- `tools/build_crops.py` — generates 224×224 body + snout crops for
  every labeled frame, two pipelines (`mdv6_raw`,
  `yolo11x_rotcrop`), live HTML dashboard. Includes proper
  human-label filtering (`--label-mode=human`) and direction-aware
  snout cropping. Exits get `effective_prey=0` automatically.
- `dataset/crops_yolo11x_rotcrop/`:
  - `<burst>/f{NN}_body.jpg` — 224×224 body crop, label per `_index.csv`
  - `<burst>/f{NN}_snout.jpg` — 224×224 snout crop
  - `<burst>/bboxes.json` — full per-frame bbox metadata
  - `<burst>/preview.jpg` — highest-conf frame with body+snout overlay
  - `_index.csv` — one row per frame; has `effective_prey`,
    `label_source`, `cat_conf`, full bbox
  - `_dashboard.html` — live progress + prey-burst gallery (auto-
    refresh every 15 s)

## Bench numbers in one place

Run on `82 bursts / 818 frames` (Mac M4 CPU, single thread):

```
                              raw            rot CCW        rot+crop160
                              ─────────      ─────────      ─────────
                              frame burst    frame burst    frame burst    ms
nano_ncnn 384×640              32 %  76 %    17 %  47 %    29 %  67 %      8
yolo11n_640                    28 %  69 %    29 %  65 %     -    -        24
yolo11s_640                    35 %  79 %    53 %  93 %    61 %  96 %     20
yolo11m_640                    63 %  97 %    66 %  97 %     -    -        42
yolo11x_640                    61 %  97 %    78 % 100 %    73 % 100 %    104
yolo11x_960                    73 % 100 %    80 % 100 %     -    -       267
mdv6_yolov9c (1280)            49 %  93 %    17 %  58 %     -    -       760
mdv6_yolov10c (1280)           95 % 100 %    80 % 100 %    80 %  96 %    158

empty-burst FPs:
nano_ncnn                       0 %            0 %            0 %
yolo11s_640                     0 %            0 %            0 %
yolo11x_640                     5 %           15 %            2 %
mdv6_yolov10c (1280)            0 %           48 %           65 %
```

## What's next (in execution order)

1. **Bench Stage-2 snout localisers** on the body crops we just
   generated. New tool `tools/test_snout_localisers.py` that runs
   Haar + DeepFaune + (optionally) animal-pose on every prey-burst
   body crop, scores them by "did it land on a snout?" (human spot
   check) and reports inference time. Output: contact-sheet HTML.
2. **Build the v2 training pipeline** with a `--input` flag for
   {body, snout-by-heuristic, snout-by-haar, snout-by-pose} so we
   can train four variants on the same code.
3. **First training run** on body crops alone. Establishes baseline.
4. **Compare to snout variants.** Whichever is best becomes the
   default Stage 2.
5. **Threshold calibration + burst aggregation** on val split.
6. **Error gallery** of test-set FN/FP for hard-negative mining if
   needed.
7. **Production wrapper** + ONNX export.

## Decisions log

- **2026-05-30** Picked **YOLO11x on rot+crop160** as the v2 crop
  generator, not MegaDetector, because MD's empty-frame FPs make it
  unusable for offline crop generation on rotated frames.
- **2026-05-30** Confirmed firmware rotation **should NOT** happen
  on-device: tanks nano recall from 32 % → 17 % per-frame.
- **2026-05-30** Adopted niciBume's cascade (body → snout →
  classifier) as the v2 architecture. Implemented Stage 1; Stage 2
  is the open question being benched next.
- **2026-05-30** Decided to include cat **exit** bursts in training
  with `effective_prey=0` override (cat can't carry prey out).
  Frees 142 hard negatives.
- **2026-05-30** Dropped sub-snout heuristic of "narrow snout
  horizontally" because the cat is column-shaped at the cat-flap
  entry; full-width preserves paws/prey context.
