# VOXL2 Inference Acceleration — Findings

Working notes for moving beacon detection off CPU on the ModalAI Starling 2 Max
(QRB5165). Source material is in `references/`. The upstream `voxl-tflite-server/`
and `voxl-docker/` checkouts were dropped once the GPU-container direction was
chosen; they remain in commit `f959d7a`. The unbuilt server patch described below
was never committed: it is saved as `references/voxl-tflite-server_config.patch`
(apply from the repo root on top of `f959d7a`).

Provenance is marked throughout: **[measured]** = observed on hardware in this
session, **[source]** = read from the checked-in upstream code, **[reported]** =
stated by someone, not independently verified.

---

## Bench hardware

Bench drone (no props, bench PSU) at `root@192.168.50.104`. **Not** the drone
prior work was done on — camera pipe names differ (`hires_down_*` here vs
`hires_front_*` in `configs/modal_config.json`), so treat per-drone config as
unverified against the real target.

| | |
|---|---|
| system-image | `1.8.06-M0054-14.1a-perf`, kernel 4.19.125 |
| voxl-suite | `1.6.4~beta3` (repo `dists/qrb5165/sdk-1.6`) |
| voxl-tflite-server | **0.5.1** — matches `references/voxl_tflite_server.pdf` and the checkout here |
| qrb5165-tflite | 2.17.2 |
| voxl-mpa-to-ros2 | 0.0.7, active |
| host Python | **3.6.9**, numpy 1.13.3, **no cv2, no tflite_runtime** |
| ROS on host | Foxy + Melodic (no Humble) |
| container | `voxl2-humble`, Python 3.10.12, no tflite_runtime |

**The host Python cannot run our detector.** ROS2 Humble + cv2 + modern numpy
don't exist there and py36/aarch64 wheels are unobtainable. The application is
already containerised (`configs/modal_config.json` references `/workspace/...`).
So the live question is never "container or bare metal" — it is only **where
inference runs**, host-side or in-process.

---

## Model architecture [source]

Both detectors are **Ultralytics YOLOv8n**, read from the model config pickled
inside each checkpoint (`data.pkl`):

| | `best_beacon_square3.pt` (stage 1) | `beacon_square_top93b.pt` (stage 2 crop) |
|---|---|---|
| blocks | `C2f` + `SPPF` | `C2f` + `SPPF` |
| depth / width multiple | 0.33 / 0.25 | 0.33 / 0.25 |
| nc | 4 | 1 |
| imgsz | 640 | 640 |
| ultralytics | 8.4.30 | 8.4.45 |

- **v8, not v11:** `C2f` is the YOLOv8 block; YOLO11 uses `C3k2` / `C2PSA`, which
  appear in neither checkpoint.
- **n scale:** 0.33 / 0.25 is YOLOv8n's `[depth, width]`. YOLO11n would be 0.50 / 0.25.
- `8.4.x` is the **ultralytics package** version, not the architecture.
- `models/best_square3.tflite` has **no ultralytics metadata**, so it can't be tied
  to a specific `.pt` from the file alone. Its `[1, 8, 8400]` output (4 box + 4
  classes) and ~3.35 MB size are consistent with a 4-class YOLOv8n export.

Server config: `"model_architecture": "YOLOV8"`.

---

## Benchmarks [measured]

`/usr/bin/benchmark_model_mai` (upstream TFLite `benchmark_model`), 30 runs,
3 warmup. **Times are `Invoke()` only** — no preprocessing, NMS, postprocessing,
capture or transport. Bursts on a propless bench with no airflow; sustained
thermal behaviour is untested.

Both models are `[1,640,640,3]` float32 input.

| Config | `models/best_square3.tflite` | shipped `yolov8n_float16` |
|---|---|---|
| CPU 1 thread | — | 728.7 ms |
| CPU 4 threads | **198.9 ms** | 362.3 ms |
| CPU 8 threads | — | 714.7 ms |
| XNNPACK 8 threads | — | 705.1 ms |
| GPU, default | — | 44.8 ms |
| **GPU, sustained-speed** | **36.0–36.9 ms** | 35.7 ms |
| GPU, fp32 compute | 53.9 ms | 55.1 ms |

Defensible claim: **GPU is 5.4× the best CPU config**, same tool/model/board.

- GPU reports `Created 1 GPU delegate kernels` + *"graph will be completely
  executed by the delegate"* — full op coverage, zero partitioning, for both models.
- **8 threads is worse than 4** (QRB5165 is 4×A77 + 4×A55; threads 5-8 hit little
  cores and the join waits on the slowest). Note `voxl-tflite-server` hardcodes
  `num_threads = 8` for XNNPACK (`model_helper.cpp:324`) — its CPU fallback runs
  in the worst configuration.
- **fp16 is already on by default** and already worth ~1.5×. Both the benchmark
  default (`gpu_precision_loss_allowed=true`) and the server
  (`SetAllowFp16PrecisionForFp32(true)`, `model_helper.cpp:180`) enable it.
  There is no unclaimed fp16 win.
- **Weight precision is performance-neutral.** Our int8-weighted 4-class model
  (59 `kTfLiteInt8` tensors, 3.35 MB) and the shipped fp16-weighted 80-class
  model (131 `kTfLiteFloat16`, 6.48 MB) both land at ~36 ms. The Adreno 650 is
  compute-bound here, not weight-bandwidth-bound. **int16 has no GPU path** —
  TFLite's 16x8 mode isn't implemented in the GPU delegate.
- **Input resolution is the only real lever**, and it needs a **re-export** —
  input shape is not a runtime knob. Ultralytics bakes in grid dims; resizing the
  input tensor fails at node 107 (`CONCATENATION`, 40 = 640/16) and drops the
  delegate entirely. Expect ~quadratic scaling, traded against small-object recall.
- **GPU init is ~6.2 s** (OpenCL kernel compilation), every process start.
  `--delegate_serialize_dir` / `--delegate_serialize_token` can cache it.

For comparison, the deployed Ultralytics `.pt` CPU path was **[reported]** as
"over a second" (Sam, from memory, "don't remember exact numbers"). Scope and
hardware unknown, and `.track()` includes Kalman + association work our benchmark
doesn't. **Not comparable to the table above** — time the real path before
quoting any speedup against it.

---

## The NPU is not reachable the way the reference PDF assumes

`references/voxl_npu_convo.pdf` speculates about a Hexagon delegate or QNN
(`libQnnTFLiteDelegate.so`). Neither applies here:

- ModalAI docs state plainly: *"There is no Hexagon delegate; nnapi picks its
  accelerator automatically."*
- The real path is NNAPI with `accelerator_name = "libunifiedhal-driver.so2"`
  **[source]** `model_helper.cpp:343-358`. `/usr/lib/libneuralnetworks.so` and
  `/usr/lib/nn/libunifiedhal-driver.so` exist **[measured]**.
- `tflite_hexagon_detector.py`'s `delegate_path` → `libhexagon_delegate.so` will
  never load. It is caught and falls back to CPU **silently**. Anything run under
  `backend: "tflite_hexagon"` so far has been CPU.
- Python cannot reach NNAPI at all — `tflite_runtime` has no way to set
  `accelerator_name` or build a `StatefulNnApiDelegate`.
- `benchmark_model_mai` has **no NNAPI support compiled in** (no `--use_nnapi`,
  no NNAPI strings in the binary), so the NPU could not be benchmarked here.

### voxl-tflite-server's quantised path is broken for YOLOv8 [source]

Not globally broken — coherent for the uint8 MobileNet classifiers it ships
(`mobilenetv1_nnapi_*.tflite`), which is what it was built for. Broken for the
architecture we need. **There is no dequantisation code anywhere in the repo**
(`grep zero_point|quantization.scale|params.scale` → nothing).

| Helper | Output read | Quantised model |
|---|---|---|
| `generic_classification:41` | `TensorData<uint8_t>` | OK — raw uint8 compared to a threshold, monotonic |
| `generic_object_detection:56-61` | `TensorData<float>` ×4 | Fails type check → `nullptr` |
| `yolov8:70` | `typed_tensor<float>` | Fails **silently** — int8 read as float32, 4× buffer overread |

Plus the generic input fill (`model_helper.cpp:400-429`) copies raw `[0,255]`
bytes into int8/uint8 tensors with no scale/zero-point — correct only when input
quantisation is (scale=1/255, zp=0). Ultralytics int8 exports use zp=−128.

**This cannot be fixed in Python under Path A.** The server publishes exactly two
pipes **[source]** `main.cpp:160-215` — `<prefix>_tflite`
(`camera_image_metadata_t`, 16 MB) and `<prefix>_tflite_data` (`ai_detection_t`,
**16 KB**). No tensor crosses the boundary; a `[1,84,8400]` fp32 tensor is 2.8 MB
and could not fit regardless. `worker()` calls `postprocess()` **first**
(`yolov8_model_helper.cpp:22-36`), so decode, NMS and pixel-box conversion have
all happened before a byte is written. Python is strictly downstream.

---

## Paths

**A — host `voxl-tflite-server` + container consumes `/run/mpa`.**
Config only, no code. Our model is already float32 with the exact `[1, 8, 8400]`
output the server's YOLOv8 decoder expects (`rows=data[2]`, `dims=data[1]`, then
transpose — `yolov8_model_helper.cpp:62-76`), and boxes come out in **original
camera pixel coordinates**, no letterboxing (`:106-110`). Available today.
Costs: thresholds hardcoded (YOLOv8 score 0.45 / conf 0.25 / NMS 0.5, rebuild to
change); and **frame association is unsolvable exactly** — `timestamp_ns` is
stamped at postprocess time, `frame_id` is the server's own counter, and
`worker()` even overwrites the overlay frame's timestamp (`:32`). Neither pipe
carries a capture timestamp.

**B′ — wrapper `.so` + interpreter in our own Python process.**
Removes the dequantisation objection, the hardcoded thresholds and the frame
association problem in one move. `tflite_hexagon_detector.py` already handles
input quantisation and output dequantisation correctly. Needs a delegate binary
that does not exist — see below.

**A″ — add a tensor pipe to the server.** Possible but strictly more work than
fixing dequantisation in C++ directly, plus ~84 MB/s of marshalling at 30 Hz, and
it doesn't fix the input-side bug.

### What blocks the OpenCL container path (`references/voxl_opencl_docker.pdf`)

That recipe is **step 1 of 3** and verifies with `clinfo` — which proves the
Adreno is reachable and says nothing about TFLite being able to use it.

1. **OpenCL in container — not done, mechanical.** Container's `libOpenCL.so.1`
   is `ocl-icd-libopencl1`, the generic ICD *loader*, with no
   `/etc/OpenCL/vendors/` — zero drivers registered, enumerates nothing. Real
   driver is host-side (`/usr/lib/libOpenCL.so` from `qti-adreno`). All five
   packages the recipe repacks are installed at the named versions;
   `/dev/kgsl-3d0` and `/dev/ion` exist. Snags: `dpkg-repack` not installed
   (needs network); **`/dev/dma_heap` does not exist** on this 4.19 kernel —
   `/dev/ion` is the allocator, as the conversation PDF correctly guessed.
2. **A loadable GPU delegate — THE BLOCKER.** `qrb5165-tflite` ships **only
   static archives, zero `.so`**; nothing resembling a GPU delegate exists on the
   filesystem. It must be **built**. It is buildable: `TfLiteGpuDelegateV2Create`
   *and* `StatefulNnApiDelegate` are both defined in
   `/usr/lib64/libtensorflow-lite.a` **[measured, `nm`]**, and the board has
   g++ 7.5 + `/usr/include/tensorflow`. ~60 lines exporting
   `tflite_plugin_create_delegate` — the same C API
   `--external_delegate_path` and Python's `load_delegate()` both use, so it is
   testable with `benchmark_model_mai` and serves the NPU branch equally.
3. **`tflite_runtime` in the container**, ABI-matched to 2.17.x.

None of this is required for the GPU speedup — Path A already has it natively.

---

## Bugs found

**1. Dropped classes — FIXED, verified.** `beacon_camera.py` hardcoded
`class_names=["beacon"]` for both backends, and both detectors do
`if cls_idx >= len(self.class_names): continue`. But
`models/best_beacon_square3.pt` (what `configs/modal_config.json` loads) has
**four** classes: `{0: unknown_beacon, 1: red_beacon, 2: green_beacon,
3: blue_beacon}`. So only `unknown_beacon` survived — every detection the model
successfully colour-classified was silently discarded. Confirmed unintended by
the model's author. Affected the **live ROS path only**; the video/batch paths
call `YOLO()` directly and use ultralytics' own `names`, so offline evaluation
looked correct while flight lost detections.
Fix: both backends now self-determine the class count — `.pt` reads
`model.names`, `.tflite` derives `nc` from its `[1, 4+nc, anchors]` output shape.
New optional `detection.class_names` config key overrides.

**2. No tracker off the Ultralytics path — FIXED.** All persistent IDs came from
ultralytics `.track(persist=True)` (BoT-SORT); there is **no tracking code in
this repo**. `TFLiteHexagonDetector` returned a constant `-1`, which collapses
every target onto one shared `_get_blink_detector(tracking_id)`
(`blink_detector.py:257-260`) and one `_tracker_colors[tid]` entry.
Fix: greedy IoU matcher in `_track()`. Matching **ignores class** (stage 1 flips
a beacon between unknown/red/green/blue; colour is decided downstream anyway).
`track_max_age` defaults to 30 frames because **a blinking beacon is undetectable
for its whole off-phase** (~14 frames at 1 Hz / 50% duty / 27 fps) — too short and
the beacon returns as a new id and its blink history restarts. Empty frames now
route through the tracker too, so stale tracks expire during dropouts.

**3. Normalised box decode — FIXED.** `_decode()` treated the model's box
outputs as pixels in the letterboxed 640 frame. Ultralytics TFLite exports emit
**normalised [0,1]** xywh — verified on-device: all four box channels max at
~1.0. Every box therefore collapsed to `(0,0,0,0)` after un-letterboxing, so the
TFLite backend had **never produced a valid detection**. Consistent with it never
having run on real hardware (the delegate never loaded, so it silently sat on
CPU). `voxl-tflite-server` does the equivalent conversion at
`yolov8_model_helper.cpp:106-110`, multiplying by the camera frame dims because
it stretch-resizes rather than letterboxing.

**4. BGR fed to an RGB model — FIXED.** `beacon_camera.py:294` uses cv_bridge
`desired_encoding='bgr8'` and `camera_interface.get_rgb()` documents BGR. The
`.pt` path survives this because ultralytics converts BGR->RGB internally; the
TFLite backend did **no** conversion, so the model saw R and B swapped.
`voxl-tflite-server` feeds RGB too (`CV_YUV2RGB_*`, `model_helper.cpp:237`),
confirming RGB is the expected input.

Measured on the demo frame, same model, same GPU:

| input | result | `unknown_beacon` peak |
|---|---|---|
| BGR (as fed today) | `blue_beacon` **0.4941** — below config `conf: 0.5`, **rejected** | 0.2161 |
| RGB (converted) | `blue_beacon` **0.6372** — **accepted** | 0.0452 |

+0.143 confidence (+29% relative) and ~5x less class confusion. On this frame it
is the difference between a detection and a miss. Fixed in `detect()` with an
`input_is_bgr=True` flag; verified bit-exact against a manual conversion.

**5. imgsz/model mismatch — HARDENED.** `imgsz` came from JSON config while the
model's input size is fixed at export. A mismatch throws in `set_tensor()`, and
since `_decode()` scales normalised boxes by `imgsz`, a silent disagreement would
misplace every box. `start()` now reads the real size from the input tensor and
overrides with a warning.

**Deliberate divergence (not a bug):** NMS here is class-**agnostic**, unlike
ultralytics' per-class default. One physical beacon can score on several of the
four colour classes at once; per-class NMS would emit a duplicate detection per
class, each then paying a full stage-2 pass. Colour is decided downstream by
`classify_beacon_color()` regardless.

**Recommendation:** set `detection.class_names` explicitly for the tflite
backend. A `.tflite` carries no names, so the reconciler can only generate
`class_0..class_3` placeholders — harmless (labels are unused downstream) but it
makes logs unreadable.

**6. Not a bug (retracted).** `staged_`/`burst_` use per-frame indices as
`tracking_id`, but only ever as log columns — `_analyse_burst` builds one local
`BlinkDetector` per burst. No persistent per-id state, nothing to corrupt.

---

## End-to-end check on a real frame

`seabird_dataset/beacon_debug/full_frames/nodet_20260910_t1789042048.50.png`,
run through the repo's own `_letterbox` / `_prepare_input` / `_decode` with
inference on the Adreno via `benchmark_model_mai`. Outputs in `pipeline_demo/`.

Result: **`blue_beacon`, conf 0.494, bbox (524,365,576,416), 52x51 px.**

- The frame is named `nodet_` because the old `class_names=["beacon"]` dropped
  class 3. This is direct evidence of bug 1 on real flight data.
- conf **0.494 vs the config's `conf: 0.5`** — even with the fix this frame is
  missed by a hair. Note `voxl-tflite-server`'s hardcoded YOLOv8 score threshold
  of 0.45 *would* accept it. Worth reviewing the configured threshold.
- The 52x51 crop is the stage-2 input. Step A would letterbox that up ~12x
  linearly toward 640 before inference.

## Stage 2 — where its cost is, and how to remove it

**Priority note:** GPU acceleration of stage 1 is the primary win and what moves
this into usable territory. Everything below is a follow-on optimisation — worth
recording now, not worth blocking the GPU work on.

`isolate_and_classify()` is two very different jobs fused into one call:

- **Step A — localise the lit top.** A second ultralytics `.pt` YOLO
  (`beacon_square_top93b.pt`, 1 class `beacon_top`) run on the stage-1 crop.
  No `imgsz` is passed, so ultralytics uses its 640 default and `scaleup=True`
  **enlarges** the crop toward 640 (a 52x51 crop is scaled ~12x linearly; with
  `auto=True` a non-square crop pads to stride multiples, e.g. 224x640, not
  always a full 640²). Cost is therefore at 640-scale, not crop-scale.
  ~200 ms **[proxy: our measured TFLite CPU-4 figure for a comparable YOLOv8n
  at 640; the real `.pt` path is slower. NOT directly measured.]**
- **Step B — classify colour + measure intensity.** `classify_beacon_color()`,
  pure OpenCV/numpy. Timings below.

Step B **[measured** — on-device, inside the `humble_stack` container, real repo
code with `ultralytics` stubbed; synthetic square-face imagery, so real crops
with messier contour structure will take the branchier path and cost a little
more**]**:

| crop | ms/call | calls/s |
|---|---|---|
| 16x16 | 0.584 | 1714 |
| 24x24 | 0.600 | 1666 |
| 32x32 | 0.627 | 1595 |
| 48x48 | 0.728 | 1373 |
| 64x64 | 0.814 | 1229 |
| 96x96 | 1.118 | 895 |
| 128x128 | 1.480 | 676 |
| 192x192 | 2.579 | 388 |

~0.55 ms fixed overhead plus a per-pixel term. At a realistic 32-64 px beacon
crop: **~0.6-0.8 ms**, i.e. ~2% of a 37 ms frame budget per beacon.

**Step A is ~250-300x step B.** All of stage 2's cost is neural localisation;
none of it is the measurement that actually produces colour and the blink signal.

Per-frame cost today is roughly `(1 + N_beacons) x` a full-model pass. This is
almost certainly what the burst/staged variants were built to work around.

### Why the upscale exists (don't just lower imgsz)

Upscaling adds no detail — interpolation cannot create information. It exists to
put the object at **the pixel scale the model was trained at** (`imgsz=640` in
the checkpoint); CNN detectors are not scale-invariant, and YOLOv8's 8/16/32
strides assign objects to levels by pixel extent. So simply passing a smaller
`imgsz` at inference is **not** a free win — it manufactures a train/test scale
mismatch and would likely cost recall. The principled version is to retrain or
fine-tune the crop model at a smaller `imgsz`.

What the two-stage design *does* legitimately buy: the crop is taken from the
**original** frame, not the downscaled tensor stage 1 ran on. On the demo frame
stage 1 saw the beacon at ~32 px (letterbox `scale=0.625`); stage 2 gets the
full 52 px. ~1.6x more true linear resolution, plus correct scale presentation.

### Proposal: cache the localisation per track

Because the crop model is `task=detect`, `masks` is always `None`, so
`display_mask` is always a filled rectangle and `mask_region` is always all-255.
**Step B consumes nothing from step A except a rectangle** — so a cached frame
can be bit-identical to a live one. The only question is whether the rectangle
is still right.

1. Store the lit region as **fractions of the stage-1 crop** (not absolute px —
   the bbox moves and rescales), keyed by `tracking_id`. This is why the tracker
   had to land first.
2. Re-project onto the current crop each frame and run step B only:
   `lit = crop[int(fy1*ch):int(fy2*ch), int(fx1*cw):int(fx2*cw)]`,
   `mask = np.full(lit.shape[:2], 255, np.uint8)`.
3. Refresh step A on track creation, every K frames (**staggered round-robin
   across tracks** so refreshes don't spike one frame), on large bbox
   area/aspect change, or on validity failure.
4. **Distinguish "beacon off" from "cache stale"**: threshold the *whole* crop
   (sub-ms) and check for lit pixels *outside* the cached rect. Lit pixels
   elsewhere -> drifted, refresh. None anywhere -> genuinely off, keep cache and
   record the low-intensity sample.

| | per beacon/frame |
|---|---|
| today | ~200 ms |
| K=10 | ~21 ms |
| K=30 (~1.1 s @ 27 Hz) | ~7.7 ms |

~26x at K=30, including the validity check. No GPU, delegate, container or
re-export needed.

**It also fixes blink measurement.** Today `if beacon_color == "no_top":
continue` (`beacon_detector_config.py:1823`) **skips the blink update entirely**
when the top isn't found — so `BlinkDetector` only ever sees on-phase samples and
the off-phase must be inferred from gaps via `_BLINK_GAP_OFF_SEC = 5.0`, which
**never fires for a 1 Hz blink** (0.5 s off-phase < 5 s threshold; that constant
was sized for the ~1 Hz detection regime). With a cached rectangle you keep
sampling regardless, so the off-phase becomes a *measured low-intensity sample*
— a real waveform with amplitude, phase and duty cycle instead of gap inference.

**Risk to watch: bbox jitter.** YOLO boxes wobble a few px per frame; a
fractional rect on a wobbling box wobbles too, injecting intensity noise — and
blink detection is edge detection on intensity, so that manufactures false edges.
Mitigate with an EMA-smoothed per-track bbox and a small margin on the cached
rect. Acceptance test: intensity variance on a *static* beacon, before vs after.

Implementation: a `LitRegionCache` keyed by `tracking_id` plus an
`isolate_and_classify_cached(...)` wrapper; swap only the call site at
`beacon_detector_config.py:1821` and leave `isolate_and_classify` untouched so
the video/batch paths are unaffected. Evict on track death.

### Cheaper thing to test first

`classify_beacon_color()` **already** does blob selection — threshold for lit
pixels, then if multiple contours prefer the *square-ish* one (`aspect <= 1.6`,
`extent >= 0.5`) precisely because the beacon top is a cube face. That is already
"find the lit square face in this image", which may make the crop model largely
redundant.

Before building the cache: run `classify_beacon_color()` on the **whole crop**
vs on the crop-model's `lit_region` across the saved `seabird_dataset/beacon_debug`
crops and measure agreement. High agreement means step A can be **dropped** for
most frames rather than cached — simpler than all of the above. Purely offline,
on data already in the repo.

## Preprocessing mismatch: the server stretches, the model expects letterbox

**Known and accepted for now.** `voxl-tflite-server` is being fed a **stretched**
input, which is technically incorrect for this model. We are testing with it as-is
before modifying the server.

The model's input is fixed `[1,640,640,3]` and the checkpoint records `imgsz: 640,
task: detect` — ultralytics trains by **letterboxing** to that square (aspect
preserved, 114-grey padding). The server instead **stretch-resizes**:
`mcv_init_resize_map(meta.width, meta.height, model_width, model_height)`
(`model_helper.cpp:211-214`), so e.g. the live 1440x1080 NV12 pipe is squashed to
640x640 — a 1.333x vertical distortion. That is a distribution shift, and it
cannot be changed without rebuilding the C++.

Our own `TFLiteHexagonDetector._letterbox()` letterboxes, i.e. matches training.
So the two paths do **not** run the same inference, and a confidence threshold
calibrated through one does not transfer to the other.

Measured once (n=1, demo frame, RGB, same model, same GPU): letterbox 0.6372 vs
stretch **0.7686**. Do **not** read that as stretch being better — stretch scales
y by 0.833 vs letterbox's 0.625, making the beacon ~33% taller in the tensor, and
small-object detection is strongly scale-sensitive. The likely reading is an
object-size confound, not a vindication of the distortion. Caveat: for the
BGR/RGB test the box was decoded and validated against the image; for the stretch
test only *peak score per class* was recorded — the box was never decoded, so it
is not a like-for-like comparison.

If the object-size effect is the real mechanism, the bigger lever is the
aggressive 1440x1080 -> 640 downscale, not the letterbox-vs-stretch choice.

**To settle it:** sweep all 210 frames in `seabird_dataset/beacon_debug/full_frames`
across {BGR, RGB} x {letterbox, stretch}, decoding properly each time. Those
frames are *all* named `nodet`, so the same sweep also quantifies how many real
detections the class-drop bug was discarding on flight data.

### Where the distortion actually lands (and a correction that does NOT work)

The **crop is not distorted**: `rgb_clean = rgb.copy()` is the untouched camera
frame (`beacon_detector_config.py:1798`) and `crop = rgb_clean[y1:y2, x1:x2]`
(`:1820`). The server also returns boxes in *original camera pixel* coordinates,
so stage 2 sees undistorted pixels.

**Stage 2 does not infer scale — stage 1 does.** `estimate_distance_from_bbox`
(`:1002`): `slant = (beacon_height_m * fy) / bbox_h`. Range comes from the
stage-1 **bbox height** plus the known physical beacon height, and
`modal_config.json` sets `depth_source: "bbox"`, so this path is live. Since
`slant` is inversely proportional to `bbox_h`, a 10% height bias is ~10% range
error.

**Rejected mitigation: scaling bbox width by 1.333.** The stretch is already
inverted exactly by the `* input_width` / `* input_height` mapping, so an extra
1.333x on width *introduces* error rather than removing it — and width does not
feed ranging at all, only `bbox_h` does. Any residual error is a **model
prediction bias** (trained on aspect-correct beacons, shown one 1.333x
vertically elongated, the net may pull its box toward the learned aspect), which
is empirical and unknown in sign. If the sweep shows a consistent bias the
correction is a *measured scalar on `bbox_h`* — not 1.333, and not on width.

**So the sweep should record `bbox_h` per frame under letterbox vs stretch and
test for systematic bias, not just confidence.**

## SWEEP RESULTS (n=129 frames) — settled

Ran `seabird_dataset/beacon_debug/full_frames` x {BGR,RGB} x {letterbox,stretch},
516 inferences, tflite_runtime 2.13.0 CPU in the `humble_stack` container, boxes
decoded and NMS'd properly in every condition.

**Caveat: 81 of the 210 frames are unfetched Git LFS pointers** (132-byte stubs;
`git-lfs` is not installed on this dev machine). Analysis is n=**129**. Running
`git lfs pull` and re-running would complete it.

| condition | detects @0.25 | detects @0.5 | mean conf | median conf |
|---|---|---|---|---|
| bgr / letterbox | 61 | 43 | 0.6176 | 0.6078 |
| **rgb / letterbox** | **107** | **86** | **0.6865** | **0.7330** |
| bgr / stretch | 63 | 39 | 0.5764 | 0.5858 |
| rgb / stretch | 106 | 88 | 0.6709 | 0.7085 |

### 1. The BGR bug was halving detections

Paired on 61 frames where both detect: **mean +0.1428, median +0.1479, RGB higher
on 90%**. At the configured `conf: 0.5` it is **43 -> 86 detections, exactly 2x**.

### 2. The class-drop bug was discarding ~82% of detections

All 129 frames were labelled `nodet` by the old code. With RGB+letterbox:

| threshold | frames detecting | class split | would have been dropped |
|---|---|---|---|
| 0.25 | 107/129 (83%) | blue 59, unknown 19, green 18, red 11 | **88/107 (82%)** |
| 0.45 | 90/129 (70%) | blue 58, unknown 16, green 12, red 4 | **74/90 (82%)** |
| 0.50 | 86/129 (67%) | blue 58, unknown 16, green 9, red 3 | **70/86 (81%)** |

**67% of "no detection" frames actually contained a detectable beacon**, and four
fifths of those were thrown away purely by `class_names=["beacon"]`.

### 3. letterbox vs stretch: a wash on confidence

Paired on 106 frames: mean **-0.0187**, median -0.0099, stretch higher on only
**45%**. The earlier n=1 result (stretch 0.7686 vs letterbox 0.6372) **did not
generalise and reversed** — it was a single-sample artefact, as suspected.
Detection counts are within noise (107 vs 106 @0.25, 86 vs 88 @0.5).

### 4. ...but stretch biases ranging by +12%

stretch/letterbox **bbox_h ratio: mean 0.8931, median 0.8947, stdev 0.0482**
(n=106). Stretch systematically predicts boxes **~10.7% shorter**. Since
`slant = beacon_height_m * fy / bbox_h`, that is a **+12.3% range over-estimate**.

Mechanism consistent with prediction: in stretched space the beacon appears 1.333x
taller; a net trained on correct aspect pulls its box toward the learned aspect,
under-estimating the elongation, and the inverse mapping then yields a short box.

The bias is **tight** (stdev 0.048 on mean 0.893), so it is systematic and
correctable by a measured scalar (~1.12 on `bbox_h`) — exactly as predicted, and
*not* the rejected 1.333-on-width idea. Better to fix the preprocessing.

**Conclusion:** stretch is acceptable for crude detection testing (confidence is a
wash) but **must not be trusted for ranging** until either the server letterboxes
or a measured `bbox_h` correction is applied.

## Future work: make voxl-tflite-server configurable instead of rebuilding per model

Rebuilding the server is acceptable — the goal would be to make it free-form so it
does not need a rebuild for every model. Minimal patch set, mostly one file:

- **Preprocessing mode** — `mcv_init_resize_map()` builds a stretch map; letterbox
  needs aspect-preserving scale + 114 padding, and the inverse applied in
  `postprocess()` (currently a plain `* input_width`,
  `yolov8_model_helper.cpp:106-110`). Config key.
- **Thresholds** — `model_score_threshold` / `model_confidence_threshold` /
  `model_nms_threshold` are compile-time per architecture (YOLOv8/v11:
  0.45 / 0.25 / 0.5). Config keys.
- **Quantised input scale/zero-point** (`model_helper.cpp:400-429`) and **YOLOv8
  output dequantisation** (`yolov8_model_helper.cpp:70`) — both required before
  the NPU is usable at all, and both absent repo-wide.

*Superseded: the GPU path (`voxl/beacon_gpu/`, `tflite_gpu.py`) builds on the drone
host and needs neither the server nor `voxl-cross`; `voxl-docker/` is no longer
checked out.* Build path was: `voxl-docker/` provides `voxl-cross`; use
`install_build_deps.sh qrb5165 sdk-1.6` to match voxl-suite 1.6.4~beta3 (docs note
the branch arg must match the branch of the source checked out, and master needs
voxl-cross >= 4.3).

Once the first two land, Path A stops being "same model, different input" and the
A vs B' choice reduces to where the interpreter should live rather than which one
is correct.

### Implemented (unbuilt, untested on hardware)

New keys in `/etc/modalai/voxl-tflite-server.conf`, defaults preserve upstream
behaviour except `num_threads`:

| key | default | notes |
|---|---|---|
| `resize_mode` | `STRETCH` | `LETTERBOX` = Ultralytics convention (min scale, centred, 114 pad). Refused at startup for non-YOLOV8/V11 architectures, whose postprocess doesn't undo the padding |
| `score_threshold` | `0.45` | YOLOV8/V11. The old `model_confidence_threshold = 0.25` passed to `NMSBoxes` was dead — every candidate had already passed 0.45 — so it's folded into this one |
| `nms_threshold` | `0.5` | YOLOV8/V11 |
| `num_threads` | `4` | interpreter + XNNPACK; was 8 on QRB5165, measured slower |

Also changed (YOLOV8/V11): `ai_detection_t.timestamp_ns` is now the camera
**capture** timestamp and `frame_id` the camera's `frame_id`, and the overlay frame
keeps its capture timestamp. The old `frame_id` read a counter the preprocess thread
increments concurrently, so it wasn't reliable even as a server counter. Boxes are
now clipped to the frame, matching `tflite_hexagon_detector._decode()`.

## Flight drone (`root@192.168.50.121`) [measured 2026-10-11]

Same software as the bench drone: system-image `1.8.06-M0054-14.1a-perf`,
voxl-suite `1.6.4~beta3`, qrb5165-tflite 2.17.2, voxl-tflite-server 0.5.1. Its SSH
host key differs from the one previously stored for this IP.

- **Cameras:** both `hires_front_*` and `hires_down_*` pipes exist.
  `hires_front_small_color` is **1024x768** (`voxl-camera-server.conf`). This
  confirms the flight frames are 1024x768, not the 640x480 the configs assume.
- **No hires intrinsics:** `/data/modalai` has `opencv_tracking_*` only, and
  mpa-to-ros2 publishes **no `camera_info`**. hires_front needs a calibration at
  1024x768 before any geometry is trusted.
- **Extrinsics** (`/etc/modalai/extrinsics.conf`):
  - `hires_front` wrt `imu_apps`: T `[0.0408, 0.006, 0.0186]`, RPY `[0, 90, 90]`,
    i.e. **level and forward-facing**. The configs' `pitch_deg: 15` and
    `mount_offset_xyz [0.30, 0, 0.05]` are sim values, not this airframe.
  - `imu_apps` wrt `body`: T `[0.0303, -0.0057, -0.0116]`.
- **`humble_stack` container:**
  - Image `voxl2-humble`, built from `/home/root/humble_docker/` (Dockerfile +
    compose): `ros:humble-ros-base-jammy` + px4_msgs `release/1.14`.
  - Host network/IPC, not privileged, no `/dev/kgsl-3d0` or `/dev/ion`, only
    `/home/root/bags` mounted, no repo at `/workspace`.
  - Python 3.10.12, cv2 4.5.4 (`ocl.haveOpenCL()` False), numpy 1.21.5. **No
    tflite_runtime, scipy or ultralytics** (unlike the bench drone's container).
- **OpenCL prerequisites:**
  - All five packages the OpenCL recipe repacks are installed at the recipe's versions.
  - `/dev/kgsl-3d0` and `/dev/ion` exist; there is no `/dev/dma_heap`.
  - `dpkg-repack` is absent, but the drone has internet, so `apt install` works.
- **GPS/heading (indoors):** no GPS fix (`fix_type 0`, `ref_lat` nan); VIO-fed
  `xy_valid` true. Magnetometer present, but `heading_good_for_control` false.

### PX4 topics missing from ROS 2 — FIXED

**Symptom:** `microdds_client` reported "Running, connected" to `MicroXRCEAgent
udp4 -p 8888` (`voxl-microdds-agent` 3.0.0), but no `/fmu/*` topics appeared in
`humble_stack`.

**Cause:** PX4 param `XRCE_DDS_DOM_ID = 8` (saved, non-default). PX4 published on
DDS domain 8, while the container and mpa-to-ros2 (cameras, VIO) use domain 0.
With `ROS_DOMAIN_ID=8`, all `/fmu/in|out/*` topics appeared and decoded with the
container's px4_msgs `release/1.14`.

**Fix (drone disarmed):**
1. `px4-param set XRCE_DDS_DOM_ID 0`
2. `px4-param save`
3. Restart the client with the same command `voxl-px4-start:294` uses:
   `px4-microdds_client stop`, then
   `px4-microdds_client start -t udp -h 127.0.0.1 -p 8888`.

**Verified:** on domain 0, 24 `/fmu/*` topics coexist with `/hires_front_small_color`
and `/vvhub_body_wrt_local/pose`. `vehicle_local_position`, `vehicle_attitude` and
`vehicle_gps_position` all decode.

**Rollback:** `px4-param set XRCE_DDS_DOM_ID 8 && px4-param save`, then reboot.
Anything that was listening on domain 8 no longer sees PX4.

### Drone reconfiguration for the mission [measured 2026-10-11]

Mission profile **[reported]**: open air over water with full GPS, so VIO and ToF
obstacle avoidance are unneeded. Every change below was made disarmed.

| Change | Rollback |
|---|---|
| `voxl-open-vins-server` disabled + stopped (qvio was already off) | `systemctl enable --now voxl-open-vins-server` |
| `voxl-vision-hub` disabled + stopped (VIO forwarding, VOA, figure-8 offboard) | `systemctl enable --now voxl-vision-hub` |
| PX4 `EKF2_EV_CTRL` 15 -> 0 (saved): EKF uses GPS + mag only | set `15`, `px4-param save` |
| PX4 `CP_DIST` 1.9144 -> -1 (saved): collision prevention off | set `1.9144`, `px4-param save` |
| `voxl-camera-server.conf`: `tof` disabled | restore `.bak-pre-tof-off`, restart server |
| `voxl-camera-server.conf`: `hires_front` and `tracking_down` disabled; `hires_down` `en_large_video` and `en_snapshot` off; `small_video` kept at 1024x768 | restore `.bak-pre-camera-trim`, restart server |

- The flight scripts are unaffected: `sweep_rrt.py`, `sweep_lawnmower.py` and
  `element3_safety_check.py` command PX4 through MAVSDK offboard, not vision-hub.
- **Breaks the current detector's world position:** `configs/modal_config*.json`
  use `/vvhub_body_wrt_local/pose` (now stale) and `/hires_front_small_color` (now
  gone). The pose source must move to PX4 `/fmu/out/vehicle_local_position` and
  `/fmu/out/vehicle_attitude`.
- **Before flight:**
  - Confirm outdoors that PX4 has a GPS fix, `xy_global` set and
    `heading_good_for_control` true. There is no VIO fallback any more.
  - Heading now rests on the magnetometer calibration, which feeds directly into
    beacon lat/lon.
- **CPU after the changes:**
  - `voxl-camera-server` dropped from ~55% to ~21%.
  - The largest remaining consumers are `voxl_mpa_to_ros2` (~90-100%),
    `/usr/local/bin/mavlink-serial-bridge.py` (~85-90%), and `republish`
    (~40%, `hires_down_small_color` -> compressed). None were touched.
- **Not a regression:** camera-server logs `json_from_yaml, failed to open file`
  on every start. MISP looks for `tracking_front_intrinsics.yml`, but the file on
  disk is `opencv_tracking_front_intrinsics.yml`.
- **GPU contention to watch:** camera-server reports "Detected 1 GPU device(s)".
  MISP for `tracking_front` runs on the same Adreno as inference.

## Bench board (`root@192.168.50.146`, m0054) [measured 2026-10-19]

The damaged VOXL2, now a bench/test board. **Not** the flight drone: `.121` above
stays the flight drone, with its TFLite 2.17 setup. Nothing of ours was on it.

- **Software (older than `.121`):** voxl-suite 1.5.0, qrb5165-tflite **2.8.0**
  (static archives only; `TfLiteGpuDelegateV2Create` present), voxl-tflite-server
  0.4.1 (inactive), voxl-camera-server 2.2.4. g++ 7.5, kernel 4.19.125.
- **GPU:** `/dev/kgsl-3d0`, `/dev/ion` and all of `run.sh`'s `GPU_LIBS` exist;
  `qti-adreno 1.0-r0`. Its driver can differ from `.121`'s, so GPU timings here are
  indicative only.
- **`tracking_front`:** 1280x800 at 30 fps, `en_rotate: true`, same extrinsics as
  `.121`. Its `/data/modalai/opencv_tracking_front_intrinsics.yml` is **not**
  `.121`'s: fx 445.81 / fy 446.15, cx/cy 627.99/367.08 (`.121`: 446.73 / 446.75,
  669.30/379.34). Both files say 2023-03-02.
- **Docker:** no `voxl2-humble`; `seabird` (Ubuntu 22.04, ROS Humble, px4_msgs,
  numpy 1.26.4, cv2 4.11) serves as `BASE` for `beacon_overlay`.
- **ROS:** the host bridge is `voxl_mpa_to_ros2` on **Foxy**, publishing on
  **`ROS_DOMAIN_ID=8`** over UDP only (`/home/voxluser/fastdds_profile.xml`:
  192.168.225.1 + 127.0.0.1, no shared memory). The Humble container receives
  `/tracking_front` at ~18 Hz once it uses domain 8:
  `ROS_DOMAIN_ID=8 voxl/beacon_overlay/run.sh start`.
- Internet works; 8 cores, 7.5 GB RAM, 55 GB free on `/data`.
- Ours on it now: code rsynced to `/home/root/seaowl`, the 2-class model in
  `/data/models_eval/`, and the `beacon-overlay` image (see below).

## Pose model

### Current: `YOLOv11n_pose_diamond_beacon.pt` (2-class) [source; measured 2026-10-19]

- **Classes:** `{0: beacon, 1: diamond}`, `kpt_shape [4,3]`. `diamond` carries the
  real corners (0 = bottom, 1 = left, 2 = top, 3 = right); `beacon` is box-only,
  with a placeholder `dummy` keypoint in the RoboFlow labels **[reported]**.
- **Training:** YOLO11n-pose from `yolo11n-pose.pt`, Ultralytics 8.4.55, trained
  2026-10-16, `imgsz 640`, 300 epochs (patience 90). Augmentation on this time:
  mosaic 1.0, fliplr 0.5, hsv 0.015/0.7/0.4, scale 0.5. The checkpoint doesn't
  record `flip_idx`; without `[0,3,2,1]`, Ultralytics turns fliplr off for pose.
- **Validation (checkpoint):** box P 0.889 / R 0.709 / mAP50 0.711 / mAP50-95
  0.537; pose P 0.462 / R 0.338 / mAP50 0.396 / mAP50-95 0.323. Pose metrics
  include `beacon`, whose keypoints are placeholders.
- **TFLite export** (TF 2.19.0, `half=True`): input `[1,640,640,3]` float32 RGB in
  [0,1]; output `[1,18,8400]` = 4 box + 2 class scores + 4 x (x, y, conf).
- **Input is STRETCHED, not letterboxed.** The training frames are 1280x800
  tracking images stretched to 640x640 **[reported]**, confirmed against held-out
  labels (`pose_data_gen/beacon_labeler/coco_export`, fp32 export, IoU > 0.5):

  | split | preprocessing | beacon recall@0.25 | diamond recall@0.25 | diamond kp median err |
  |---|---|---|---|---|
  | valid (87 beacons / 164 diamonds) | **stretch** | 1.00 | **0.41** | **1.21 px** |
  | valid | letterbox | 0.99 | 0.20 | 1.78 px |
  | test (44 / 83) | **stretch** | 0.98 | **0.40** | **1.43 px** |
  | test | letterbox | 0.98 | 0.35 | 1.94 px |

  `TFLiteHexagonDetector(resize_mode="stretch")` undoes it per axis (1280x800:
  x / 0.5, y / 0.8); `beacon_overlay.py --resize stretch` is the default.
  Diamond recall is low at 0.25: most misses are small or oblique diamonds.

### Previous checkpoint: `YOLOv11n_diamond_detect.pt` (1-class, superseded) [source]

- **Architecture:** YOLO11n-pose (`C3k2`, `C2PSA`, `DFL`; `yolo11n-pose.yaml`,
  0.50/0.25). Ultralytics 8.4.55, trained 2026-10-09.
- **Classes:** 1 (`diamond`), `kpt_shape [4,3]`. This is Strategy B
  (single-diamond detector) from `references/gemini_convo_oct_8.pdf`.
- **Training:** `imgsz 640`, 300 epochs with patience 10.
- **Augmentation:** all Ultralytics augmentation was off (`mosaic`, `fliplr`,
  `hsv_*` = 0), so RoboFlow's Symmetries setting had no effect.
- **Validation:**
  - Box: P 0.947, R 0.813, mAP50 0.862, mAP50-95 0.492.
  - Pose: P 0.928, R 0.882, mAP50 0.902, mAP50-95 0.703.
- **Keypoint order** (from RoboFlow's named output):
  **0 = bottom, 1 = left, 2 = top, 3 = right**.
- **The names are face-relative, not image directions.** Under perspective,
  "left" can sit to the right of "top".
- **Degenerate quads need gating on geometry.** One RoboFlow example had
  `right` and `bottom` 1 px apart with per-keypoint confidence of 0.76-0.93.

### Beacon geometry [reported, user-measured]

- Origin at the base centre, Z up. Full beacon: height 0.52605 m, envelope
  0.559 x 0.559 m. The 0.559 m is the **skirt** at the base, wider than the faces.
- **White section** (square frustum carrying the diamonds) [CAD]: face width
  0.432 m at the bottom and 0.254 m at the top, vertical height 0.291548 m.
  - Its sloped length is 0.3048 m = 12.0 in. An earlier tape figure of
    "12 in tall" was this sloped length; the tape widths (16.75 / 7.5 in) are
    superseded by CAD.
- Face-0 diamond corners: T `(0, -0.14074, 0.32499)`, B `(0, -0.2, 0.13063)`,
  L `(-0.1016, -0.17037, 0.22781)`, R `(0.1016, -0.17037, 0.22781)`.
- Faces 1-3 are face 0 rotated k*90 deg about Z.
- **Checked:**
  - The diamond is a coplanar square, with diagonals 0.2032 m and perpendicular.
  - Face tilt from vertical: 16.96 deg from the diamonds, 16.98 deg from CAD.
  - Fitting the CAD frustum to the diamond plane puts the white section at
    **z = 0.0782-0.3697 m**. Every diamond corner lies within **0.1 mm** of the
    CAD face, so the corners and the CAD agree.
- **Derived:** the skirt is ~0.078 m tall. The top above the white section is
  ~0.156 m (lit top; shape not modelled).
- `detection.beacon_height_m: 0.3048` in the configs is the white section's
  sloped length, not the beacon height. Its vertical height is 0.2915 m.
- `voxl/visualize_beacon_mesh.py` renders this model (diamonds from
  `beacon_pose.BeaconModel`, CAD white section, skirt, envelope) and prints the
  consistency figures above.

### TFLite export, 1-class model [measured]

- **Environment:** dev machine, venv `~/.venvs/yolo-pose`, ultralytics 8.4.55,
  torch 2.14.1+cpu, tensorflow 2.19.0.
- **Pin needed:** `onnx==1.17.0`. The 1.23.2 that Ultralytics pulls in needs
  protobuf >= 6.31, but TF 2.19 pins 5.29.6.
- **Output files:** `yolo export ... format=tflite imgsz=640 half=True` writes
  `YOLOv11n_diamond_detect_saved_model/{float16,float32}.tflite`.
- **Tensors:** input `[1,640,640,3]` float32 RGB in [0,1]; output `[1,17,8400]`
  float32.
  - Rows 0-3: box xywh, normalised to [0,1] of 640.
  - Row 4: score, already a probability.
  - Rows 5-16: 4 x (x, y, conf). x/y are normalised like the box; conf is already
    a probability.
- **Against the `.pt`:** keypoints agree within ~1 px after un-letterboxing, and
  keypoint confidences within 0.01.
- **Score difference:** 0.225 vs 0.326 on the same frame. Likely `.pt` predict
  uses a rect 640x480 input instead of a 640x640 letterbox (unverified).
- **On flight frames:** `seabird_dataset/beacon_debug/full_frames` gave only 7
  weak detections (conf ~0.33, diamonds 6-20 px). 81 of the 210 PNGs there are
  unreadable.

### GPU benchmark on `192.168.50.121`, 1-class model, host TFLite 2.17 [measured]

`benchmark_model_mai`, fp16 weights, 30 runs, 3 warmup. Times are `Invoke()` only.

| Config | YOLO11n-pose (diamond) | YOLOv8n reference (above) |
|---|---|---|
| GPU, default | 52.0 ms (~19 Hz) | 44.8 ms |
| **GPU, sustained-speed** | **42.5 ms (~23 Hz)** | 35.7-36.9 ms |
| CPU, 4 threads | 378.6 ms | 198.9 / 362.3 ms |
| GPU init | 5.3 s / 9.7 s (sustained) | ~6.2 s |

- **Whole graph on the GPU:** "completely executed by the delegate", 1 kernel.
  This includes `C2PSA` attention and `DFL`, which the Gemini PDF expected to be
  a problem.
- **Cost vs YOLOv8n:** ~1.2x on the GPU. The PDF's "9-14 ms" is ~4x
  optimistic.
- **GPU vs CPU:** 8.9x faster. CPU alone (~2.6 Hz) would miss the 0.5 s
  requirement.
- Short burst on the bench, peak 48.7 C. Sustained thermals untested.
- **10 Hz target [reported]:** leaves ~57 ms for transport, preprocessing,
  decode/NMS and PnP. These stages can be pipelined around the GPU call.
- The test model is still on the drone at `/data/models_eval/`
  (`rm -r` to remove).

### Two-stage design [reported 2026-10-11]

- **Stage 1:** `tracking_front` detects beacons by their diamonds and locates them.
- **Stage 2:** `hires_down` (`small_video` 1024x768) observes colour and blink
  when the drone flies over.
- **Deployment:** a container with OpenCL + the TFLite GPU delegate (from
  `references/voxl_opencl_docker.pdf`), **not** `voxl-tflite-server`.

**`tracking_front` [measured]:**
- AR0144, **mono8** 1280x800 at 30 fps, `en_rotate: true`.
- **Fisheye** calibration `/data/modalai/opencv_tracking_front_intrinsics.yml`:
  fx/fy 446.7, cx/cy 669.3/379.3, 4-coefficient `fisheye` model, 0.29 px
  reprojection error, dated 2023-03-02. Whether it matches the rotated image is
  unverified.
- Extrinsics: wrt `imu_apps`, T `[0.039, 0, 0.0006]`, RPY `[0, 90, 90]`
  (forward, level).

**Consequences:**
- **The model has only seen RGB.** It must be tested on grey replicated to 3
  channels from real tracking frames. Retraining with greyscale or tracking-camera
  images may be needed.
- **Diamonds are small** on this lens: ~91/Z px at full resolution, ~45/Z px
  after the 1280 -> 640 letterbox. The ~12 px needed for reliable PnP is reached
  at ~3.8 m (640 letterbox) or ~7.5 m (full resolution).
- **Beyond that range, position comes from the ray.** The detection ray is
  intersected with the water plane, using GPS/baro altitude and PX4 attitude.
  At 5 m altitude and 20 m range: ~0.2 m per pixel, ~0.7 m per 0.5 deg of
  attitude error.
- **Options to extend PnP range:** 640-wide tiles at full resolution across the
  horizon band (~2x range, 2 inferences), or a 1280x800 export (likely
  >= ~100 ms).

## TFLite built in the container [2026-10-19]

`beacon_overlay`'s image now builds TFLite from source instead of linking the
drone's `qrb5165-tflite`, so `.121` (2.17) and `.146` (2.8) run the same runtime.

- **Version: 2.19.0**, the TensorFlow that exports the models. Every op version in
  an export is then known to the runtime. Newer means moving to LiteRT
  (`tf.lite` is deprecated from TF 2.20), a separate job; bump exporter and
  runtime together.
- **Build:** `voxl/beacon_overlay/Dockerfile` stage 1 fetches the v2.19.0 source
  tarball, then `voxl/beacon_gpu/CMakeLists.txt` adds `tensorflow/lite` as a
  subdirectory (`TFLITE_ENABLE_GPU=ON`, XNNPACK on) and links it with
  `tflite_gpu_shim.cc` into `libtflite_gpu_shim.so` (only `bg_*` exported). Stage 2
  copies it to `/opt/beacon_gpu/` in `BASE` (`voxl2-humble` on `.121`, `seabird` on
  `.146`). `run.sh build` / `BASE=seabird run.sh build`.
- **CPU and GPU share it:** `GPU=0` passes `--cpu`, so the shim runs the same TFLite
  on XNNPACK. `tflite-runtime 2.13` is no longer in the image. The host build
  (`beacon_gpu/build.sh`) is gone.
- **`XNNPACK_ENABLE_KLEIDIAI=OFF`:** XNNPACK 2.19 downloads KleidiAI as a GitLab
  archive whose SHA256 no longer matches the pinned one (GitLab regenerates
  archives). KleidiAI only accelerates int8/int4 matmuls; these models are fp16/fp32.
- **absl log:** 2.19's `compiler/mlir/lite/core/api/flatbuffer_conversions.cc` uses
  `LOG()` but the `tensorflow-lite` target doesn't link absl logging; the shim
  links `absl::log` + `absl::log_internal_message` itself (97 undefined
  `LogMessage` refs otherwise).
- **Build time on `.146`:** ~15 min for TFLite (`JOBS=4`, peak RAM fine), in its own
  layer; a shim-only edit just relinks. Image 10.6 GB (`seabird` base); the shim is
  10 MB, exports only the 8 `bg_*`, and needs only libc/libm/libstdc++.

**Measured on `.146` [2026-10-19]** (2-class fp16 model, one held-out tracking
frame stretched to 640x640, compared with TF 2.19 on the dev machine):

| | invoke (median / min, 20 runs) | init | vs TF 2.19 |
|---|---|---|---|
| GPU (fp16, sustained) | **38.1 / 37.1 ms** (~26 Hz) | 6.0 s | anchors scoring > 0.25: box 0.28 px, kp 0.26 px, score 0.0016 |
| CPU (XNNPACK, 4 threads) | 645 / 341 ms | 0.0 s | identical (max 0.002 px) |

- "Created 1 GPU delegate kernels": the whole graph runs on the GPU under 2.19
  too. Same speed as `.121`'s host 2.17 build (~38 ms) on this board's driver.
- Worst-case GPU difference over all 8400 anchors is ~4 px, on anchors far below
  threshold (fp16 rounding); irrelevant after the score cut.
- The CPU spread (341-645 ms) is contention with the board's other services;
  either way the CPU is a fallback, not the plan.
- Not yet done: the same image on `.121` (`run.sh build`, `BASE=voxl2-humble`), and
  a run of `beacon_overlay` on live `tracking_front` frames.

## Beacon localisation (`beacon_pose.py`)

Detections -> beacon base centre in PX4 local NED -> lat/lon. Pure numpy/OpenCV,
with no ROS and no scipy. Tested offline in `tests/test_beacon_pose.py` (16 tests,
synthetic scenes through the real `tracking_front` fisheye calibration and
extrinsics). Tunables live in `configs/beacon_pose.json`, which the loader checks
strictly (unknown keys are an error). That file is wired to the detector via
`detection.pose_config`.

**Priority per beacon:**
1. `"pnp"`: a diamond passing `diamond_quality`. Gravity-constrained 4-DoF fit
   over all of that beacon's diamonds.
2. `"box"`: a `beacon`-class bbox.
3. `"ray"`: diamond centre ray to the water plane.

A box claims the diamonds whose centres fall inside it. If a pnp fix exists, the
box is recorded in that fix's `box_members` instead of producing its own fix.

### Box localisation [source; synthetic results measured]

Purpose **[reported]**: give the drone a rough bearing and range to start flying
towards a beacon too far away for diamonds. Bearing comes from the box centre
(pixel-level). Range only needs to be roughly right, because fixes improve as the
drone closes in (box -> pnp), and the map weights them by covariance.

`locate_box()` fuses two estimates horizontally:
- **Size:** range = apparent height / angular height of the box. Angles come from
  undistorted rays, so the fisheye is handled.
  - Height only. Width was dropped because it depends on the unknown yaw
    (0.559 m face-on to 0.79 m corner-on).
  - The look-down correction (`box_height_elev_correction`, **on**) uses apparent
    height = H cos(e) + depth sin(e). Seen from above, the box includes part of
    the top surface.
  - Without the correction, range is short by 4% (2 m altitude, 40 m range) up to
    18% (8 m altitude, 30 m range). Use `box_size_rel_sigma` ~0.15 if it's off;
    0.08 with it on.
- **Ray:** through the bbox bottom-centre to the water. The skirt sits on the
  water, so no height assumption is needed. The point is then pushed back by half
  the skirt depth to reach the base centre.
- **Consistency gate:** the two estimates must agree within chi2(2) <
  `box_consistency_chi2` (13.82), or the box is rejected. This catches false
  detections and a wrong water height: a 3 m water-height error rejects >80% of
  boxes in the test.

Synthetic box-only error (horizontal, median / p90; box edges +/-1 px, attitude
error 0.5 deg, altitude error 0.3 m; 94-100% of errors inside the reported 3 sigma):

| Range / alt | Box height | Correction on | Correction off | Diamond-centre ray |
|---|---|---|---|---|
| 8 m / 4 m | 44 px | 0.25 / 0.63 m | 0.60 / 1.08 m | (pnp) |
| 12 m / 5 m | 28 px | 0.37 / 0.83 m | 0.62 / 1.30 m | 0.54 / 1.35 m |
| 20 m / 6 m | 16 px | 0.66 / 1.66 m | 0.92 / 1.91 m | 0.76 / 2.04 m |
| 30 m / 8 m | 10 px | 0.86 / 2.17 m | 1.00 / 2.36 m | 1.01 / 2.60 m |
| 45 m / 8 m | 6 px | 2.03 / 4.86 m | 2.09 / 4.86 m | (no diamonds) |
| 60 m / 10 m | 5 px | 2.32 / 6.15 m | 2.31 / 6.00 m | (no diamonds) |

**Assumptions:**
- **Box convention** **[reported]**: `beacon` boxes cover the whole beacon, skirt
  to top, matching H = 0.52605 m. A box stopping at the white section (0.370 m)
  would make every size range ~42% long.
- **Lit top shape:** taken as the CAD white-section top width, 0.254 m. It only
  sets how much top surface shows from above (the correction's depth term,
  scaled by sin(e)). A 0.15 m top would change apparent height by <3% at 15 deg
  look-down, well inside `box_size_rel_sigma` 0.08. The light itself (colour and
  blink) is stage 2's job, not this one's.
- Drone GPS error (~1-2 m) is not modelled. It is common to every fix and does
  not average out.

### Related pieces

- **Detector:**
  - `Detection.keypoints` is (K, 3) in original-frame pixels.
  - The TFLite decode needs `detection.kpt_shape` (e.g. `[4, 3]`), because a pose
    output's channel count is ambiguous.
  - mono8 frames are replicated to 3 channels.
  - Against the `.pt`: keypoints within 0.36 px, identical keypoint confidences.
- **NMS:** `nms_iou` 0.65 from `configs/beacon_pose.json`. Box IoU cannot separate
  a duplicate detection of one diamond from adjacent-face diamonds: on a RoboFlow
  test frame the two duplicate pairs had IoU 0.6 and 0.40.
- **Keypoint dedupe** (`kp_dedupe`, `kp_dedupe_frac` 0.6): drops the weaker
  detection when all 4 same-named corners are within 0.6 x diagonal. Real
  duplicates measured 0.37. Adjacent faces are >= 1.04 (min over 1,449
  synthetic pairs).
- **Front-face constraint:** a diamond's back is never visible, so the opposite
  winding is rejected as `back_facing` (mislabelled keypoints). Any IPPE or
  joint-fit solution that shows a diamond's back is discarded.
- **Real-model fit** (RoboFlow test image, new 2-class model): two diamonds fit
  the CAD geometry at 0.38 px RMS. Only face pair (0, 3) survives the front-face
  constraint. But both diamonds are 9-10 px, below `min_diag_px` 12, so the
  pipeline would not use pnp here. `min_diag_px` should be set from measured
  keypoint error against labels, not from this fit's RMS.

## Replay with telemetry (`beacon_overlay.py --telemetry`) [measured 2026-10-19]

`telemetry_viewer.py` recordings (`<ts>.mkv` + `<ts>.csv`, one row per frame) play
on loop through the full localisation: per fix a marker at the estimated base,
method, slant/horizontal distance and lat/lon +/- 1 sigma (via the drone's own GPS
fix + NED offset), and fused beacons (reset each loop) bottom-left. On `.146`:

    REPLAY_DIR=/data/replay ROS_DOMAIN_ID=8 voxl/beacon_overlay/run.sh start \
        --video /replay/20261015_154558_510023.mkv \
        --telemetry /replay/20261015_154558_510023.csv --water-z 1.6

**Bug found and fixed: `CameraModel.normalize()` / `project()` on strided arrays.**
Detector keypoints are `(K, 3)`; every caller passes the `[:, :2]` view, and
OpenCV's fisheye undistort silently misread that buffer. IPPE then put the diamond
0.17 m from the camera at 464 px error (7.09 m and 0.23 px from a contiguous copy),
so **PnP never produced a fix on real frames**, and the overlay's camera-only
diamond ranges were wrong too. Both now take `np.ascontiguousarray`;
`test_camera_strided_input` fails without it.

Recording `20261015_154558_510023` (engineering west pool, 710 frames, ~10 fps,
1 beacon, drone 1.6-2.5 m above PX4's origin, 6-9 m from the beacon):
- Telemetry horizon (attitude + extrinsics + `.146` calibration) lands on the real
  horizon, roll included: frame conventions are right.
- 685 beacon boxes, 379 diamonds (223 pass `diamond_quality`). With the fix: 215
  PnP fits at 0.42 px median rms (`.121`'s calibration: 214 at 0.41 px, so this
  recording can't tell which board shot it).
- **Water surface ~1.6 m below PX4's origin**: PnP base z median 1.63 m, and the
  box size-vs-ray check is most consistent there (chi2 median 1.7, every box
  accepted). It armed above the water, so `--water-z 0` rejects most boxes.
- PnP and box agree within each 12 s window, but the estimate **moves ~2 m over
  the 70 s** ((5.4, -7.4) -> (6.7, -5.4) NED). Beacon drift, drone GPS/EKF drift,
  or heading error; this recording can't separate them.
- Per-fix covariances leave out drone GPS error, so with `BeaconMap`'s default
  0.05 m floor two thirds of fixes were "outliers". `--map-floor 0.5` (overlay
  default): 4 outliers out of ~680.
- **`BeaconMap` is now one Kalman filter per beacon** with a horizontal random walk
  (`drift_mps`): covariance grows by drift^2 * dt between fixes, entries are never
  dropped (a multi-beacon map persists; an unseen beacon re-attaches with a wider
  gate), and entries that converge within `gate_m` and agree are merged. `drift 0`
  is the old information fusion. On this recording (floor 0.5 m): drift 0 -> 3
  entries, 136 outliers; **drift 0.15 m/sqrt(s) -> 1 entry, 683 fixes, 3
  outliers**, ending at (6.80, -5.26) NED. Overlay defaults: `--drift 0.15`,
  `--min-fixes 3`; map time is the CSV `frame_stamp`. Two beacons closer than
  `gate_m` (3 m) can still merge: set it from the minimum beacon spacing.
- 8 fps on `.146` GPU (inference ~55 ms of it).

## Beacon map on ROS (`/seabird/beacon_map`) [2026-10-19]

`beacon_overlay.py` publishes the map as `std_msgs/String` JSON (repo convention,
like `/seabird/beacon_detections`) once per processed frame while localising, live
or replay (`--map-topic`, `--no-publish`). Schema in the module docstring:
`beacons` (map entries with >= `--min-fixes`: id, ned, lat/lon, sigma_m incl.
drift, n, outliers, age_s, seen) and this frame's `fixes` (method, ned, lat/lon,
sigma_m, range_m, horiz_m, rms_px). Kept off `/seabird/beacon_detections`:
`sweep_rrt.py` treats every message there as a colour detection, and colour is
stage 2's job.

- **Live:** one rclpy node takes `/tracking_front` and PX4's
  `/fmu/out/vehicle_local_position`, `vehicle_attitude`, `vehicle_gps_position`
  (best effort; both <= 0.5 s old, `xy_valid` and `z_valid` required). Lat/lon
  from PX4's `ref_lat`/`ref_lon`; replay uses the drone GPS + NED offset.
- **Pose at capture time** [measured on `.146`]: `/tracking_front` `header.stamp`
  is the capture time in Unix time and arrives 35 ms later (34-51 ms); PX4's
  microDDS `timestamp`/`timestamp_sample` are on the same clock and arrive ~3 ms
  after. Position and attitude are buffered by `timestamp_sample`
  (`beacon_pose.StateBuffer`, linear / slerp) and looked up at the image stamp
  (+ `--image-time-offset` ms) in both `beacon_overlay.py` (`Px4State`) and
  `telemetry_stream.py` (`snapshot_at`; groups get `at_frame`). Before, the
  latest pose at arrival was used: 3.2 deg of heading at 90 deg/s, ~0.45 m at
  8 m. Stamps > 1 s apart: warn once, fall back to the latest pose. Checked live:
  30/30 `telemetry_stream` frames matched (`at_frame: true`). Recordings made
  before this carry the lagged pose (one row per frame; not recoverable).
  `--image-time-offset` is still to be fitted from a brisk-yaw flight (minimise
  fix scatter / horizon error), e.g. half the exposure if the stamp is its start.
- **Bench caveat:** with the replay overlay running (~2.5 cores) the UDP-only
  camera bridge on `.146` drops `/tracking_front` to < 1 Hz; with it stopped
  22 Hz. The camera pipe itself stays at 30 fps (`voxl-inspect-cam`).
- **`.146` [measured]:** PX4 is on domain 8 like the camera; in the container
  attitude ~90 Hz, local position ~55 Hz, GPS ~5 Hz. Indoors `xy_valid` false and
  no global ref, so live localises/publishes nothing (by design). Replay publishes
  at 8.2 Hz. (`ros2 topic echo` of the px4_msgs topics prints nothing here; an
  rclpy subscriber receives them.)
- Not yet consumed by anything; the mission scripts still read
  `/seabird/beacon_detections` / `/seabird/buoy_detections`.

## Open (to do) [updated 2026-10-19]

Done items are written up in their own sections above.

**Flight drone (`.121`), next time it's reachable**
1. Build and run the container there: rsync the repo to `/home/root/seaowl`, copy
   `YOLOv11n_pose_diamond_beacon_float16.tflite` to `/data/models_eval/`, then
   `voxl/beacon_overlay/run.sh build` (`BASE=voxl2-humble`, ~15 min) and
   `run.sh start` (domain 0). Confirm the whole graph is delegated and measure
   GPU invoke time for the 2-class model under TFLite 2.19.
2. Outdoor live test with GPS: confirm `/seabird/beacon_map` publishes, the log's
   "pose @capture" count rises (not "latest" / "none"), and fixes land on the
   beacon.
3. Fit `--image-time-offset` (beacon_overlay and telemetry_stream) from a
   brisk-yaw pass in front of a beacon: pick the offset that minimises fix scatter
   or horizon error. Currently 0.
4. Sustained thermals with the GPU running (only short bursts measured).

**Localisation / map**
5. Decide `--water-z` for the mission: water height relative to where PX4 arms
   (the pool recording needed 1.6 m). Either measure it per site or estimate it
   online from PnP base heights.
6. Set `BeaconMap` `gate_m` (3 m) from the minimum beacon spacing at the
   competition: beacons closer than the gate can merge.
7. The ~2 m drift of the beacon estimate over the 70 s pool recording is
   unexplained: beacon drift, drone GPS/EKF drift, or heading error (mag
   calibration). Repeat with a moored/fixed beacon to separate them.
8. Write a mission consumer for `/seabird/beacon_map`; the sweep scripts still read
   `/seabird/beacon_detections` / `/seabird/buoy_detections`.

**Model / data**
9. COCO -> YOLO-pose converter **[reported: deferred]**: ignore RoboFlow's
   placeholder `dummy` keypoint on `beacon`, write beacon rows as box + four
   `0 0 0`, `kpt_shape [4,3]`, `flip_idx [0,3,2,1]` only with horizontal flips.
10. Check whether training set `flip_idx` (checkpoint has `fliplr 0.5` but no
    record of it; without it Ultralytics disables flips for pose).
11. Add far-range beacons (5-20 px boxes) to the dataset; current boxes are
    35-527 px.
12. Diamond recall is low (0.41 at conf 0.25 on held-out frames): mostly small or
    oblique diamonds.
13. Which board shot the 2026-10-15 pool session is unresolved: the labeler's
    calibration matches `.146`, the training images are reported to be from `.121`,
    and the replay PnP fits equally well with either. If `.121`, re-run the
    labeler's CAD refit with `.121`'s calibration (labels themselves are raw
    pixels and stand).

**Tooling**
14. `voxl/telemetry_stream/run.sh` has no `ROS_DOMAIN_ID` (needs 8 on `.146`) and
    hard-codes the `voxl2-humble` image (absent on `.146`).
15. Overlay runs at ~8 fps with ~55 ms inference: profile resize / decode / draw /
    JPEG encode if a faster stream is wanted. On `.146` the replay's CPU load also
    starves the UDP camera bridge.

Target requirement **[reported]**: a detection at least every 0.5 s to resolve a
1 Hz blink. Met with margin on the GPU (and on the CPU alone).
