# VOXL2 Inference Acceleration — Findings

Working notes for moving beacon detection off CPU on the ModalAI Starling 2 Max
(QRB5165). Source material is in `references/`; `voxl-tflite-server/` and
`voxl-docker/` are upstream checkouts kept here for reference.

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

## Open

- Which model Sam settles on (1-class vs the untested 4-class).
- Target requirement **[reported]**: a detection at least every 0.5 s to resolve a
  1 Hz blink. TFLite on CPU alone (199 ms ≈ 5 Hz) already clears this — the GPU is
  headroom, not a prerequisite.
- Real-drone camera pipe names and SDK version unverified.
- Sustained thermal behaviour unmeasured.
