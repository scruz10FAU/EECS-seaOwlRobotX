# Beacon Detection Scripts

Detection pipeline for colored, optionally blinking, beacon lights mounted on a drone or buoy. Uses a two-stage YOLO model followed by HSV color classification and a rolling-window blink estimator.

---

## File Overview

| File | Role |
|---|---|
| `beacon_detector.py` | Entry point. Color classification pipeline, video and ROS live modes. |
| `beacon_detector_config.py` | JSON-configured version of `beacon_detector.py`. Loads all arguments and ROS topic names from a JSON config file instead of command-line flags. |
| `beacon_config.json` | Default configuration file for `beacon_detector_config.py`. Controls model paths, thresholds, run mode, and ROS topic overrides. |
| `modal_config.json` | Config for Modal AI VOXL hardware. Overrides camera/depth topics, drone pose, and GPS topics for that platform. |
| `blink_detector.py` | `BlinkDetector` class — rolling-window blink frequency estimator. |
| `beacon_camera.py` | `BeaconCamera` ROS2 node — camera image subscriptions, depth decoding, and pose/GPS callbacks. |
| `batch_detect.py` | Headless batch processor — runs detection over multiple video files and writes a CSV + summary. |

---

## beacon_detector.py

The main script. Handles color classification, frame annotation, video playback modes, and the ROS live camera loop. Imports `BlinkDetector` from `blink_detector.py` and lazily imports `BeaconCamera` from `beacon_camera.py` only when ROS mode is invoked.

### Two-stage detection pipeline

1. **Stage 1 — Beacon localization** (`one_beacon.pt`): YOLO detects the beacon bounding box in the full camera frame.
2. **Stage 2 — Lit-area isolation** (`best_crop.pt`): A second YOLO model runs on the beacon crop to find the glowing portion. Supports both detection (bbox) and segmentation (mask) output. Falls back to HSV brightness thresholding if nothing is found.
3. **Stage 3 — Color classification**: `classify_beacon_color()` masks pixels by saturation (≥ 60) and brightness (≥ 160), then runs a per-pixel hue vote over the lit region to assign one of: `red`, `green`, `blue`, `white`, `unknown`.

### Color classification details

Hue votes are computed in OpenCV HSV space (0–180°) using strict band membership with intentional gaps between bands so ambiguous hues fall through to `other` rather than snapping to the wrong color.

| Color | Hue range |
|---|---|
| red | 0–20° and 150–180° (wrap-around) |
| green | 35–95° |
| blue | 105–135° |

Red gets priority: if `vote_red ≥ 0.10` the result is `red`, even if more pixels are blue (the beacon housing reads blue when the LED is off).

### Public functions

```python
classify_beacon_color(bgr_crop) -> (color, color_conf, light_mask, intensity, votes)
isolate_and_classify(beacon_crop, crop_model, conf=0.3) -> (color, color_conf, display_mask, intensity, votes)
```

`isolate_and_classify` wraps `classify_beacon_color` with the second YOLO model pass. Use this in all calling code.

### Run modes

```
python3 beacon_detector.py                          # ROS live mode (ZED camera)
python3 beacon_detector.py -d                       # ROS live mode + OpenCV display window
python3 beacon_detector.py -rv footage.mp4          # video file → ROS publisher
python3 beacon_detector.py -v footage.mp4           # video file, no ROS
```

**Common flags**

| Flag | Default | Description |
|---|---|---|
| `--model / -m` | `models/one_beacon.pt` | Stage-1 YOLO beacon model |
| `--crop-model / -cm` | `models/best_crop.pt` | Stage-2 lit-area model |
| `--conf / -c` | `0.5` | Detection confidence threshold |
| `--display / -d` | off | Show OpenCV window (ROS and ROS-video modes) |
| `--save / -s` | off | Write annotated output video (video modes) |
| `--log / -l` | off | Write per-frame CSV log |
| `--save-crops / -sc` | off | Save each beacon bounding-box crop as a PNG for post-run analysis |

### Crop output (--save-crops)

When `--save-crops` is set, the raw bounding-box crop passed to the color classifier is written to disk for every detection. Files are named `crop_f{frame:06d}_d{det_idx:02d}_{color}.png` and saved in a directory alongside the input video: `<video_stem>_beacon_crops/`. In ROS live mode the directory is `~/seabird_dataset/beacon_debug/beacon_crops/` and filenames use the YOLO tracking ID instead of a per-frame detection index (`_t{tracking_id:02d}_`).

### CSV log columns

`timestamp, frame, color, color_confidence, intensity, vote_red, vote_green, vote_blue, vote_other, det_confidence, x1, y1, x2, y2, tracking_id, pos3d_x, pos3d_y, pos3d_z, blink_is_blinking, blink_hz, blink_phase`

### ROS topics (live and ROS-video modes)

| Topic | Direction | Type | Content |
|---|---|---|---|
| `/seabird/beacon_detections` | publish | `std_msgs/String` | JSON per detection (see below) |
| `/mavros/local_position/pose` | subscribe | `PoseStamped` | Drone local position + orientation |
| `/mavros/global_position/gp_origin` | subscribe | `GeoPointStamped` | GPS origin for ENU→GPS conversion |

**Published JSON fields**

```json
{
  "color": "green",
  "blink": {"is_blinking": true, "blink_color": "green", "blink_hz": 1.02, "phase": "on"},
  "label": "beacon",
  "color_confidence": 0.42,
  "intensity": 0.73,
  "hue_votes": {"red": 0.0, "green": 0.91, "blue": 0.07, "other": 0.02},
  "confidence": 0.87,
  "bbox": [120, 80, 210, 170],
  "position_3d": [0.12, -0.05, 4.82],
  "world_position": [1.3, 0.4, 4.8],
  "gps_position": {"latitude": 26.3712, "longitude": -80.1034, "altitude": 12.1},
  "drone_position": [0.0, 0.0, 5.0],
  "tracking_id": 2,
  "timestamp": 1748549271.34
}
```

`position_3d`, `world_position`, and `gps_position` are `null` if depth or pose data is unavailable.

---

## beacon_detector_config.py

A drop-in alternative to `beacon_detector.py` that reads all runtime parameters from a JSON file instead of command-line flags. Useful when the same node needs to run with different topic namespaces or model paths across deployments without modifying source code.

The detection logic, color classification, blink estimation, and ROS publishing are identical to `beacon_detector.py`. The only differences are how configuration is loaded and how `BeaconCamera` topic names are resolved.

### Usage

```
python3 beacon_detector_config.py                          # uses beacon_config.json
python3 beacon_detector_config.py --config custom.json    # use a different config file
```

The run mode (ROS live / video-only / video+ROS) is determined by the `video` and `ros_video` fields in the config rather than by which flag is passed.

### beacon_config.json

Default config file. Copy and edit to create environment-specific configs.

The config fully replaces `seabird_config.py` — `beacon_detector_config.py` does not import it at all. Camera intrinsics and the body-to-camera rotation matrix are derived at load time from the `camera` block.

**Top-level fields**

| Key | Type | Default | Description |
|---|---|---|---|
| `model` | string | `"models/one_beacon.pt"` | Path to stage-1 YOLO beacon model |
| `crop_model` | string | `"models/best_crop.pt"` | Path to stage-2 lit-area isolation model |
| `conf` | float | `0.5` | Detection confidence threshold |
| `display` | bool | `false` | Show OpenCV window in ROS live mode |
| `true_dist` | float | `0.4826` | Known ground-truth distance to target in metres (ROS mode) |
| `save` | bool | `false` | Write annotated output video alongside input (video modes) |
| `log` | bool | `false` | Write per-frame CSV detection log |
| `save_crops` | bool | `false` | Save each beacon bounding-box crop as a PNG for post-run color classification analysis |
| `save_det_images` | bool | `false` | Save a padded bbox crop for every detection, with color and blink status in the filename (ROS live mode only) |
| `target_color` | string \| null | `null` | Expected beacon color (`"red"`, `"green"`, `"blue"`, etc.). When set, a `target_match` column is added to the CSV log. |
| `target_blinking` | bool \| null | `null` | Expected blink state (`true` = blinking, `false` = steady). Combined with `target_color` for the `target_match` column. |
| `video` | string \| null | `null` | Path to video file for video-only mode |
| `ros_video` | string \| null | `null` | Path to video file for video+ROS mode. Takes priority over `video`. |
| `topics` | object | see below | ROS topic name overrides |
| `camera` | object | see below | Camera intrinsics and mount geometry |
| `paths` | object | see below | Filesystem paths for scripts, datasets, logs |
| `isaac` | object | see below | Isaac Sim USD prim paths |
| `buoys` | object | see below | Buoy physical properties and world positions |
| `drone_spawn` | object | see below | Drone initial position and orientation in simulation |
| `px4` | object | see below | PX4 flight controller parameters |
| `labeler` | object | see below | Dataset labeler settings |

**`topics` object**

| Key | Default | Description |
|---|---|---|
| `camera_prefix` | `"/zed/zed_node"` | Prefix used to derive `image`, `camera_info`, and `depth` if those keys are absent |
| `image` | `"{prefix}/rgb/color/rect/image"` | Full topic path for the RGB image stream |
| `camera_info` | `"{prefix}/rgb/color/rect/camera_info"` | Full topic path for camera info (optional — intrinsics are set from the `camera` block) |
| `depth` | `"{prefix}/depth/depth_registered"` | Full topic path for the depth image |
| `drone_pose` | `"/mavros/local_position/pose"` | Drone local position + orientation (`PoseStamped`) |
| `gps_origin` | `"/mavros/global_position/gp_origin"` | GPS reference origin for ENU→GPS conversion (`GeoPointStamped`) |
| `detections_pub` | `"/seabird/beacon_detections"` | Topic detections are published to |

If `image`, `camera_info`, or `depth` are omitted from the config, they are derived automatically from `camera_prefix` using the ZED path convention. Set them explicitly when the camera driver does not follow that convention (e.g. `modal_config.json` sets `"image": "/hires_front_small_color"` and `"depth": "/tof_depth"`).

**`camera` object**

Matches the ZED 2i wide-lens parameters used in `seabird_config.py`. `fx`, `fy`, `cx`, `cy`, the body-to-camera rotation matrix, and the mount offset array are computed automatically by `load_config()` — do not add them to the file.

| Key | Default | Description |
|---|---|---|
| `focal_length_mm` | `2.1` | Lens focal length in millimetres |
| `h_aperture_mm` | `6.0` | Horizontal sensor size in millimetres |
| `v_aperture_mm` | `4.5` | Vertical sensor size in millimetres |
| `clipping_near` | `0.1` | Near clip plane in metres |
| `clipping_far` | `200.0` | Far clip plane in metres |
| `img_w` | `640` | Render/stream width in pixels |
| `img_h` | `480` | Render/stream height in pixels |
| `mount_offset_xyz` | `[0.30, 0.0, 0.05]` | Camera position relative to drone body frame in metres (FLU) |
| `pitch_deg` | `15.0` | Nose-down camera tilt in degrees |

**`paths` object**

| Key | Default | Description |
|---|---|---|
| `scripts_dir` | `"~/seabird/scripts"` | ROS scripts and Python modules |
| `dataset_dir` | `"~/seabird_dataset"` | Training data and debug frames |
| `logs_dir` | `"~/seabird/logs"` | Runtime logs |
| `px4_dir` | `"~/seabird/PX4-Autopilot"` | PX4 firmware checkout |
| `assets_dir` | `"~/seabird/assets"` | USD assets and meshes |

**`isaac` object**

| Key | Default | Description |
|---|---|---|
| `drone_prim_path` | `"/World/Iris"` | Root USD prim for the drone |
| `drone_body_path` | `"/World/Iris/body"` | Drone body prim |
| `camera_prim_path` | `"/World/Iris/body/front_cam"` | Camera prim |

**`buoys` object**

| Key | Default | Description |
|---|---|---|
| `radius_m` | `0.2286` | Physical buoy radius in metres (18" diameter) |
| `classes` | `{"red_buoy": 0, "green_buoy": 1, "blue_buoy": 2}` | YOLO class index mapping |
| `positions` | see `beacon_config.json` | Known world-frame XYZ positions for each buoy (bbox centre) |

**`drone_spawn` object**

| Key | Default | Description |
|---|---|---|
| `position` | `[0.0, -8.0, 2.5]` | Spawn position in Isaac world frame (metres) |
| `quat_wxyz` | `[0.707, 0.0, 0.0, -0.707]` | Spawn orientation as `[w, x, y, z]` quaternion (facing −Y toward buoys) |

**`px4` object**

| Key | Default | Description |
|---|---|---|
| `connection_type` | `"tcpin"` | MAVLink connection type passed to MAVSDK/pymavlink |
| `takeoff_alt_m` | `1.25` | `MIS_TAKEOFF_ALT` parameter value in metres |

**`labeler` object**

| Key | Default | Description |
|---|---|---|
| `save_every_n` | `10` | Save a labelled frame every N render frames |
| `max_frames` | `2000` | Stop after this many saved frames |
| `debug_mode` | `true` | Draw bounding boxes on debug images |
| `min_bbox_px` | `6` | Minimum bounding-box side length in pixels to save |

**Run mode selection**

| `ros_video` | `video` | Mode |
|---|---|---|
| path string | any | Video file + ROS publishing |
| null | path string | Video file only, no ROS |
| null | null | ROS live camera mode |

**Example — remap to a second ZED camera**

```json
{
  "model": "models/one_beacon.pt",
  "crop_model": "models/best_crop.pt",
  "conf": 0.5,
  "display": false,
  "true_dist": 0.4826,
  "save": false,
  "log": false,
  "video": null,
  "ros_video": null,
  "topics": {
    "camera_prefix":  "/zed2/zed_node",
    "drone_pose":     "/mavros/local_position/pose",
    "gps_origin":     "/mavros/global_position/gp_origin",
    "detections_pub": "/seabird/beacon_detections_cam2"
  },
  "camera": {
    "focal_length_mm": 2.1,
    "h_aperture_mm":   6.0,
    "v_aperture_mm":   4.5,
    "clipping_near":   0.1,
    "clipping_far":  200.0,
    "img_w":          640,
    "img_h":          480,
    "mount_offset_xyz": [0.30, 0.0, 0.05],
    "pitch_deg":       15.0
  }
}
```

### Detection image output (save_det_images)

When `save_det_images` is `true`, a padded bounding-box crop is saved for every detection in ROS live mode. Images are written to `~/seabird_dataset/beacon_debug/beacon_det_images/` with filenames that encode the detection result:

```
det_f000030_t01_blue_conf65_blink0.25hz.png   ← blinking confirmed
det_f000031_t01_blue_conf72_steady.png         ← confirmed not blinking
det_f000032_t01_blue_conf58.png                ← blink status still deciding
```

Fields: `det_f{frame}_t{tracking_id}_{color}_conf{det_conf%}[_blink{hz}hz|_steady]`

The crop is taken from the clean (un-annotated) frame with 20 px padding on each side.

### Target matching (target_color / target_blinking)

Setting `target_color` and/or `target_blinking` adds a `target_match` column to the CSV log. Each row gets `True` if the detection satisfies all non-null criteria, `False` otherwise. Leaving both `null` omits the column entirely (empty string).

| `target_color` | `target_blinking` | `target_match = True` when |
|---|---|---|
| `"blue"` | `true` | color is blue AND confirmed blinking |
| `"blue"` | `null` | color is blue (any blink state) |
| `null` | `true` | confirmed blinking (any color) |
| `null` | `null` | *(column left empty)* |

`target_blinking: true` requires `is_blinking = True` — rows where blink is still undecided (`None`) count as `False`.

### CSV log columns (ROS live mode)

`timestamp, frame, color, color_confidence, intensity, vote_red, vote_green, vote_blue, vote_other, det_confidence, x1, y1, x2, y2, tracking_id, pos3d_x, pos3d_y, pos3d_z, blink_is_blinking, blink_hz, blink_phase[, target_match]`

`target_match` is only present when at least one of `target_color` / `target_blinking` is set. Log files are written to `~/seabird_dataset/beacon_debug/beacon_log_<YYYYMMDD_HHMMSS>.csv` and flushed after every row.

### How topic and intrinsic overrides work

`BeaconCamera` normally reads topic names from module-level constants in `beacon_camera.py` and reads intrinsics (`FX`, `FY`, `CX`, `CY`, `IMG_W`, `IMG_H`) from `seabird_config.py`. `beacon_detector_config.py` creates a subclass of `BeaconCamera` at runtime via `_make_beacon_camera(topics, cfg_camera)`, capturing all config values by closure and overriding the topic subscriptions. Camera intrinsics are applied immediately when `open()` is called — no `camera_info` message is required. Neither `beacon_camera.py` nor `seabird_config.py` is modified.

---

## blink_detector.py

Standalone module — no ROS or OpenCV dependency. Imported directly by `beacon_detector.py` and `batch_detect.py`.

### BlinkDetector

Maintains an 8-second rolling window of `(timestamp, color, intensity, color_conf)` samples and estimates whether the beacon is blinking and at what frequency.

```python
detector = BlinkDetector()
result = detector.update(ts, color, intensity, color_conf)
# result: {"is_blinking": True|False|None, "blink_color": str, "blink_hz": float|None, "phase": "on"|"off"|"unknown"}
```

`color_conf` is the `color_confidence` value from `classify_beacon_color`. It is used to filter out low-confidence non-blue readings (e.g. a dimming blue beacon misclassified as red) that would otherwise force the detector into the wrong color mode.

`is_blinking` has three states:
- `None` — not enough data yet (window < 4 s)
- `False` — confirmed not blinking
- `True` — confirmed blinking at `blink_hz` Hz

### Algorithm

**Red / Green beacons:** A rising edge is a transition from `blue` (beacon off, housing visible) to the signal color (beacon on). Only non-blue readings with `color_conf ≥ 0.10` are counted when selecting the dominant signal color — this prevents low-confidence noise from a dimming blue beacon triggering the wrong detection path.

**Blue beacons:** On/off state is determined from detection presence and absence rather than intensity oscillations. When the beacon turns off, YOLO stops detecting it entirely; the resulting gap in the sample stream is the primary blink signal. A synthetic `_off_` marker is injected whenever two consecutive samples are more than `_BLINK_GAP_OFF_SEC` (0.5 s) apart, making the off period visible to the rising-edge detector. `unknown` samples (YOLO detected but color classification failed) are also treated as off. If no gaps or unknowns appear in the window, the detector falls back to intensity oscillation detection.

Guards applied before declaring `True`:

| Guard | Purpose |
|---|---|
| Minimum data span (4 s) | Avoids decisions on too little history |
| Minimum rising edges (3 for blue, 2 for red/green) | Requires at least one complete blink cycle |
| Duty-cycle check (non-blue, 2-edge case) | Rejects solid beacons whose color-classification noise produces exactly 2 spurious edges while `on_fraction > 65%` |
| Max inter-onset interval (5.0 s blue / 5.5 s color) | Rejects windows where a YOLO detection gap swallows a full cycle |

### Key constants

| Constant | Value | Meaning |
|---|---|---|
| `_BLINK_WINDOW_SEC` | 8.0 s | Rolling window length |
| `_BLINK_MIN_DATA_SEC` | 4.0 s | Minimum history before deciding |
| `_BLINK_HZ_RANGE` | 0.2–2.0 Hz | Valid blink frequency range |
| `_BLINK_MIN_EDGE_GAP` | 0.20 s | Debounce: minimum gap between rising edges |
| `_BLINK_GAP_OFF_SEC` | 0.5 s | Detection gap longer than this injects an off marker |
| `_BLINK_COLOR_CONF_MIN` | 0.10 | Minimum `color_conf` for a non-blue reading to count as signal |
| `_BLINK_MAX_IOI_SEC` | 5.0 s | Max inter-onset interval for blue beacons |
| `_BLINK_MAX_IOI_SEC_COLOR` | 5.5 s | Max inter-onset interval for red/green (absorbs YOLO detection gaps) |

### Helper

```python
_get_blink_detector(tracking_id: int) -> BlinkDetector
```

Returns the `BlinkDetector` for a given YOLO tracking ID, creating one on first call. Used by the ROS live mode to maintain per-track state across frames.

---

## beacon_camera.py

ROS2 node that wraps camera subscriptions, depth synchronization, drone pose, and GPS origin. Imported by `beacon_detector.py` inside `_import_ros()` so ROS packages are never loaded unless ROS mode is actually invoked.

### BeaconCamera(Node)

```python
cam = BeaconCamera(topic_prefix="/zed/zed_node")
```

**Lifecycle**

| Method | Description |
|---|---|
| `open()` | Subscribe to RGB+depth image topics, pose, and GPS. Used for live camera mode. |
| `open_for_video()` | Minimal setup for video-file mode: create publisher and subscribe to pose + GPS only. |
| `close()` | Mark node as closed. |
| `grab()` | Spin once and return `True` if a new synchronized frame arrived. |
| `enable_detection(model_path)` | Start `YoloDetector` with object tracking on the live RGB stream. |

**Data accessors**

| Method | Returns |
|---|---|
| `get_rgb()` | Latest BGR frame as `np.ndarray`, or `None` |
| `get_depth()` | Latest float32 depth map, or `None` |
| `get_drone_pose()` | `(pos_xyz, quat_wxyz)` numpy arrays, or `(None, None)` |
| `get_gps_origin()` | `(lat, lon, alt)` tuple, or `None` |
| `get_detections()` | List of `Detection` objects from `YoloDetector` |

**Subscribed topics**

| Topic | Type | Purpose |
|---|---|---|
| `image` topic | `sensor_msgs/Image` | RGB frames (synced with depth) |
| `depth` topic | `sensor_msgs/Image` | Depth map — any encoding (see below) |
| `drone_pose` topic | `geometry_msgs/PoseStamped` | Drone ENU position + quaternion |
| `gps_origin` topic | `geographic_msgs/GeoPointStamped` | GPS reference origin |

Topic names come from the config file (see [How topic and intrinsic overrides work](#how-topic-and-intrinsic-overrides-work)). The `camera_info` subscription is not required — intrinsics are set from config values when `open()` is called.

RGB and depth frames are synchronized with `message_filters.ApproximateTimeSynchronizer` (50 ms slop).

**Depth decoding**

The depth callback decodes the raw message based on `encoding`:

| Encoding | dtype | Scale |
|---|---|---|
| `32FC1` | `float32` | metres, no scaling |
| `16UC1` | `uint16` | × 0.001 → metres |
| `8UC1` / other | `uint8` | raw value, no scaling |

If the decoded depth map has a different resolution than the RGB frame (e.g. 240×180 ToF vs 640×480 RGB), it is resized to match using `cv2.INTER_NEAREST` to preserve depth values without interpolation.

---

## batch_detect.py

Headless batch processor for running detection over multiple video files without opening any display window. Results are written to a timestamped CSV and a human-readable summary text file.

### Usage

```
python3 batch_detect.py video1.mp4 video2.mp4 ...
python3 batch_detect.py -m models/one_beacon.pt -cm models/best_crop.pt videos/*.mp4
python3 batch_detect.py --output-dir /path/to/logs video1.mp4
```

**Flags**

| Flag | Default | Description |
|---|---|---|
| `--model / -m` | `models/one_beacon.pt` | Stage-1 YOLO beacon model |
| `--crop-model / -cm` | `models/best_crop.pt` | Stage-2 lit-area model |
| `--conf / -c` | `0.5` | Detection confidence threshold |
| `--output-dir / -o` | directory of first video | Where to write output files |

### Output files

Both files are written to `--output-dir` with a `YYYYMMDD_HHMMSS` timestamp suffix.

**`batch_detections_<ts>.csv`** — one row per detection per frame across all videos.

Columns: `video, timestamp, frame, color, color_confidence, intensity, is_blinking, blink_hz, blink_phase, vote_red, vote_green, vote_blue, vote_other, det_confidence, x1, y1, x2, y2`

The `timestamp` column is the video presentation timestamp in seconds (`CAP_PROP_POS_MSEC / 1000`), not wall-clock time, so blink frequency estimates match the video's actual frame rate regardless of decode speed.

**`batch_summary_<ts>.txt`** — human-readable per-video breakdown printed to stdout and written to disk. Includes frame count, detection rate, color breakdown, and blink statistics.

### Design notes

- Both YOLO models are loaded once and reused across all videos.
- Each video gets its own `BlinkDetector` instance so timing windows do not bleed between files.
- A `VideoStats` accumulator tracks per-video color counts and blink state counts for the summary.

---

## Timestamp note (video modes)

When processing video files, `cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0` is used as the blink detector timestamp rather than `time.time()`. This ensures the blink frequency estimate reflects the video's actual frame timing even when frames are decoded faster or slower than real time. In live ROS mode `time.time()` is used, which is correct for a real camera stream.
