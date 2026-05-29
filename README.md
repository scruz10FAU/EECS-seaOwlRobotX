# Beacon Detection Scripts

Detection pipeline for colored, optionally blinking, beacon lights mounted on a drone or buoy. Uses a two-stage YOLO model followed by HSV color classification and a rolling-window blink estimator.

---

## File Overview

| File | Role |
|---|---|
| `beacon_detector.py` | Entry point. Color classification pipeline, video and ROS live modes. |
| `blink_detector.py` | `BlinkDetector` class — rolling-window blink frequency estimator. |
| `beacon_camera.py` | `BeaconCamera` ROS2 node — ZED camera subscriptions and pose/GPS callbacks. |
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

### CSV log columns

`timestamp, frame, color, color_confidence, intensity, vote_red, vote_green, vote_blue, vote_other, det_confidence, x1, y1, x2, y2, tracking_id, pos3d_x, pos3d_y, pos3d_z`

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

## blink_detector.py

Standalone module — no ROS or OpenCV dependency. Imported directly by `beacon_detector.py` and `batch_detect.py`.

### BlinkDetector

Maintains a 4-second rolling window of `(timestamp, color, intensity)` samples and estimates whether the beacon is blinking and at what frequency.

```python
detector = BlinkDetector()
result = detector.update(ts, color, intensity)
# result: {"is_blinking": True|False|None, "blink_color": str, "blink_hz": float|None, "phase": "on"|"off"|"unknown"}
```

`is_blinking` has three states:
- `None` — not enough data yet (window < 2 s)
- `False` — confirmed not blinking
- `True` — confirmed blinking at `blink_hz` Hz

### Algorithm

**Red / Green beacons:** A rising edge is a transition from `blue` (beacon off, housing visible) to the signal color (beacon on). Edge timing gives the blink period.

**Blue beacons:** Edges are detected from intensity oscillations relative to the window mean. Requires a minimum peak-to-peak intensity swing (`_BLINK_INTENSITY_MIN_SWING = 0.05`) to rule out ambient noise.

Guards applied before declaring `True`:

| Guard | Purpose |
|---|---|
| Minimum data span (2 s) | Avoids decisions on too little history |
| Minimum rising edges (3 for blue, 2 for red/green) | Requires at least one complete blink cycle |
| Duty-cycle check (non-blue, 2-edge case) | Rejects solid beacons whose color-classification noise produces exactly 2 spurious edges while `on_fraction > 65%` |
| Max inter-onset interval (2.0 s blue / 2.5 s color) | Rejects windows where a YOLO detection gap swallows a full cycle |
| Max consecutive off duration (1.0 s, blue only) | Rejects slow intensity drifts |
| On/off mean separation (blue only) | Ensures the signal has real amplitude, not noise around the mean |

### Key constants

| Constant | Value | Meaning |
|---|---|---|
| `_BLINK_WINDOW_SEC` | 4.0 s | Rolling window length |
| `_BLINK_MIN_DATA_SEC` | 2.0 s | Minimum history before deciding |
| `_BLINK_HZ_RANGE` | 0.5–2.0 Hz | Valid blink frequency range |
| `_BLINK_MIN_EDGE_GAP` | 0.20 s | Debounce: minimum gap between rising edges |
| `_BLINK_MAX_IOI_SEC` | 2.0 s | Max inter-onset interval for blue beacons |
| `_BLINK_MAX_IOI_SEC_COLOR` | 2.5 s | Max inter-onset interval for red/green (absorbs YOLO detection gaps) |

### Helper

```python
_get_blink_detector(tracking_id: int) -> BlinkDetector
```

Returns the `BlinkDetector` for a given YOLO tracking ID, creating one on first call. Used by the ROS live mode to maintain per-track state across frames.

---

## beacon_camera.py

ROS2 node that wraps ZED camera subscriptions, depth synchronization, drone pose, and GPS origin. Imported by `beacon_detector.py` inside `_import_ros()` so ROS packages are never loaded unless ROS mode is actually invoked.

### BeaconCamera(Node)

```python
cam = BeaconCamera(topic_prefix="/zed/zed_node")
```

**Lifecycle**

| Method | Description |
|---|---|
| `open()` | Subscribe to RGB+depth image topics, camera info, pose, GPS. Used for live camera mode. |
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
| `{prefix}/rgb/color/rect/image` | `sensor_msgs/Image` | RGB frames (synced with depth) |
| `{prefix}/depth/depth_registered` | `sensor_msgs/Image` | Float32 depth map |
| `{prefix}/rgb/color/rect/camera_info` | `sensor_msgs/CameraInfo` | Triggers intrinsics initialization (one-shot) |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | Drone ENU position + quaternion |
| `/mavros/global_position/gp_origin` | `geographic_msgs/GeoPointStamped` | GPS reference origin |

RGB and depth frames are synchronized with `message_filters.ApproximateTimeSynchronizer` (50 ms slop).

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
