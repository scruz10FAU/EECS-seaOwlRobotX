# Beacon Detection Scripts

Detection pipeline for colored, optionally blinking, beacon lights mounted on a drone or buoy. Uses a two-stage YOLO model followed by HSV color classification and a rolling-window blink estimator.

---

## File Overview

| File | Role |
|---|---|
| `beacon_detector.py` | Entry point. Color classification pipeline, video and ROS live modes. |
| `beacon_detector_config.py` | JSON-configured version of `beacon_detector.py`. Loads all arguments and ROS topic names from a JSON config file instead of command-line flags. Works with any camera — not ZED-specific. |
| `beacon_config.json` | Default configuration file for `beacon_detector_config.py`. |
| `beacon_config_physical.json` | Config for physical drone (ModalAI VOXL2 / Starling 2). Uses modal camera and ToF depth topics. |
| `beacon_config_sim.json` | Config for Isaac Sim. Uses `/iris_0/front_cam/*` topics; pose and GPS are disabled. |
| `modal_config.json` | Reference config for Modal AI VOXL hardware topic names. |
| `sweep_lawnmower.py` | Autonomous boustrophedon (lawnmower) flight mission. Searches a 5 ft × 5 ft area. |
| `data_recorder.py` | ROS2 node that saves camera frames and detection labels to disk during a mission. |
| `start_seabird_beacon.sh` | Mission launcher. Starts the detector, sweep, and recorder as tagged, logged child processes. |
| `blink_detector.py` | `BlinkDetector` class — rolling-window blink frequency estimator. |
| `beacon_camera.py` | `BeaconCamera` ROS2 node — camera image subscriptions, depth decoding, and pose/GPS callbacks. |
| `batch_detect.py` | Headless batch processor — runs detection over multiple video files and writes a CSV + summary. |

---

## start_seabird_beacon.sh

Mission launcher that starts all three nodes as tagged, color-coded child processes with unified terminal output and per-component log files.

**Run from the directory containing all component scripts:**

```bash
./start_seabird_beacon.sh                                          # Isaac Sim (default)
DETECTOR_ARGS="--config beacon_config_physical.json" \
  ./start_seabird_beacon.sh                                        # physical drone
DETECTOR_ARGS="--no-yolo" ./start_seabird_beacon.sh               # HSV-only fallback
MUTE=SWEEP ./start_seabird_beacon.sh                              # suppress SWEEP output
```

**Components started (in order):**

| Tag | Script | Color |
|---|---|---|
| `DETECTOR` | `beacon_detector_config.py` | green |
| `SWEEP` | `sweep_lawnmower.py` | magenta |
| `RECORDER` | `data_recorder.py` | cyan |

**Logs** are written to `logs/<timestamp>/` in the working directory — one `.log` and one `.stderr` per component, plus a `launch.log` recording start times and exit codes.

**Stop:** Ctrl-C. The trap sends SIGTERM to all children, waits 3 s, then SIGKILLs any survivors.

### Safety warning

RC takeover requires `COM_RC_OVERRIDE` to be non-zero. Verify before every flight:

```
pxh> param show COM_RC_OVERRIDE   # must be non-zero (3 = offboard + auto)
```

Note: `NAV_RCL_ACT=0` disables the RC-loss failsafe — the mission continues if the RC signal drops. Use Ctrl-C in this terminal to abort.

---

## sweep_lawnmower.py

Flies a boustrophedon (lawnmower) pattern over a **5 ft × 5 ft search area** (1.524 m × 1.524 m), centred on the drone spawn origin. Rows run east–west, advancing northward by `LAWN_ROW_SPACING_M` per row.

### Search area constants

```python
LAWN_ROW_SPACING_M = 0.5     # 0.5 m gap between E-W rows (~4 rows in the 5 ft area)
LAWN_NORTH_M       =  0.762  # northern edge  (+2.5 ft from origin)
LAWN_SOUTH_M       = -0.762  # southern edge  (−2.5 ft from origin)
LAWN_EAST_M        =  0.762  # eastern edge   (+2.5 ft from origin)
LAWN_WEST_M        = -0.762  # western edge   (−2.5 ft from origin)
```

To change the search area, edit these four constants. All values are in metres (NED frame, relative to the spawn origin).

### Mission configuration

| Constant | Value | Description |
|---|---|---|
| `TAKEOFF_ALT_M` | `5.0` | Takeoff altitude in metres |
| `WAYPOINT_TOL_M` | `2.5` | Distance tolerance to declare a waypoint reached |
| `WAYPOINT_TIMEOUT` | `60.0` | Max seconds to spend flying toward any single waypoint |
| `HOVER_STABILIZE_S` | `8.0` | Seconds to hover after takeoff before starting the sweep |
| `TARGET_COLORS` | `{"red","green","blue"}` | Mission ends early when all colors are detected |
| `MAVSDK_ADDRESS` | `"udp://:14540"` | MAVSDK connection string |

### Behaviour

- Subscribes to `/seabird/buoy_detections` in a background thread. Stops the sweep as soon as all `TARGET_COLORS` are detected.
- Publishes the executed flight path on `/seabird/flight_path` (`nav_msgs/Path`) and `/seabird/path_marker` (`visualization_msgs/Marker`) for RViz2.
- On completion (or early exit), returns to launch and lands.

### PX4 parameters (set once)

```
pxh> param set MPC_XY_CRUISE 2.0
pxh> param set MPC_XY_VEL_MAX 3.0
pxh> param set MPC_Z_VEL_MAX_UP 1.5
pxh> param set MPC_Z_VEL_MAX_DN 1.0
pxh> param set SYS_HAS_MAG 0
pxh> param set COM_ARM_MAG_STR 0
pxh> param set EKF2_ABL_LIM 5.0
pxh> param save
```

---

## beacon_detector_config.py

A drop-in alternative to `beacon_detector.py` that reads all runtime parameters from a JSON file instead of command-line flags. Works with any camera — not ZED-specific. Topic names, model paths, camera intrinsics, and detection thresholds are all driven by the config file.

Three changes were made relative to the original `beacon_detector_config.py` to support non-ZED cameras and the Seabird mission:

1. **`sys.path`** points to `dirname(__file__)` so the script can be run from any directory.
2. **Null-gated subscriptions** — `drone_pose` and `gps_origin` topics are skipped when set to `null` in the config (required for Isaac Sim, which has no pose/GPS topics).
3. **Dual publish** — every detection is also published to `buoy_pub` (`/seabird/buoy_detections`) so `sweep_lawnmower.py` receives it without modification.

### Usage

```bash
python3 beacon_detector_config.py                                  # uses beacon_config.json
python3 beacon_detector_config.py --config beacon_config_physical.json
python3 beacon_detector_config.py --config beacon_config_sim.json
```

The run mode (ROS live / video-only / video+ROS) is determined by the `video` and `ros_video` fields in the config.

---

## Config files

### beacon_config_physical.json — Physical drone (ModalAI Starling 2 / VOXL2)

Uses the modal camera driver and ToF depth sensor topics.

| Field | Value |
|---|---|
| `topics.image` | `/hires_front_small_color` |
| `topics.camera_info` | `/hires_front_small_color/camera_info` |
| `topics.depth` | `/tof_depth` |
| `topics.drone_pose` | `/vvhub_body_wrt_local/pose` |
| `topics.gps_origin` | `/fmu/out/vehicle_gps_position` |

**Prerequisites before launching:**
- Modal camera driver running (publishes `/hires_front_small_color` and `/tof_depth`)
- VVHub running (publishes `/vvhub_body_wrt_local/pose`)
- micro-ROS agent running (bridges PX4 → `/fmu/out/vehicle_gps_position`)

### beacon_config_sim.json — Isaac Sim

Uses the Pegasus Simulator front camera. Pose and GPS are disabled (`null`) since the simulation does not provide them.

| Field | Value |
|---|---|
| `topics.image` | `/iris_0/front_cam/rgb` |
| `topics.camera_info` | `/iris_0/front_cam/camera_info` |
| `topics.depth` | `/iris_0/front_cam/depth` |
| `topics.drone_pose` | `null` |
| `topics.gps_origin` | `null` |

**Prerequisites before launching:**
- Isaac Sim running with the marina scene
- Drone spawned via `init_scene.py` (`[init] Done`)
- PX4 SITL running (`make px4_sitl none_iris`)

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
| `--save-crops / -sc` | off | Save each beacon bounding-box crop as a PNG |
| `--save-det-images / -sdi` | off | Save each detection as a padded image with color and blink status in the filename |
| `--target-color / -tc` | `None` | Expected beacon color (`blue`, `red`, etc.) |
| `--target-blinking / -tb` | `None` | Expected blink state (`true` or `false`) |

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

Maintains an 8-second rolling window of `(timestamp, color, intensity, color_conf)` samples and estimates whether the beacon is blinking and at what frequency.

```python
detector = BlinkDetector()
result = detector.update(ts, color, intensity, color_conf)
# result: {"is_blinking": True|False|None, "blink_color": str, "blink_hz": float|None, "phase": "on"|"off"|"unknown"}
```

`color_conf` is the `color_confidence` value from `classify_beacon_color`. It is used to filter out low-confidence non-blue readings that would otherwise force the detector into the wrong color mode.

`is_blinking` has three states:
- `None` — not enough data yet (window < 4 s)
- `False` — confirmed not blinking
- `True` — confirmed blinking at `blink_hz` Hz

### Algorithm

**Red / Green beacons:** A rising edge is a transition from `blue` (beacon off, housing visible) to the signal color (beacon on). Only non-blue readings with `color_conf ≥ 0.10` are counted when selecting the dominant signal color.

**Blue beacons:** On/off state is determined from detection presence and absence rather than intensity oscillations. When the beacon turns off, YOLO stops detecting it entirely; the resulting gap in the sample stream is the primary blink signal. A synthetic `_off_` marker is injected whenever two consecutive samples are more than `_BLINK_GAP_OFF_SEC` (0.5 s) apart.

### Key constants

| Constant | Value | Meaning |
|---|---|---|
| `_BLINK_WINDOW_SEC` | `8.0 s` | Rolling window length |
| `_BLINK_MIN_DATA_SEC` | `4.0 s` | Minimum history before deciding |
| `_BLINK_HZ_RANGE` | `0.2–2.0 Hz` | Valid blink frequency range |
| `_BLINK_MIN_EDGE_GAP` | `0.20 s` | Debounce: minimum gap between rising edges |
| `_BLINK_GAP_OFF_SEC` | `0.5 s` | Detection gap longer than this injects an off marker |
| `_BLINK_COLOR_CONF_MIN` | `0.10` | Minimum `color_conf` for a non-blue reading to count as signal |
| `_BLINK_MAX_IOI_SEC` | `5.0 s` | Max inter-onset interval for blue beacons |
| `_BLINK_MAX_IOI_SEC_COLOR` | `5.5 s` | Max inter-onset interval for red/green |

### Helper

```python
_get_blink_detector(tracking_id: int) -> BlinkDetector
```

Returns the `BlinkDetector` for a given YOLO tracking ID, creating one on first call.

---

## beacon_camera.py

ROS2 node that wraps camera subscriptions, depth synchronization, drone pose, and GPS origin. Imported by `beacon_detector.py` inside `_import_ros()` so ROS packages are never loaded unless ROS mode is actually invoked.

### BeaconCamera(Node)

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

**Depth decoding**

| Encoding | dtype | Scale |
|---|---|---|
| `32FC1` | `float32` | metres, no scaling |
| `16UC1` | `uint16` | × 0.001 → metres |
| `8UC1` / other | `uint8` | raw value, no scaling |

If the decoded depth map has a different resolution than the RGB frame, it is resized to match using `cv2.INTER_NEAREST`.

RGB and depth frames are synchronized with `message_filters.ApproximateTimeSynchronizer` (50 ms slop).

---

## batch_detect.py

Headless batch processor for running detection over multiple video files without opening any display window.

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

**`batch_detections_<ts>.csv`** — one row per detection per frame across all videos.

Columns: `video, timestamp, frame, color, color_confidence, intensity, is_blinking, blink_hz, blink_phase, vote_red, vote_green, vote_blue, vote_other, det_confidence, x1, y1, x2, y2`

**`batch_summary_<ts>.txt`** — human-readable per-video breakdown. Includes frame count, detection rate, color breakdown, and blink statistics.

---

## Timestamp note (video modes)

When processing video files, `cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0` is used as the blink detector timestamp rather than `time.time()`. This ensures the blink frequency estimate reflects the video's actual frame timing even when frames are decoded faster or slower than real time. In live ROS mode `time.time()` is used, which is correct for a real camera stream.
