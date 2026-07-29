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
| `sweep_lawnmower.py` | Autonomous boustrophedon (lawnmower) flight mission. Searches a fixed rectangular area. |
| `sweep_rrt.py` | Autonomous RRT beacon search. Explores a configurable radius from takeoff, hovering to verify blink status on each new detection. |
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
| `SWEEP` | `sweep_rrt.py` | magenta |
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

- Subscribes to `/seabird/beacon_detections` in a background thread. Stops the sweep as soon as all `TARGET_COLORS` are detected.
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

## sweep_rrt.py

Explores the search area using a **Rapidly-exploring Random Tree (RRT)**. The drone grows a tree of visited positions outward from takeoff within a configurable radius. When a new beacon color appears on `/seabird/beacon_detections` the drone hovers in place and waits for `blink_is_blinking` to become `True` or `False` before continuing. The search always runs until the node limit is reached — it does not stop early when all target colors are found, because multiple beacons of the same color may exist.

### Configuration (top of file)

| Constant | Default | Description |
|---|---|---|
| `MAX_SEARCH_RADIUS_M` | `10.0` | Hard boundary — no node placed beyond this from takeoff |
| `RRT_STEP_M` | `1.5` | Max edge length per tree extension |
| `RRT_GOAL_BIAS` | `0.08` | Probability of biasing sample toward least-explored sector |
| `RRT_MAX_NODES` | *(derived)* | Computed as `max(20, int(2π × R² / S²))` — enough to cover the disc ~twice |
| `BLINK_VERIFY_TIMEOUT_S` | `20.0` | Max hover time per beacon before resuming search |
| `TAKEOFF_ALT_M` | `5.0` | Flight altitude in metres |

`RRT_MAX_NODES` is derived automatically from `MAX_SEARCH_RADIUS_M` and `RRT_STEP_M` — reduce the radius to reduce the search time proportionally.

**Example values:**

| `MAX_SEARCH_RADIUS_M` | `RRT_MAX_NODES` | Max flight distance |
|---|---|---|
| 3.0 m | 25 | 38 m |
| 5.0 m | 69 | 104 m |
| 10.0 m | 279 | 418 m |

### Behaviour

- **RRT sampling**: uniform random sample inside the search disc; 8% of samples are biased toward the least-explored sector (opposite of the current node centroid) so the tree spreads outward rather than clustering near the start.
- **Beacon detection**: checks for new colors after arriving at each node. The same color can trigger verification multiple times at different locations.
- **Blink verification**: hovers at the detection node and polls every 0.25 s until `blink_is_blinking` is non-`None` or `BLINK_VERIFY_TIMEOUT_S` elapses. Records the result either way and clears the pending entry so the color can re-trigger at a new location.
- **No obstacle avoidance**: flies straight-line paths between nodes. Suitable for open outdoor areas with surface-level obstacles below flight altitude.

### RViz2 topics

| Topic | Type | Content |
|---|---|---|
| `/seabird/flight_path` | `nav_msgs/Path` | Visited positions |
| `/seabird/path_marker` | `visualization_msgs/Marker` (LINE_STRIP) | Same path as a line |
| `/seabird/rrt_tree` | `visualization_msgs/Marker` (LINE_LIST) | Every RRT edge |
| `/seabird/rrt_nodes` | `visualization_msgs/Marker` (SPHERE_LIST) | Every RRT node |

---

## beacon_detector_config.py

A drop-in alternative to `beacon_detector.py` that reads all runtime parameters from a JSON file instead of command-line flags. Works with any camera — not ZED-specific. Topic names, model paths, camera intrinsics, and detection thresholds are all driven by the config file.

Three changes were made relative to the original `beacon_detector_config.py` to support non-ZED cameras and the Seabird mission:

1. **`sys.path`** points to `dirname(__file__)` so the script can be run from any directory.
2. **Null-gated subscriptions** — `drone_pose` and `gps_origin` topics are skipped when set to `null` in the config (required for Isaac Sim, which has no pose/GPS topics).
3. **ArUco ground truth** — when `aruco.enabled` is `true`, runs ArUco detection once per frame and publishes distance + pose to `/seabird/aruco_ground_truth` and writes ground-truth columns to the CSV log.

### Usage

```bash
python3 beacon_detector_config.py                                  # uses beacon_config.json
python3 beacon_detector_config.py --config beacon_config_physical.json
python3 beacon_detector_config.py --config beacon_config_sim.json
```

The run mode (ROS live / video-only / video+ROS) is determined by the `video` and `ros_video` fields in the config.

### Config file schema

All keys are optional — omitted keys fall back to their defaults.

**Top-level fields**

| Field | Default | Description |
|---|---|---|
| `model` | `models/one_beacon.pt` | Stage-1 YOLO model path |
| `crop_model` | `models/best_crop.pt` | Stage-2 lit-area model path |
| `conf` | `0.5` | Detection confidence threshold |
| `display` | `false` | Show OpenCV window |
| `video` | `null` | Path to a local video file (video-only mode) |
| `ros_video` | `null` | Path to a local video file (video + ROS publish mode) |
| `save` | `false` | Write annotated output video |
| `log` | `false` | Write per-frame CSV detection log |
| `save_crops` | `false` | Save lit-area crop PNG per detection |
| `save_det_images` | `false` | Save detection bbox PNG per detection |
| `save_frames` | `false` | Save full unannotated frame per detection |
| `target_color` | `null` | Expected beacon color for log annotation |
| `target_blinking` | `null` | Expected blink state for log annotation |
| `true_dist` | `0.4826` | Known ground-truth distance (metres) written to log |

**`topics` section — ROS topic names**

| Field | Default | Description |
|---|---|---|
| `topics.camera_prefix` | `/zed/zed_node` | Prefix used to derive image/depth/camera_info defaults. Override the three below explicitly to ignore this. |
| `topics.image` | `<prefix>/rgb/color/rect/image` | Camera image subscription |
| `topics.camera_info` | `<prefix>/rgb/color/rect/camera_info` | Camera info subscription |
| `topics.depth` | `<prefix>/depth/depth_registered` | Depth map subscription |
| `topics.drone_pose` | `/mavros/local_position/pose` | Drone pose subscription (`null` to disable) |
| `topics.gps_origin` | `/mavros/global_position/gp_origin` | GPS origin subscription (`null` to disable) |
| `topics.detections_pub` | `/seabird/beacon_detections` | Beacon detection publish topic |
| `topics.aruco_pub` | `/seabird/aruco_ground_truth` | ArUco ground-truth publish topic |

**`camera` section — intrinsics and mount**

| Field | Description |
|---|---|
| `focal_length_mm` | Lens focal length in mm (used for theoretical intrinsics when `calibration_file` is `null`) |
| `h_aperture_mm` / `v_aperture_mm` | Sensor aperture in mm |
| `img_w` / `img_h` | Frame resolution in pixels |
| `mount_offset_xyz` | Camera position relative to drone body frame in metres `[x, y, z]` |
| `pitch_deg` | Camera downward pitch angle in degrees |

**`detection` section**

| Field | Default | Description |
|---|---|---|
| `confirm_frames` | `3` | Consecutive frames required before publishing a new detection |
| `pub_cooldown_s` | `1.0` | Minimum seconds between publishes for the same beacon |
| `depth_min_m` / `depth_max_m` | `1.0` / `60.0` | Valid depth range for 3D position estimation |
| `min_area_frac` | `0.001` | Minimum detection area as a fraction of frame area |
| `stage2_conf` | `0.30` | Stage-2 model confidence threshold |
| `depth_source` | `"topic"` | Distance source: `"topic"` uses the depth ROS topic; `"bbox"` estimates distance from the bounding box height and drone altitude (no depth sensor required) |
| `beacon_height_m` | `0.3048` | Physical beacon height in metres (12 in) — used only when `depth_source` is `"bbox"` |
| `beacon_z_m` | `0.0` | Known beacon altitude in the ENU world frame (metres) — used to remove the vertical offset from the slant range when `depth_source` is `"bbox"` |
| `max_detections` | `null` | Stop after this many published detections. `null` = unlimited |
| `imgsz` | `640` | YOLO inference resolution in pixels. Should match model training size |
| `red_threshold` | `0.40` | Minimum fraction of lit pixels that must vote red, AND red must outscore green and blue |
| `winner_threshold` | `0.25` | Minimum fraction required for green or blue to be declared the winner |
| `red_hue_high` | `10` | Half-width of the wrap-around red hue band near 180°. Band covers `(180 − value)–180°`. Reduce to avoid classifying purple/blue hues as red |

**`aruco` section**

| Field | Default | Description |
|---|---|---|
| `enabled` | `false` | Enable per-frame ArUco ground-truth detection |
| `dictionary` | `"DICT_4X4_50"` | ArUco dictionary name |
| `marker_size_m` | `0.15` | Physical marker side length in metres |
| `calibration_file` | `null` | Path to a `.npz` camera calibration file (from `cv2.calibrateCamera`). If `null`, falls back to theoretical intrinsics derived from the `camera` section. |

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

**All flags**

| Flag | Short | Default | Description |
|---|---|---|---|
| `--model` | `-m` | `models/one_beacon.pt` | Stage-1 YOLO beacon localization model |
| `--crop-model` | `-cm` | `models/best_crop.pt` | Stage-2 lit-area isolation model |
| `--conf` | `-c` | `0.5` | Detection confidence threshold |
| `--display` | `-d` | off | Show OpenCV window (ROS and ROS-video modes) |
| `--video` | `-v` | `None` | Run on a local video file — no ROS, CV only |
| `--ros-video` | `-rv` | `None` | Run on a local video file and publish detections to ROS |
| `--save` | `-s` | off | Write annotated output video alongside the input (video modes) |
| `--log` | `-l` | off | Write per-frame CSV detection log |
| `--save-crops` | `-sc` | off | Save isolated lit-area crop for each detection as a PNG |
| `--save-det-images` | `-sdi` | off | Save each detection bbox (+ 20 px padding); filename encodes color, confidence, and blink status |
| `--save-frames` | `-sf` | off | Save full unannotated frame on every frame that contains a detection |
| `--target-color` | `-tc` | `None` | Expected beacon color (`blue`, `red`, `green`); adds `target_match` column to the CSV log |
| `--target-blinking` | `-tb` | `None` | Expected blink state (`true` or `false`); adds `target_blinking` column to the CSV log |
| `--true_dist` | `-td` | `0.4826` | Known ground-truth distance in metres (ROS live mode; written to log for calibration) |

### Image output directories

Three independent image-saving flags can be combined freely. All write to a directory derived from the input path (video modes) or `~/seabird_dataset/beacon_debug/` (ROS live mode). Images are only written on frames that contain at least one detection.

| Flag | Directory suffix | Contents |
|---|---|---|
| `--save-crops / -sc` | `_beacon_crops/` | Tight crop of the isolated lit area used for color classification |
| `--save-det-images / -sdi` | `_beacon_det_images/` | Detection bbox + 20 px padding; filename encodes color, confidence, blink |
| `--save-frames / -sf` | `_beacon_frames/` | Full unannoted frame |

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

**Red / Green beacons:** YOLO loses the beacon entirely when the LED turns off. A rising edge is therefore a color transition from absent/`unknown` back to the signal color. If two consecutive detections are more than `_BLINK_GAP_OFF_SEC` apart, a synthetic `_off_` marker is injected between them to represent the missed off-period.

**Blue beacons:** The beacon housing is always visible so inter-frame gaps are *not* off-periods — gap injection is skipped. Instead, `color_confidence` (fraction of lit pixels) separates the LED-on state (~0.4+) from the housing-only state (~0.02–0.05). Readings above `_BLINK_CC_ON_THRESHOLD` are treated as "on"; readings below (including `unknown` frames) are treated as "off".

### Key constants

| Constant | Value | Meaning |
|---|---|---|
| `_BLINK_WINDOW_SEC` | `12.0 s` | Rolling window length |
| `_BLINK_MIN_DATA_SEC` | `4.0 s` | Minimum history before deciding |
| `_BLINK_HZ_RANGE` | `0.12–2.0 Hz` | Valid blink frequency range |
| `_BLINK_MIN_EDGE_GAP` | `0.20 s` | Debounce: minimum gap between rising edges |
| `_BLINK_GAP_OFF_SEC` | `5.0 s` | Red/green: gap longer than this injects an off marker |
| `_BLINK_CC_ON_THRESHOLD` | `0.15` | Blue beacons: `color_conf` above this = LED on |
| `_BLINK_COLOR_CONF_MIN` | `0.001` | Minimum `color_conf` for a non-blue reading to count toward color mode |
| `_BLINK_MAX_IOI_SEC` | `5.0 s` | Max inter-onset interval for blue beacons |
| `_BLINK_MAX_IOI_SEC_COLOR` | `8.0 s` | Max inter-onset interval for red/green (allows long on-periods) |

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
| `enable_detection(model_path, imgsz=640)` | Start `YoloDetector` with object tracking on the live RGB stream. `imgsz` controls YOLO inference resolution. |

**Data accessors**

| Method | Returns |
|---|---|
| `get_rgb()` | Latest BGR frame as `np.ndarray`, or `None` |
| `get_depth()` | Latest float32 depth map, or `None` |
| `get_frame_timestamp()` | ROS header timestamp of the latest frame as `float` seconds, or `None` |
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
