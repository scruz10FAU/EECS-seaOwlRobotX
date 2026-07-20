# ArUco Marker Detection

Tools for generating, detecting, and estimating the pose of ArUco markers. Three usage paths are provided: standalone webcam scripts, a basic ROS2 subscriber pair, and a full ROS2 + ZED node driven by the project's JSON config system.

---

## Files

| File | Description |
|---|---|
| `generator.py` | Generate and save a marker image as a PNG |
| `detector.py` | Detect markers from a webcam (no pose, no ROS) |
| `poseEstimator.py` | Detect markers + estimate 6DoF pose using a calibrated camera |
| `calibrateCamera.py` | Capture checkerboard images and compute camera calibration |
| `camera_calibration.npz` | Saved calibration output used by `poseEstimator.py` |
| `aruco_marker_0–5.png` | Pre-generated DICT_4X4_50 markers, IDs 0–5 |
| `calib_images/` | Checkerboard images captured during calibration |
| `aruco_detector_ros.py` | ROS2 + ZED detector driven by `configs/aruco_config.json` |
| `ROS2/webcam_publisher.py` | Publishes a webcam stream to `/camera/image_raw` |
| `ROS2/detector_ros2.py` | Detects markers from any `/camera/image_raw` ROS2 topic |

---

## Dependencies

```
pip install opencv-contrib-python numpy
```

ROS2 scripts additionally require a sourced ROS2 workspace with `cv_bridge` and `sensor_msgs`. `aruco_detector_ros.py` also requires the packages from the repo root (`beacon_camera`, `camera_interface`).

---

## Standalone Scripts (no ROS)

### 1 — Generate a marker

Edit `marker_id` (0–49 for `DICT_4X4_50`) and run:

```bash
python3 generator.py
```

Outputs `aruco_marker_<id>.png` with a white border. Print it, measure the black square area (excluding the border) in metres, and record that number — it is required for accurate pose estimation.

Pre-generated markers for IDs 0–5 are already in this folder.

### 2 — Detect markers (webcam, no pose)

```bash
python3 detector.py
```

Opens the default webcam. Detected markers are outlined and their IDs are printed to the console. Press `q` to quit.

### 3 — Calibrate your camera

Accurate distance and pose estimation require a calibration file specific to your camera. Skip this step only if you are using the ZED camera with `aruco_detector_ros.py`, which reads intrinsics from the JSON config instead.

**Steps:**

1. Print a 10×7 checkerboard ([printable PDF](https://github.com/kyle-bersani/opencv-examples/blob/master/CalibrationByChessboard/chessboard-to-print.pdf)) and attach it to a flat board.
2. Measure the physical size of one square in metres and update `SQUARE_SIZE` in `calibrateCamera.py` (line 19, default `0.0254 m`).
3. Run:

```bash
python3 calibrateCamera.py
```

4. A live preview opens. **Left-click** while the detection overlay is visible to capture an image. Capture at least 15–20 photos from different angles and distances.
5. Press `q` when done. Calibration runs automatically and saves `camera_calibration.npz`.
6. Check the reported reprojection error — values below 0.5 are good.

### 4 — Detect markers with pose estimation (webcam)

Requires `camera_calibration.npz` from step 3. Update `MARKER_SIZE` in `poseEstimator.py` (line 4) to match the printed marker's black square area in metres (default `0.0635 m`).

```bash
python3 poseEstimator.py
```

Each detected marker is annotated with its ID, distance in metres, and a 3D axis overlay showing orientation. Press `q` to quit.

---

## ROS2 — Basic Subscriber Pair

This pair lets you test detection over any ROS2 image topic, including a local webcam bridged into ROS.

### Publish a webcam as a ROS2 image topic

```bash
python3 ROS2/webcam_publisher.py
```

Publishes frames from the default webcam at 30 fps to `/camera/image_raw`.

### Run the detector against that topic

In a second terminal:

```bash
python3 ROS2/detector_ros2.py
```

Subscribes to `/camera/image_raw`. Detected marker IDs are printed to the console each frame. To use a different image topic, update the topic string on line 37 of `detector_ros2.py`.

---

## ROS2 — ZED Camera + JSON Config (`aruco_detector_ros.py`)

This script integrates with the Seabird project's camera and config infrastructure. It reuses `beacon_camera.py` for ZED topic subscriptions and reads all parameters from `configs/aruco_config.json`.

### Config file

`configs/aruco_config.json` controls all runtime parameters:

```json
{
  "topics": {
    "camera_prefix": "/zed/zed_node",
    "drone_pose":    "/mavros/local_position/pose",
    "gps_origin":    "/mavros/global_position/gp_origin",
    "aruco_pub":     "/seabird/aruco_detections"
  },
  "camera": {
    "focal_length_mm": 2.1,
    "h_aperture_mm":   6.0,
    "v_aperture_mm":   4.5,
    "img_w":           640,
    "img_h":           480
  },
  "aruco": {
    "dictionary":    "DICT_4X4_50",
    "marker_size_m": 0.15,
    "display":       false,
    "video":         null
  }
}
```

| Field | Description |
|---|---|
| `topics.camera_prefix` | Camera namespace (used as the default `image` topic if `image` is omitted) |
| `topics.image` | ROS2 topic publishing the camera image — set this to your actual topic |
| `topics.aruco_pub` | Topic detections are published to as a JSON string |
| `camera.focal_length_mm` | Lens focal length — used to compute `fx` / `fy` |
| `camera.h_aperture_mm` | Horizontal sensor aperture |
| `camera.v_aperture_mm` | Vertical sensor aperture |
| `camera.img_w` / `img_h` | Image resolution |
| `aruco.dictionary` | ArUco dictionary name (see table below) |
| `aruco.marker_size_m` | Physical marker side length in metres (black area only) |
| `aruco.display` | Show annotated frames in a window while running |
| `aruco.video` | Path to a video file for standalone mode; `null` for ROS live |

**Supported dictionaries:**

`DICT_4X4_50`, `DICT_4X4_100`, `DICT_4X4_250`, `DICT_4X4_1000`, `DICT_5X5_50`, `DICT_5X5_100`, `DICT_5X5_250`, `DICT_5X5_1000`, `DICT_6X6_250`, `DICT_7X7_1000`, `DICT_ARUCO_ORIGINAL`

### Usage

**ROS2 live (ZED camera):**
```bash
python3 aruco_detector_ros.py
python3 aruco_detector_ros.py --config configs/aruco_config.json   # explicit path
```

**Video file (no ROS required):**
```bash
python3 aruco_detector_ros.py --video path/to/video.mp4
```

**Webcam (no ROS required):**
```bash
python3 aruco_detector_ros.py --video 0
```

### Published message format

Each frame with detections publishes a JSON string to `topics.aruco_pub`:

```json
{
  "timestamp": 1783454350.147,
  "markers": [
    {
      "id": 3,
      "corners": [[x,y], [x,y], [x,y], [x,y]],
      "rvec": [rx, ry, rz],
      "tvec": [tx, ty, tz],
      "dist_m": 1.234
    }
  ]
}
```

`rvec` is the rotation vector (Rodrigues), `tvec` is the translation vector, both in the camera frame. `dist_m` is Euclidean distance to the marker centre.

---

## Workflow Summary

```
Print markers          →  generator.py
                                │
                    ┌───────────┴──────────────┐
                    │                          │
         Standalone / webcam            ROS2 / ZED
                    │                          │
         ┌──────────┴──────────┐        aruco_detector_ros.py
         │                     │         (reads aruco_config.json,
    detector.py          calibrateCamera.py    publishes JSON to ROS)
    (no pose)                  │
                         poseEstimator.py
                         (6DoF pose + distance)
```
