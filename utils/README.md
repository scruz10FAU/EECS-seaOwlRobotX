# Utility Modules

Shared helper modules used by the top-level scripts in the main folder. None of these files are run directly — they have no `__main__` entry point. Import as a package from the main folder, e.g. `from utils.blink_detector import BlinkDetector`.

---

## File Overview

| File | Role |
|---|---|
| `camera_interface.py` | Abstract `CameraInterface` base class, plus the shared `Detection`, `Intrinsics`, `CameraConfig` dataclasses every detector backend and camera wrapper is built on. |
| `yolo_detector.py` | `YoloDetector` — Ultralytics YOLO `.pt` inference backend (CPU/GPU), with optional ByteTrack persistent tracking. |
| `tflite_hexagon_detector.py` | `TFLiteHexagonDetector` — int8 TFLite inference backend for the ModalAI VOXL2/Starling2 Max Hexagon NPU delegate. Drop-in alternative to `YoloDetector`; falls back to CPU if the delegate isn't available. |
| `blink_detector.py` | `BlinkDetector` class — rolling-window blink frequency estimator. |
| `beacon_camera.py` | `BeaconCamera` ROS2 node — camera image subscriptions, depth decoding, pose/GPS callbacks, and detector backend selection. |
| `beacon_mavlink_protocol.py` | Binary wire format (pack/unpack) for relaying beacon detections to the ground station as a MAVLink `MEMORY_VECT` message. Shared by `beacon_mavlink_bridge.py` (drone) and `beacon_mavlink_ground.py` (ground station). |

---

## camera_interface.py

Defines the shared data contract every camera/detector implementation in this repo uses, so mission code (`sweep_lawnmower.py`, `sweep_rrt.py`, etc.) never has to know which backend produced a detection.

### Detection

```python
@dataclass
class Detection:
    tracking_id: int              # persistent ID across frames (-1 if no tracker)
    label: str                    # class name
    confidence: float              # 0.0-1.0
    bbox_2d: Tuple[int, int, int, int]        # (x_min, y_min, x_max, y_max) px
    position_3d: Optional[Tuple[float, float, float]] = None   # (x, y, z) metres, camera frame
    velocity_3d: Optional[Tuple[float, float, float]] = None   # (vx, vy, vz) m/s, camera frame
```

### Intrinsics

```python
@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
```

### CameraInterface(ABC)

Base class camera wrappers can implement. Provides a default `back_project(u, v, depth_m)` pinhole back-projection method; subclasses may override it if their SDK provides a more accurate one (e.g. ZED's point cloud).

---

## yolo_detector.py

Pure "image in → `List[Detection]` out" wrapper around Ultralytics YOLO. Knows nothing about ROS, camera backend, or drone pose.

```python
from utils.yolo_detector import YoloDetector

det = YoloDetector(weights="path/to/best.pt", class_names=["red_buoy", ...],
                    imgsz=320, conf_thresh=0.5, device=None)
det.start(enable_tracking=True)          # ByteTrack for persistent IDs
detections = det.detect(rgb_frame, depth_map, intrinsics)
```

`detect()` calls `model.track()` when `enable_tracking=True` (persistent `tracking_id` via ByteTrack) or `model.predict()` otherwise (`tracking_id=-1`). `position_3d` is filled only when both `depth` and `intrinsics` are passed — sampled as the median of a 5×5 patch around the bbox center, then back-projected through the pinhole model.

---

## tflite_hexagon_detector.py

Drop-in alternative to `YoloDetector` for running on the ModalAI Starling2 Max's Hexagon DSP/NPU (Qualcomm QRB5165) instead of CPU/GPU. Same public interface (`start()` / `detect()`), so `BeaconCamera.enable_detection()` can select either backend via config.

```python
from utils.tflite_hexagon_detector import TFLiteHexagonDetector

det = TFLiteHexagonDetector(weights="model_int8.tflite", class_names=["beacon"],
                             delegate_path="/usr/lib/libhexagon_delegate.so")
det.start()
detections = det.detect(rgb_frame, depth_map, intrinsics)
```

**Requires** an int8-quantized `.tflite` export of the model (produced offline, not at runtime) and, to actually use the NPU, the VOXL2 SDK's Hexagon delegate shared library. `delegate_path` is board/SDK-version specific — if it's absent or fails to load, inference silently falls back to the interpreter's default CPU backend rather than raising.

Manually decodes the raw YOLOv8-style TFLite output tensor (`[1, 4+num_classes, num_anchors]`, no built-in NMS): dequantizes via the output tensor's scale/zero-point, thresholds by confidence, converts xywh→xyxy, undoes letterbox padding/scale back to original-frame pixel coordinates, then runs `cv2.dnn.NMSBoxes`. `tracking_id` is always `-1` — there is no ByteTrack equivalent for raw TFLite output in this backend.

---

## blink_detector.py

Standalone module — no ROS or OpenCV dependency. Imported directly by `beacon_detector.py`, `beacon_detector_config.py`, and `burst_beacon_detector.py`.

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

If all samples in the window are "on" (color_conf never drops below the threshold, as happens with some beacon types), the detector falls back to intensity oscillation: if the peak-to-peak swing of intensity across the window exceeds `_BLINK_INTENSITY_MIN_SWING`, the mean intensity is used to split samples into on/off and rising edges are counted as usual.

**`blink_color` vs `color`:** `blink_color` in the result dict reflects the most common non-blue color seen across the entire rolling window — it is populated as soon as any non-blue frame enters the window, regardless of whether blinking is confirmed. When `is_blinking=False`, `blink_color` may differ from the current frame's `color` (e.g. LED is currently off → `color="blue"` but `blink_color="red"` from recent history). Trust `blink_color` only when `is_blinking=True`; use the frame-level `color` for instantaneous classification.

### Key constants

| Constant | Default | Meaning |
|---|---|---|
| `_BLINK_WINDOW_SEC` | `12.0 s` | Rolling window length |
| `_BLINK_MIN_DATA_SEC` | `4.0 s` | Minimum history before deciding. Configurable via `blink_min_data_sec` |
| `_BLINK_HZ_RANGE` | `0.12–2.0 Hz` | Valid blink frequency range |
| `_BLINK_MIN_EDGE_GAP` | `0.20 s` | Debounce: minimum gap between rising edges. Configurable via `blink_min_edge_gap` |
| `_BLINK_GAP_OFF_SEC` | `5.0 s` | Red/green: gap longer than this injects an off marker |
| `_BLINK_CC_ON_THRESHOLD` | `0.15` | Blue beacons: `color_conf` above this = LED on |
| `_BLINK_COLOR_CONF_MIN` | `0.001` | Minimum `color_conf` for a non-blue reading to count toward color mode |
| `_BLINK_MAX_IOI_SEC` | `5.0 s` | Max inter-onset interval for blue beacons |
| `_BLINK_MAX_IOI_SEC_COLOR` | `8.0 s` | Max inter-onset interval for red/green (allows long on-periods) |
| `_BLINK_INTENSITY_MIN_SWING` | `0.05` | Min peak-to-peak intensity swing to activate intensity-fallback path. Configurable via `blink_intensity_min_swing` |
| `_BLINK_MAX_IOI_RATIO` | `None` | Max ratio of longest to shortest IOI (blue beacons only). `None` = disabled. Configurable via `blink_max_ioi_ratio` |

Parameters marked "Configurable" can be set per-deployment via the `detection` section of the JSON config (applied at startup by `_apply_color_config`).

### Helper

```python
_get_blink_detector(tracking_id: int) -> BlinkDetector
```

Returns the `BlinkDetector` for a given YOLO tracking ID, creating one on first call.

---

## beacon_camera.py

ROS2 node that wraps camera subscriptions, depth synchronization, drone pose, GPS origin, and detector backend selection. Lazily imported by `beacon_detector.py`/`beacon_detector_config.py` inside `_import_ros()` so ROS packages are never loaded unless ROS mode is actually invoked.

### BeaconCamera(Node)

**Lifecycle**

| Method | Description |
|---|---|
| `open()` | Subscribe to RGB+depth image topics, camera info, pose, GPS. Used for live camera mode. |
| `open_for_video()` | Minimal setup for video-file mode: create publisher and subscribe to pose + GPS only. |
| `close()` | Mark node as closed. |
| `grab()` | Spin once and return `True` if a new synchronized frame arrived. |
| `enable_detection(model_path, imgsz=640, backend="ultralytics", conf_thresh=0.5, delegate_path=None)` | Start detection on the live RGB stream. `backend="ultralytics"` uses `YoloDetector` with ByteTrack; `backend="tflite_hexagon"` uses `TFLiteHexagonDetector` (Hexagon NPU via `delegate_path`, no tracking). |

**Data accessors**

| Method | Returns |
|---|---|
| `get_rgb()` | Latest BGR frame as `np.ndarray`, or `None` |
| `get_depth()` | Latest float32 depth map, or `None` |
| `get_frame_timestamp()` | ROS header timestamp of the latest frame as `float` seconds, or `None` |
| `get_drone_pose()` | `(pos_xyz, quat_wxyz)` numpy arrays, or `(None, None)` |
| `get_gps_origin()` | `(lat, lon, alt)` tuple, or `None` |
| `get_detections()` | List of `Detection` objects from the active detector backend |

**Depth decoding**

| Encoding | dtype | Scale |
|---|---|---|
| `32FC1` | `float32` | metres, no scaling |
| `16UC1` | `uint16` | × 0.001 → metres |
| `8UC1` / other | `uint8` | raw value, no scaling |

If the decoded depth map has a different resolution than the RGB frame, it is resized to match using `cv2.INTER_NEAREST`.

RGB and depth frames are synchronized with `message_filters.ApproximateTimeSynchronizer` (50 ms slop).

---

## beacon_mavlink_protocol.py

Pure encode/decode module — no ROS or `pymavlink` dependency, just `struct`. Shared verbatim by `beacon_mavlink_bridge.py` (drone side) and `beacon_mavlink_ground.py` (ground station side); see the main `README.md` for the full picture of how the two scripts use it.

```python
from utils.beacon_mavlink_protocol import pack_value_array, unpack_value_array

value = pack_value_array(latitude, longitude, color, is_blinking)   # -> 32-entry signed-int8 list
# ... send `value` as a MAVLink MEMORY_VECT message ...
beacon = unpack_value_array(value)   # -> BeaconMavlinkMessage(latitude, longitude, color, is_blinking)
```

Packs `latitude`/`longitude` as int32, ×1e7-scaled (same convention as MAVLink's own `GLOBAL_POSITION_INT`), plus `color` and `is_blinking` as `uint8` enums, into the first 10 bytes of the 32-byte `value` array MAVLink's `MEMORY_VECT` message carries — the remaining bytes are zero-padded. `BEACON_MAVLINK_ADDRESS` (`0xBEAC`) is used as `MEMORY_VECT.address`, a sentinel the ground-station decoder filters on to distinguish this traffic from any other real memory-vector use on the link. `BEACON_MAVLINK_VERSION` is carried in `MEMORY_VECT.ver` so a future payload-layout change can be detected and rejected cleanly instead of silently misdecoding.

| Enum | Values |
|---|---|
| `color` | `unknown`=0, `red`=1, `green`=2, `blue`=3, `white`=4 |
| `is_blinking` | `None`=0, `False`=1, `True`=2 |
