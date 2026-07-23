#!/usr/bin/env python3
"""
beacon_detector_config.py — beacon_detector.py driven by a JSON configuration file.

All runtime parameters (model paths, thresholds, ROS topics, camera intrinsics,
and mount geometry) are loaded from a JSON config file. seabird_config.py is not
imported — the JSON config is the single source of truth for all Seabird parameters.

Usage:
    python3 beacon_detector_config.py                           # uses beacon_config.json
    python3 beacon_detector_config.py --config my_config.json  # custom config path

See beacon_config.json for the full schema and default values.
"""

import sys
import argparse
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import csv
import numpy as np
from typing import Tuple
import json
import time
import cv2
import math

from blink_detector import BlinkDetector, _get_blink_detector

EARTH_RADIUS_M = 6378137.0

_DEFAULT_CONFIG = "beacon_config.json"

_DEFAULT_TOPICS = {
    "camera_prefix":  "/zed/zed_node",
    "image":          "/zed/zed_node/rgb/color/rect/image",
    "camera_info":    "/zed/zed_node/rgb/color/rect/camera_info",
    "depth":          "/zed/zed_node/depth/depth_registered",
    "drone_pose":     "/mavros/local_position/pose",
    "gps_origin":     "/mavros/global_position/gp_origin",
    "detections_pub": "/seabird/beacon_detections",
    "aruco_pub":      "/seabird/aruco_ground_truth",
}

def _merge_topics(raw: dict) -> dict:
    merged = {**_DEFAULT_TOPICS, **raw}
    prefix = merged["camera_prefix"]
    if "image" not in raw:
        merged["image"] = f"{prefix}/rgb/color/rect/image"
    if "camera_info" not in raw:
        merged["camera_info"] = f"{prefix}/rgb/color/rect/camera_info"
    if "depth" not in raw:
        merged["depth"] = f"{prefix}/depth/depth_registered"
    return merged


_DEFAULT_CAMERA = {
    "focal_length_mm":  2.1,
    "h_aperture_mm":    6.0,
    "v_aperture_mm":    4.5,
    "clipping_near":    0.1,
    "clipping_far":   200.0,
    "img_w":           640,
    "img_h":           480,
    "mount_offset_xyz": [0.30, 0.0, 0.05],
    "pitch_deg":        15.0,
}

_DEFAULT_ARUCO = {
    "enabled":          False,
    "dictionary":       "DICT_4X4_50",
    "marker_size_m":    0.15,
    "calibration_file": None,
}

_DEFAULT_DETECTION = {
    "confirm_frames":  3,
    "pub_cooldown_s":  1.0,
    "depth_min_m":     1.0,
    "depth_max_m":     60.0,
    "min_area_frac":   0.001,
    "stage2_conf":     0.30,
    "depth_source":    "topic",   # "topic" = use depth ROS topic; "bbox" = estimate from bounding box
    "beacon_height_m": 0.3048,    # physical beacon height in metres (12 in) — used for bbox estimation
    "beacon_z_m":      0.0,       # known beacon altitude in ENU frame (metres)
    "max_detections":  None,      # stop after this many published detections (null = unlimited)
}


# ── Config loader ─────────────────────────────────────────────────────────────

def load_config(path: str) -> dict:
    """
    Load the JSON config file. Missing keys fall back to defaults.
    Computes derived camera values (fx, fy, cx, cy, rotation matrix, mount offset)
    so callers never need to import seabird_config.
    """
    with open(path) as fh:
        raw = json.load(fh)

    cfg = {
        "model":      raw.get("model",      "models/one_beacon.pt"),
        "crop_model": raw.get("crop_model", "models/best_crop.pt"),
        "conf":       float(raw.get("conf",      0.5)),
        "display":    bool(raw.get("display",    False)),
        "true_dist":  float(raw.get("true_dist", 0.4826)),
        "save":       bool(raw.get("save",        False)),
        "log":        bool(raw.get("log",         False)),
        "save_crops":      bool(raw.get("save_crops",      False)),
        "save_det_images": bool(raw.get("save_det_images", False)),
        "save_frames":     bool(raw.get("save_frames",     False)),
        "target_color":    raw.get("target_color",    None),
        "target_blinking": raw.get("target_blinking", None),
        "video":      raw.get("video",      None),
        "ros_video":  raw.get("ros_video",  None),
        "topics":     _merge_topics(raw.get("topics", {})),
        "camera":     {**_DEFAULT_CAMERA,  **raw.get("camera",  {})},
        "paths":      raw.get("paths",      {}),
        "isaac":      raw.get("isaac",      {}),
        "drone_spawn":raw.get("drone_spawn",{}),
        "px4":        raw.get("px4",        {}),
        "labeler":    raw.get("labeler",    {}),
        "aruco":      {**_DEFAULT_ARUCO,     **raw.get("aruco",     {})},
        "detection":  {**_DEFAULT_DETECTION, **raw.get("detection", {})},
    }

    # Compute derived camera intrinsics and mount geometry
    cam = cfg["camera"]
    fl, ha, va = cam["focal_length_mm"], cam["h_aperture_mm"], cam["v_aperture_mm"]
    w,  h      = cam["img_w"], cam["img_h"]
    cam["fx"] = fl * w / ha
    cam["fy"] = fl * h / va
    cam["cx"] = w / 2.0
    cam["cy"] = h / 2.0

    # If a calibration file is provided (same one used for ArUco), load the
    # calibrated fx/fy/cx/cy from it — these are more accurate than the
    # theoretical values derived from lens specs above.
    cal_path = cfg["aruco"].get("calibration_file")
    if cal_path:
        cal_path = os.path.expanduser(cal_path)
        if os.path.isfile(cal_path):
            _cal = np.load(cal_path)
            _K   = _cal["camera_matrix"]
            cam["fx"] = float(_K[0, 0])
            cam["fy"] = float(_K[1, 1])
            cam["cx"] = float(_K[0, 2])
            cam["cy"] = float(_K[1, 2])

    cam["_mount_offset"] = np.array(cam["mount_offset_xyz"], dtype=np.float64)

    # Body-to-camera rotation: Isaac FLU body frame → OpenCV camera frame,
    # then pitched nose-down by pitch_deg.
    #   cam_X (right)   = -body_Y
    #   cam_Y (down)    = -body_Z
    #   cam_Z (forward) =  body_X
    pr = np.radians(cam["pitch_deg"])
    _R_base = np.array([[0, -1,  0],
                         [0,  0, -1],
                         [1,  0,  0]], dtype=np.float64)
    _R_pitch = np.array([[1,            0,           0],
                          [0,  np.cos(pr), -np.sin(pr)],
                          [0,  np.sin(pr),  np.cos(pr)]], dtype=np.float64)
    cam["_R_body_to_cam"] = _R_pitch @ _R_base

    return cfg


# ── Lazily imported only when ROS mode is used ────────────────────────────────

_BeaconCameraBase = None  # set by _import_ros()


def _import_ros():
    global rclpy, String, _BeaconCameraBase
    import rclpy as _rclpy; rclpy = _rclpy
    from std_msgs.msg import String as _Str; String = _Str
    from beacon_camera import BeaconCamera as _BC; _BeaconCameraBase = _BC


def _camera_to_world(p_cam, drone_pos, drone_quat_wxyz, mount_offset, R_body_to_cam):
    """
    Transform a 3D point from camera frame to world frame (ENU).
    Replaces seabird_config.camera_to_world using values from the JSON config.
      p_cam:             (3,) point in OpenCV camera frame (X-right, Y-down, Z-forward)
      drone_pos:         (3,) drone position in world frame
      drone_quat_wxyz:   (4,) [w, x, y, z] quaternion
      mount_offset:      cfg["camera"]["_mount_offset"]  (np.ndarray)
      R_body_to_cam:     cfg["camera"]["_R_body_to_cam"] (3x3 np.ndarray)
    """
    from scipy.spatial.transform import Rotation
    p_body = R_body_to_cam.T @ np.asarray(p_cam) + mount_offset
    w, x, y, z = drone_quat_wxyz
    R_body_to_world = Rotation.from_quat([x, y, z, w]).as_matrix()
    return R_body_to_world @ p_body + np.asarray(drone_pos)


def _make_beacon_camera(topics: dict, cfg_camera: dict, cfg_detection: dict):
    """
    Return a BeaconCamera instance whose ROS topic names and camera intrinsics
    are taken from the JSON config. Creates a subclass at call-time (after
    _import_ros()) so all values are captured by closure — seabird_config.py
    is never imported.
    """
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import PoseStamped
    from geographic_msgs.msg import GeoPointStamped
    from camera_interface import Intrinsics
    from cv_bridge import CvBridge as _CvBridge
    import message_filters

    _rgb_bridge = _CvBridge()

    camera_prefix    = topics["camera_prefix"]
    image_topic      = topics["image"]
    depth_topic      = topics["depth"]
    drone_pose_topic  = topics["drone_pose"]
    gps_topic         = topics["gps_origin"]
    detections_topic  = topics["detections_pub"]
    aruco_gt_topic    = topics.get("aruco_pub", "/seabird/aruco_ground_truth")
    depth_source      = cfg_detection.get("depth_source", "topic")

    fx, fy   = cfg_camera["fx"],    cfg_camera["fy"]
    cx, cy   = cfg_camera["cx"],    cfg_camera["cy"]
    img_w    = cfg_camera["img_w"]
    img_h    = cfg_camera["img_h"]

    class _ConfiguredBeaconCamera(_BeaconCameraBase):
        def _on_rgb_only(self, rgb_msg):
            """RGB-only callback used when depth_source == 'bbox'."""
            if rgb_msg.encoding in ('bgr8', 'yuv422', 'yuv422_yuy2'):
                bgr = _rgb_bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            else:
                channels = len(rgb_msg.data) // (rgb_msg.height * rgb_msg.width)
                rgb_arr = np.frombuffer(rgb_msg.data, dtype=np.uint8).reshape(
                    rgb_msg.height, rgb_msg.width, channels
                )
                bgr = rgb_arr[:, :, :3][:, :, ::-1].copy()
            ts = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
            with self._frame_lock:
                self._rgb      = bgr
                self._depth    = None
                self._frame_ts = ts
                self._new_frame = True

        def open(self):
            if self._is_open:
                return True
            self._intrinsics = Intrinsics(
                fx=fx, fy=fy, cx=cx, cy=cy, width=img_w, height=img_h
            )
            self.get_logger().info(
                f"Intrinsics set from config: fx={fx:.1f} fy={fy:.1f} "
                f"cx={cx:.1f} cy={cy:.1f} {img_w}x{img_h}"
            )
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            if depth_source == "bbox":
                # RGB-only — depth comes from bounding-box projection, no sync needed
                self.create_subscription(Image, image_topic, self._on_rgb_only, qos)
                self.get_logger().info("depth_source=bbox — subscribing to RGB only")
            else:
                rgb_sub = message_filters.Subscriber(
                    self, Image, image_topic, qos_profile=qos
                )
                depth_sub = message_filters.Subscriber(
                    self, Image, depth_topic, qos_profile=qos
                )
                self._sync = message_filters.ApproximateTimeSynchronizer(
                    [rgb_sub, depth_sub], queue_size=5, slop=0.05
                )
                self._sync.registerCallback(self._on_synced_frame)
            self.detection_pub  = self.create_publisher(String, detections_topic, 10)
            self.aruco_gt_pub   = self.create_publisher(String, aruco_gt_topic,   10)
            if drone_pose_topic:
                self._pose_sub = self.create_subscription(
                    PoseStamped, drone_pose_topic, self._on_drone_pose, qos
                )
            if gps_topic:
                origin_qos = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    depth=1,
                )
                self._origin_sub = self.create_subscription(
                    GeoPointStamped, gps_topic, self._on_gps_origin, origin_qos
                )
            self._is_open = True
            self.get_logger().info("BeaconCamera open — waiting for frames…")
            return True

        def open_for_video(self):
            if self._is_open:
                return True
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.detection_pub  = self.create_publisher(String, detections_topic, 10)
            self.aruco_gt_pub   = self.create_publisher(String, aruco_gt_topic,   10)
            if drone_pose_topic:
                self._pose_sub = self.create_subscription(
                    PoseStamped, drone_pose_topic, self._on_drone_pose, qos
                )
            if gps_topic:
                origin_qos = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    depth=1,
                )
                self._origin_sub = self.create_subscription(
                    GeoPointStamped, gps_topic, self._on_gps_origin, origin_qos
                )
            self._is_open = True
            self.get_logger().info("BeaconCamera open (video-file mode) — pose + GPS only")
            return True

    return _ConfiguredBeaconCamera(topic_prefix=camera_prefix)


# ── Color classification ───────────────────────────────────────────────────────

_SAT_MIN = 40    # lowered from 60 to catch overexposed LEDs
_VAL_MIN = 160

# Previous bands (revert here if needed):
# _HUE_BANDS = [
#     (  0, 20, "red"),
#     ( 65, 30, "green"),
#     (120, 15, "blue"),   # 105–135°
#     (165, 15, "red"),    # 150–180°
# ]
_HUE_BANDS = [
    (  0, 20, "red"),    # 0–20°
    ( 65, 30, "green"),  # 35–95°
    (157, 22, "red"),    # 135–179° — expanded to catch purple-tinted red LEDs; processed before blue so the 135° boundary goes to red
    (120, 15, "blue"),   # 105–134° — only claims hues not already taken by red
]

_RED_THRESHOLD = 0.1


def _hue_votes(hues: np.ndarray) -> dict:
    n = max(len(hues), 1)
    hues_f = hues.astype(np.float32)
    matched = np.zeros(len(hues), dtype=bool)
    label_counts: dict = {}

    for center, half, label in _HUE_BANDS:
        dist = np.abs(hues_f - center)
        dist = np.minimum(dist, 180.0 - dist)
        in_band = (dist <= half) & ~matched
        label_counts[label] = label_counts.get(label, 0) + int(np.sum(in_band))
        matched |= in_band

    result = {"red": 0, "green": 0, "blue": 0, "other": int(np.sum(~matched))}
    for label, count in label_counts.items():
        result[label] = result.get(label, 0) + count
    return {k: result[k] / n for k in result}


def classify_beacon_color(bgr_crop: np.ndarray) -> Tuple[str, float, np.ndarray, float, dict]:
    _empty_votes = {"red": 0.0, "green": 0.0, "blue": 0.0, "other": 0.0}
    if bgr_crop is None or bgr_crop.size == 0:
        return "unknown", 0.0, np.zeros((1, 1), dtype=np.uint8), 0.0, _empty_votes

    hsv = cv2.cvtColor(bgr_crop, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]

    light_mask = ((s >= _SAT_MIN) & (v >= _VAL_MIN)).astype(np.uint8) * 255

    lit_pixels   = np.count_nonzero(light_mask)
    total_pixels = bgr_crop.shape[0] * bgr_crop.shape[1]
    color_conf   = lit_pixels / max(total_pixels, 1)

    if lit_pixels < max(3, total_pixels * 0.02):
        very_bright = (v >= 220)
        bright_count = int(np.count_nonzero(very_bright))
        if bright_count > total_pixels * 0.1:
            intensity = float(np.mean(v[very_bright]) / 255.0)
            return "white", float(bright_count / total_pixels), light_mask, intensity, _empty_votes
        return "unknown", 0.0, light_mask, 0.0, _empty_votes

    hues      = h[light_mask > 0]
    intensity = float(np.mean(v[light_mask > 0]) / 255.0)

    votes = _hue_votes(hues)

    if votes["red"] >= _RED_THRESHOLD:
        color = "red"
    else:
        winner = max(("green", "blue"), key=lambda c: votes[c])
        color = winner if votes[winner] >= 0.30 else "unknown"

    return color, color_conf, light_mask, intensity, votes


def isolate_and_classify(beacon_crop: np.ndarray, crop_model,
                         conf: float = 0.3) -> Tuple[str, float, np.ndarray, float, dict]:
    _empty = ("unknown", 0.0, np.zeros((1, 1), dtype=np.uint8), 0.0,
              {"red": 0.0, "green": 0.0, "blue": 0.0, "other": 0.0})
    if beacon_crop is None or beacon_crop.size == 0:
        return _empty

    h, w = beacon_crop.shape[:2]
    display_mask = np.zeros((h, w), dtype=np.uint8)

    results = crop_model(beacon_crop.copy(), conf=conf, verbose=False)
    boxes   = results[0].boxes

    if len(boxes) == 0:
        color, color_conf, light_mask, intensity, votes = classify_beacon_color(beacon_crop)
        return color, color_conf, light_mask, intensity, votes, beacon_crop

    if results[0].masks is not None:
        best_idx = int(np.argmax([float(b.conf[0]) for b in boxes]))
        seg_mask = results[0].masks.data[best_idx].cpu().numpy()
        seg_resized = cv2.resize(seg_mask, (w, h), interpolation=cv2.INTER_NEAREST)
        display_mask = (seg_resized > 0.5).astype(np.uint8) * 255
    else:
        best_box = max(boxes, key=lambda b: float(b.conf[0]))
        lx1, ly1, lx2, ly2 = map(int, best_box.xyxy[0].tolist())
        lx1 = max(0, lx1); ly1 = max(0, ly1)
        lx2 = min(w, lx2); ly2 = min(h, ly2)
        if lx2 > lx1 and ly2 > ly1:
            display_mask[ly1:ly2, lx1:lx2] = 255

    if not display_mask.any():
        color, color_conf, light_mask, intensity, votes = classify_beacon_color(beacon_crop)
        return color, color_conf, light_mask, intensity, votes, beacon_crop

    rows = np.any(display_mask > 0, axis=1)
    cols = np.any(display_mask > 0, axis=0)
    rmin, rmax = np.where(rows)[0][[0, -1]]
    cmin, cmax = np.where(cols)[0][[0, -1]]
    lit_region = beacon_crop[rmin:rmax + 1, cmin:cmax + 1]

    color, color_conf, _, intensity, votes = classify_beacon_color(lit_region)
    return color, color_conf, display_mask, intensity, votes, lit_region


# ── Detection logger ──────────────────────────────────────────────────────────

_LOG_HEADER = [
    "timestamp", "frame", "img_w", "img_h", "color", "color_confidence", "intensity",
    "vote_red", "vote_green", "vote_blue", "vote_other",
    "det_confidence", "x1", "y1", "x2", "y2", "tracking_id",
    "pos3d_x", "pos3d_y", "pos3d_z",
    "blink_is_blinking", "blink_hz", "blink_phase",
    "target_color", "target_blinking", "target_match",
    "gt_aruco_id", "gt_dist_m", "gt_tvec_x", "gt_tvec_y", "gt_tvec_z",
]


def _open_log(path: str):
    fh = open(path, "w", newline="")
    writer = csv.writer(fh)
    writer.writerow(_LOG_HEADER)
    print(f"[beacon] Logging detections → {path}")
    return fh, writer


def _write_log_row(log_writer, frame_idx: int, color: str,
                   color_conf: float, intensity: float, votes: dict,
                   det_conf: float, bbox, tracking_id: int = -1,
                   pos3d=None, blink_info: dict = None,
                   target_color=None, target_blinking=None,
                   gt_info=None, img_w=None, img_h=None) -> None:
    x1, y1, x2, y2 = bbox
    px = py = pz = ""
    if pos3d is not None:
        px, py, pz = f"{pos3d[0]:.4f}", f"{pos3d[1]:.4f}", f"{pos3d[2]:.4f}"
    bi = blink_info or {}
    blink_blinking = "" if bi.get("is_blinking") is None else str(bi.get("is_blinking"))
    blink_hz    = f"{bi['blink_hz']:.3f}" if bi.get("blink_hz") is not None else ""
    blink_phase = bi.get("phase", "")
    if target_color is None and target_blinking is None:
        tc_col = tb_col = target_match = ""
    else:
        tc_col = target_color   if target_color   is not None else ""
        tb_col = str(target_blinking) if target_blinking is not None else ""
        color_ok    = target_color    is None or color == target_color
        blinking_ok = target_blinking is None or bi.get("is_blinking") == target_blinking
        target_match = str(color_ok and blinking_ok)
    if gt_info is None:
        gt_id = gt_dist = gt_tx = gt_ty = gt_tz = ""
    else:
        _gt_id, _gt_dist, _gt_tvec = gt_info
        gt_id  = str(_gt_id)
        gt_dist = f"{_gt_dist:.4f}"
        gt_tx, gt_ty, gt_tz = [f"{v:.4f}" for v in _gt_tvec]
    log_writer.writerow([
        f"{time.time():.3f}", frame_idx,
        img_w if img_w is not None else "", img_h if img_h is not None else "",
        color, f"{color_conf:.4f}", f"{intensity:.4f}",
        f"{votes.get('red',0):.4f}", f"{votes.get('green',0):.4f}",
        f"{votes.get('blue',0):.4f}", f"{votes.get('other',0):.4f}",
        f"{det_conf:.4f}", x1, y1, x2, y2, tracking_id,
        px, py, pz,
        blink_blinking, blink_hz, blink_phase,
        tc_col, tb_col, target_match,
        gt_id, gt_dist, gt_tx, gt_ty, gt_tz,
    ])


# ── Helper ────────────────────────────────────────────────────────────────────

def local_enu_to_gps(world_pos: np.ndarray,
                     origin_lat: float,
                     origin_lon: float,
                     origin_alt: float) -> Tuple[float, float, float]:
    east, north, up = world_pos[0], world_pos[1], world_pos[2]
    dlat = np.degrees(north / EARTH_RADIUS_M)
    dlon = np.degrees(east / (EARTH_RADIUS_M * np.cos(np.radians(origin_lat))))
    return (origin_lat + dlat, origin_lon + dlon, origin_alt + up)


def estimate_distance_from_bbox(
    x1: int, y1: int, x2: int, y2: int,
    fx: float, fy: float, cx: float, cy: float,
    drone_z_m: float = 0.0,
    beacon_z_m: float = 0.0,
    beacon_height_m: float = 0.3048,
) -> "list | None":
    """
    Estimate a camera-frame 3D point for a beacon detection using the pinhole
    model and known physical beacon height, corrected for drone altitude.

    Returns [X, Y, Z] in OpenCV camera frame (X-right, Y-down, Z-forward) so
    the result can be passed directly to _camera_to_world(), or None if the
    geometry is degenerate (too few pixels, drone directly above beacon, etc.).

    Slant distance:  slant = (beacon_height_m * fy) / bbox_height_px
    Altitude offset: dz    = drone_z_m - beacon_z_m
    Horizontal dist: horiz = sqrt(slant^2 - dz^2)
    Camera-frame pt: project the slant distance along the ray through the bbox centre.
    """
    bbox_h = y2 - y1
    if bbox_h < 4:
        return None
    slant = (beacon_height_m * fy) / bbox_h
    dz = drone_z_m - beacon_z_m
    horiz_sq = slant ** 2 - dz ** 2
    if horiz_sq <= 0:
        return None
    horiz = math.sqrt(horiz_sq)
    # Reconstruct slant from horizontal + vertical for the projection step
    slant_corrected = math.sqrt(horiz ** 2 + dz ** 2)
    # Normalised image coordinates of the bbox centre
    nu = ((x1 + x2) / 2.0 - cx) / fx
    nv = ((y1 + y2) / 2.0 - cy) / fy
    # Scale the unit ray by slant_corrected to get the camera-frame 3D point
    ray_len = math.sqrt(nu ** 2 + nv ** 2 + 1.0)
    depth_cam = slant_corrected / ray_len
    return [nu * depth_cam, nv * depth_cam, depth_cam]


# ── ArUco ground truth helpers ────────────────────────────────────────────────

_ARUCO_DICT_MAP = {
    "DICT_4X4_50":         cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100":        cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250":        cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000":       cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50":         cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100":        cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250":        cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000":       cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_250":        cv2.aruco.DICT_6X6_250,
    "DICT_7X7_1000":       cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


def _build_aruco_detector(aruco_cfg: dict):
    dict_name = aruco_cfg.get("dictionary", "DICT_4X4_50")
    dict_id   = _ARUCO_DICT_MAP.get(dict_name, cv2.aruco.DICT_4X4_50)
    return cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(dict_id),
        cv2.aruco.DetectorParameters(),
    )


def _load_aruco_calibration(aruco_cfg: dict, cam_cfg: dict):
    cal_path = aruco_cfg.get("calibration_file")
    if cal_path:
        cal_path = os.path.expanduser(cal_path)
        if os.path.isfile(cal_path):
            data = np.load(cal_path)
            return data["camera_matrix"], data["dist_coeffs"]
    fl, ha, va = cam_cfg["focal_length_mm"], cam_cfg["h_aperture_mm"], cam_cfg["v_aperture_mm"]
    w, h       = cam_cfg["img_w"], cam_cfg["img_h"]
    fx, fy     = fl * w / ha, fl * h / va
    camera_mat = np.array([[fx, 0, w / 2.0], [0, fy, h / 2.0], [0, 0, 1]], dtype=np.float64)
    return camera_mat, np.zeros((4, 1), dtype=np.float64)


def detect_aruco_ground_truth(frame, detector, camera_mat, dist_coeffs, marker_size_m: float):
    """Run ArUco on frame; return (id, dist_m, [tx,ty,tz]) for closest marker, or None."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None or len(ids) == 0:
        return None
    half    = marker_size_m / 2.0
    obj_pts = np.array([[-half, half, 0], [half, half, 0],
                        [half, -half, 0], [-half, -half, 0]], dtype=np.float32)
    best_dist, best = float("inf"), None
    for i, corner in enumerate(corners):
        ok, _, tvec = cv2.solvePnP(obj_pts, corner[0], camera_mat, dist_coeffs)
        if not ok:
            continue
        dist = float(np.linalg.norm(tvec))
        if dist < best_dist:
            best_dist = dist
            best = (int(ids[i][0]), dist, tvec.flatten().tolist())
    return best


# ── Video test mode (no ROS) ───────────────────────────────────────────────────

_COLOR_BGR = {
    "red":     (0,   0,   255),
    "green":   (0,   200,   0),
    "blue":    (255,  80,   0),
    "white":   (255, 255, 255),
    "unknown": (180, 180, 180),
}


def _annotate_frame(frame: np.ndarray, boxes, names: dict, crop_model,
                    log_writer=None, frame_idx: int = 0,
                    blink_detector: BlinkDetector = None,
                    video_ts: float = None,
                    save_crops_dir: str = None,
                    target_color: str = None,
                    target_blinking=None,
                    aruco_gt=None,
                    img_w=None, img_h=None) -> np.ndarray:
    clean = frame.copy()  # unmodified source for crops — keeps drawn annotations out of saved images
    for det_idx, box in enumerate(boxes):
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        conf  = float(box.conf[0])
        cls   = int(box.cls[0])
        label = names.get(cls, str(cls))

        crop = clean[max(y1, 0):max(y2, 1), max(x1, 0):max(x2, 1)]
        beacon_color, color_conf, light_mask, intensity, votes, lit_region = isolate_and_classify(crop, crop_model)

        draw_color = _COLOR_BGR.get(beacon_color, (180, 180, 180))

        blink_info = None
        if blink_detector is not None:
            ts = video_ts if video_ts is not None else time.time()
            blink_info = blink_detector.update(ts, beacon_color, intensity, color_conf)

        if save_crops_dir is not None and lit_region.size > 0:
            _date = time.strftime("%Y%m%d")
            _gt   = (f"gt-{target_color or 'unk'}-"
                     f"{'blink' if target_blinking is True else 'steady' if target_blinking is False else 'unk'}")
            _b    = blink_info.get("is_blinking") if blink_info else None
            _det  = (f"det-blink{blink_info.get('blink_hz', 0):.2f}hz" if _b is True
                     else "det-steady" if _b is False else "det-acc")
            fname = (f"crop_{_date}_f{frame_idx:06d}_d{det_idx:02d}_{beacon_color}"
                     f"_{_det}_r{int(votes['red']*100)}g{int(votes['green']*100)}b{int(votes['blue']*100)}"
                     f"_{_gt}.png")
            cv2.imwrite(os.path.join(save_crops_dir, fname), lit_region)

        cv2.rectangle(frame, (x1, y1), (x2, y2), draw_color, 2)

        if light_mask is not None and light_mask.any():
            lm_full = np.zeros(frame.shape[:2], dtype=np.uint8)
            lm_h = min(light_mask.shape[0], y2 - y1)
            lm_w = min(light_mask.shape[1], x2 - x1)
            lm_full[y1:y1+lm_h, x1:x1+lm_w] = light_mask[:lm_h, :lm_w]
            contours, _ = cv2.findContours(lm_full, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(frame, contours, -1, draw_color, 2)

        txt = (f"{label} [{beacon_color}] det={conf:.2f} int={intensity:.2f} "
               f"r={votes['red']:.0%} g={votes['green']:.0%} b={votes['blue']:.0%}")
        if blink_info:
            if blink_info["is_blinking"]:
                txt += f" blink={blink_info['blink_hz']:.2f}Hz"
            elif blink_info["is_blinking"] is None:
                txt += " blink=?"
        cv2.putText(frame, txt, (x1, max(y1 - 6, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, draw_color, 1)

        print(f"  {txt}  bbox=({x1},{y1},{x2},{y2})")

        if log_writer is not None:
            _write_log_row(log_writer, frame_idx, beacon_color, color_conf,
                           intensity, votes, conf, (x1, y1, x2, y2),
                           blink_info=blink_info,
                           target_color=target_color,
                           target_blinking=target_blinking,
                           gt_info=aruco_gt,
                           img_w=img_w, img_h=img_h)

    return frame


def run_video(cfg: dict) -> None:
    """Run beacon detection on a local video file. No ROS required."""
    video_path      = cfg["video"]
    model_path      = cfg["model"]
    crop_model_path = cfg["crop_model"]
    save_output     = cfg["save"]
    conf            = cfg["conf"]
    log             = cfg["log"]
    save_crops      = cfg.get("save_crops", False)

    try:
        from ultralytics import YOLO
    except ImportError:
        print("[beacon-video] ultralytics not installed: pip install ultralytics")
        return

    if not os.path.exists(model_path):
        print(f"[beacon-video] Model not found: {model_path}")
        return

    if not os.path.exists(crop_model_path):
        print(f"[beacon-video] Crop model not found: {crop_model_path}")
        return

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"[beacon-video] Cannot open video: {video_path}")
        return

    model      = YOLO(model_path)
    crop_model = YOLO(crop_model_path)
    fps    = cap.get(cv2.CAP_PROP_FPS) or 30.0
    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total  = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"[beacon-video] {video_path}  {width}x{height} @ {fps:.1f}fps  {total} frames")
    print(f"[beacon-video] Beacon model : {model_path}  conf≥{conf}")
    print(f"[beacon-video] Crop model   : {crop_model_path}")
    print("[beacon-video] Press 'q' to quit, SPACE to pause")

    writer = None
    if save_output:
        out_path = os.path.splitext(video_path)[0] + "_beacon_out.mp4"
        fourcc   = cv2.VideoWriter_fourcc(*"mp4v")
        writer   = cv2.VideoWriter(out_path, fourcc, fps, (width, height))
        print(f"[beacon-video] Saving output → {out_path}")

    log_fh = log_writer = None
    if log:
        ts = time.strftime("%Y%m%d_%H%M%S")
        log_path = os.path.splitext(video_path)[0] + f"_beacon_log_{ts}.csv"
        log_fh, log_writer = _open_log(log_path)

    crops_dir = None
    if save_crops:
        crops_dir = os.path.splitext(video_path)[0] + "_beacon_crops"
        os.makedirs(crops_dir, exist_ok=True)
        print(f"[beacon-video] Saving crops → {crops_dir}/")

    blink_detector = BlinkDetector()
    frame_idx     = 0
    paused        = False
    display_frame = None

    aruco_detector = aruco_camera_mat = aruco_dist_coeffs = None
    if cfg["aruco"].get("enabled"):
        aruco_detector = _build_aruco_detector(cfg["aruco"])
        aruco_camera_mat, aruco_dist_coeffs = _load_aruco_calibration(cfg["aruco"], cfg["camera"])
        print(f"[beacon-video] ArUco ground truth enabled  "
              f"dict={cfg['aruco']['dictionary']}  marker={cfg['aruco']['marker_size_m']}m")

    cv2.namedWindow("Beacon Detector — Video Test", cv2.WINDOW_AUTOSIZE)

    try:
        while True:
            if not paused:
                ret, raw = cap.read()
                if not ret:
                    print("[beacon-video] End of video")
                    break
                frame_idx += 1

                display_frame = raw.copy()
                video_ts = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0

                aruco_gt = None
                if aruco_detector is not None:
                    aruco_gt = detect_aruco_ground_truth(
                        raw, aruco_detector, aruco_camera_mat, aruco_dist_coeffs,
                        cfg["aruco"]["marker_size_m"])

                results = model(raw, conf=conf, verbose=False)
                boxes   = results[0].boxes
                names   = results[0].names

                print(f"Frame {frame_idx}/{total} — {len(boxes)} detection(s)")
                display_frame = _annotate_frame(display_frame, boxes, names, crop_model,
                                                log_writer=log_writer, frame_idx=frame_idx,
                                                blink_detector=blink_detector,
                                                video_ts=video_ts,
                                                save_crops_dir=crops_dir,
                                                target_color=cfg.get("target_color"),
                                                target_blinking=cfg.get("target_blinking"),
                                                aruco_gt=aruco_gt,
                                                img_w=width, img_h=height)

                if writer:
                    writer.write(display_frame)

            if display_frame is not None:
                cv2.imshow("Beacon Detector — Video Test", display_frame)
            key = cv2.waitKey(10 if not paused else 50) & 0xFF
            if key == ord("q"):
                break
            elif key == ord(" "):
                paused = not paused

    except KeyboardInterrupt:
        print("\n[beacon-video] Interrupted")
    finally:
        cap.release()
        if writer:
            writer.release()
        if log_fh:
            log_fh.close()
        cv2.destroyAllWindows()
        print(f"[beacon-video] Done — {frame_idx} frames processed")


# ── Video + ROS mode ──────────────────────────────────────────────────────────

def run_video_ros(cfg: dict) -> None:
    """Read frames from a local video file and publish detections to ROS."""
    video_path      = cfg["ros_video"]
    model_path      = cfg["model"]
    crop_model_path = cfg["crop_model"]
    save_output     = cfg["save"]
    conf            = cfg["conf"]
    log             = cfg["log"]
    display         = cfg["display"]
    topics          = cfg["topics"]
    save_crops      = cfg.get("save_crops", False)
    date_tag = time.strftime("%Y%m%d")
    _tc = cfg.get("target_color") or "unk"
    _tb = cfg.get("target_blinking")
    gt_tag   = f"gt-{_tc}-{'blink' if _tb is True else 'steady' if _tb is False else 'unk'}"

    _import_ros()

    try:
        from ultralytics import YOLO
    except ImportError:
        print("[beacon-ros-video] ultralytics not installed: pip install ultralytics")
        return

    if not os.path.exists(model_path):
        print(f"[beacon-ros-video] Model not found: {model_path}")
        return

    if not os.path.exists(crop_model_path):
        print(f"[beacon-ros-video] Crop model not found: {crop_model_path}")
        return

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"[beacon-ros-video] Cannot open video: {video_path}")
        return

    fps    = cap.get(cv2.CAP_PROP_FPS) or 30.0
    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total  = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    print(f"[beacon-ros-video] {video_path}  {width}x{height} @ {fps:.1f}fps  {total} frames")
    print(f"[beacon-ros-video] Model: {model_path}  conf≥{conf}")
    print(f"[beacon-ros-video] Publishing → {topics['detections_pub']}")
    if display:
        print("[beacon-ros-video] Press 'q' to quit, SPACE to pause")

    writer = None
    if save_output:
        out_path = os.path.splitext(video_path)[0] + "_beacon_ros_out.mp4"
        fourcc   = cv2.VideoWriter_fourcc(*"mp4v")
        writer   = cv2.VideoWriter(out_path, fourcc, fps, (width, height))
        print(f"[beacon-ros-video] Saving output → {out_path}")

    log_fh = log_writer = None
    if log:
        ts = time.strftime("%Y%m%d_%H%M%S")
        log_path = os.path.splitext(video_path)[0] + f"_beacon_log_{ts}.csv"
        log_fh, log_writer = _open_log(log_path)

    crops_dir = None
    if save_crops:
        crops_dir = os.path.splitext(video_path)[0] + "_beacon_crops"
        os.makedirs(crops_dir, exist_ok=True)
        print(f"[beacon-ros-video] Saving crops → {crops_dir}/")

    depth_source = cfg["detection"].get("depth_source", "topic")
    max_dets     = cfg["detection"].get("max_detections")
    det_count    = 0
    blink_detector = BlinkDetector()
    rclpy.init()
    cam        = _make_beacon_camera(topics, cfg["camera"], cfg["detection"])
    model      = YOLO(model_path)
    crop_model = YOLO(crop_model_path)
    print(f"[beacon-ros-video] Beacon model : {model_path}  conf≥{conf}")
    print(f"[beacon-ros-video] Crop model   : {crop_model_path}")

    aruco_detector = aruco_camera_mat = aruco_dist_coeffs = None
    if cfg["aruco"].get("enabled"):
        aruco_detector = _build_aruco_detector(cfg["aruco"])
        aruco_camera_mat, aruco_dist_coeffs = _load_aruco_calibration(cfg["aruco"], cfg["camera"])
        print(f"[beacon-ros-video] ArUco ground truth enabled  "
              f"dict={cfg['aruco']['dictionary']}  marker={cfg['aruco']['marker_size_m']}m")

    if not cam.open_for_video():
        print("[beacon-ros-video] Failed to open ROS node")
        rclpy.shutdown()
        cap.release()
        return

    frame_idx     = 0
    paused        = False
    display_frame = None

    if display:
        cv2.namedWindow("Beacon Detector — Video + ROS", cv2.WINDOW_AUTOSIZE)

    try:
        while rclpy.ok():
            rclpy.spin_once(cam, timeout_sec=0.0)

            if not paused:
                ret, raw = cap.read()
                if not ret:
                    print("[beacon-ros-video] End of video")
                    break
                frame_idx += 1

                drone_pos, _ = cam.get_drone_pose()
                display_frame = raw.copy()
                video_ts = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0

                aruco_gt = None
                if aruco_detector is not None:
                    aruco_gt = detect_aruco_ground_truth(
                        raw, aruco_detector, aruco_camera_mat, aruco_dist_coeffs,
                        cfg["aruco"]["marker_size_m"])
                if aruco_gt is not None:
                    _ag = String()
                    _ag.data = json.dumps({
                        "marker_id": aruco_gt[0],
                        "dist_m":    round(aruco_gt[1], 4),
                        "tvec":      [round(v, 4) for v in aruco_gt[2]],
                        "timestamp": round(time.time(), 3),
                    })
                    cam.aruco_gt_pub.publish(_ag)

                results = model(raw, conf=conf, verbose=False)
                boxes   = results[0].boxes

                print(f"Frame {frame_idx}/{total} — {len(boxes)} detection(s)")

                for det_idx, box in enumerate(boxes):
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    det_conf = float(box.conf[0])

                    crop = raw[max(y1, 0):max(y2, 1), max(x1, 0):max(x2, 1)]
                    beacon_color, color_conf, light_mask, intensity, votes, lit_region = isolate_and_classify(crop, crop_model)
                    draw_color = _COLOR_BGR.get(beacon_color, (180, 180, 180))

                    blink_info = blink_detector.update(video_ts, beacon_color, intensity, color_conf)

                    # Estimate 3D position from bounding box when depth_source == "bbox"
                    pos3d = None
                    if depth_source == "bbox":
                        det_cfg = cfg["detection"]
                        pos3d = estimate_distance_from_bbox(
                            x1, y1, x2, y2,
                            cfg["camera"]["fx"], cfg["camera"]["fy"],
                            cfg["camera"]["cx"], cfg["camera"]["cy"],
                            drone_pos[2] if drone_pos is not None else 0.0,
                            det_cfg.get("beacon_z_m", 0.0),
                            det_cfg.get("beacon_height_m", 0.3048),
                        )

                    cv2.rectangle(display_frame, (x1, y1), (x2, y2), draw_color, 2)

                    if light_mask is not None and light_mask.any():
                        lm_full = np.zeros(display_frame.shape[:2], dtype=np.uint8)
                        lm_h = min(light_mask.shape[0], y2 - y1)
                        lm_w = min(light_mask.shape[1], x2 - x1)
                        lm_full[y1:y1+lm_h, x1:x1+lm_w] = light_mask[:lm_h, :lm_w]
                        contours, _ = cv2.findContours(lm_full, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        cv2.drawContours(display_frame, contours, -1, draw_color, 2)

                    label_txt = (
                        f"beacon [{beacon_color}] det={det_conf:.2f} int={intensity:.2f} "
                        f"r={votes['red']:.0%} g={votes['green']:.0%} b={votes['blue']:.0%}"
                    )
                    if blink_info["is_blinking"]:
                        label_txt += f" blink={blink_info['blink_hz']:.2f}Hz"
                    elif blink_info["is_blinking"] is None:
                        label_txt += " blink=?"
                    if pos3d is not None:
                        label_txt += f" dist={pos3d[2]:.2f}m"
                    cv2.putText(display_frame, label_txt, (x1, max(y1 - 6, 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, draw_color, 1)

                    msg = String()
                    msg.data = json.dumps({
                        "color":            beacon_color,
                        "blink":            blink_info,
                        "label":            "beacon",
                        "color_confidence": color_conf,
                        "intensity":        intensity,
                        "hue_votes":        votes,
                        "confidence":       det_conf,
                        "bbox":             [x1, y1, x2, y2],
                        "position_3d":      pos3d,
                        "world_position":   None,
                        "gps_position":     None,
                        "drone_position":   drone_pos.tolist() if drone_pos is not None else None,
                        "tracking_id":      -1,
                        "timestamp":        time.time(),
                    })
                    cam.detection_pub.publish(msg)
                    print(f"  {label_txt}")
                    det_count += 1
                    if max_dets is not None and det_count >= max_dets:
                        print(f"[beacon-ros-video] Reached max_detections={max_dets} — stopping")
                        break

                    if log_writer is not None:
                        _write_log_row(log_writer, frame_idx, beacon_color, color_conf,
                                       intensity, votes, det_conf, (x1, y1, x2, y2),
                                       blink_info=blink_info,
                                       target_color=cfg.get("target_color"),
                                       target_blinking=cfg.get("target_blinking"),
                                       gt_info=aruco_gt,
                                       img_w=cfg["camera"]["img_w"],
                                       img_h=cfg["camera"]["img_h"])

                    if crops_dir is not None and lit_region.size > 0:
                        _b   = blink_info.get("is_blinking") if blink_info else None
                        _det = (f"det-blink{blink_info.get('blink_hz', 0):.2f}hz" if _b is True
                                else "det-steady" if _b is False else "det-acc")
                        fname = (f"crop_{date_tag}_f{frame_idx:06d}_d{det_idx:02d}_{beacon_color}"
                                 f"_{_det}_r{int(votes['red']*100)}g{int(votes['green']*100)}b{int(votes['blue']*100)}"
                                 f"_{gt_tag}.png")
                        cv2.imwrite(os.path.join(crops_dir, fname), lit_region)

                if writer:
                    writer.write(display_frame)

            if max_dets is not None and det_count >= max_dets:
                break

            if display:
                if display_frame is not None:
                    cv2.imshow("Beacon Detector — Video + ROS", display_frame)
                key = cv2.waitKey(10 if not paused else 50) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord(" "):
                    paused = not paused

    except KeyboardInterrupt:
        print("\n[beacon-ros-video] Interrupted")
    finally:
        cap.release()
        if writer:
            writer.release()
        if log_fh:
            log_fh.close()
        cam.close()
        if display:
            cv2.destroyAllWindows()
        rclpy.shutdown()
        print(f"[beacon-ros-video] Done — {frame_idx} frames processed")


# ── Main loop (ROS live mode) ─────────────────────────────────────────────────

def main(cfg: dict) -> None:
    model_path      = cfg["model"]
    crop_model_path = cfg["crop_model"]
    display         = cfg["display"]
    log             = cfg["log"]
    topics          = cfg["topics"]

    _import_ros()

    DEBUG_DIR    = os.path.expanduser("seabird_dataset/beacon_debug")
    SAVE_EVERY_N = 30
    os.makedirs(DEBUG_DIR, exist_ok=True)

    rclpy.init()
    cam = _make_beacon_camera(topics, cfg["camera"], cfg["detection"])

    if not cam.open():
        print("[beacon] Failed to open camera")
        rclpy.shutdown()
        return

    if not os.path.exists(model_path):
        print(f"[beacon] Model not found: {model_path}")
        cam.close()
        rclpy.shutdown()
        return

    if not os.path.exists(crop_model_path):
        print(f"[beacon] Crop model not found: {crop_model_path}")
        cam.close()
        rclpy.shutdown()
        return

    from ultralytics import YOLO
    print(f"[beacon] Loading model: {model_path}")
    print(f"[beacon] Loading crop model: {crop_model_path}")
    crop_model = YOLO(crop_model_path)
    if not cam.enable_detection(model_path):
        print("[beacon] Detection failed to start")
        cam.close()
        rclpy.shutdown()
        return

    print("[beacon] Detection ENABLED — class: beacon (color determined by CV)")
    print(f"[beacon] Publishing → {topics['detections_pub']}")

    if display:
        cv2.namedWindow("Beacon Detector", cv2.WINDOW_AUTOSIZE)

    log_fh = log_writer = None
    if log:
        ts = time.strftime("%Y%m%d_%H%M%S")
        log_path = os.path.join(DEBUG_DIR, f"beacon_log_{ts}.csv")
        log_fh, log_writer = _open_log(log_path)

    save_crops = cfg.get("save_crops", False)
    crops_dir = None
    if save_crops:
        crops_dir = os.path.join(DEBUG_DIR, "beacon_crops")
        os.makedirs(crops_dir, exist_ok=True)
        print(f"[beacon] Saving crops → {crops_dir}/")

    save_det_images = cfg.get("save_det_images", False)
    det_images_dir = None
    if save_det_images:
        det_images_dir = os.path.join(DEBUG_DIR, "beacon_det_images")
        os.makedirs(det_images_dir, exist_ok=True)
        print(f"[beacon] Saving detection images → {det_images_dir}/")

    save_frames = cfg.get("save_frames", False)
    frames_dir = None
    if save_frames:
        frames_dir = os.path.join(DEBUG_DIR, "beacon_frames")
        os.makedirs(frames_dir, exist_ok=True)
        print(f"[beacon] Saving full frames → {frames_dir}/")

    date_tag = time.strftime("%Y%m%d")
    _tc = cfg.get("target_color") or "unk"
    _tb = cfg.get("target_blinking")
    gt_tag   = f"gt-{_tc}-{'blink' if _tb is True else 'steady' if _tb is False else 'unk'}"

    aruco_detector = aruco_camera_mat = aruco_dist_coeffs = None
    if cfg["aruco"].get("enabled"):
        aruco_detector = _build_aruco_detector(cfg["aruco"])
        aruco_camera_mat, aruco_dist_coeffs = _load_aruco_calibration(cfg["aruco"], cfg["camera"])
        print(f"[beacon] ArUco ground truth enabled  "
              f"dict={cfg['aruco']['dictionary']}  marker={cfg['aruco']['marker_size_m']}m")

    depth_source  = cfg["detection"].get("depth_source", "topic")
    max_dets      = cfg["detection"].get("max_detections")
    frame_count   = 0
    det_count     = 0
    intrinsics_printed = False

    try:
        while rclpy.ok():
            if not cam.grab():
                continue

            frame_count += 1
            rgb        = cam.get_rgb()
            depth      = cam.get_depth()
            intr       = cam._intrinsics
            drone_pos, drone_quat = cam.get_drone_pose()
            frame_ts   = cam.get_frame_timestamp() or time.time()

            if intr and not intrinsics_printed:
                print(f"[beacon] Intrinsics ready: {intr.width}x{intr.height} "
                      f"fx={intr.fx:.1f} fy={intr.fy:.1f}")
                intrinsics_printed = True

            if rgb is None:
                continue

            aruco_gt = None
            if aruco_detector is not None:
                aruco_gt = detect_aruco_ground_truth(
                    rgb, aruco_detector, aruco_camera_mat, aruco_dist_coeffs,
                    cfg["aruco"]["marker_size_m"])
            if aruco_gt is not None:
                _ag = String()
                _ag.data = json.dumps({
                    "marker_id": aruco_gt[0],
                    "dist_m":    round(aruco_gt[1], 4),
                    "tvec":      [round(v, 4) for v in aruco_gt[2]],
                    "timestamp": round(time.time(), 3),
                })
                cam.aruco_gt_pub.publish(_ag)

            dets = cam.get_detections()
            rgb_clean = rgb.copy()  # snapshot before drawing so crops are annotation-free

            if frames_dir is not None and dets:
                fname = f"frame_{date_tag}_f{frame_count:06d}_{gt_tag}.png"
                cv2.imwrite(os.path.join(frames_dir, fname), rgb_clean)

            for d in dets:
                x1, y1, x2, y2 = [int(v) for v in d.bbox_2d]

                # Resolve 3D position — use depth topic result, or fall back to bbox estimation
                pos3d = d.position_3d
                if pos3d is None and depth_source == "bbox":
                    det_cfg = cfg["detection"]
                    pos3d = estimate_distance_from_bbox(
                        x1, y1, x2, y2,
                        cfg["camera"]["fx"], cfg["camera"]["fy"],
                        cfg["camera"]["cx"], cfg["camera"]["cy"],
                        drone_pos[2] if drone_pos is not None else 0.0,
                        det_cfg.get("beacon_z_m", 0.0),
                        det_cfg.get("beacon_height_m", 0.3048),
                    )

                crop = rgb_clean[max(y1, 0):max(y2, 1), max(x1, 0):max(x2, 1)]
                beacon_color, color_conf, light_mask, intensity, votes, lit_region = isolate_and_classify(crop, crop_model)
                if crops_dir is not None and lit_region.size > 0:
                    fname = f"crop_{date_tag}_f{frame_count:06d}_t{d.tracking_id:02d}_{beacon_color}_{gt_tag}.png"
                    cv2.imwrite(os.path.join(crops_dir, fname), lit_region)
                blink_info = _get_blink_detector(d.tracking_id).update(
                    frame_ts, beacon_color, intensity, color_conf
                )

                draw_color = _COLOR_BGR.get(beacon_color, (180, 180, 180))

                cv2.rectangle(rgb, (x1, y1), (x2, y2), draw_color, 2)

                if light_mask is not None and light_mask.any():
                    lm_full = np.zeros(rgb.shape[:2], dtype=np.uint8)
                    lm_h = min(light_mask.shape[0], y2 - y1)
                    lm_w = min(light_mask.shape[1], x2 - x1)
                    lm_full[y1:y1+lm_h, x1:x1+lm_w] = light_mask[:lm_h, :lm_w]
                    contours, _ = cv2.findContours(lm_full, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(rgb, contours, -1, draw_color, 2)

                label_txt = (
                    f"beacon [{beacon_color}] conf={d.confidence:.2f} int={intensity:.2f} "
                    f"r={votes['red']:.0%} g={votes['green']:.0%} b={votes['blue']:.0%}"
                )
                if blink_info["is_blinking"]:
                    label_txt += f" blink={blink_info['blink_hz']:.2f}Hz"
                elif blink_info["is_blinking"] is None:
                    label_txt += " blink=?"
                if d.tracking_id >= 0:
                    label_txt += f" #{d.tracking_id}"

                cv2.putText(rgb, label_txt, (x1, y1 - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, draw_color, 1)

                world_pos  = None
                gps_coords = None
                if pos3d is not None and drone_pos is not None:
                    world_pos = _camera_to_world(
                        pos3d, drone_pos, drone_quat,
                        cfg["camera"]["_mount_offset"],
                        cfg["camera"]["_R_body_to_cam"],
                    )
                    origin    = cam.get_gps_origin()
                    if origin is not None:
                        lat, lon, alt = local_enu_to_gps(world_pos, *origin)
                        gps_coords    = {"latitude": lat, "longitude": lon, "altitude": alt}

                    cv2.putText(
                        rgb,
                        f"W({world_pos[0]:.1f},{world_pos[1]:.1f},{world_pos[2]:.1f})",
                        (x1, y2 + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, draw_color, 1,
                    )

                msg = String()
                msg.data = json.dumps({
                    "color":            beacon_color,
                    "blink":            blink_info,
                    "label":            "beacon",
                    "color_confidence": color_conf,
                    "intensity":        intensity,
                    "hue_votes":        votes,
                    "confidence":       float(d.confidence),
                    "bbox":           [int(v) for v in d.bbox_2d],
                    "position_3d":    list(pos3d) if pos3d is not None else None,
                    "world_position": world_pos.tolist()     if world_pos     is not None else None,
                    "gps_position":   gps_coords,
                    "drone_position": drone_pos.tolist()     if drone_pos     is not None else None,
                    "tracking_id":    int(d.tracking_id),
                    "timestamp":      time.time(),
                })
                cam.detection_pub.publish(msg)
                det_count += 1
                if max_dets is not None and det_count >= max_dets:
                    print(f"[beacon] Reached max_detections={max_dets} — stopping")
                    break

                if log_writer is not None:
                    _write_log_row(log_writer, frame_count, beacon_color, color_conf,
                                   intensity, votes, float(d.confidence), d.bbox_2d,
                                   tracking_id=d.tracking_id,
                                   pos3d=pos3d,
                                   blink_info=blink_info,
                                   target_color=cfg.get("target_color"),
                                   target_blinking=cfg.get("target_blinking"),
                                   gt_info=aruco_gt,
                                   img_w=cfg["camera"]["img_w"],
                                   img_h=cfg["camera"]["img_h"])

                if crops_dir is not None and lit_region.size > 0:
                    _b   = blink_info.get("is_blinking")
                    _det = (f"det-blink{blink_info.get('blink_hz', 0):.2f}hz" if _b is True
                            else "det-steady" if _b is False else "det-acc")
                    fname = (f"crop_{date_tag}_f{frame_count:06d}_t{d.tracking_id:02d}_{beacon_color}"
                             f"_{_det}_r{int(votes['red']*100)}g{int(votes['green']*100)}b{int(votes['blue']*100)}"
                             f"_{gt_tag}.png")
                    cv2.imwrite(os.path.join(crops_dir, fname), lit_region)

                if det_images_dir is not None:
                    pad = 20
                    h_img, w_img = rgb_clean.shape[:2]
                    ix1 = max(x1 - pad, 0)
                    iy1 = max(y1 - pad, 0)
                    ix2 = min(x2 + pad, w_img)
                    iy2 = min(y2 + pad, h_img)
                    det_crop = rgb_clean[iy1:iy2, ix1:ix2].copy()
                    blink_tag = ""
                    if blink_info.get("is_blinking") is True:
                        blink_tag = f"_blink{blink_info['blink_hz']:.2f}hz"
                    elif blink_info.get("is_blinking") is False:
                        blink_tag = "_steady"
                    fname = (f"det_{date_tag}_f{frame_count:06d}_t{d.tracking_id:02d}_{beacon_color}"
                             f"_conf{int(d.confidence*100):02d}{blink_tag}_{gt_tag}.png")
                    cv2.imwrite(os.path.join(det_images_dir, fname), det_crop)

                print(f"[beacon] {label_txt}"
                      + (f" dist={pos3d[2]:.2f}m" if pos3d is not None else ""))

            if max_dets is not None and det_count >= max_dets:
                break

            if display and rgb is not None:
                cv2.imshow("Beacon Detector", rgb)
                if cv2.waitKey(10) & 0xFF == ord("q"):
                    break

            if frame_count % SAVE_EVERY_N == 0 and rgb is not None:
                out_path = os.path.join(DEBUG_DIR, f"frame_{frame_count:06d}.png")
                cv2.imwrite(out_path, rgb)
                print(f"[beacon] Saved {out_path}")

    except KeyboardInterrupt:
        print("\n[beacon] Interrupted")
    finally:
        if log_fh:
            log_fh.close()
        cam.close()
        cv2.destroyAllWindows()
        rclpy.shutdown()
        print(f"[beacon] Done — {frame_count} frames processed")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="beacon_detector_config",
        description="Beacon detector configured via a JSON file",
    )
    parser.add_argument(
        "--config", "-cfg",
        default=_DEFAULT_CONFIG,
        metavar="CONFIG_PATH",
        help=f"Path to JSON config file (default: {_DEFAULT_CONFIG})",
    )
    args = parser.parse_args()

    if not os.path.exists(args.config):
        print(f"[beacon] Config not found: {args.config}")
        print(f"[beacon] Create one based on beacon_config.json or pass --config <path>")
        sys.exit(1)

    cfg = load_config(args.config)
    print(f"[beacon] Loaded config: {args.config}")
    
    if cfg["ros_video"] is not None:
        run_video_ros(cfg)
    elif cfg["video"] is not None:
        run_video(cfg)
    else:
        main(cfg)
