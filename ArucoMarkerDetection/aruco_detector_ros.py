#!/usr/bin/env python3
"""
aruco_detector_ros.py — ArUco marker detector driven by a JSON config file.

Mirrors the structure of beacon_detector_config.py: camera topics and intrinsics
are loaded from a JSON config; a runtime BeaconCamera subclass is created via
closure so beacon_camera.py and seabird_config.py are not modified.

Run modes
---------
  ROS2 live      python3 aruco_detector_ros.py
  Video file     python3 aruco_detector_ros.py --video path/to/video.mp4
  Custom config  python3 aruco_detector_ros.py --config my_config.json

Published topic
---------------
  <topics.aruco_pub>  — JSON string per frame, e.g.:
      {"timestamp": 1.23, "markers": [{"id": 3, "corners": [...],
                                        "rvec": [...], "tvec": [...]}]}
"""

import sys
import os
import argparse
import json
import threading

import numpy as np
import cv2

# Allow imports from the repo root (beacon_camera.py, camera_interface.py, …)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

_DEFAULT_CONFIG = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "configs", "aruco_config.json"
)

_ARUCO_DICTS = {
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

_DEFAULT_TOPICS = {
    "camera_prefix": "/hires_front_small_color",
    "image":         "/hires_front_small_color",
    "drone_pose":    "/mavros/local_position/pose",
    "gps_origin":    "/mavros/global_position/gp_origin",
    "aruco_pub":     "/seabird/aruco_detections",
}

_DEFAULT_CAMERA = {
    "focal_length_mm": 2.1,
    "h_aperture_mm":   6.0,
    "v_aperture_mm":   4.5,
    "img_w":           640,
    "img_h":           480,
}

_DEFAULT_ARUCO = {
    "dictionary":      "DICT_4X4_50",
    "marker_size_m":   0.15,
    "calibration_file": None,
    "display":         False,
    "log":             False,
    "video":           None,
}

# ---------------------------------------------------------------------------
# Config loader
# ---------------------------------------------------------------------------

def _merge_topics(raw: dict) -> dict:
    # raw keys override defaults; no auto-derivation from prefix
    # (the image topic for this camera is the prefix itself, not a sub-topic)
    return {**_DEFAULT_TOPICS, **raw}


def load_config(path: str) -> dict:
    """
    Load JSON config. Missing keys fall back to defaults.
    Derives camera intrinsics (fx, fy, cx, cy) from physical lens parameters.
    """
    with open(path) as fh:
        raw = json.load(fh)

    cfg = {
        "topics": _merge_topics(raw.get("topics", {})),
        "camera": {**_DEFAULT_CAMERA, **raw.get("camera", {})},
        "aruco":  {**_DEFAULT_ARUCO,  **raw.get("aruco",  {})},
    }

    cam = cfg["camera"]
    fl, ha, va = cam["focal_length_mm"], cam["h_aperture_mm"], cam["v_aperture_mm"]
    w, h = cam["img_w"], cam["img_h"]
    cam["fx"] = fl * w / ha
    cam["fy"] = fl * h / va
    cam["cx"] = w / 2.0
    cam["cy"] = h / 2.0

    return cfg


# ---------------------------------------------------------------------------
# Lazy ROS2 import (mirrors beacon_detector_config.py)
# ---------------------------------------------------------------------------

rclpy          = None
String         = None
_BeaconCameraBase = None


def _import_ros():
    global rclpy, String, _BeaconCameraBase
    import rclpy as _rclpy;                      rclpy = _rclpy
    from std_msgs.msg import String as _S;       String = _S
    from beacon_camera import BeaconCamera as _BC; _BeaconCameraBase = _BC


# ---------------------------------------------------------------------------
# ArUco camera subclass factory
# ---------------------------------------------------------------------------

def _make_aruco_camera(topics: dict, cfg_camera: dict):
    """
    Return a BeaconCamera subclass whose ROS topic subscriptions and camera
    intrinsics come from the JSON config instead of seabird_config.py.

    Depth is not used for ArUco detection, so this subscribes to the image topic
    directly (no ApproximateTimeSynchronizer) — frames arrive immediately without
    waiting for a paired depth message.
    """
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import PoseStamped
    from geographic_msgs.msg import GeoPointStamped
    from camera_interface import Intrinsics
    from cv_bridge import CvBridge

    image_topic      = topics["image"]
    camera_prefix    = topics["camera_prefix"]
    drone_pose_topic = topics.get("drone_pose", "")
    gps_topic        = topics.get("gps_origin", "")
    aruco_topic      = topics.get("aruco_pub", "/seabird/aruco_detections")

    fx, fy = cfg_camera["fx"],   cfg_camera["fy"]
    cx, cy = cfg_camera["cx"],   cfg_camera["cy"]
    img_w  = cfg_camera["img_w"]
    img_h  = cfg_camera["img_h"]

    class _ArucoCamera(_BeaconCameraBase):
        def __init__(self, topic_prefix=camera_prefix):
            import rclpy.node as _node
            _node.Node.__init__(self, "aruco_camera")
            self._topic_prefix     = topic_prefix
            self._rgb              = None
            self._depth            = None
            self._intrinsics       = None
            self._new_frame        = False
            self._frame_ts         = None
            self._frame_lock       = threading.Lock()
            self._drone_pos        = None
            self._drone_quat_wxyz  = None
            self._pose_lock        = threading.Lock()
            self._gps_origin       = None
            self._gps_origin_lock  = threading.Lock()
            self._is_open          = False
            self._detector         = None
            self.detection_pub     = None
            self._bridge           = CvBridge()

        def _on_image_frame(self, msg):
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ts  = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            with self._frame_lock:
                self._rgb       = bgr
                self._frame_ts  = ts
                self._new_frame = True

        def open(self):
            if self._is_open:
                return True
            self._intrinsics = Intrinsics(
                fx=fx, fy=fy, cx=cx, cy=cy, width=img_w, height=img_h
            )
            self.get_logger().info(
                f"Intrinsics from config: fx={fx:.1f} fy={fy:.1f} "
                f"cx={cx:.1f} cy={cy:.1f} {img_w}×{img_h}"
            )
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self._image_sub = self.create_subscription(
                Image, image_topic, self._on_image_frame, qos
            )
            self.detection_pub = self.create_publisher(String, aruco_topic, 10)
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
            self.get_logger().info(f"ArucoCamera open on {image_topic} — waiting for frames…")
            return True

        def open_for_video(self):
            """Pose + GPS subscriptions only; image frames come from a video file."""
            if self._is_open:
                return True
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.detection_pub = self.create_publisher(String, aruco_topic, 10)
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
            self.get_logger().info("ArucoCamera open (video-file mode)")
            return True

    return _ArucoCamera(topic_prefix=camera_prefix)


# ---------------------------------------------------------------------------
# ArUco detection helpers
# ---------------------------------------------------------------------------

def _build_aruco_detector(aruco_cfg: dict) -> cv2.aruco.ArucoDetector:
    dict_name = aruco_cfg.get("dictionary", "DICT_4X4_50")
    if dict_name not in _ARUCO_DICTS:
        raise ValueError(
            f"Unknown ArUco dictionary '{dict_name}'. "
            f"Valid options: {list(_ARUCO_DICTS)}"
        )
    aruco_dict   = cv2.aruco.getPredefinedDictionary(_ARUCO_DICTS[dict_name])
    aruco_params = cv2.aruco.DetectorParameters()
    return cv2.aruco.ArucoDetector(aruco_dict, aruco_params)


def _camera_matrix(cam_cfg: dict) -> np.ndarray:
    return np.array(
        [[cam_cfg["fx"], 0,             cam_cfg["cx"]],
         [0,             cam_cfg["fy"], cam_cfg["cy"]],
         [0,             0,             1            ]],
        dtype=np.float64,
    )


def _load_calibration(aruco_cfg: dict, cam_cfg: dict):
    """
    Return (camera_matrix, dist_coeffs).
    If aruco.calibration_file is set and the file exists, load measured values
    from the .npz. Otherwise fall back to the theoretical matrix from the config
    with zero distortion.
    """
    cal_path = aruco_cfg.get("calibration_file")
    if cal_path:
        cal_path = os.path.expanduser(cal_path)
        if os.path.isfile(cal_path):
            data = np.load(cal_path)
            print(f"Loaded calibration from {cal_path}")
            return data["camera_matrix"], data["dist_coeffs"]
        print(f"Warning: calibration_file '{cal_path}' not found — using theoretical matrix")
    return _camera_matrix(cam_cfg), np.zeros((4, 1), dtype=np.float64)


def detect_and_annotate(frame_bgr, detector, camera_mat, dist_coeffs, marker_size_m):
    """
    Detect ArUco markers in frame_bgr, estimate 6-DoF pose for each, and
    draw detected markers + coordinate axes onto the frame.

    Returns
    -------
    detections : list of dict
        Each dict: {"id": int, "corners": [[x,y]×4], "rvec": [rx,ry,rz],
                    "tvec": [tx,ty,tz], "dist_m": float}
    annotated  : np.ndarray  (frame_bgr with drawings in-place)
    """
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is None:
        return [], frame_bgr

    cv2.aruco.drawDetectedMarkers(frame_bgr, corners, ids)

    half    = marker_size_m / 2.0
    obj_pts = np.array(
        [[-half,  half, 0],
         [ half,  half, 0],
         [ half, -half, 0],
         [-half, -half, 0]],
        dtype=np.float32,
    )

    detections = []
    for i, corner in enumerate(corners):
        img_pts = corner[0].astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, camera_mat, dist_coeffs)
        if not ok:
            continue
        cv2.drawFrameAxes(
            frame_bgr, camera_mat, dist_coeffs, rvec, tvec, marker_size_m * 0.5
        )
        marker_id = int(ids[i][0])
        dist_m    = float(np.linalg.norm(tvec))
        detections.append({
            "id":      marker_id,
            "corners": corner[0].tolist(),
            "rvec":    rvec.flatten().tolist(),
            "tvec":    tvec.flatten().tolist(),
            "dist_m":  round(dist_m, 4),
        })

    return detections, frame_bgr


# ---------------------------------------------------------------------------
# Run modes
# ---------------------------------------------------------------------------

def run_ros(cfg: dict):
    """Subscribe to ZED camera via ROS2, detect ArUco markers, publish results."""
    _import_ros()
    rclpy.init()

    cam         = _make_aruco_camera(cfg["topics"], cfg["camera"])
    aruco_cfg   = cfg["aruco"]
    detector    = _build_aruco_detector(aruco_cfg)
    camera_mat, dist_coeffs = _load_calibration(cfg["aruco"], cfg["camera"])
    marker_size = float(aruco_cfg.get("marker_size_m", 0.15))
    display     = bool(aruco_cfg.get("display", False))

    cam.open()
    print("ArUco detector running (ROS2 mode) — Ctrl-C to stop")

    try:
        while rclpy.ok():
            if not cam.grab():
                continue
            frame = cam.get_rgb()
            if frame is None:
                continue
            ts        = cam.get_frame_timestamp()
            annotated = frame.copy()
            detections, annotated = detect_and_annotate(
                annotated, detector, camera_mat, dist_coeffs, marker_size
            )
            if detections:
                msg = String()
                msg.data = json.dumps({"timestamp": ts, "markers": detections})
                cam.detection_pub.publish(msg)
                for d in detections:
                    print(
                        f"  marker {d['id']:3d}  dist={d['dist_m']:.3f}m  "
                        f"tvec={[round(v,3) for v in d['tvec']]}"
                    )
            if display:
                cv2.imshow("ArUco Detection (ROS2)", annotated)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cam.close()
        rclpy.shutdown()
        if display:
            cv2.destroyAllWindows()


def run_video(cfg: dict, video_source):
    """Standalone mode: video file or webcam index (no ROS required)."""
    aruco_cfg   = cfg["aruco"]
    detector    = _build_aruco_detector(aruco_cfg)
    camera_mat, dist_coeffs = _load_calibration(cfg["aruco"], cfg["camera"])
    marker_size = float(aruco_cfg.get("marker_size_m", 0.15))

    src = 0 if video_source is None else video_source
    cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        print(f"Error: could not open video source {src!r}")
        return

    print(f"ArUco detector running (video mode: {src!r}) — 'q' to stop")
    frame_n = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        frame_n += 1
        detections, annotated = detect_and_annotate(
            frame, detector, camera_mat, dist_coeffs, marker_size
        )
        if detections:
            ids = [d["id"] for d in detections]
            print(
                f"Frame {frame_n}: {len(detections)} marker(s) — "
                f"IDs {ids}  dists {[d['dist_m'] for d in detections]}"
            )
        cv2.imshow("ArUco Detection", annotated)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="ArUco marker detector — ROS2 live or standalone video/webcam"
    )
    parser.add_argument(
        "--config",
        default=_DEFAULT_CONFIG,
        help=f"Path to JSON config file (default: configs/aruco_config.json)",
    )
    parser.add_argument(
        "--video",
        default=None,
        help=(
            "Video file path or webcam index for standalone mode. "
            "Omit to use ROS2 live camera."
        ),
    )
    args = parser.parse_args()

    cfg = load_config(args.config)

    video_src = args.video or cfg["aruco"].get("video")
    if video_src is not None:
        # Convert numeric strings ("0", "1") to int for webcam indices
        try:
            video_src = int(video_src)
        except (ValueError, TypeError):
            pass
        run_video(cfg, video_src)
    else:
        run_ros(cfg)


if __name__ == "__main__":
    main()
