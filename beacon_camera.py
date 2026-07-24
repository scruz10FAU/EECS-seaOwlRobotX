#!/usr/bin/env python3
"""
beacon_camera.py — ROS2 BeaconCamera node for ZED camera topic subscriptions.

Imported by beacon_detector.py inside _import_ros() so that ROS packages are
only loaded when ROS mode is actually invoked.
"""

import threading
import numpy as np

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPointStamped
from std_msgs.msg import String
import message_filters

from camera_interface import CameraInterface, CameraConfig, Detection, Intrinsics
from seabird_config import IMG_W, IMG_H, FX, FY, CX, CY
from yolo_detector import YoloDetector

_bridge = CvBridge()

DEFAULT_TOPIC_PREFIX = "/zed/zed_node"
DRONE_POSE_TOPIC     = "/mavros/local_position/pose"
GPS_TOPIC            = "/mavros/global_position/gp_origin"


class BeaconCamera(Node):
    """
    Subscribes to ZED camera topics and runs beacon detection + color classification.

    Publishes:
        /seabird/beacon_detections  — JSON with label "beacon", detected color, position
    """

    def __init__(self, topic_prefix=DEFAULT_TOPIC_PREFIX):
        super().__init__("beacon_camera")
        self._topic_prefix = topic_prefix

        self._rgb        = None
        self._depth      = None
        self._intrinsics = None
        self._new_frame  = False
        self._frame_ts   = None
        self._frame_lock = threading.Lock()

        self._drone_pos       = None
        self._drone_quat_wxyz = None
        self._pose_lock       = threading.Lock()

        self._gps_origin      = None
        self._gps_origin_lock = threading.Lock()

        self._is_open   = False
        self._detector  = None
        self.detection_pub = None

    # ── Lifecycle ──────────────────────────────────────────────────────────

    def open(self):
        if self._is_open:
            return True

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._info_sub = self.create_subscription(
            CameraInfo,
            f"{self._topic_prefix}/rgb/color/rect/camera_info",
            self._on_camera_info,
            qos,
        )

        rgb_sub   = message_filters.Subscriber(
            self, Image, f"{self._topic_prefix}/rgb/color/rect/image", qos_profile=qos
        )
        depth_sub = message_filters.Subscriber(
            self, Image, f"{self._topic_prefix}/depth/depth_registered", qos_profile=qos
        )
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=5, slop=0.05
        )
        self._sync.registerCallback(self._on_synced_frame)

        self.detection_pub = self.create_publisher(String, "/seabird/beacon_detections", 10)

        self._pose_sub = self.create_subscription(
            PoseStamped, DRONE_POSE_TOPIC, self._on_drone_pose, qos
        )

        origin_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self._origin_sub = self.create_subscription(
            GeoPointStamped, GPS_TOPIC, self._on_gps_origin, origin_qos
        )

        self._is_open = True
        self.get_logger().info("BeaconCamera open — waiting for frames…")
        return True

    def open_for_video(self):
        """
        Minimal ROS setup for video-file input mode.
        Creates the detection publisher and subscribes to drone pose + GPS,
        but skips all camera image subscriptions (frames come from cv2.VideoCapture).
        """
        if self._is_open:
            return True

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.detection_pub = self.create_publisher(String, "/seabird/beacon_detections", 10)

        self._pose_sub = self.create_subscription(
            PoseStamped, DRONE_POSE_TOPIC, self._on_drone_pose, qos
        )

        origin_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self._origin_sub = self.create_subscription(
            GeoPointStamped, GPS_TOPIC, self._on_gps_origin, origin_qos
        )

        self._is_open = True
        self.get_logger().info("BeaconCamera open (video-file mode) — pose + GPS only")
        return True

    def close(self):
        self._is_open = False

    def grab(self):
        if not self._is_open:
            return False
        rclpy.spin_once(self, timeout_sec=0.05)
        with self._frame_lock:
            if self._new_frame:
                self._new_frame = False
                return True
        return False

    def enable_detection(self, model_path, imgsz=640):
        self._detector = YoloDetector(
            weights=model_path,
            class_names=["beacon"],
            imgsz=imgsz,
            conf_thresh=0.5,
        )
        ok = self._detector.start(enable_tracking=True)
        if not ok:
            self._detector = None
            self.get_logger().error("YoloDetector failed to start")
        return ok

    def get_rgb(self):
        with self._frame_lock:
            return self._rgb.copy() if self._rgb is not None else None

    def get_depth(self):
        with self._frame_lock:
            return self._depth.copy() if self._depth is not None else None

    def get_frame_timestamp(self) -> float:
        with self._frame_lock:
            return self._frame_ts

    def get_drone_pose(self):
        with self._pose_lock:
            if self._drone_pos is None:
                return None, None
            return self._drone_pos.copy(), self._drone_quat_wxyz.copy()

    def get_gps_origin(self):
        with self._gps_origin_lock:
            return self._gps_origin

    def get_detections(self):
        if self._detector is None:
            return []
        with self._frame_lock:
            rgb   = self._rgb.copy()   if self._rgb   is not None else None
            depth = self._depth.copy() if self._depth is not None else None
        if rgb is None:
            return []
        return self._detector.detect(rgb, depth, self._intrinsics)

    # ── ROS2 Callbacks ──────────────────────────────────────────────────────

    def _on_camera_info(self, _msg):
        if self._intrinsics is not None:
            return
        self._intrinsics = Intrinsics(
            fx=FX, fy=FY, cx=CX, cy=CY, width=IMG_W, height=IMG_H
        )
        self.get_logger().info(
            f"Intrinsics set from config: fx={FX:.1f} fy={FY:.1f} "
            f"cx={CX:.1f} cy={CY:.1f} {IMG_W}x{IMG_H}"
        )
        self.destroy_subscription(self._info_sub)

    def _on_synced_frame(self, rgb_msg, depth_msg):
        if rgb_msg.encoding in ('bgr8', 'yuv422', 'yuv422_yuy2'):
            bgr = _bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        else:
            channels = len(rgb_msg.data) // (rgb_msg.height * rgb_msg.width)
            rgb = np.frombuffer(rgb_msg.data, dtype=np.uint8).reshape(
                rgb_msg.height, rgb_msg.width, channels
            )
            bgr = rgb[:, :, :3][:, :, ::-1].copy()
        enc = depth_msg.encoding
        if enc == '32FC1':
            depth = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(
                depth_msg.height, depth_msg.width
            ).copy()
        elif enc == '16UC1':
            depth = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(
                depth_msg.height, depth_msg.width
            ).astype(np.float32) * 0.001
        else:
            depth = np.frombuffer(depth_msg.data, dtype=np.uint8).reshape(
                depth_msg.height, depth_msg.width
            ).astype(np.float32)
        if depth.shape != (bgr.shape[0], bgr.shape[1]):
            import cv2 as _cv2
            depth = _cv2.resize(depth, (bgr.shape[1], bgr.shape[0]),
                                interpolation=_cv2.INTER_NEAREST)
        ts = rgb_msg.header.stamp.sec + rgb_msg.header.stamp.nanosec * 1e-9
        with self._frame_lock:
            self._rgb       = bgr
            self._depth     = depth
            self._frame_ts  = ts
            self._new_frame = True

    def _on_drone_pose(self, msg):
        p, q = msg.pose.position, msg.pose.orientation
        with self._pose_lock:
            self._drone_pos       = np.array([p.x, p.y, p.z])
            self._drone_quat_wxyz = np.array([q.w, q.x, q.y, q.z])

    def _on_gps_origin(self, msg):
        with self._gps_origin_lock:
            self._gps_origin = (
                msg.position.latitude,
                msg.position.longitude,
                msg.position.altitude,
            )
        self.get_logger().info(
            f"GPS origin: lat={msg.position.latitude:.7f} "
            f"lon={msg.position.longitude:.7f}"
        )
