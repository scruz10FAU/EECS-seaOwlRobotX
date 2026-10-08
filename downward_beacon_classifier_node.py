#!/usr/bin/env python3
"""
downward_beacon_classifier_node.py — one-shot color/blink/GPS classification
of a beacon's circular top face, using a downward-facing camera while the
drone hovers directly above it. Assumes another script already positioned
the drone there — this script only classifies.

Reuses beacon_detector_config.py's JSON config loader (for camera geometry
and detection thresholds) and BeaconCamera.open_for_video() — the same
minimal "pose + GPS only, no image/depth sync" mode already used for
video-file testing elsewhere in this repo — rather than writing a new
pose/GPS pipeline. Only the downward image subscription
(_DownwardImageGrabber below) is new; the actual classification logic lives
in utils/downward_beacon_classifier.py (pure, no ROS dependency).

Publishes one result to topics.detections_pub using the same JSON schema the
rest of the pipeline uses (color, blink, gps_position, world_position), so
it's immediately usable by beacon_mavlink_bridge.py and anything else
already consuming that topic.

Usage:
    python3 downward_beacon_classifier_node.py --config configs/my_config.json
    python3 downward_beacon_classifier_node.py --config configs/my_config.json --duration 8.0
"""

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import beacon_detector_config as bdc
from utils.downward_beacon_classifier import classify_beacon_downward

_bridge = CvBridge()


class _DownwardImageGrabber(Node):
    """
    Minimal node subscribing only to the downward camera's image topic,
    buffering (timestamp, bgr_frame) pairs. Deliberately far lighter than
    BeaconCamera -- no depth sync, no pose/gps, no detector backend.
    """

    def __init__(self, image_topic: str):
        super().__init__("downward_image_grabber")
        self._frames = []
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, image_topic, self._on_image, qos)
        self.get_logger().info(f"Subscribed to downward camera: {image_topic}")

    def _on_image(self, msg: Image) -> None:
        bgr = _bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._frames.append((ts, bgr))

    def drain_frames(self):
        """Return and clear all frames buffered so far."""
        out, self._frames[:] = list(self._frames), []
        return out


def _wait_for_pose_data(pose_source, timeout_s: float) -> bool:
    """Poll until get_gps_origin()/get_drone_height_agl() both have data, or timeout."""
    t_end = time.time() + timeout_s
    while time.time() < t_end:
        rclpy.spin_once(pose_source, timeout_sec=0.1)
        if pose_source.get_gps_origin() is not None and pose_source.get_drone_height_agl() is not None:
            return True
    return False


def classify_beacon_at_current_position(pose_source, image_grabber, cfg: dict,
                                         duration_s: float) -> dict:
    """
    Spin for duration_s seconds collecting downward-camera frames, then run
    classify_beacon_downward() against them using the drone's pose/GPS/height
    sampled at the end of that window.

    pose_source: anything exposing get_drone_pose(), get_gps_origin(),
                 get_drone_height_agl() -- e.g. a BeaconCamera opened via
                 open_for_video() (pose + GPS only, no image/depth).
    image_grabber: a _DownwardImageGrabber instance, already subscribed.
    """
    t_end = time.time() + duration_s
    while time.time() < t_end:
        rclpy.spin_once(pose_source, timeout_sec=0.02)
        rclpy.spin_once(image_grabber, timeout_sec=0.02)

    frames = image_grabber.drain_frames()
    drone_pos, drone_quat = pose_source.get_drone_pose()
    gps_origin = pose_source.get_gps_origin()
    drone_height_agl = pose_source.get_drone_height_agl()

    return classify_beacon_downward(
        frames, drone_pos, drone_quat, gps_origin, drone_height_agl,
        cfg["downward_camera"], cfg["detection"], duration_s,
    )


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--config", default=bdc._DEFAULT_CONFIG,
                     help=f"JSON config path (default: {bdc._DEFAULT_CONFIG})")
    ap.add_argument("--duration", type=float, default=8.0,
                     help="Seconds to sample the downward camera before classifying (default: 8.0)")
    ap.add_argument("--pose-wait-timeout", type=float, default=15.0,
                     help="Max seconds to wait for drone height/GPS origin before giving up (default: 15.0)")
    args = ap.parse_args()

    cfg = bdc.load_config(args.config)
    down_cfg = cfg["downward_camera"]
    if not down_cfg.get("image_topic"):
        print("[downward] ERROR: downward_camera.image_topic is not set in the config")
        return

    bdc._import_ros()  # populates beacon_detector_config._BeaconCameraBase
    rclpy.init()

    pose_source = bdc._make_beacon_camera(cfg["topics"], cfg["camera"], cfg["detection"])
    pose_source.open_for_video()  # pose + GPS only, no image/depth sync
    image_grabber = _DownwardImageGrabber(down_cfg["image_topic"])

    print(f"[downward] Waiting up to {args.pose_wait_timeout:.0f}s for GPS origin / AGL height...")
    if not _wait_for_pose_data(pose_source, args.pose_wait_timeout):
        print("[downward] ERROR: GPS origin and/or AGL height never arrived -- aborting")
        pose_source.destroy_node()
        image_grabber.destroy_node()
        rclpy.shutdown()
        return

    print(f"[downward] Sampling downward camera for {args.duration:.1f}s...")
    result = classify_beacon_at_current_position(pose_source, image_grabber, cfg, args.duration)
    print(f"[downward] Result: {json.dumps(result, indent=2)}")

    if result["n_frames_used"] > 0:
        msg = String()
        msg.data = json.dumps(result)
        pose_source.detection_pub.publish(msg)
        print(f"[downward] Published to {cfg['topics']['detections_pub']}")
    else:
        print("[downward] No frames matched the expected circle -- nothing published")

    pose_source.destroy_node()
    image_grabber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
