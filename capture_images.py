#!/usr/bin/env python3
"""
capture_images.py — save frames from a ROS2 camera topic to disk.

Subscribes to a sensor_msgs/Image topic (default /hires_front_small_color,
the VOXL front hires camera via voxl_mpa_to_ros2) and writes BGR images,
rate-limited, until --count / --duration is reached or Ctrl+C.

Files: <out>/frame_NNNNN_<sec>.<nsec>.<fmt>  (stamp = image header time)

Usage:
    python3 capture_images.py                              # 1 img/s to captures/<timestamp>/
    python3 capture_images.py --rate 5 --count 200         # 5 img/s, stop after 200
    python3 capture_images.py --rate 0 --duration 10       # every frame for 10 s
    python3 capture_images.py --format png --out ~/data/front_run1
    ROS_DOMAIN_ID=8 python3 capture_images.py              # VOXL bench board domain
"""

import argparse
import os
import time
from datetime import datetime

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

DEFAULT_TOPIC = "/hires_front_small_color"
NO_FRAME_WARN_S = 5.0


class ImageCapture(Node):
    def __init__(self, topic, out_dir, rate_hz, count, fmt):
        super().__init__("image_capture")
        self._bridge = CvBridge()
        self._out_dir = out_dir
        self._min_interval = 1.0 / rate_hz if rate_hz > 0 else 0.0
        self._count = count
        self._fmt = fmt
        self._last_saved = None   # monotonic time of last save
        self.saved = 0
        self.received = 0
        self.done = False

        # VOXL / PX4 publish sensor data BEST_EFFORT -- a RELIABLE subscriber
        # would silently receive nothing.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, topic, self._on_image, qos)
        self.get_logger().info(f"Capturing {topic} -> {out_dir}")

    def _on_image(self, msg):
        if self.done:
            return
        self.received += 1
        now = time.monotonic()
        if self._last_saved is not None and now - self._last_saved < self._min_interval:
            return
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge conversion failed ({msg.encoding}): {exc}",
                                   throttle_duration_sec=5.0)
            return

        name = (f"frame_{self.saved:05d}_{msg.header.stamp.sec}."
                f"{msg.header.stamp.nanosec:09d}.{self._fmt}")
        if not cv2.imwrite(os.path.join(self._out_dir, name), bgr):
            self.get_logger().error(f"Failed to write {name}")
            return
        self._last_saved = now
        self.saved += 1
        if self.saved == 1 or self.saved % 10 == 0:
            self.get_logger().info(f"Saved {self.saved} ({bgr.shape[1]}x{bgr.shape[0]}) -> {name}")
        if self._count and self.saved >= self._count:
            self.done = True


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default=DEFAULT_TOPIC,
                    help=f"sensor_msgs/Image topic (default: {DEFAULT_TOPIC})")
    ap.add_argument("--out", default=None,
                    help="Output directory (default: captures/<YYYYmmdd_HHMMSS>)")
    ap.add_argument("--rate", type=float, default=1.0,
                    help="Max images saved per second (default: 1; 0 = every frame)")
    ap.add_argument("--count", type=int, default=0,
                    help="Stop after this many images (default: 0 = unlimited)")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="Stop after this many seconds (default: 0 = unlimited)")
    ap.add_argument("--format", choices=["jpg", "png"], default="jpg",
                    help="Image format (default: jpg; png is lossless)")
    args = ap.parse_args()

    out_dir = os.path.expanduser(
        args.out or os.path.join("captures", datetime.now().strftime("%Y%m%d_%H%M%S")))
    os.makedirs(out_dir, exist_ok=True)

    rclpy.init()
    node = ImageCapture(args.topic, out_dir, args.rate, args.count, args.format)
    start = time.monotonic()
    warned = False
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = time.monotonic() - start
            if args.duration and elapsed >= args.duration:
                break
            if not warned and node.received == 0 and elapsed >= NO_FRAME_WARN_S:
                node.get_logger().warn(
                    f"No frames on {args.topic} after {NO_FRAME_WARN_S:.0f}s -- is the camera "
                    f"publishing (ros2 topic hz {args.topic}) and ROS_DOMAIN_ID "
                    f"({os.environ.get('ROS_DOMAIN_ID', '0')}) correct?")
                warned = True
    except KeyboardInterrupt:
        pass
    finally:
        print(f"[capture] {node.saved} images saved to {out_dir} "
              f"({node.received} frames received)")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
