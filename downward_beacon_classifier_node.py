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


def classify_beacon_at_current_position(
    pose_source, image_grabber, cfg: dict, duration_s: float,
    fallback_gps_origin=None, fallback_height_agl=None, debug_image_dir=None,
    expected_radius_px_override=None,
) -> dict:
    """
    Spin for duration_s seconds collecting downward-camera frames, then run
    classify_beacon_downward() against them using the drone's pose/GPS/height
    sampled at the end of that window.

    pose_source: anything exposing get_drone_pose(), get_gps_origin(),
                 get_drone_height_agl() -- e.g. a BeaconCamera opened via
                 open_for_video() (pose + GPS only, no image/depth).
    image_grabber: a _DownwardImageGrabber instance, already subscribed.
    fallback_gps_origin: (lat, lon, alt) used ONLY if get_gps_origin() is
                 still None -- e.g. no GPS fix / GPS-denied flight. Lets
                 color/blink classification (and a local-frame
                 world_position) proceed; the resulting "gps_position" in
                 the output is then a placeholder, not a real fix.
    fallback_height_agl: used if get_drone_height_agl() is still None, OR if
                 it returned a real but implausible value (at or below the
                 beacon's own height -- the drone can't be below the beacon
                 top while hovering above it, so that's telemetry to
                 distrust, not a legitimate reading). Unlike the GPS
                 fallback, this isn't just cosmetic -- it drives
                 depth_m = height_agl - beacon_height_m, which sizes the
                 expected circle radius, so a bad height blocks circle
                 detection entirely without a fallback here.
    expected_radius_px_override: if set, bypasses the height-derived circle
                 radius entirely -- see classify_beacon_downward(). For
                 hand-held bench testing where height telemetry doesn't
                 reflect the actual (much closer) test distance at all.
    """
    t_end = time.time() + duration_s
    while time.time() < t_end:
        rclpy.spin_once(pose_source, timeout_sec=0.02)
        rclpy.spin_once(image_grabber, timeout_sec=0.02)

    frames = image_grabber.drain_frames()
    drone_pos, drone_quat = pose_source.get_drone_pose()
    gps_origin = pose_source.get_gps_origin()
    drone_height_agl = pose_source.get_drone_height_agl()
    beacon_height_m = cfg["downward_camera"]["beacon_height_m"]

    if gps_origin is None and fallback_gps_origin is not None:
        print(f"[downward] WARNING: no GPS origin -- using fallback {fallback_gps_origin} "
              f"(gps_position in the result will be a placeholder, not a real fix)")
        gps_origin = fallback_gps_origin

    if drone_height_agl is None:
        if fallback_height_agl is not None:
            print(f"[downward] WARNING: no AGL height -- using fallback {fallback_height_agl:.2f}m")
            drone_height_agl = fallback_height_agl
    elif drone_height_agl <= beacon_height_m:
        print(f"[downward] WARNING: AGL height ({drone_height_agl:.2f}m) is at or below the "
              f"beacon height ({beacon_height_m:.2f}m) -- the drone can't be below the beacon "
              f"top while hovering above it, so this telemetry looks wrong"
              + (f"; using fallback {fallback_height_agl:.2f}m instead" if fallback_height_agl is not None else ""))
        if fallback_height_agl is not None:
            drone_height_agl = fallback_height_agl

    if drone_pos is None:
        # No pose at all (e.g. VIO not running). Fall back to a nominal
        # "directly above the beacon at drone_height_agl" position so
        # _camera_to_world() has something to work with -- world_position/
        # gps_position in the result are then placeholders too.
        print("[downward] WARNING: no drone pose -- assuming identity "
              "position/orientation above the beacon")
        drone_pos  = [0.0, 0.0, drone_height_agl if drone_height_agl is not None else 0.0]
        drone_quat = [1.0, 0.0, 0.0, 0.0]

    return classify_beacon_downward(
        frames, drone_pos, drone_quat, gps_origin, drone_height_agl,
        cfg["downward_camera"], cfg["detection"], duration_s,
        debug_image_dir=debug_image_dir,
        expected_radius_px_override=expected_radius_px_override,
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
    ap.add_argument("--fallback-latitude", type=float, default=0.0,
                     help="GPS origin latitude to use if no real GPS fix arrives (default: 0.0)")
    ap.add_argument("--fallback-longitude", type=float, default=0.0,
                     help="GPS origin longitude to use if no real GPS fix arrives (default: 0.0)")
    ap.add_argument("--fallback-altitude", type=float, default=0.0,
                     help="GPS origin altitude to use if no real GPS fix arrives (default: 0.0)")
    ap.add_argument("--fallback-height-agl", type=float, default=1.0,
                     help="Drone AGL height (metres) to assume if no real height arrives "
                          "(default: 1.0 -- must exceed downward_camera.beacon_height_m)")
    ap.add_argument("--save-debug-images", default=None, metavar="DIR",
                     help="Write per-frame debug images (circle overlay, crop, mask -- or "
                          "the raw frame when nothing was found) to this directory, showing "
                          "exactly what each frame's color/blink classification was based on")
    ap.add_argument("--expected-radius-px", type=float, default=None,
                     help="Bypass the height-derived circle radius entirely and use this "
                          "pixel radius directly -- for hand-held bench testing where "
                          "--fallback-height-agl doesn't reflect the actual (much closer) "
                          "test distance. Measure it from a debug image: the radius of the "
                          "disc outline you want detected, in pixels.")
    args = ap.parse_args()

    cfg = bdc.load_config(args.config)
    down_cfg = cfg["downward_camera"]
    if not down_cfg.get("image_topic"):
        print("[downward] ERROR: downward_camera.image_topic is not set in the config")
        return
    if args.fallback_height_agl <= down_cfg["beacon_height_m"]:
        print(f"[downward] ERROR: --fallback-height-agl ({args.fallback_height_agl:.2f}m) must "
              f"exceed downward_camera.beacon_height_m ({down_cfg['beacon_height_m']:.2f}m) -- "
              f"the drone must be above the beacon top")
        return

    bdc._import_ros()  # populates beacon_detector_config._BeaconCameraBase
    rclpy.init()

    pose_source = bdc._make_beacon_camera(cfg["topics"], cfg["camera"], cfg["detection"])
    pose_source.open_for_video()  # pose + GPS only, no image/depth sync
    image_grabber = _DownwardImageGrabber(down_cfg["image_topic"])

    print(f"[downward] Waiting up to {args.pose_wait_timeout:.0f}s for GPS origin / AGL height...")
    if not _wait_for_pose_data(pose_source, args.pose_wait_timeout):
        print("[downward] GPS origin and/or AGL height didn't arrive in time -- "
              "continuing with fallback value(s) where needed")

    if args.save_debug_images:
        print(f"[downward] Saving debug images to {args.save_debug_images}/")
    if args.expected_radius_px is not None:
        print(f"[downward] Using --expected-radius-px override: {args.expected_radius_px:.1f}px "
              f"(ignoring height-derived sizing)")

    print(f"[downward] Sampling downward camera for {args.duration:.1f}s...")
    result = classify_beacon_at_current_position(
        pose_source, image_grabber, cfg, args.duration,
        fallback_gps_origin=(args.fallback_latitude, args.fallback_longitude, args.fallback_altitude),
        fallback_height_agl=args.fallback_height_agl,
        debug_image_dir=args.save_debug_images,
        expected_radius_px_override=args.expected_radius_px,
    )
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
