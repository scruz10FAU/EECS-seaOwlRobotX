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

Publishes one result to topics.downward_detections_pub (default
/seabird/beacon_detections_downward) -- deliberately NOT topics.detections_pub,
which the oblique forward-camera pipeline uses for a different JSON shape
(bbox/position_3d/world_position/det_confidence/etc.); this classifier's
result is a flatter schema (gps_position, blink, blink_color, blink_hz,
phase, color, tracking_id, timestamp), so a subscriber expecting one
shouldn't silently receive the other.

Usage:
    python3 downward_beacon_classifier_node.py --config configs/my_config.json
    python3 downward_beacon_classifier_node.py --config configs/my_config.json --duration 8.0
"""

import argparse
import csv
import json
import os
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

_LOG_HEADER = [
    "timestamp", "color", "blink", "blink_color", "blink_hz", "phase",
    "latitude", "longitude", "altitude", "tracking_id",
    "target_color", "target_blinking", "color_match", "blink_match",
]

_FRAME_LOG_HEADER = [
    "frame_idx", "timestamp", "source", "bx", "by", "color", "color_conf", "intensity",
    "hue_median", "hue_mean", "hue_mode", "hue_variance", "hue_min", "hue_max", "hue_n",
    "vote_red", "vote_green", "vote_blue", "vote_other",
    "target_color", "target_blinking",
]


def _build_log_path(base_path: str, target_color, target_blinking, ts: str, suffix: str = "") -> str:
    """
    Derive a unique, self-describing log filename from the configured base
    path: base_gt-<color>-<blink|steady|unk>_<timestamp>[_suffix].csv -- so
    repeated runs against the same --log-file/config value never collide or
    silently overwrite each other's results, and the ground truth for that
    run is visible from the filename alone without opening it. Same gt tag
    convention and time.strftime() format beacon_detector_config.py's own
    loggers already use elsewhere in this repo. ts is passed in (rather than
    computed here) so the summary and per-frame logs for the same run share
    the exact same timestamp tag.
    """
    root, _ext = os.path.splitext(base_path)
    tc = target_color or "unk"
    tb = "blink" if target_blinking is True else "steady" if target_blinking is False else "unk"
    return f"{root}_gt-{tc}-{tb}_{ts}{suffix}.csv"


def _write_log_row(log_path: str, result: dict, target_color, target_blinking) -> None:
    """
    Write this run's result as a one-row CSV (plus header) to log_path --
    comparing against ground truth (target_color/target_blinking), read the
    same way as the rest of this repo's pipeline (beacon_detector_config.py's
    top-level cfg["target_color"]/cfg["target_blinking"], set via the
    config's own "target_color"/"target_blinking" fields, not anything new
    here). log_path is expected to already be unique per run (see
    _build_log_path()), so this always starts a fresh file rather than
    appending.
    """
    gps = result.get("gps_position") or {}
    with open(log_path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(_LOG_HEADER)
        color_match = "" if target_color is None else str(result["color"] == target_color)
        blink_match = "" if target_blinking is None else str(result["blink"] == target_blinking)
        writer.writerow([
            result["timestamp"], result["color"], result["blink"], result["blink_color"],
            result["blink_hz"], result["phase"],
            gps.get("latitude", ""), gps.get("longitude", ""), gps.get("altitude", ""),
            result["tracking_id"], target_color or "", target_blinking if target_blinking is not None else "",
            color_match, blink_match,
        ])
    print(f"[downward] Logged result → {log_path}")


def _write_frame_log(log_path: str, frame_log: list, target_color, target_blinking) -> None:
    """
    Write one row per sampled frame to log_path -- the intensity/hue
    statistics classify_beacon_color() already computes on every call
    (median/mean/mode/variance/min/max/n, plus the red/green/blue/other hue
    votes) previously only ever printed to stdout and lost once the
    terminal scrolled past, never captured anywhere structured. Ground
    truth is repeated on every row (not just once) so each row is
    self-contained for debugging without cross-referencing the summary log.
    """
    with open(log_path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(_FRAME_LOG_HEADER)
        tc_col = target_color or ""
        tb_col = target_blinking if target_blinking is not None else ""
        for rec in frame_log:
            writer.writerow([
                rec["frame_idx"], rec["timestamp"], rec["source"], rec["bx"], rec["by"],
                rec["color"], rec["color_conf"], rec["intensity"],
                rec["hue_median"], rec["hue_mean"], rec["hue_mode"], rec["hue_variance"],
                rec["hue_min"], rec["hue_max"], rec["hue_n"],
                rec["vote_red"], rec["vote_green"], rec["vote_blue"], rec["vote_other"],
                tc_col, tb_col,
            ])
    print(f"[downward] Logged {len(frame_log)} per-frame rows → {log_path}")


class _DownwardImageGrabber(Node):
    """
    Minimal node subscribing only to the downward camera's image topic,
    buffering (timestamp, bgr_frame) pairs. Deliberately far lighter than
    BeaconCamera -- no depth sync, no pose/gps, no detector backend.
    """

    def __init__(self, image_topic: str, max_sample_hz: float = None, detections_topic: str = None):
        """
        max_sample_hz: if set, drop frames arriving sooner than 1/max_sample_hz
        after the last one kept, instead of buffering every frame the topic
        publishes. The topic's native rate is typically far higher than
        needed -- e.g. a 0.5 Hz blink only needs >1 Hz (Nyquist) to detect at
        all, and a handful of Hz for a clean edge/phase read, not 20-30 fps --
        so this cuts down the per-frame debug-image I/O, per-frame log
        volume, and total CV work for a run, all proportionally to how far
        above max_sample_hz the native rate is. None (default) keeps every
        frame, same as before.
        detections_topic: if set, creates self.detection_pub here -- a
        publisher dedicated to this classifier's own result schema, deliberately
        separate from the oblique pipeline's topics.detections_pub (different
        JSON shape; see topics.downward_detections_pub in beacon_detector_config.py).
        None (default) leaves self.detection_pub unset.
        """
        super().__init__("downward_image_grabber")
        self._frames = []
        self._min_interval = (1.0 / max_sample_hz) if max_sample_hz else 0.0
        self._last_kept_ts = None
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, image_topic, self._on_image, qos)
        self.get_logger().info(f"Subscribed to downward camera: {image_topic}")

        self.detection_pub = None
        if detections_topic:
            self.detection_pub = self.create_publisher(String, detections_topic, 10)

    def _on_image(self, msg: Image) -> None:
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._last_kept_ts is not None and (ts - self._last_kept_ts) < self._min_interval:
            return
        bgr = _bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self._frames.append((ts, bgr))
        self._last_kept_ts = ts

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
    expected_radius_px_override=None, tracking_id: int = -1,
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
    tracking_id: passthrough to classify_beacon_downward() -- set this if a
                 multi-beacon mission script is calling in per beacon, so
                 the published result is tagged with which one this was.
                 Default -1 (no real tracker; see classify_beacon_downward()).
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
        tracking_id=tracking_id,
    )


def setup_downward_classifier(cfg: dict, max_sample_hz: float = None, pose_wait_timeout: float = 15.0):
    """
    Stand up the pose/GPS source and downward-image subscriber needed for a
    classification pass, and wait up to pose_wait_timeout seconds for pose
    data to arrive. Factored out of main() so an importable one-shot entry
    point (classify_beacon_downward_once()) and the CLI can share it.

    Safe to call whether or not the calling process already has its own
    active rclpy context (e.g. a mission script running its own ROS node on
    a background thread) -- only calls rclpy.init() if rclpy.ok() is
    currently False. The returned owns_rclpy flag tells the caller whether
    IT is responsible for eventually calling rclpy.shutdown() (via
    teardown_downward_classifier()) -- never tear down a context this call
    didn't create, or you'll kill the caller's own ROS node out from under
    it.

    Returns (pose_source, image_grabber, owns_rclpy).
    """
    down_cfg = cfg["downward_camera"]
    if not down_cfg.get("image_topic"):
        raise ValueError("downward_camera.image_topic is not set in the config")

    bdc._import_ros()  # populates beacon_detector_config._BeaconCameraBase
    owns_rclpy = not rclpy.ok()
    if owns_rclpy:
        rclpy.init()

    pose_source = bdc._make_beacon_camera(cfg["topics"], cfg["camera"], cfg["detection"])
    pose_source.open_for_video()  # pose + GPS only, no image/depth sync
    image_grabber = _DownwardImageGrabber(
        down_cfg["image_topic"], max_sample_hz=max_sample_hz,
        detections_topic=cfg["topics"]["downward_detections_pub"],
    )
    if max_sample_hz:
        print(f"[downward] Sampling downward camera frames at up to {max_sample_hz:.1f} Hz")

    print(f"[downward] Waiting up to {pose_wait_timeout:.0f}s for GPS origin / AGL height...")
    if not _wait_for_pose_data(pose_source, pose_wait_timeout):
        print("[downward] GPS origin and/or AGL height didn't arrive in time -- "
              "continuing with fallback value(s) where needed")

    return pose_source, image_grabber, owns_rclpy


def teardown_downward_classifier(pose_source, image_grabber, owns_rclpy: bool) -> None:
    """
    Destroy the nodes setup_downward_classifier() created. Only calls
    rclpy.shutdown() if owns_rclpy is True -- i.e. this classification pass
    was the one that initialized rclpy -- so a caller with its own
    already-active ROS node never has its context torn down from under it.
    """
    pose_source.destroy_node()
    image_grabber.destroy_node()
    if owns_rclpy:
        rclpy.shutdown()


def classify_beacon_downward_once(
    cfg: dict, duration: float = None, max_sample_hz: float = None,
    pose_wait_timeout: float = None, fallback_latitude: float = None,
    fallback_longitude: float = None, fallback_altitude: float = None,
    fallback_height_agl: float = None, save_debug_images: str = None,
    expected_radius_px: float = None, tracking_id: int = None,
    log_file: str = None, publish: bool = True,
) -> dict:
    """
    Importable one-shot entry point: run a single downward-camera
    classification pass and return the result dict directly -- for another
    script (e.g. a mission/sweep script, once it's hovering over a beacon)
    to call in-process, without going through a subprocess or needing to
    parse the published topic itself.

    Every parameter defaults to None and is resolved against
    cfg["downward_test"] exactly like the CLI's own flags are (see
    beacon_detector_config.py's _DEFAULT_DOWNWARD_TEST) -- pass a value here
    to override that config's default for this one call.

    Safe to call whether or not the calling process already has its own
    active rclpy context elsewhere (see setup_downward_classifier()). Each
    call creates and fully destroys its own pose/image nodes, so repeated
    calls (e.g. once per beacon across a mission) don't collide.

    Raises ValueError if downward_camera.image_topic isn't set, or if the
    resolved fallback_height_agl doesn't exceed downward_camera.beacon_height_m
    -- raising (rather than printing + silently returning, as main() did
    before this was factored out) so a caller can't mistake a config error
    for a legitimate "nothing found" result.
    """
    down_cfg = cfg["downward_camera"]
    test_cfg = cfg["downward_test"]

    def _resolved(name, value):
        return value if value is not None else test_cfg[name]

    duration             = _resolved("duration", duration)
    max_sample_hz         = _resolved("max_sample_hz", max_sample_hz)
    pose_wait_timeout     = _resolved("pose_wait_timeout", pose_wait_timeout)
    fallback_latitude     = _resolved("fallback_latitude", fallback_latitude)
    fallback_longitude    = _resolved("fallback_longitude", fallback_longitude)
    fallback_altitude     = _resolved("fallback_altitude", fallback_altitude)
    fallback_height_agl   = _resolved("fallback_height_agl", fallback_height_agl)
    save_debug_images     = _resolved("save_debug_images", save_debug_images)
    expected_radius_px    = _resolved("expected_radius_px", expected_radius_px)
    tracking_id           = _resolved("tracking_id", tracking_id)
    log_file              = _resolved("log_file", log_file)

    if fallback_height_agl <= down_cfg["beacon_height_m"]:
        raise ValueError(
            f"fallback_height_agl ({fallback_height_agl:.2f}m) must exceed "
            f"downward_camera.beacon_height_m ({down_cfg['beacon_height_m']:.2f}m) -- "
            f"the drone must be above the beacon top"
        )

    if save_debug_images:
        print(f"[downward] Saving debug images to {save_debug_images}/")
    if expected_radius_px is not None:
        print(f"[downward] Using expected-radius-px override: {expected_radius_px:.1f}px "
              f"(ignoring height-derived sizing)")

    pose_source, image_grabber, owns_rclpy = setup_downward_classifier(
        cfg, max_sample_hz=max_sample_hz, pose_wait_timeout=pose_wait_timeout,
    )
    try:
        print(f"[downward] Sampling downward camera for {duration:.1f}s...")
        result = classify_beacon_at_current_position(
            pose_source, image_grabber, cfg, duration,
            fallback_gps_origin=(fallback_latitude, fallback_longitude, fallback_altitude),
            fallback_height_agl=fallback_height_agl,
            debug_image_dir=save_debug_images,
            expected_radius_px_override=expected_radius_px,
            tracking_id=tracking_id,
        )
        n_frames_used = result.pop("n_frames_used", None)
        frame_log = result.pop("frame_log", [])

        print(f"[downward] Result: {json.dumps(result, indent=2)}")

        if log_file:
            target_color, target_blinking = cfg.get("target_color"), cfg.get("target_blinking")
            ts_tag = time.strftime("%Y%m%d_%H%M%S")
            log_path = _build_log_path(log_file, target_color, target_blinking, ts_tag)
            _write_log_row(log_path, result, target_color, target_blinking)
            frames_path = _build_log_path(log_file, target_color, target_blinking, ts_tag, suffix="_frames")
            _write_frame_log(frames_path, frame_log, target_color, target_blinking)

        if publish:
            if n_frames_used > 0:
                msg = String()
                msg.data = json.dumps(result)
                image_grabber.detection_pub.publish(msg)
                print(f"[downward] Published to {cfg['topics']['downward_detections_pub']}")
                # detection_pub (created in _DownwardImageGrabber.__init__) uses
                # the default VOLATILE durability (no explicit QoSProfile) -- a
                # subscriber only receives messages published *after* it finishes
                # connecting, nothing is replayed. Since this script publishes
                # exactly once, a listener that isn't already up and discovered
                # before this call (ros2 topic echo started late, or started at
                # nearly the same moment as this script) will simply never see it,
                # regardless of whether classification succeeded. Spin briefly here
                # so the message actually has time to flush over the wire and so a
                # listener whose discovery was still completing gets a chance to
                # receive it, before the node (and the publisher with it) is torn
                # down immediately below.
                t_end = time.time() + 1.0
                while time.time() < t_end:
                    rclpy.spin_once(pose_source, timeout_sec=0.1)
            else:
                print("[downward] No frames matched the expected circle -- nothing published")

        return result
    finally:
        teardown_downward_classifier(pose_source, image_grabber, owns_rclpy)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--config", default=bdc._DEFAULT_CONFIG,
                     help=f"JSON config path (default: {bdc._DEFAULT_CONFIG})")
    # Every flag below defaults to None here -- the config's downward_test
    # section (see beacon_detector_config.py's _DEFAULT_DOWNWARD_TEST) is the
    # actual source of truth for each one; a flag only overrides its config
    # value when explicitly passed on the command line (resolved just below,
    # after the config is loaded).
    ap.add_argument("--duration", type=float, default=None,
                     help="Seconds to sample the downward camera before classifying "
                          "(default: downward_test.duration, 8.0)")
    ap.add_argument("--max-sample-hz", type=float, default=None,
                     help="Drop frames arriving sooner than 1/this many seconds after the "
                          "last one kept, instead of buffering every frame the topic "
                          "publishes -- the native rate is typically far higher than needed "
                          "for a slow blink signal. Pass 0 to disable and keep every frame "
                          "(default: downward_test.max_sample_hz, 5.0)")
    ap.add_argument("--pose-wait-timeout", type=float, default=None,
                     help="Max seconds to wait for drone height/GPS origin before giving up "
                          "(default: downward_test.pose_wait_timeout, 15.0)")
    ap.add_argument("--fallback-latitude", type=float, default=None,
                     help="GPS origin latitude to use if no real GPS fix arrives "
                          "(default: downward_test.fallback_latitude, 0.0)")
    ap.add_argument("--fallback-longitude", type=float, default=None,
                     help="GPS origin longitude to use if no real GPS fix arrives "
                          "(default: downward_test.fallback_longitude, 0.0)")
    ap.add_argument("--fallback-altitude", type=float, default=None,
                     help="GPS origin altitude to use if no real GPS fix arrives "
                          "(default: downward_test.fallback_altitude, 0.0)")
    ap.add_argument("--fallback-height-agl", type=float, default=None,
                     help="Drone AGL height (metres) to assume if no real height arrives -- "
                          "must exceed downward_camera.beacon_height_m "
                          "(default: downward_test.fallback_height_agl, 1.0)")
    ap.add_argument("--save-debug-images", default=None, metavar="DIR",
                     help="Write per-frame debug images (circle overlay, crop, mask -- or "
                          "the raw frame when nothing was found) to this directory, showing "
                          "exactly what each frame's color/blink classification was based on "
                          "(default: downward_test.save_debug_images, disabled)")
    ap.add_argument("--expected-radius-px", type=float, default=None,
                     help="Bypass the height-derived circle radius entirely and use this "
                          "pixel radius directly -- for hand-held bench testing where "
                          "--fallback-height-agl doesn't reflect the actual (much closer) "
                          "test distance. Measure it from a debug image: the radius of the "
                          "disc outline you want detected, in pixels "
                          "(default: downward_test.expected_radius_px, disabled)")
    ap.add_argument("--tracking-id", type=int, default=None,
                     help="Tag the published result with this ID -- for a multi-beacon "
                          "mission script calling this in per beacon "
                          "(default: downward_test.tracking_id, -1, meaning no real tracker)")
    ap.add_argument("--log-file", default=None, metavar="PATH",
                     help="Append this run's result as one CSV row to PATH (creating it with "
                          "a header if it doesn't exist yet), including a color_match/"
                          "blink_match comparison against the config's top-level "
                          "target_color/target_blinking ground-truth fields -- the same "
                          "fields beacon_detector_config.py's own logger reads, not anything "
                          "new. Those columns are left blank when a target isn't set "
                          "(default: downward_test.log_file, disabled)")
    args = ap.parse_args()

    cfg = bdc.load_config(args.config)
    try:
        classify_beacon_downward_once(
            cfg,
            duration=args.duration,
            max_sample_hz=args.max_sample_hz,
            pose_wait_timeout=args.pose_wait_timeout,
            fallback_latitude=args.fallback_latitude,
            fallback_longitude=args.fallback_longitude,
            fallback_altitude=args.fallback_altitude,
            fallback_height_agl=args.fallback_height_agl,
            save_debug_images=args.save_debug_images,
            expected_radius_px=args.expected_radius_px,
            tracking_id=args.tracking_id,
            log_file=args.log_file,
        )
    except ValueError as e:
        print(f"[downward] ERROR: {e}")


if __name__ == "__main__":
    main()
