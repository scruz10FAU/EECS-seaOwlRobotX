#!/usr/bin/env python3
"""
Beacon overlay stream — detections + camera-only range estimates drawn on the
tracking_front feed, served as MJPEG over HTTP.

    browser / VLC / ffplay:  http://<drone-ip>:8091/          (viewer page)
                             http://<drone-ip>:8091/stream.mjpg
                             http://<drone-ip>:8091/snapshot.jpg

Each streamed frame is the frame the detector ran on, so boxes never lag the
image; the stream therefore updates at inference rate, not camera rate.

Camera-only ranges (no drone pose needed):
  diamond  slant range from the angle subtended by its top-bottom diagonal
           (beacon_pose.size_range). Green = passes diamond_quality (PnP-grade
           keypoints); amber = size-only, with the rejection reason. If the
           keypoints collapsed toward the centre ("kp_collapsed") the range
           comes from the diamond's box height instead.
  beacon   slant range = full beacon height / angle between the rays through the
           box's top- and bottom-centre. Assumes a roughly level line of sight
           (no elevation correction without attitude).

With --telemetry (a telemetry_viewer.py recording: <ts>.mkv + <ts>.csv, one row
per frame) each frame also gets the full beacon_pose localisation, using that
row's PX4 position/attitude and GPS: per fix, a marker at the estimated beacon
base, its method (pnp / box / ray), slant and horizontal distance, and lat/lon
+/- 1 sigma; fixes are fused over the loop (BeaconMap: one Kalman filter per
beacon with --drift random walk, reset when the video restarts) and listed
bottom-left once they have --min-fixes fixes.

Live (ROS) does the same localisation from PX4 over microDDS
(/fmu/out/vehicle_local_position, vehicle_attitude, vehicle_gps_position; the
pose interpolated to each image's capture stamp, see Px4State), with lat/lon
from PX4's own reference origin. --no-locate turns it off.

Published on --map-topic (std_msgs/String, JSON; default /seabird/beacon_map),
once per processed frame while localising, live or replay:
  {"stamp", "source": "live"|"replay", "frame", "drone": {"ned", "lat", "lon"},
   "water_z_ned",
   "beacons": [{"id", "ned", "lat", "lon", "sigma_m", "n", "outliers", "age_s",
                "seen"}],                    # the map, entries with >= --min-fixes
   "fixes":   [{"method", "ned", "lat", "lon", "sigma_m", "range_m", "horiz_m",
                "rms_px", "n_diamonds", "n_boxes"}]}   # this frame
ned is PX4 local NED (m), lat/lon null without a global reference, sigma_m the
horizontal 1-sigma (map: including drift since last seen).

Sources:
  ROS 2 (default)  sensor_msgs/Image on --topic (mono8, bgr8 or rgb8)
  --video PATH     a video file, looped (offline testing, no ROS needed)
  --image PATH     one image, repeated

Usage:
    python3 beacon_overlay.py --model /data/models/YOLOv11n_diamond_detect_float16.tflite
    python3 beacon_overlay.py --model m.tflite --classes beacon,diamond
    python3 beacon_overlay.py --model m.tflite --image frame.png --intrinsics cal.yml --once out.jpg
    python3 beacon_overlay.py --model m.tflite --classes beacon,diamond \
        --video rec/20261015_154558_510023.mkv --telemetry rec/20261015_154558_510023.csv
"""

import argparse
import csv
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

import cv2
import numpy as np

from beacon_pose import (BeaconBoxObs, BeaconMap, BeaconModel, CameraModel, DiamondObs,
                         StateBuffer, camera_pose_from_drone, dedupe_diamonds, diamond_quality,
                         load_modalai_extrinsics, load_pose_config, locate_beacons,
                         ned_to_lla, size_range)
from tflite_hexagon_detector import TFLiteHexagonDetector

HERE = os.path.dirname(os.path.abspath(__file__))

# BGR. Keypoint order: 0 bottom, 1 left, 2 top, 3 right.
KPT_COLORS = [(0, 0, 255), (0, 255, 255), (255, 0, 255), (255, 255, 0)]
GOOD, WEAK, BOX = (0, 220, 0), (0, 170, 255), (255, 160, 0)
FIX_COLORS = {"pnp": (0, 255, 0), "box": (255, 200, 0), "ray": (0, 140, 255)}
FUSED = (255, 0, 255)


# ─── MJPEG server ─────────────────────────────────────────────────────

class FrameHub:
    """Latest encoded frame + a condition clients wait on. Keeps one frame only,
    so a slow client skips frames instead of building a backlog."""

    def __init__(self):
        self._cond = threading.Condition()
        self._jpeg: Optional[bytes] = None
        self._seq = 0

    def publish(self, jpeg: bytes):
        with self._cond:
            self._jpeg, self._seq = jpeg, self._seq + 1
            self._cond.notify_all()

    def wait(self, last_seq: int, timeout: float = 5.0):
        with self._cond:
            self._cond.wait_for(lambda: self._seq != last_seq, timeout)
            return self._seq, self._jpeg


VIEWER_HTML = b"""<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Beacon overlay</title>
<style>body{margin:0;background:#111;color:#ccc;font:14px sans-serif}
img{display:block;max-width:100%;margin:0 auto}p{text-align:center}</style></head>
<body><img src="/stream.mjpg"><p><a href="/snapshot.jpg" style="color:#8cf">snapshot</a></p></body></html>"""


def make_handler(hub: FrameHub):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):  # quiet: one line per request is noise
            pass

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                self._send(200, "text/html", VIEWER_HTML)
            elif path == "/snapshot.jpg":
                _, jpeg = hub.wait(-1, timeout=0)
                if jpeg is None:
                    self._send(503, "text/plain", b"no frame yet\n")
                else:
                    self._send(200, "image/jpeg", jpeg)
            elif path == "/stream.mjpg":
                self._stream()
            else:
                self._send(404, "text/plain", b"not found\n")

        def _send(self, code, ctype, body):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _stream(self):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            seq = -1
            try:
                while True:
                    new_seq, jpeg = hub.wait(seq)
                    if jpeg is None or new_seq == seq:
                        continue
                    seq = new_seq
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     b"Content-Length: %d\r\n\r\n" % len(jpeg))
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


# ─── Frame sources ────────────────────────────────────────────────────

class LatestFrame:
    """Single-slot mailbox: producers overwrite, the consumer takes the newest.
    Items are (frame, telemetry row or None, loop index)."""

    def __init__(self):
        self._cond = threading.Condition()
        self._frame = None

    def put(self, frame):
        with self._cond:
            self._frame = frame
            self._cond.notify()

    def take(self, timeout=1.0):
        with self._cond:
            self._cond.wait_for(lambda: self._frame is not None, timeout)
            frame, self._frame = self._frame, None
            return frame


def image_msg_to_array(msg) -> Optional[np.ndarray]:
    """sensor_msgs/Image -> numpy without cv_bridge. Honours row padding (step)."""
    buf = np.frombuffer(msg.data, np.uint8)
    if msg.encoding == "mono8":
        return buf.reshape(msg.height, msg.step)[:, :msg.width]
    if msg.encoding in ("bgr8", "rgb8"):
        img = buf.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        return img if msg.encoding == "bgr8" else img[:, :, ::-1]
    # ROS "yuv422" is UYVY, "yuv422_yuy2" is YUYV; 2 bytes/pixel (VOXL hires_down_small_color).
    if msg.encoding in ("yuv422", "yuv422_yuy2"):
        yuv = buf.reshape(msg.height, msg.step)[:, :msg.width * 2].reshape(msg.height, msg.width, 2)
        code = cv2.COLOR_YUV2BGR_UYVY if msg.encoding == "yuv422" else cv2.COLOR_YUV2BGR_YUYV
        return cv2.cvtColor(yuv, code)
    return None


def image_stamp(msg, offset_s: float = 0.0) -> float:
    """Capture time of a sensor_msgs/Image (Unix s): header.stamp + offset_s, or the
    receive time if the publisher left the stamp empty. On VOXL2 the stamp is the
    camera capture time on the same clock as PX4's microDDS timestamps."""
    st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    return (st if st > 0 else time.time()) + offset_s


class Px4State:
    """PX4 state from microDDS as a telemetry row (load_telemetry's keys, plus
    ref_lat/ref_lon/ref_alt). Position and attitude are buffered by PX4's
    timestamp_sample and interpolated to the image capture stamp; the rest
    (validity, reference, GPS) is the latest value."""

    MAX_AGE_S = 0.5          # local position / attitude must be this fresh
    MAX_EXTRAP_S = 0.05      # capture stamp may be this far past the newest sample
    CLOCK_MISMATCH_S = 1.0   # beyond this, stamps aren't comparable: use the latest

    def __init__(self):
        self._lock = threading.Lock()
        self._row, self._rx = {}, {}
        self.pos, self.att = StateBuffer(), StateBuffer(quat=True)
        self.stats = {"interp": 0, "latest": 0, "none": 0}
        self._warned = False

    def update(self, group, **kv):
        with self._lock:
            self._row.update(kv)
            self._rx[group] = time.monotonic()

    def snapshot(self, stamp: float) -> dict:
        """Row for an image captured at `stamp` (Unix s). Pose keys are None unless
        local position and attitude are fresh and PX4 reports xy/z valid."""
        now = time.monotonic()
        with self._lock:
            row = dict(self._row)
            fresh = {g: now - t <= self.MAX_AGE_S for g, t in self._rx.items()}
        if not fresh.get("gps"):
            row["gps_fix_type"] = None
        p = q = None
        if fresh.get("local") and fresh.get("attitude") and row.get("pose_valid"):
            p, q = self.pos.at(stamp, self.MAX_EXTRAP_S), self.att.at(stamp, self.MAX_EXTRAP_S)
            if p is not None and q is not None:
                self.stats["interp"] += 1
                row["pose_dt_ms"] = (self.att.latest_t - stamp) * 1e3
            else:
                lt = self.att.latest_t
                if lt is not None and abs(stamp - lt) > self.CLOCK_MISMATCH_S:
                    if not self._warned:
                        print(f"[overlay] WARNING: image stamp {stamp:.3f} vs PX4 {lt:.3f}: "
                              f"clocks differ, using the latest pose (no lag compensation)")
                        self._warned = True
                    p, q = self.pos.latest(), self.att.latest()
                    self.stats["latest"] += 1
        if p is None or q is None:
            self.stats["none"] += 1
            for k in ("local_x", "local_y", "local_z", "qw", "qx", "qy", "qz"):
                row[k] = None
        else:
            row.update(local_x=float(p[0]), local_y=float(p[1]), local_z=float(p[2]),
                       qw=float(q[0]), qx=float(q[1]), qy=float(q[2]), qz=float(q[3]))
        return row


class RosIO:
    """One rclpy node for the camera, PX4 and the beacon-map publisher."""

    def __init__(self):
        import rclpy
        self._rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node("beacon_overlay")
        self._spinning = False

    def spin(self):
        if not self._spinning:
            threading.Thread(target=self._rclpy.spin, args=(self.node,), daemon=True).start()
            self._spinning = True

    def subscribe_image(self, topic: str, box: LatestFrame, px4: Optional[Px4State],
                        offset_s: float = 0.0):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image

        warned = set()
        count = [0]

        def on_image(msg):
            img = image_msg_to_array(msg)
            if img is None:
                if msg.encoding not in warned:
                    warned.add(msg.encoding)
                    self.node.get_logger().error(f"unsupported encoding '{msg.encoding}' on {topic}")
                return
            telem = None
            if px4 is not None:
                stamp = image_stamp(msg, offset_s)
                telem = px4.snapshot(stamp)
                telem.update(frame=count[0], frame_stamp=stamp)
            count[0] += 1
            box.put((img, telem, 0))

        # Best effort matches both best-effort and reliable publishers.
        self.node.create_subscription(Image, topic, on_image, qos_profile_sensor_data)
        print(f"[overlay] subscribed to {topic}")

    def subscribe_px4(self) -> Optional[Px4State]:
        try:
            from px4_msgs.msg import SensorGps, VehicleAttitude, VehicleLocalPosition
        except ImportError:
            print("[overlay] WARNING: px4_msgs not available; no live localisation")
            return None
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        st = Px4State()

        def on_local(m):
            glob = bool(m.xy_global) and math.isfinite(m.ref_lat) and math.isfinite(m.ref_lon)
            st.pos.add(m.timestamp_sample * 1e-6, (m.x, m.y, m.z))
            st.update("local", pose_valid=bool(m.xy_valid and m.z_valid),
                      height_agl_m=float(-m.z) if m.z_valid else None,
                      heading_deg=math.degrees(m.heading) if math.isfinite(m.heading) else None,
                      ref_lat=float(m.ref_lat) if glob else None,
                      ref_lon=float(m.ref_lon) if glob else None,
                      ref_alt=float(m.ref_alt) if glob and m.z_global else None)

        def on_attitude(m):
            st.att.add(m.timestamp_sample * 1e-6, [float(v) for v in m.q])
            st.update("attitude")

        def on_gps(m):
            # lat/lon int32 1e-7 deg, alt int32 mm; fix_type < 3 = no 3D fix.
            st.update("gps", gps_lat=m.lat * 1e-7, gps_lon=m.lon * 1e-7, gps_alt_msl_m=m.alt * 1e-3,
                      gps_fix_type=float(m.fix_type), gps_sats=float(m.satellites_used),
                      gps_eph=float(m.eph))

        for msg_type, t, cb in ((VehicleLocalPosition, "/fmu/out/vehicle_local_position", on_local),
                                (VehicleAttitude, "/fmu/out/vehicle_attitude", on_attitude),
                                (SensorGps, "/fmu/out/vehicle_gps_position", on_gps)):
            self.node.create_subscription(msg_type, t, cb, qos)
        print("[overlay] subscribed to PX4 local position / attitude / GPS")
        return st

    def string_publisher(self, topic: str):
        from std_msgs.msg import String
        pub = self.node.create_publisher(String, topic, 10)
        print(f"[overlay] publishing beacon map on {topic}")

        def publish(text: str):
            msg = String()
            msg.data = text
            pub.publish(msg)
        return publish


def start_file_source(path: str, is_video: bool, box: LatestFrame, fps: float):
    def run():
        period = 1.0 / fps
        if not is_video:
            img = cv2.imread(path, cv2.IMREAD_UNCHANGED)
            if img is None:
                raise SystemExit(f"cannot read {path}")
            while True:
                box.put((img, None, 0))
                time.sleep(period)
        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            raise SystemExit(f"cannot open {path}")
        while True:
            ok, img = cap.read()
            if not ok:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue
            box.put((img, None, 0))
            time.sleep(period)

    threading.Thread(target=run, daemon=True).start()


TELEMETRY_FLOATS = ("frame_stamp", "gps_lat", "gps_lon", "gps_alt_msl_m", "gps_eph", "height_agl_m",
                    "local_x", "local_y", "local_z", "heading_deg", "roll_deg", "pitch_deg",
                    "qw", "qx", "qy", "qz")


def load_telemetry(path: str):
    """telemetry_viewer.py recording CSV -> one dict per video frame (None = field
    missing in that row, e.g. no GPS yet)."""
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            row = {"frame": int(r["frame"])}
            for k in TELEMETRY_FLOATS + ("gps_fix_type", "gps_sats"):
                try:
                    row[k] = float(r[k])
                except (KeyError, TypeError, ValueError):
                    row[k] = None
            rows.append(row)
    return rows


def start_replay_source(path: str, rows, box: LatestFrame, fps: float):
    """Video + its telemetry rows, frame i paired with row i, looped. The loop
    index lets the consumer reset state that must not span a restart."""
    def run():
        period = 1.0 / fps
        loop = 0
        while True:
            cap = cv2.VideoCapture(path)
            if not cap.isOpened():
                raise SystemExit(f"cannot open {path}")
            i = 0
            while i < len(rows):
                ok, img = cap.read()
                if not ok:
                    break
                box.put((img, rows[i], loop))
                i += 1
                time.sleep(period)
            cap.release()
            if loop == 0 and i != len(rows):
                print(f"[overlay] WARNING: video has {i} frames, telemetry {len(rows)} rows; "
                      f"using the first {i}", flush=True)
            loop += 1

    threading.Thread(target=run, daemon=True).start()


# ─── Ranges ───────────────────────────────────────────────────────────

def scaled_camera(cam: CameraModel, w: int, h: int) -> CameraModel:
    """Calibration rescaled to a frame of a different resolution (same FOV)."""
    if (w, h) == (cam.width, cam.height):
        return cam
    sx, sy = w / cam.width, h / cam.height
    K = cam.K.copy()
    K[0] *= sx
    K[1] *= sy
    print(f"[overlay] WARNING: frame {w}x{h} != calibration {cam.width}x{cam.height}; "
          f"scaling intrinsics by ({sx:.3f}, {sy:.3f})")
    return CameraModel(K, cam.D, cam.model, w, h)


def box_size_range(cam: CameraModel, bbox, height_m: float) -> Optional[float]:
    """Slant range from the angular height of a box around an object height_m tall
    (level line of sight)."""
    x1, y1, x2, y2 = map(float, bbox)
    if y2 <= y1:
        return None
    cx = (x1 + x2) / 2
    n = cam.normalize([(cx, y1), (cx, y2)])
    v = np.c_[n, np.ones(2)]
    v /= np.linalg.norm(v, axis=1, keepdims=True)
    th = math.acos(float(np.clip(v[0] @ v[1], -1, 1)))
    return height_m / th if th > 1e-6 else None


# ─── Drawing ──────────────────────────────────────────────────────────

def put_label(img, text, org, color, scale=0.5):
    x, y = int(org[0]), int(org[1])
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def fmt_range(r):
    return f"{r:.1f} m" if r is not None and math.isfinite(r) else "? m"


class Overlay:
    def __init__(self, args):
        self.params, nms_iou = load_pose_config(args.pose_config)
        self.model = BeaconModel()
        c = self.model.corners_zup
        self.diamond_height_m = c["top"][2] - c["bottom"][2]   # vertical extent, ~0.194 m
        self.diamond_label = args.diamond_label
        self.cam_cal = CameraModel.from_modalai_yml(args.intrinsics)
        self.cam = None
        classes = [c.strip() for c in args.classes.split(",") if c.strip()]
        self.detector = TFLiteHexagonDetector(
            weights=args.model, class_names=classes, conf_thresh=args.conf,
            iou_thresh=nms_iou, delegate_path=args.delegate, num_threads=args.threads,
            kpt_shape=(4, 3), gpu_lib=args.gpu_lib, use_gpu=not args.cpu,
            resize_mode=args.resize)
        if not self.detector.start(enable_tracking=False):
            raise SystemExit(f"failed to load {args.model}")
        self.locate = args.locate
        self.source = "replay" if args.telemetry else "live"
        self.publish = None       # callable(str), set by main() when publishing
        if self.locate:
            self.R_body_cam, self.t_body_cam = load_modalai_extrinsics(
                args.extrinsics, args.camera_frame, "body")
            self.water_z = args.water_z
            self.map_args = dict(cov_floor_m=args.map_floor, drift_mps=args.drift)
            self.min_fixes = args.min_fixes
            self.map, self.map_loop = BeaconMap(**self.map_args), 0

    def process(self, frame: np.ndarray, telem: Optional[dict] = None, loop: int = 0):
        h, w = frame.shape[:2]
        if self.cam is None or (self.cam.width, self.cam.height) != (w, h):
            self.cam = scaled_camera(self.cam_cal, w, h)

        t0 = time.monotonic()
        dets = self.detector.detect(frame)
        infer_ms = (time.monotonic() - t0) * 1e3

        out = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR) if frame.ndim == 2 else frame.copy()
        diamonds = [d for d in dets if d.label == self.diamond_label and d.keypoints is not None]
        boxes = [d for d in dets if d.label != self.diamond_label]

        obs = [DiamondObs(kpts=d.keypoints[:, :2], conf=d.keypoints[:, 2],
                          score=d.confidence, bbox=d.bbox_2d) for d in diamonds]
        keep = dedupe_diamonds(obs, self.params.kp_dedupe_frac) if self.params.kp_dedupe \
            else range(len(obs))

        for d in boxes:
            x1, y1, x2, y2 = d.bbox_2d
            cv2.rectangle(out, (x1, y1), (x2, y2), BOX, 2)
            r = box_size_range(self.cam, d.bbox_2d, self.model.height_m)
            put_label(out, f"{d.label} {d.confidence:.2f} | {fmt_range(r)}",
                      (x1, max(y1 - 6, 12)), BOX)

        for i in keep:
            d, o = diamonds[i], obs[i]
            ok, reason = diamond_quality(o, self.params)
            color = GOOD if ok else WEAK
            x1, y1, x2, y2 = d.bbox_2d
            cv2.rectangle(out, (x1, y1), (x2, y2), color, 1)
            q = np.round(o.kpts).astype(int)
            cv2.polylines(out, [q.reshape(-1, 1, 2)], True, color, 1, cv2.LINE_AA)
            for k, (pt, c) in enumerate(zip(q, o.conf)):
                cv2.circle(out, tuple(int(v) for v in pt), 3 if c >= self.params.min_kpt_conf else 1,
                           KPT_COLORS[k], -1, cv2.LINE_AA)
            if reason == "kp_collapsed":
                # Corners regressed to the centre, but the box is still sized right.
                r = box_size_range(self.cam, d.bbox_2d, self.diamond_height_m)
                reason = "kp_collapsed, box range"
            else:
                r = size_range(self.cam, o, self.model) \
                    if min(o.conf[0], o.conf[2]) >= self.params.min_kpt_conf else None
            tag = "" if ok else f" ({reason})"
            put_label(out, f"{d.label} {d.confidence:.2f} | {fmt_range(r)}{tag}",
                      (x1, max(y1 - 6, 12)), color)

        if self.locate and telem is not None:
            self._localise(out, telem, loop, obs, boxes)
        return out, infer_ms, len(keep), len(boxes)

    # ── Localisation (--telemetry replay, or live PX4) ──

    def _localise(self, out, telem, loop, obs, boxes):
        need = ("local_x", "local_y", "local_z", "qw", "qx", "qy", "qz")
        if any(telem.get(k) is None for k in need):
            put_label(out, "no valid PX4 pose: not localising", (6, 40), WEAK)
            return
        if loop != self.map_loop:
            self.map, self.map_loop = BeaconMap(**self.map_args), loop
        drone_ned = np.array([telem["local_x"], telem["local_y"], telem["local_z"]])
        pose = camera_pose_from_drone(drone_ned, [telem[k] for k in ("qw", "qx", "qy", "qz")],
                                      self.R_body_cam, self.t_body_cam)
        box_obs = [BeaconBoxObs(bbox=d.bbox_2d, score=d.confidence) for d in boxes]
        fixes = locate_beacons(obs, self.cam, pose, self.water_z, self.model, self.params,
                               boxes=box_obs)
        ref = (telem["ref_lat"], telem["ref_lon"], telem.get("ref_alt") or 0.0) \
            if telem.get("ref_lat") is not None else None
        gps = (telem["gps_lat"], telem["gps_lon"], telem["gps_alt_msl_m"]) \
            if telem.get("gps_fix_type") and telem["gps_fix_type"] >= 3 else None

        def lat_lon(pos):
            """Lat/lon of an NED point: from PX4's reference origin (live), else via
            the drone's own GPS fix and NED offset (recordings), else None."""
            if ref is not None:
                return ned_to_lla(pos, *ref)[:2]
            if gps is not None:
                return ned_to_lla(np.asarray(pos) - drone_ned, *gps)[:2]
            return None

        def where(pos):
            ll = lat_lon(pos)
            return f"{ll[0]:.6f}, {ll[1]:.6f}" if ll else f"NED {pos[0]:.1f}, {pos[1]:.1f}"

        for f in fixes:
            px = self._project(pose, f.pos_ned)
            sig = math.sqrt(max(float(np.trace(f.cov[:2, :2])), 0.0))
            horiz = float(np.linalg.norm(f.pos_ned[:2] - pose.c_ned[:2]))
            color = FIX_COLORS.get(f.method, WEAK)
            if px is None:
                continue
            x, y = int(px[0]), int(px[1])
            cv2.drawMarker(out, (x, y), color, cv2.MARKER_TILTED_CROSS, 16, 2, cv2.LINE_AA)
            put_label(out, f"{f.method}: {f.range_m:.1f} m slant, {horiz:.1f} m horiz",
                      (x + 10, y + 16), color)
            put_label(out, f"{where(f.pos_ned)} +/-{sig:.1f} m", (x + 10, y + 32), color)

        t = telem["frame_stamp"] if telem.get("frame_stamp") is not None else telem["frame"] / 10
        seen = {b["id"] for b in self.map.update(fixes, t)}
        if self.publish is not None:
            self._publish(telem, t, pose, drone_ned, fixes, seen, lat_lon)
        rows = []
        for b in self.map.beacons:
            if b["n"] < self.min_fixes:
                continue
            px = self._project(pose, b["pos"])
            if px is not None:
                cv2.circle(out, (int(px[0]), int(px[1])), 7, FUSED, 2, cv2.LINE_AA)
                put_label(out, f"B{b['id']}", (int(px[0]) - 8, int(px[1]) - 10), FUSED)
            sig = math.sqrt(max(float(np.trace(self.map.cov_at(b, t)[:2, :2])), 0.0))
            dist = float(np.linalg.norm(b["pos"][:2] - pose.c_ned[:2]))
            rows.append(f"B{b['id']}  {where(b['pos'])} +/-{sig:.2f} m  "
                        f"dist {dist:.1f} m  n={b['n']} out={b['outliers']}")
        y0 = out.shape[0] - 10 - 18 * len(rows)
        if rows:
            cv2.rectangle(out, (0, y0 - 18), (out.shape[1], out.shape[0]), (0, 0, 0), -1)
            put_label(out, "fused (this loop):" if self.source == "replay" else "beacon map:",
                      (6, y0 - 4), FUSED)
        for k, r in enumerate(rows):
            put_label(out, r, (6, y0 + 14 + 18 * k), FUSED)

    def _publish(self, telem, t, pose, drone_ned, fixes, seen, lat_lon):
        def ll(pos):
            v = lat_lon(pos)
            return (round(v[0], 8), round(v[1], 8)) if v else (None, None)

        def sig(cov):
            return round(math.sqrt(max(float(np.trace(cov[:2, :2])), 0.0)), 3)

        def ned(pos):
            return [round(float(v), 3) for v in pos]

        beacons = []
        for b in self.map.beacons:
            if b["n"] < self.min_fixes:
                continue
            lat, lon = ll(b["pos"])
            beacons.append(dict(id=b["id"], ned=ned(b["pos"]), lat=lat, lon=lon,
                                sigma_m=sig(self.map.cov_at(b, t)), n=b["n"],
                                outliers=b["outliers"], age_s=round(t - b["last_t"], 3),
                                seen=b["id"] in seen))
        fx = []
        for f in fixes:
            lat, lon = ll(f.pos_ned)
            fx.append(dict(method=f.method, ned=ned(f.pos_ned), lat=lat, lon=lon,
                           sigma_m=sig(f.cov), range_m=round(f.range_m, 3),
                           horiz_m=round(float(np.linalg.norm(f.pos_ned[:2] - pose.c_ned[:2])), 3),
                           rms_px=None if f.rms_px is None else round(f.rms_px, 3),
                           n_diamonds=len(f.members), n_boxes=len(f.box_members)))
        lat, lon = ll(drone_ned)
        self.publish(json.dumps(dict(
            stamp=t, source=self.source, frame=telem.get("frame"),
            drone=dict(ned=ned(drone_ned), lat=lat, lon=lon), water_z_ned=self.water_z,
            beacons=beacons, fixes=fx)))

    def _project(self, pose, p_ned) -> Optional[np.ndarray]:
        """NED point -> pixel, or None if behind the camera / far off-frame."""
        pc = pose.R_ned_cam.T @ (np.asarray(p_ned, float) - pose.c_ned)
        if pc[2] <= 0.1:
            return None
        px = self.cam.project(pc[None])[0]
        h, w = self.cam.height, self.cam.width
        if not (-w <= px[0] <= 2 * w and -h <= px[1] <= 2 * h):
            return None
        return px


def draw_hud(img, text):
    cv2.rectangle(img, (0, 0), (img.shape[1], 22), (0, 0, 0), -1)
    put_label(img, text, (6, 16), (255, 255, 255), 0.5)


# ─── Main ─────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--model", required=True, help=".tflite pose model")
    ap.add_argument("--classes", default="diamond",
                    help="comma-separated, in model class order (new model: beacon,diamond)")
    ap.add_argument("--diamond-label", default="diamond",
                    help="class carrying keypoints; every other class is drawn as a box")
    ap.add_argument("--conf", type=float, default=0.5)
    ap.add_argument("--threads", type=int, default=4)
    ap.add_argument("--delegate", default=None, help="TFLite delegate .so (default: CPU)")
    ap.add_argument("--gpu-lib", default=None,
                    help="libtflite_gpu_shim.so: run on the VOXL2 GPU (voxl/beacon_gpu)")
    ap.add_argument("--cpu", action="store_true",
                    help="with --gpu-lib: run the shim's TFLite on the CPU instead")
    ap.add_argument("--resize", choices=("stretch", "letterbox"), default="stretch",
                    help="frame -> model input; must match training (the 2-class pose "
                         "model was trained stretched)")
    ap.add_argument("--topic", default="/tracking_front")
    ap.add_argument("--image-time-offset", type=float, default=0.0, metavar="MS",
                    help="live: added to the image header stamp before looking up the PX4 "
                         "pose (residual capture-time error, e.g. half the exposure)")
    ap.add_argument("--intrinsics", default="/data/modalai/opencv_tracking_front_intrinsics.yml")
    ap.add_argument("--pose-config", default=os.path.join(HERE, "configs", "beacon_pose.json"))
    ap.add_argument("--port", type=int, default=8091)
    ap.add_argument("--quality", type=int, default=80, help="JPEG quality")
    ap.add_argument("--out-width", type=int, default=0, help="downscale stream to this width")
    ap.add_argument("--max-fps", type=float, default=0, help="cap processing rate (0 = none)")
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--video", help="video file instead of ROS")
    src.add_argument("--image", help="single image instead of ROS")
    ap.add_argument("--file-fps", type=float, default=10, help="--video/--image feed rate")
    ap.add_argument("--telemetry", metavar="CSV",
                    help="with --video: telemetry_viewer.py CSV (one row per frame); "
                         "enables beacon localisation and lat/lon")
    ap.add_argument("--extrinsics", default="/etc/modalai/extrinsics.conf")
    ap.add_argument("--camera-frame", default="tracking_front",
                    help="extrinsics child frame of the camera")
    ap.add_argument("--water-z", type=float, default=0.0,
                    help="water surface in PX4 local NED z (m, down positive); 0 = the "
                         "height of PX4's local origin (where it armed)")
    ap.add_argument("--map-floor", type=float, default=0.5,
                    help="per-fix horizontal noise floor (m) when fusing: covers drone GPS "
                         "error, which the per-fix covariance leaves out")
    ap.add_argument("--drift", type=float, default=0.15,
                    help="beacon horizontal random walk (m/sqrt(s)) in the map: follows a "
                         "floating beacon instead of splitting it; 0 = static beacons")
    ap.add_argument("--min-fixes", type=int, default=3,
                    help="draw a mapped beacon only once it has this many fixes")
    ap.add_argument("--no-locate", action="store_true",
                    help="live: don't localise from PX4 (camera-only ranges)")
    ap.add_argument("--map-topic", default="/seabird/beacon_map",
                    help="std_msgs/String JSON beacon map, published while localising")
    ap.add_argument("--no-publish", action="store_true", help="don't publish --map-topic")
    ap.add_argument("--once", metavar="OUT.jpg",
                    help="process one frame, write the annotated JPEG and exit (no server)")
    args = ap.parse_args()
    if args.telemetry and not args.video:
        ap.error("--telemetry needs --video")
    live = not (args.video or args.image)
    args.locate = bool(args.telemetry) or (live and not args.no_locate)

    ros = None
    if live or (args.locate and not args.no_publish and not args.once):
        try:
            ros = RosIO()
        except ImportError:
            if live:
                raise
            print("[overlay] WARNING: rclpy not available; not publishing")
    px4 = ros.subscribe_px4() if live and args.locate else None
    if live and px4 is None:
        args.locate = False

    overlay = Overlay(args)
    if ros is not None and args.locate and not args.no_publish and not args.once:
        overlay.publish = ros.string_publisher(args.map_topic)
    frames = LatestFrame()
    if args.telemetry:
        rows = load_telemetry(args.telemetry)
        print(f"[overlay] replay {args.video} with {len(rows)} telemetry rows")
        start_replay_source(args.video, rows, frames, args.file_fps)
    elif args.video or args.image:
        start_file_source(args.video or args.image, bool(args.video), frames, args.file_fps)
    else:
        ros.subscribe_image(args.topic, frames, px4, args.image_time_offset / 1e3)
    if ros is not None:
        ros.spin()

    hub = FrameHub()
    if not args.once:
        server = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(hub))
        server.daemon_threads = True
        threading.Thread(target=server.serve_forever, daemon=True).start()
        print(f"[overlay] serving http://0.0.0.0:{args.port}/  (stream: /stream.mjpg)")

    enc = [cv2.IMWRITE_JPEG_QUALITY, args.quality]
    fps, last, last_log, n = 0.0, time.monotonic(), 0.0, 0
    while True:
        item = frames.take()
        if item is None:
            continue
        frame, telem, loop = item
        out, infer_ms, nd, nb = overlay.process(frame, telem, loop)

        now = time.monotonic()
        dt, last = now - last, now
        fps = 1 / dt if fps == 0 else 0.9 * fps + 0.1 / max(dt, 1e-6)
        hud = (f"{fps:4.1f} fps | {overlay.detector.backend} {infer_ms:4.0f} ms | "
               f"diamonds {nd} | beacons {nb}")
        if telem is not None:
            alt = telem.get("height_agl_m")
            hdg = telem.get("heading_deg")
            if alt is not None and hdg is not None and telem.get("local_x") is None:
                alt = None    # live: pose not valid/fresh, don't show stale numbers
            hud += (f" | frame {telem['frame']} | alt {alt:.1f} m | hdg {hdg:.0f} deg"
                    if alt is not None and hdg is not None else f" | frame {telem['frame']}")
            if telem.get("gps_fix_type") is not None:
                hud += f" | GPS fix {telem['gps_fix_type']:.0f}, {telem.get('gps_sats') or 0:.0f} sats"
        else:
            hud += " | range: camera-only (size)"
        draw_hud(out, hud)
        if args.out_width and out.shape[1] > args.out_width:
            s = args.out_width / out.shape[1]
            out = cv2.resize(out, (args.out_width, int(round(out.shape[0] * s))),
                             interpolation=cv2.INTER_AREA)
        ok, jpeg = cv2.imencode(".jpg", out, enc)
        if not ok:
            continue
        if args.once:
            with open(args.once, "wb") as f:
                f.write(jpeg.tobytes())
            print(f"[overlay] {hud}\n[overlay] wrote {args.once}")
            return
        hub.publish(jpeg.tobytes())

        n += 1
        if now - last_log > 5:
            sync = ""
            if px4 is not None:
                sync = " | pose @capture {interp}, latest {latest}, none {none}".format(**px4.stats)
                if telem is not None and telem.get("pose_dt_ms") is not None:
                    sync += f" (newest PX4 sample {telem['pose_dt_ms']:+.0f} ms from capture)"
            print(f"[overlay] {hud} | frames {n}{sync}", flush=True)
            last_log = now
        if args.max_fps > 0:
            time.sleep(max(0.0, 1 / args.max_fps - (time.monotonic() - now)))


if __name__ == "__main__":
    main()
