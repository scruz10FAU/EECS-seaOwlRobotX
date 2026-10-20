#!/usr/bin/env python3
"""
Telemetry stream — tracking_front frames + PX4 GPS / altitude / attitude / IMU,
served over HTTP for telemetry_viewer.py on a laptop.

    http://<drone-ip>:8092/                 (browser viewer, image only)
    http://<drone-ip>:8092/stream.mjpg      MJPEG; every part carries an
                                            X-Telemetry: <json> header for
                                            that frame (pose at its capture)
    http://<drone-ip>:8092/telemetry.json   latest telemetry snapshot
    http://<drone-ip>:8092/snapshot.jpg
    POST /camera?exp_ms=2.0&gain=100        manual exposure (voxl-camera-server
    POST /camera?ae=1                       set_exp_gain) / back to auto exposure

Exposure commands go straight into the camera server's MPA control pipe
(<mpa-dir>/<camera>/control), the same thing voxl-send-command does, so the
container needs /run/mpa mounted read-write (run.sh does this).

Telemetry (PX4 over microDDS, best effort):
  /fmu/out/vehicle_gps_position    lat, lon, MSL alt, fix type, satellites
  /fmu/out/vehicle_local_position  NED x/y/z, height above ground (-z), heading
  /fmu/out/vehicle_attitude        quaternion (FRD body -> NED) + roll/pitch/yaw
  /fmu/out/sensor_combined         gyro (rad/s) + accelerometer (m/s^2), body FRD
Every group carries age_s (seconds since its last message); a group is null
until its first message arrives.

Per frame, local x/y/z (+ height_agl_m) and the attitude are interpolated to the
image's capture stamp (header.stamp + --image-time-offset) from PX4's
timestamp_sample history, and those groups get at_frame: true. On VOXL2 both are
on the same clock and frames arrive ~35 ms after capture, so the latest values
would lag the image by that much. Without a match (stamp outside the history,
e.g. clocks differ) the group keeps its latest values and at_frame: false.

Sources:
  ROS 2 (default)  sensor_msgs/Image on --topic (mono8, bgr8 or rgb8)
  --video PATH     a video file, looped (offline testing)
  --image PATH     one image, repeated
Without rclpy/px4_msgs (e.g. on a laptop) telemetry stays null.

Usage (on the drone, see voxl/telemetry_stream/run.sh):
    python3 telemetry_stream.py
    python3 telemetry_stream.py --out-width 640 --max-fps 15
    python3 telemetry_stream.py --image frame.png        # offline
"""

import argparse
import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional
from urllib.parse import parse_qs

import cv2

from beacon_overlay import FrameHub, LatestFrame, image_msg_to_array, image_stamp, start_file_source
from beacon_pose import StateBuffer


# ─── Telemetry ────────────────────────────────────────────────────────

def quat_to_euler_deg(w, x, y, z):
    """FRD->NED quaternion -> (roll, pitch, yaw) in degrees, ZYX convention."""
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class Telemetry:
    """Latest value per group, stamped with the local receive time, plus local
    position / attitude history (PX4 timestamp_sample) for snapshot_at()."""

    GROUPS = ("gps", "local", "attitude", "imu")
    MAX_EXTRAP_S = 0.05

    def __init__(self):
        self._lock = threading.Lock()
        self._data = {g: None for g in self.GROUPS}
        self._rx = {g: None for g in self.GROUPS}
        self.pos, self.att = StateBuffer(), StateBuffer(quat=True)

    def set(self, group, values):
        with self._lock:
            self._data[group] = values
            self._rx[group] = time.monotonic()

    def snapshot(self) -> dict:
        now = time.monotonic()
        with self._lock:
            out = {}
            for g in self.GROUPS:
                if self._data[g] is None:
                    out[g] = None
                else:
                    out[g] = dict(self._data[g], age_s=round(now - self._rx[g], 3))
            return out

    def snapshot_at(self, stamp: float) -> dict:
        """snapshot(), with local x/y/z and attitude interpolated to `stamp`."""
        out = self.snapshot()
        loc, att = out["local"], out["attitude"]
        p = self.pos.at(stamp, self.MAX_EXTRAP_S)
        if loc is not None:
            loc["at_frame"] = p is not None
            if p is not None:
                loc.update(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                if loc.get("z_valid"):
                    loc["height_agl_m"] = float(-p[2])
        q = self.att.at(stamp, self.MAX_EXTRAP_S)
        if att is not None:
            att["at_frame"] = q is not None
            if q is not None:
                w, x, y, z = (float(v) for v in q)
                roll, pitch, yaw = quat_to_euler_deg(w, x, y, z)
                att.update(qw=w, qx=x, qy=y, qz=z, roll_deg=roll, pitch_deg=pitch, yaw_deg=yaw)
        return out


def start_ros(topic: Optional[str], frames: LatestFrame, telem: Telemetry,
              offset_s: float = 0.0):
    """One node: image (if topic) + PX4 telemetry. Returns False if ROS is missing."""
    try:
        import rclpy
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    except ImportError:
        print("[telemetry] rclpy not available: no ROS image, telemetry stays null")
        return False

    rclpy.init()
    node = rclpy.create_node("telemetry_stream")
    log = node.get_logger()

    if topic:
        from sensor_msgs.msg import Image
        warned = set()

        def on_image(msg):
            img = image_msg_to_array(msg)
            if img is None:
                if msg.encoding not in warned:
                    warned.add(msg.encoding)
                    log.error(f"unsupported encoding '{msg.encoding}' on {topic}")
                return
            frames.put((img, image_stamp(msg, offset_s)))

        # Best effort matches both best-effort and reliable publishers.
        node.create_subscription(Image, topic, on_image, qos_profile_sensor_data)
        print(f"[telemetry] subscribed to {topic}")

    try:
        from px4_msgs.msg import SensorCombined, SensorGps, VehicleAttitude, VehicleLocalPosition
    except ImportError:
        print("[telemetry] px4_msgs not available: telemetry stays null")
    else:
        # PX4's microDDS publishers are best effort.
        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)

        def on_gps(m):
            # lat/lon int32 1e-7 deg, alt int32 mm; fix_type < 3 = no 3D fix.
            telem.set("gps", {
                "lat": m.lat * 1e-7, "lon": m.lon * 1e-7, "alt_msl_m": m.alt * 1e-3,
                "fix_type": int(m.fix_type), "satellites": int(m.satellites_used),
                "eph": float(m.eph), "epv": float(m.epv)})

        def on_local(m):
            telem.pos.add(m.timestamp_sample * 1e-6, (m.x, m.y, m.z))
            telem.set("local", {
                "x": float(m.x), "y": float(m.y), "z": float(m.z),
                "height_agl_m": float(-m.z) if m.z_valid else None,
                "heading_deg": math.degrees(m.heading) if math.isfinite(m.heading) else None,
                "xy_valid": bool(m.xy_valid), "z_valid": bool(m.z_valid)})

        def on_attitude(m):
            telem.att.add(m.timestamp_sample * 1e-6, [float(v) for v in m.q])
            w, x, y, z = (float(v) for v in m.q)
            roll, pitch, yaw = quat_to_euler_deg(w, x, y, z)
            telem.set("attitude", {"qw": w, "qx": x, "qy": y, "qz": z,
                                   "roll_deg": roll, "pitch_deg": pitch, "yaw_deg": yaw})

        def on_imu(m):
            g, a = m.gyro_rad, m.accelerometer_m_s2
            telem.set("imu", {"gyro_x": float(g[0]), "gyro_y": float(g[1]), "gyro_z": float(g[2]),
                              "accel_x": float(a[0]), "accel_y": float(a[1]), "accel_z": float(a[2])})

        for msg_type, t, cb in (
                (SensorGps, "/fmu/out/vehicle_gps_position", on_gps),
                (VehicleLocalPosition, "/fmu/out/vehicle_local_position", on_local),
                (VehicleAttitude, "/fmu/out/vehicle_attitude", on_attitude),
                (SensorCombined, "/fmu/out/sensor_combined", on_imu)):
            node.create_subscription(msg_type, t, cb, px4_qos)
        print("[telemetry] subscribed to PX4 gps / local position / attitude / sensor_combined")

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    return True


# ─── Camera control ───────────────────────────────────────────────────

class CameraControl:
    """Writes voxl-camera-server commands into <mpa_dir>/<camera>/control."""

    def __init__(self, mpa_dir: str, camera: str):
        self.path = os.path.join(mpa_dir, camera, "control")

    def send(self, cmd: str):
        # Non-blocking: fails with ENXIO instead of hanging if the server is down.
        fd = os.open(self.path, os.O_WRONLY | os.O_NONBLOCK)
        try:
            os.write(fd, cmd.encode() + b"\0")   # NUL-terminated, as libmodal_pipe sends
        finally:
            os.close(fd)

    def handle(self, query: str) -> str:
        """Query string -> command sent. Raises ValueError on bad input, OSError on pipe."""
        q = {k: v[-1] for k, v in parse_qs(query).items()}
        if q.get("ae") == "1":
            cmd = "start_ae"
        else:
            exp_ms, gain = float(q["exp_ms"]), int(float(q["gain"]))
            if not (0.0 < exp_ms <= 100.0) or gain <= 0:
                raise ValueError("exp_ms must be in (0, 100], gain > 0")
            cmd = f"set_exp_gain {exp_ms:.3f} {gain}"
        self.send(cmd)
        return cmd


# ─── HTTP ─────────────────────────────────────────────────────────────

class TelemetryHub(FrameHub):
    """FrameHub whose frames each carry the telemetry JSON they were encoded with."""

    def publish(self, jpeg: bytes, telem_json: bytes = b"{}"):
        with self._cond:
            self._telem = telem_json
            super().publish(jpeg)

    def wait(self, last_seq: int, timeout: float = 5.0):
        with self._cond:
            seq, jpeg = super().wait(last_seq, timeout)
            return seq, jpeg, getattr(self, "_telem", b"{}")


VIEWER_HTML = b"""<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Telemetry stream</title>
<style>body{margin:0;background:#111;color:#ccc;font:14px sans-serif}
img{display:block;max-width:100%;margin:0 auto}p{text-align:center}a{color:#8cf}</style></head>
<body><img src="/stream.mjpg"><p><a href="/telemetry.json">telemetry.json</a> &middot;
<a href="/snapshot.jpg">snapshot</a></p></body></html>"""


def make_handler(hub: TelemetryHub, telem: Telemetry, cam: CameraControl):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):  # quiet: one line per request is noise
            pass

        def do_POST(self):
            path, _, query = self.path.partition("?")
            if path != "/camera":
                self._send(404, "text/plain", b"not found\n")
                return
            try:
                cmd = cam.handle(query)
            except (KeyError, ValueError) as e:
                self._send(400, "application/json",
                           json.dumps({"ok": False, "error": f"bad request: {e}"}).encode())
                return
            except OSError as e:
                self._send(503, "application/json",
                           json.dumps({"ok": False, "error": f"{cam.path}: {e}"}).encode())
                return
            print(f"[telemetry] camera: {cmd}", flush=True)
            self._send(200, "application/json", json.dumps({"ok": True, "cmd": cmd}).encode())

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                self._send(200, "text/html", VIEWER_HTML)
            elif path == "/telemetry.json":
                self._send(200, "application/json", json.dumps(telem.snapshot()).encode())
            elif path == "/snapshot.jpg":
                _, jpeg, _ = hub.wait(-1, timeout=0)
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
                    new_seq, jpeg, tj = hub.wait(seq)
                    if jpeg is None or new_seq == seq:
                        continue
                    seq = new_seq
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                     b"X-Telemetry: %s\r\n"
                                     b"Content-Length: %d\r\n\r\n" % (tj, len(jpeg)))
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


# ─── Main ─────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--topic", default="/tracking_front")
    ap.add_argument("--image-time-offset", type=float, default=0.0, metavar="MS",
                    help="added to the image header stamp before interpolating the pose")
    ap.add_argument("--port", type=int, default=8092)
    ap.add_argument("--camera", default="tracking_front",
                    help="voxl-camera-server pipe that exposure commands go to")
    ap.add_argument("--mpa-dir", default="/run/mpa")
    ap.add_argument("--quality", type=int, default=85, help="JPEG quality")
    ap.add_argument("--out-width", type=int, default=0, help="downscale stream to this width")
    ap.add_argument("--max-fps", type=float, default=10, help="cap stream rate (0 = none)")
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--video", help="video file instead of ROS")
    src.add_argument("--image", help="single image instead of ROS")
    ap.add_argument("--file-fps", type=float, default=10, help="--video/--image feed rate")
    args = ap.parse_args()

    frames, telem = LatestFrame(), Telemetry()
    if args.video or args.image:
        # File frames carry no stamp; wrap them so the loop sees (img, stamp).
        raw = LatestFrame()
        start_file_source(args.video or args.image, bool(args.video), raw, args.file_fps)

        def relay():
            while True:
                item = raw.take()      # (img, telemetry, loop) from beacon_overlay
                if item is not None:
                    frames.put((item[0], time.time()))

        threading.Thread(target=relay, daemon=True).start()
        start_ros(None, frames, telem)
    elif not start_ros(args.topic, frames, telem, args.image_time_offset / 1e3):
        raise SystemExit("ROS 2 unavailable: use --video or --image")

    hub = TelemetryHub()
    server = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(hub, telem, CameraControl(args.mpa_dir, args.camera)))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"[telemetry] serving http://0.0.0.0:{args.port}/  (stream: /stream.mjpg)")

    enc = [cv2.IMWRITE_JPEG_QUALITY, args.quality]
    last_log, n = 0.0, 0
    while True:
        item = frames.take()
        if item is None:
            continue
        t0 = time.monotonic()
        img, stamp = item
        if args.out_width and img.shape[1] > args.out_width:
            s = args.out_width / img.shape[1]
            img = cv2.resize(img, (args.out_width, int(round(img.shape[0] * s))),
                             interpolation=cv2.INTER_AREA)
        ok, jpeg = cv2.imencode(".jpg", img, enc)
        if not ok:
            continue
        snap = telem.snapshot_at(stamp)
        snap["frame"] = {"stamp": stamp, "width": img.shape[1], "height": img.shape[0]}
        hub.publish(jpeg.tobytes(), json.dumps(snap, separators=(",", ":")).encode())

        n += 1
        if t0 - last_log > 5:
            have = [g for g in Telemetry.GROUPS if snap[g] is not None]
            print(f"[telemetry] frames {n} | {img.shape[1]}x{img.shape[0]} "
                  f"{len(jpeg) // 1024} KB | telemetry: {', '.join(have) or 'none'}", flush=True)
            last_log = t0
        if args.max_fps > 0:
            time.sleep(max(0.0, 1 / args.max_fps - (time.monotonic() - t0)))


if __name__ == "__main__":
    main()
