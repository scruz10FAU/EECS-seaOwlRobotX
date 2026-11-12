#!/usr/bin/env python3
"""
beacon_mavlink_bridge.py — relays /seabird/beacon_detections to the ground
station as a binary MAVLink message, riding the existing MAVLink link to
the flight controller (see utils/beacon_mavlink_protocol.py for the wire
format: a standard MEMORY_VECT message with a sentinel `address` carrying
packed lat/lon/color/blink-status fields).

This connects to the SAME MAVLink endpoint sweep_lawnmower.py/sweep_rrt.py
use via MAVSDK_ADDRESS (PX4 SITL in sim, voxl-mavlink-server on the
physical drone) — but via pymavlink instead of MAVSDK, since MAVSDK's
Python API has no generic "send an arbitrary MAVLink message" call.

Whether this message actually reaches the ground station's radio link
depends on how PX4/voxl-mavlink-server are configured to forward
third-party-injected traffic between endpoints — verify it actually
arrives (e.g. with beacon_mavlink_ground.py or a raw MAVLink inspector at
the ground station) before relying on it operationally.

Requires: pip install pymavlink

Usage:
    python3 beacon_mavlink_bridge.py                                       # sim default
    python3 beacon_mavlink_bridge.py --mavlink-endpoint udpout:127.0.0.1:14551   # physical drone
    python3 beacon_mavlink_bridge.py --repeat 3 --repeat-gap 1.0                 # 3 copies, 1 s apart
    python3 beacon_mavlink_bridge.py --log sent.csv                              # CSV row per copy sent

Each detection is sent --repeat times (default 2) with the same msg_id, so
beacon_mavlink_ground.py can drop the extra copies. --repeat-gap spaces the
copies out (default 0.5 s), since radio losses tend to come in bursts.

It also relays the RobotX task the mission is attempting: each RxTask value
on --task-topic (std_msgs/UInt8, published by the sweep script) goes out as a
STATUSTEXT "SEABIRD_TASK <n>" right away and is resent every
TASK_RESEND_S, so a dropped packet or a restarted ground station still
learns it. beacon_mavlink_ground.py turns it into VehicleStatus.current_task.

And it brings RoboCommand's RunStart down to the drone (Handbook 3.4.10:
vehicles begin the run only after the OCS verifies RunStart). The ground
station sends STATUSTEXT "SEABIRD_RUNSTART <run_id>" up ELRS; the drone's ELRS
bridge (voxl/elrs_mavlink_bridge_ratelimited.py) copies ground STATUSTEXT
starting "SEABIRD_" to --onboard-listen; this publishes it once per run_id on
--run-start-topic (latched std_msgs/String JSON {"run_id": n}) for the sweep,
and answers "SEABIRD_RUNSTART_ACK <run_id>" every time so the ground stops
resending.
"""

import argparse
import csv
import json
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String, UInt8

from pymavlink import mavutil

from utils.beacon_mavlink_protocol import (
    pack_value_array, BEACON_MAVLINK_ADDRESS, BEACON_MAVLINK_VERSION,
)

DEFAULT_MAVLINK_ENDPOINT = "udp://:14540"   # matches sweep_lawnmower.py's sim MAVSDK_ADDRESS
DEFAULT_DETECTIONS_TOPIC = "/seabird/beacon_detections_downward"
DEFAULT_REPEAT = 2
DEFAULT_REPEAT_GAP = 0.5
DEFAULT_TASK_TOPIC = "/seabird/current_task"
# Must match beacon_mavlink_ground.py's TASK_STATUSTEXT_PREFIX.
TASK_STATUSTEXT_PREFIX = "SEABIRD_TASK"
TASK_RESEND_S = 2.0
# RunStart relay -- must match beacon_mavlink_ground.py and the drone ELRS
# bridge's --onboard-ip/--onboard-port (default 127.0.0.1:14570).
DEFAULT_ONBOARD_LISTEN = "udpin:127.0.0.1:14570"
DEFAULT_RUN_START_TOPIC = "/seabird/run_start"
# WiFi redundancy (elrs_windows_router.py --wifi): the ground router sends its
# heartbeats and SEABIRD_* STATUSTEXT (RunStart) here over WiFi. RunStart is
# handled exactly like the ELRS copy (published once per run_id, ACK on every
# copy), and everything this bridge sends (beacons, SEABIRD_TASK, ALERT,
# RunStart ACKs) also goes back to every router that has sent here -- --mavlink-
# endpoint goes straight to the ELRS bridge, not through voxl-mavlink-server,
# so none of it would reach the WiFi link otherwise. "none" disables it.
DEFAULT_WIFI_LISTEN = "udpin:0.0.0.0:14572"
# Pre-flight abort reasons from the flight script (latched std_msgs/String),
# relayed as WARNING STATUSTEXT so QGC shows them and beacon_mavlink_ground.py
# logs them. STATUSTEXT holds 50 chars (MAVLink 1, no chunking): longer text
# goes as "ALERT k/n <part>" messages. Must match beacon_mavlink_ground.py.
DEFAULT_ALERT_TOPIC = "/seabird/ground_alert"
ALERT_PREFIX = "ALERT"
ALERT_PART_CHARS = 40     # 50 - len("ALERT k/n ")
ALERT_MAX_PARTS = 9
ALERT_REPEATS = 3         # whole alert resent, since ELRS loses bursts
ALERT_REPEAT_GAP_S = 2.0
RUNSTART_PREFIX = "SEABIRD_RUNSTART"
RUNSTART_ACK_PREFIX = "SEABIRD_RUNSTART_ACK"

_LOG_FIELDS = ["iso_time", "epoch_time", "copy", "msg_id", "tracking_id",
               "latitude", "longitude", "color", "is_blinking"]


class BeaconMavlinkBridge(Node):
    def __init__(self, mavlink_endpoint: str, detections_topic: str,
                 repeat: int = DEFAULT_REPEAT, repeat_gap: float = DEFAULT_REPEAT_GAP,
                 log_path: str = None, task_topic: str = DEFAULT_TASK_TOPIC,
                 onboard_listen: str = DEFAULT_ONBOARD_LISTEN,
                 run_start_topic: str = DEFAULT_RUN_START_TOPIC,
                 alert_topic: str = DEFAULT_ALERT_TOPIC,
                 wifi_listen: str = DEFAULT_WIFI_LISTEN):
        super().__init__("beacon_mavlink_bridge")
        self._lock = threading.Lock()
        self._run_lock = threading.Lock()   # ELRS and WiFi listeners share _run_id
        self._repeat = max(1, repeat)
        self._repeat_gap = max(0.0, repeat_gap)
        self._next_msg_id = 0
        self._current_task = None  # nothing sent until the mission reports a task
        self._run_id = None        # last RunStart run_id published to the sweep
        self._running = True

        self._log_file = self._log_writer = None
        if log_path:
            self._log_file = open(log_path, "a", newline="")
            self._log_writer = csv.writer(self._log_file)
            if self._log_file.tell() == 0:
                self._log_writer.writerow(_LOG_FIELDS)
                self._log_file.flush()
            self.get_logger().info(f"Logging each sent copy to {log_path}")

        self.get_logger().info(f"Connecting to MAVLink endpoint: {mavlink_endpoint}")
        self._mav = mavutil.mavlink_connection(
            mavlink_endpoint,
            source_system=1,
            source_component=191
        )

        self.create_subscription(String, detections_topic, self._on_detection, 10)
        self.get_logger().info(f"Subscribed to {detections_topic} — relaying to MAVLink "
                               f"({self._repeat}x, {self._repeat_gap:.2f} s apart)")

        self.create_subscription(UInt8, task_topic, self._on_task, 10)
        self.create_timer(TASK_RESEND_S, self._send_task)
        self.get_logger().info(f"Subscribed to {task_topic} — relaying as STATUSTEXT "
                               f"'{TASK_STATUSTEXT_PREFIX} <n>' every {TASK_RESEND_S:.0f} s")

        # Latched on both sides: an alert published before this bridge started
        # (or reconnected) is still delivered.
        self.create_subscription(
            String, alert_topic, self._on_alert,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.get_logger().info(f"Subscribed to {alert_topic} — relaying alerts as WARNING "
                               f"STATUSTEXT '{ALERT_PREFIX} k/n ...' ({ALERT_REPEATS}x)")

        self._run_start_pub = self.create_publisher(
            String, run_start_topic, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._onboard = mavutil.mavlink_connection(onboard_listen)
        threading.Thread(target=self._listen_onboard, args=(self._onboard, "ELRS"),
                         daemon=True).start()
        self.get_logger().info(f"Listening on {onboard_listen} for '{RUNSTART_PREFIX} <run_id>' "
                               f"from the ground — publishing RunStart on {run_start_topic}")

        self._wifi = None
        if wifi_listen and wifi_listen.lower() != "none":
            self._wifi = mavutil.mavlink_connection(
                wifi_listen, source_system=1, source_component=191)
            threading.Thread(target=self._listen_onboard, args=(self._wifi, "WiFi"),
                             daemon=True).start()
            self.get_logger().info(f"WiFi: listening on {wifi_listen} for RunStart; beacons, "
                                   f"task, alerts and ACKs also sent to the ground routers "
                                   f"that send there")

    def _mav_send(self, send) -> None:
        """send(connection) on the ELRS link and, if any ground router has
        reached it, the WiFi link. A WiFi failure never blocks the ELRS copy."""
        with self._lock:
            send(self._mav)
            if self._wifi is not None:
                try:
                    send(self._wifi)
                except OSError:
                    pass

    def _listen_onboard(self, conn, label: str) -> None:
        """Ground STATUSTEXT: copied onboard by the drone ELRS bridge, or sent
        over WiFi by elrs_windows_router.py --wifi."""
        while self._running:
            try:
                msg = conn.recv_match(type="STATUSTEXT", blocking=True, timeout=1.0)
            except OSError:
                time.sleep(1.0)
                continue
            if msg is None:
                continue
            parts = str(msg.text).strip("\x00 ").split()
            if len(parts) != 2 or parts[0] != RUNSTART_PREFIX or not parts[1].isdigit():
                continue
            run_id = int(parts[1])
            with self._run_lock:
                new = run_id != self._run_id
                if new:
                    self._run_id = run_id
                    self._run_start_pub.publish(String(data=json.dumps(
                        {"run_id": run_id, "received": time.time()})))
            if new:
                self.get_logger().info(f"RunStart run_id={run_id} received from the ground "
                                       f"({label}) — the sweep may begin the run")
            # Every copy: the ground resends until it sees this.
            self._mav_send(lambda m: m.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO, f"{RUNSTART_ACK_PREFIX} {run_id}".encode()))

    def _on_alert(self, msg: String) -> None:
        text = " ".join(str(msg.data).split())   # one line
        if not text:
            return
        parts = [text[i:i + ALERT_PART_CHARS]
                 for i in range(0, len(text), ALERT_PART_CHARS)][:ALERT_MAX_PARTS]
        self.get_logger().warn(f"ground alert: {text}")
        for r in range(ALERT_REPEATS):
            t = threading.Timer(r * ALERT_REPEAT_GAP_S, self._send_alert, args=(parts,))
            t.daemon = True
            t.start()

    def _send_alert(self, parts: list) -> None:
        n = len(parts)

        def send(m):
            for k, part in enumerate(parts, 1):
                m.mav.statustext_send(
                    mavutil.mavlink.MAV_SEVERITY_WARNING,
                    f"{ALERT_PREFIX} {k}/{n} {part}".encode()[:50])
        self._mav_send(send)

    def _on_task(self, msg: UInt8) -> None:
        if msg.data == self._current_task:
            return
        self.get_logger().info(f"current_task {self._current_task} -> {msg.data}")
        self._current_task = msg.data
        self._send_task()

    def _send_task(self) -> None:
        if self._current_task is None:
            return
        text = f"{TASK_STATUSTEXT_PREFIX} {self._current_task}".encode()
        self._mav_send(lambda m: m.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, text))

    def _send(self, value: list, copy: int, log_fields: list) -> None:
        self._mav_send(lambda m: m.mav.memory_vect_send(
            BEACON_MAVLINK_ADDRESS, BEACON_MAVLINK_VERSION, 0, value))
        with self._lock:
            if self._log_writer is not None:
                # A logging failure (e.g. disk full) must never stop the relay:
                # warn once and stop logging.
                try:
                    now = time.time()
                    self._log_writer.writerow([
                        datetime.fromtimestamp(now).isoformat(timespec="milliseconds"),
                        f"{now:.6f}", copy, *log_fields,
                    ])
                    self._log_file.flush()
                except OSError as e:
                    self.get_logger().error(f"--log write failed ({e}) -- logging stopped, "
                                            "relay continues")
                    self._log_writer = None

    def close_log(self) -> None:
        # Under the lock so a pending repeat timer can't write to a closed file.
        with self._lock:
            if self._log_file is not None:
                try:
                    self._log_file.close()   # can re-raise a failed flush (disk full)
                except OSError:
                    pass
            self._log_file = self._log_writer = None

    def _on_detection(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("Received non-JSON beacon detection message — skipping")
            return

        gps = data.get("gps_position")
        if not gps or gps.get("latitude") is None or gps.get("longitude") is None:
            self.get_logger().warn(
                "Detection has no GPS position yet — skipping MAVLink relay",
                throttle_duration_sec=5.0,
            )
            return

        color = data.get("blink_color", "unknown")
        #blink_info = data.get("blink") or {}
        #is_blinking = blink_info.get("is_blinking")
        is_blinking = data.get("blink")
        tracking_id = data.get("tracking_id", -1)

        msg_id = self._next_msg_id
        self._next_msg_id = (msg_id + 1) % 65536

        value = pack_value_array(gps["latitude"], gps["longitude"], color, is_blinking,
                                 tracking_id, msg_id)

        log_fields = [msg_id, tracking_id, gps["latitude"], gps["longitude"], color, is_blinking]

        # First copy now; the rest on timers so the ROS callback never blocks.
        self._send(value, 1, log_fields)
        for i in range(1, self._repeat):
            if self._repeat_gap > 0:
                timer = threading.Timer(self._repeat_gap * i, self._send,
                                        args=(value, i + 1, log_fields))
                timer.daemon = True
                timer.start()
            else:
                self._send(value, i + 1, log_fields)

        self.get_logger().info(
            f"Relayed beacon via MAVLink: lat={gps['latitude']:.6f} "
            f"lon={gps['longitude']:.6f} color={color} blinking={is_blinking} "
            f"tracking_id={tracking_id} msg_id={msg_id} copies={self._repeat}"
        )


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument(
        "--mavlink-endpoint", default=DEFAULT_MAVLINK_ENDPOINT,
        help=f"pymavlink connection string (default: {DEFAULT_MAVLINK_ENDPOINT})",
    )
    ap.add_argument(
        "--topic", default=DEFAULT_DETECTIONS_TOPIC,
        help=f"Beacon detections topic to subscribe to (default: {DEFAULT_DETECTIONS_TOPIC})",
    )
    ap.add_argument(
        "--repeat", type=int, default=DEFAULT_REPEAT,
        help=f"Send each detection this many times (default: {DEFAULT_REPEAT}; 1 disables repeats)",
    )
    ap.add_argument(
        "--repeat-gap", type=float, default=DEFAULT_REPEAT_GAP,
        help=f"Seconds between repeated copies (default: {DEFAULT_REPEAT_GAP}; 0 = back-to-back)",
    )
    ap.add_argument(
        "--log", metavar="PATH", default=None,
        help="Append a CSV row (send time, copy number, msg_id, beacon fields) for every copy sent",
    )
    ap.add_argument(
        "--task-topic", default=DEFAULT_TASK_TOPIC,
        help=f"RxTask topic (std_msgs/UInt8) relayed as STATUSTEXT (default: {DEFAULT_TASK_TOPIC})",
    )
    ap.add_argument(
        "--onboard-listen", default=DEFAULT_ONBOARD_LISTEN,
        help="Where the drone ELRS bridge copies ground 'SEABIRD_' STATUSTEXT "
             f"(default: {DEFAULT_ONBOARD_LISTEN})",
    )
    ap.add_argument(
        "--run-start-topic", default=DEFAULT_RUN_START_TOPIC,
        help=f"Latched topic the verified RunStart is published on (default: {DEFAULT_RUN_START_TOPIC})",
    )
    ap.add_argument(
        "--alert-topic", default=DEFAULT_ALERT_TOPIC,
        help="Latched std_msgs/String alerts (e.g. why the UAV didn't arm) relayed as WARNING "
             f"STATUSTEXT (default: {DEFAULT_ALERT_TOPIC})",
    )
    ap.add_argument(
        "--wifi-listen", default=DEFAULT_WIFI_LISTEN,
        help="WiFi redundancy: RunStart from the ground router over WiFi, and where everything "
             f"sent is also copied back to it; 'none' disables (default: {DEFAULT_WIFI_LISTEN})",
    )
    args = ap.parse_args()

    rclpy.init()
    node = BeaconMavlinkBridge(args.mavlink_endpoint, args.topic, args.repeat, args.repeat_gap,
                               args.log, args.task_topic, args.onboard_listen, args.run_start_topic,
                               args.alert_topic, args.wifi_listen)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node.close_log()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
