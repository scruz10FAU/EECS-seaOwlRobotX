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
"""

import argparse
import csv
import json
import threading
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pymavlink import mavutil

from utils.beacon_mavlink_protocol import (
    pack_value_array, BEACON_MAVLINK_ADDRESS, BEACON_MAVLINK_VERSION,
)

DEFAULT_MAVLINK_ENDPOINT = "udp://:14540"   # matches sweep_lawnmower.py's sim MAVSDK_ADDRESS
DEFAULT_DETECTIONS_TOPIC = "/seabird/beacon_detections_downward"
DEFAULT_REPEAT = 2
DEFAULT_REPEAT_GAP = 0.5

_LOG_FIELDS = ["iso_time", "epoch_time", "copy", "msg_id", "tracking_id",
               "latitude", "longitude", "color", "is_blinking"]


class BeaconMavlinkBridge(Node):
    def __init__(self, mavlink_endpoint: str, detections_topic: str,
                 repeat: int = DEFAULT_REPEAT, repeat_gap: float = DEFAULT_REPEAT_GAP,
                 log_path: str = None):
        super().__init__("beacon_mavlink_bridge")
        self._lock = threading.Lock()
        self._repeat = max(1, repeat)
        self._repeat_gap = max(0.0, repeat_gap)
        self._next_msg_id = 0

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

    def _send(self, value: list, copy: int, log_fields: list) -> None:
        with self._lock:
            self._mav.mav.memory_vect_send(
                BEACON_MAVLINK_ADDRESS, BEACON_MAVLINK_VERSION, 0, value
            )
            if self._log_writer is not None:
                now = time.time()
                self._log_writer.writerow([
                    datetime.fromtimestamp(now).isoformat(timespec="milliseconds"),
                    f"{now:.6f}", copy, *log_fields,
                ])
                self._log_file.flush()

    def close_log(self) -> None:
        # Under the lock so a pending repeat timer can't write to a closed file.
        with self._lock:
            if self._log_file is not None:
                self._log_file.close()
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
    args = ap.parse_args()

    rclpy.init()
    node = BeaconMavlinkBridge(args.mavlink_endpoint, args.topic, args.repeat, args.repeat_gap,
                               args.log)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_log()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
