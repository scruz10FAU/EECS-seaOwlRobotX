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
"""

import argparse
import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pymavlink import mavutil

from utils.beacon_mavlink_protocol import (
    pack_value_array, BEACON_MAVLINK_ADDRESS, BEACON_MAVLINK_VERSION,
)

DEFAULT_MAVLINK_ENDPOINT = "udp://:14540"   # matches sweep_lawnmower.py's sim MAVSDK_ADDRESS
DEFAULT_DETECTIONS_TOPIC = "/seabird/beacon_detections"


class BeaconMavlinkBridge(Node):
    def __init__(self, mavlink_endpoint: str, detections_topic: str):
        super().__init__("beacon_mavlink_bridge")
        self._lock = threading.Lock()

        self.get_logger().info(f"Connecting to MAVLink endpoint: {mavlink_endpoint}")
        self._mav = mavutil.mavlink_connection(mavlink_endpoint)

        self.create_subscription(String, detections_topic, self._on_detection, 10)
        self.get_logger().info(f"Subscribed to {detections_topic} — relaying to MAVLink")

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

        color = data.get("color", "unknown")
        blink_info = data.get("blink") or {}
        is_blinking = blink_info.get("is_blinking")

        value = pack_value_array(gps["latitude"], gps["longitude"], color, is_blinking)

        with self._lock:
            self._mav.mav.memory_vect_send(
                BEACON_MAVLINK_ADDRESS, BEACON_MAVLINK_VERSION, 0, value
            )

        self.get_logger().info(
            f"Relayed beacon via MAVLink: lat={gps['latitude']:.6f} "
            f"lon={gps['longitude']:.6f} color={color} blinking={is_blinking}"
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
    args = ap.parse_args()

    rclpy.init()
    node = BeaconMavlinkBridge(args.mavlink_endpoint, args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
