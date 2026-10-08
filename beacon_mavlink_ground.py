#!/usr/bin/env python3
"""
beacon_mavlink_ground.py — ground-station decoder for beacon detections
relayed over MAVLink by beacon_mavlink_bridge.py (drone side).

Listens on a MAVLink connection and decodes MEMORY_VECT messages whose
`address` matches BEACON_MAVLINK_ADDRESS (see
utils/beacon_mavlink_protocol.py for the wire format), printing GPS
position, color, and blink status for each one. Messages with a different
`address` are ignored — they're not ours.

This is meant to run on the GROUND STATION machine, separate from the
drone. Copy this file AND the utils/ folder (for
utils/beacon_mavlink_protocol.py) over together, so the import path below
still resolves.

Requires: pip install pymavlink
With --publish-ros, also requires a ROS2 environment (rclpy) on this
machine -- otherwise no ROS dependency at all, matching the rest of this
script's design (nothing here needs ROS by default).

Usage:
    python3 beacon_mavlink_ground.py --mavlink-endpoint udpin:0.0.0.0:14550
    python3 beacon_mavlink_ground.py --mavlink-endpoint /dev/ttyUSB0 --baud 57600
    python3 beacon_mavlink_ground.py --mavlink-endpoint udpin:0.0.0.0:14550 --publish-ros
"""

import argparse
import json
import time

from pymavlink import mavutil

from utils.beacon_mavlink_protocol import (
    unpack_value_array, BEACON_MAVLINK_ADDRESS, BEACON_MAVLINK_VERSION,
)

DEFAULT_ROS_TOPIC = "/seabird/beacon_info"


def _build_detection_json(beacon) -> dict:
    """
    Minimal payload -- only the fields actually carried over the MAVLink
    wire format (see utils/beacon_mavlink_protocol.py), nothing invented or
    padded with nulls. GPS fields first.
    """
    return {
        "latitude":    beacon.latitude,
        "longitude":   beacon.longitude,
        "color":       beacon.color,
        "is_blinking": beacon.is_blinking,
    }


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument(
        "--mavlink-endpoint", required=True, default="udpin:127.0.0.1:14551"
        help="pymavlink connection string, e.g. udpin:127.0.0.1:14550 or /dev/ttyUSB0",
    )
    ap.add_argument(
        "--baud", type=int, default=57600,
        help="Baud rate, only used for serial endpoints (default: 57600)",
    )
    ap.add_argument(
        "--publish-ros", action="store_true",
        help="Also publish each decoded beacon as a std_msgs/String JSON ROS2 topic "
             "(requires rclpy on this machine; off by default)",
    )
    ap.add_argument(
        "--ros-topic", default=DEFAULT_ROS_TOPIC,
        help=f"ROS2 topic to publish on when --publish-ros is set (default: {DEFAULT_ROS_TOPIC})",
    )
    args = ap.parse_args()

    ros_publisher = None
    ros_node = None
    if args.publish_ros:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String

        rclpy.init()
        ros_node = Node("beacon_mavlink_ground")
        ros_publisher = ros_node.create_publisher(String, args.ros_topic, 10)
        print(f"[beacon-ground] Publishing decoded beacons to ROS2 topic {args.ros_topic}")

    print(f"[beacon-ground] Connecting to {args.mavlink_endpoint} ...")
    mav = mavutil.mavlink_connection(args.mavlink_endpoint, baud=args.baud)
    mav.wait_heartbeat()
    print(f"[beacon-ground] Heartbeat received from system {mav.target_system} "
          f"— listening for beacon messages")

    try:
        while True:
            msg = mav.recv_match(type="MEMORY_VECT", blocking=True, timeout=5.0)
            if msg is None:
                continue
            if msg.address != BEACON_MAVLINK_ADDRESS:
                continue  # not ours — some other real memory-vector traffic on this link
            if msg.ver != BEACON_MAVLINK_VERSION:
                print(f"[beacon-ground] WARNING: received beacon message with unknown "
                      f"protocol version {msg.ver} (expected {BEACON_MAVLINK_VERSION}) — skipping")
                continue

            beacon = unpack_value_array(msg.value)
            ts = time.strftime("%H:%M:%S")
            blink_str = "unknown" if beacon.is_blinking is None else str(beacon.is_blinking)
            print(f"[{ts}] beacon: lat={beacon.latitude:.6f} lon={beacon.longitude:.6f} "
                  f"color={beacon.color} blinking={blink_str}")

            if ros_publisher is not None:
                out = String()
                out.data = json.dumps(_build_detection_json(beacon))
                ros_publisher.publish(out)
    except KeyboardInterrupt:
        pass
    finally:
        if ros_node is not None:
            ros_node.destroy_node()
            import rclpy
            rclpy.shutdown()


if __name__ == "__main__":
    main()
