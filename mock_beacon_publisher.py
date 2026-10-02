#!/usr/bin/env python3
"""
mock_beacon_publisher.py — publishes synthetic /seabird/beacon_detections
messages for testing downstream consumers (beacon_mavlink_bridge.py,
sweep_rrt.py, sweep_lawnmower.py, etc.) without running the real detection
pipeline — no camera, no YOLO models, no ROS live mode needed upstream.

Publishes the same JSON schema beacon_detector_config.py's live ROS mode
publishes on /seabird/beacon_detections (see README.md's "Published JSON
fields" section).

Usage:
    python3 mock_beacon_publisher.py                         # cycles a built-in sample set at 1 Hz
    python3 mock_beacon_publisher.py --rate 2.0 --count 10    # 2 Hz, stop after 10 messages
    python3 mock_beacon_publisher.py --color red --blinking true --lat 26.3712 --lon -80.1034
                                                               # publish one fixed sample repeatedly
"""

import argparse
import itertools
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DEFAULT_TOPIC = "/seabird/beacon_detections"

# Built-in variety for unattended cycling: different colors, blink states,
# and GPS fixes so downstream consumers (e.g. beacon_mavlink_bridge.py,
# sweep_rrt.py's per-color tracking) see realistic variation without any
# flags.
_SAMPLE_FIXTURES = [
    {"color": "red",     "is_blinking": False, "lat": 26.371200, "lon": -80.103400},
    {"color": "green",   "is_blinking": True,  "lat": 26.371350, "lon": -80.103250, "blink_hz": 1.02},
    {"color": "blue",    "is_blinking": True,  "lat": 26.371500, "lon": -80.103100, "blink_hz": 0.48},
    {"color": "red",     "is_blinking": True,  "lat": 26.371650, "lon": -80.102950, "blink_hz": 1.75},
    {"color": "unknown", "is_blinking": None,  "lat": 26.371800, "lon": -80.102800},
]


def _build_message(color, is_blinking, lat, lon, tracking_id,
                    blink_hz=None, altitude=12.0, confidence=0.9) -> dict:
    """Build one detection dict matching the schema beacon_detector_config.py publishes."""
    blink_color = color if color not in (None, "unknown") else "unknown"
    phase = "unknown"
    if is_blinking is True:
        phase = "on"
    elif is_blinking is False:
        phase = "off"

    hue_votes = {"red": 0.0, "green": 0.0, "blue": 0.0, "other": 0.0}
    if color in hue_votes:
        hue_votes[color] = 0.9

    return {
        "gps_position": {"latitude": lat, "longitude": lon, "altitude": altitude},

        "blink": {
            "is_blinking": is_blinking,
            "blink_color": blink_color,
            "blink_hz": blink_hz,
            "phase": phase,
        },
        "color": color
    }


def _cycle_fixtures():
    """Infinite generator over _SAMPLE_FIXTURES, with an incrementing tracking_id."""
    tracking_id = 0
    for fixture in itertools.cycle(_SAMPLE_FIXTURES):
        yield _build_message(
            color=fixture["color"], is_blinking=fixture["is_blinking"],
            lat=fixture["lat"], lon=fixture["lon"], tracking_id=tracking_id,
            blink_hz=fixture.get("blink_hz"),
        )
        tracking_id += 1


class MockBeaconPublisher(Node):
    def __init__(self, topic: str):
        super().__init__("mock_beacon_publisher")
        self.publisher = self.create_publisher(String, topic, 10)
        self.get_logger().info(f"Publishing synthetic beacon detections on {topic}")


def _run(node: MockBeaconPublisher, message_source, rate_hz: float, count: int) -> None:
    period = 1.0 / rate_hz if rate_hz > 0 else 0.0
    sent = 0
    for msg_dict in message_source:
        if count and sent >= count:
            break
        msg_dict = {**msg_dict, "timestamp": time.time()}  # refresh even for a repeated fixed sample

        out = String()
        out.data = json.dumps(msg_dict)
        node.publisher.publish(out)

        node.get_logger().info(
            f"Published: color={msg_dict['color']} "
            f"blinking={msg_dict['blink']['is_blinking']} "
            f"gps=({msg_dict['gps_position']['latitude']:.6f}, "
            f"{msg_dict['gps_position']['longitude']:.6f}) "
            f"tracking_id={msg_dict['tracking_id']}"
        )

        sent += 1
        if count and sent >= count:
            break
        time.sleep(period)


def _parse_blinking(value: str):
    value = value.lower()
    if value in ("true", "1", "yes"):
        return True
    if value in ("false", "0", "no"):
        return False
    if value in ("none", "unknown", "null"):
        return None
    raise argparse.ArgumentTypeError(f"invalid blinking value: {value!r} (use true/false/none)")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--topic", default=DEFAULT_TOPIC,
                    help=f"Topic to publish on (default: {DEFAULT_TOPIC})")
    ap.add_argument("--rate", type=float, default=1.0,
                    help="Publish rate in Hz (default: 1.0; <= 0 publishes as fast as possible)")
    ap.add_argument("--count", type=int, default=0,
                    help="Stop after this many messages (default: 0 = unlimited)")
    ap.add_argument("--color", choices=["red", "green", "blue", "white", "unknown"],
                    default=None,
                    help="Fix a single color instead of cycling the built-in sample set")
    ap.add_argument("--blinking", type=_parse_blinking, default=None,
                    help="Fix blink status (true/false/none) for the single-sample mode")
    ap.add_argument("--blink-hz", type=float, default=None,
                    help="Fix blink_hz for the single-sample mode")
    ap.add_argument("--lat", type=float, default=26.371200,
                    help="Latitude for the single-sample mode (default: 26.371200)")
    ap.add_argument("--lon", type=float, default=-80.103400,
                    help="Longitude for the single-sample mode (default: -80.103400)")
    args = ap.parse_args()

    rclpy.init()
    node = MockBeaconPublisher(args.topic)
    try:
        if args.color is not None:
            fixed = _build_message(
                color=args.color, is_blinking=args.blinking, lat=args.lat, lon=args.lon,
                tracking_id=0, blink_hz=args.blink_hz,
            )
            source = itertools.repeat(fixed)
        else:
            source = _cycle_fixtures()
        _run(node, source, args.rate, args.count)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
