#!/usr/bin/env python3
"""
mock_robocommand_course.py — publishes a RoboCommand RxCourse for a course at
YOUR test site, over the same protocol RoboCommand uses, so the real chain can
be tested end to end without the competition server:

  this (RxCourse, MQTT) -> seaowls_ocs -> /seaowls/uav1/mission/geofence
    -> beacon_mavlink_ground.py --publish-ros -> PX4 fence + GF_MAX_VER_DIST
    -> sweep_rrt_overfly_downward_center.py (SEARCH_AREA_SHAPE = "fence")

Same message and topic as RoboNation's official rc_test/test_server.py: a
robotx.RxCourse protobuf (course_id, pinger_freq_hz, corners, sent_at,
uav_transit_area -- each polygon a closed ring, first point == last),
published RETAINED on robocommand/robotx/course. That server's course is
hard-coded to Course Alpha in Singapore; this one is wherever you put it.
Don't run both -- the last retained course published wins.

The protobuf classes come from the seaowls-robotx-robocommand repo
(vendor/robocommand/RobotX_2026/gen/python + third_party/), the same ones
seaowls_ocs imports. Set ROBOCOMMAND_ROOT to that repo if it isn't found.

Requires: paho-mqtt (not needed for --dry-run).

Usage (in WSL, with an MQTT broker on localhost:1883):
    python3 tests/mock_robocommand_course.py --center 26.37140,-80.10320
    python3 tests/mock_robocommand_course.py --center 26.37140,-80.10320 --size 30,20 --rotation-deg 15
    python3 tests/mock_robocommand_course.py --corner 26.3712,-80.1034 --corner 26.3712,-80.1031 \\
        --corner 26.3715,-80.1031 --corner 26.3715,-80.1034
    python3 tests/mock_robocommand_course.py --center 26.37140,-80.10320 \\
        --transit-corner 26.3709,-80.1034 --transit-corner 26.3709,-80.1031 \\
        --transit-corner 26.3712,-80.1031 --transit-corner 26.3712,-80.1034
    python3 tests/mock_robocommand_course.py --center 26.37140,-80.10320 --period 30   # republish
    python3 tests/mock_robocommand_course.py --center 26.37140,-80.10320 --dry-run
    python3 tests/mock_robocommand_course.py --clear     # remove the retained course from the broker

Put --center where the drone actually is: PX4 won't arm outside its fence.
"""

import argparse
import math
import os
import sys
import time
from pathlib import Path

TOPIC = "robocommand/robotx/course"
EARTH_RADIUS_M = 6378137.0   # same flat-earth radius as the sweep script's _gps_to_ne
SCHEMA_DIR = "vendor/robocommand/RobotX_2026/gen/python"


def _pair(text: str):
    try:
        a, b = (float(v) for v in text.split(","))
    except ValueError:
        raise argparse.ArgumentTypeError(f"expected two comma-separated numbers, got {text!r}")
    return a, b


def _import_schemas():
    """Put RoboCommand's generated protobuf code on sys.path the way
    seaowls_ocs does, and return (common_pb2, rx_course_pb2, Timestamp)."""
    here = Path(__file__).resolve()
    candidates = [
        os.getenv("ROBOCOMMAND_ROOT", ""),
        here.parents[2] / "seaowls-robotx-robocommand",   # sibling of this repo
        Path.home() / "seaowls-robotx-robocommand",
        "/mnt/c/Users/Precision 3571/seaowls-robotx-robocommand",
    ]
    root = next((Path(c) for c in candidates if c and (Path(c) / SCHEMA_DIR).is_dir()), None)
    if root is None:
        sys.exit("RoboCommand schemas not found; set ROBOCOMMAND_ROOT to the "
                 "seaowls-robotx-robocommand repo")
    sys.path[:0] = [str(root / SCHEMA_DIR), str(root / "third_party")]
    import google
    google.__path__ = [str(root / "third_party/google")] + list(google.__path__)
    import common_pb2
    from google.protobuf.timestamp_pb2 import Timestamp
    from robotx import rx_course_pb2
    return common_pb2, rx_course_pb2, Timestamp


def rectangle(center, width_m, length_m, rotation_deg):
    """Closed ring of (lat, lon): a width_m (east-west) x length_m (north-south)
    box centred on `center`, rotated clockwise by rotation_deg."""
    lat0, lon0 = center
    rot = math.radians(rotation_deg)
    ring = []
    for n, e in ((-length_m / 2, -width_m / 2), (-length_m / 2, width_m / 2),
                 (length_m / 2, width_m / 2), (length_m / 2, -width_m / 2)):
        rn = n * math.cos(rot) - e * math.sin(rot)
        re = n * math.sin(rot) + e * math.cos(rot)
        ring.append((lat0 + math.degrees(rn / EARTH_RADIUS_M),
                     lon0 + math.degrees(re / (EARTH_RADIUS_M * math.cos(math.radians(lat0))))))
    return ring + ring[:1]


def closed(points):
    """RoboCommand rings repeat the first point at the end."""
    points = list(points)
    return points if not points or points[0] == points[-1] else points + points[:1]


def build_course(args, common_pb2, rx_course_pb2, Timestamp):
    if args.corner:
        corners = closed(args.corner)
    else:
        corners = rectangle(args.center, *args.size, args.rotation_deg)
    transit = closed(args.transit_corner or [])
    sent_at = Timestamp()
    sent_at.GetCurrentTime()
    return rx_course_pb2.RxCourse(
        course_id=args.course_id,
        pinger_freq_hz=args.pinger_freq_hz,
        corners=[common_pb2.LatLng(latitude=lat, longitude=lon) for lat, lon in corners],
        sent_at=sent_at,
        uav_transit_area=[common_pb2.LatLng(latitude=lat, longitude=lon) for lat, lon in transit],
    )


def describe(course):
    lines = [f"RxCourse course_id={course.course_id!r} pinger_freq_hz={course.pinger_freq_hz}"]
    for name in ("corners", "uav_transit_area"):
        ring = getattr(course, name)
        lines.append(f"  {name}: {len(ring)} points" + ("" if ring else " (empty)"))
        lines += [f"    {p.latitude:.8f}, {p.longitude:.8f}" for p in ring]
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default=os.getenv("ROBOCOMMAND_HOST", "localhost"),
                    help="MQTT broker (default: $ROBOCOMMAND_HOST or localhost, as seaowls_ocs)")
    ap.add_argument("--port", type=int, default=int(os.getenv("ROBOCOMMAND_PORT", "1883")),
                    help="MQTT port (default: $ROBOCOMMAND_PORT or 1883)")
    ap.add_argument("--clear", action="store_true",
                    help="Remove the retained course from the broker and exit. Stop "
                         "seaowls_ocs first: subscribers still get the empty message, "
                         "which it would parse as an empty course")
    ap.add_argument("--dry-run", action="store_true",
                    help="Print the RxCourse instead of publishing (no broker needed)")
    ap.add_argument("--period", type=float, default=0.0,
                    help="Republish every this many seconds with a fresh sent_at "
                         "(default: 0 = publish once and exit)")

    g = ap.add_argument_group("course")
    g.add_argument("--course-id", default="MOCK", help="RxCourse.course_id (default: MOCK)")
    g.add_argument("--pinger-freq-hz", type=int, default=25000,
                   help="RxCourse.pinger_freq_hz (default: 25000, as the official test server)")
    g.add_argument("--center", type=_pair, metavar="LAT,LON",
                   help="Centre of a rectangular course boundary")
    g.add_argument("--size", type=_pair, default=(25.0, 25.0), metavar="W,L",
                   help="Rectangle width (east-west) and length (north-south) in m (default: 25,25)")
    g.add_argument("--rotation-deg", type=float, default=0.0,
                   help="Rotate the rectangle clockwise by this many degrees (default: 0)")
    g.add_argument("--corner", type=_pair, action="append", metavar="LAT,LON",
                   help="Course boundary vertex, in order (repeatable, >= 3; overrides --center)")
    g.add_argument("--transit-corner", type=_pair, action="append", metavar="LAT,LON",
                   help="UAV transit area vertex, in order (repeatable, >= 3; default: none)")
    args = ap.parse_args()

    if not args.clear:
        if args.corner is None and args.center is None:
            ap.error("give --center LAT,LON or at least 3 --corner LAT,LON")
        if args.corner is not None and len(set(args.corner)) < 3:
            ap.error("--corner needs at least 3 distinct points")
        if args.transit_corner is not None and len(set(args.transit_corner)) < 3:
            ap.error("--transit-corner needs at least 3 distinct points")
        if min(args.size) <= 0:
            ap.error("--size must be positive")

    schemas = _import_schemas()
    if args.dry_run:
        print(describe(build_course(args, *schemas)))
        return

    import paho.mqtt.client as mqtt
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.connect(args.host, args.port)
    client.loop_start()
    try:
        if args.clear:
            # An empty retained payload deletes the retained message.
            client.publish(TOPIC, b"", retain=True).wait_for_publish()
            print(f"Cleared the retained course on {TOPIC} ({args.host}:{args.port})")
            return
        while True:
            course = build_course(args, *schemas)
            client.publish(TOPIC, course.SerializeToString(), retain=True).wait_for_publish()
            print(f"Published RxCourse (retained) -> {TOPIC} on {args.host}:{args.port}")
            print(describe(course), flush=True)
            if args.period <= 0:
                break
            time.sleep(args.period)
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
