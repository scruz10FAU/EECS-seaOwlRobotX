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
    python3 tests/mock_robocommand_course.py --center 26.37140,-80.10320 --serve
                                                          # also act as RoboCommand: check the
                                                          # RunDeclaration, RunStart when all AUTO
    python3 tests/mock_robocommand_course.py --center 26.37140,-80.10320 --serve --no-wait-auto
                                                          # props-off bench: RunStart 3 s after an
                                                          # accepted declaration
    python3 tests/mock_robocommand_course.py --center 26.37140,-80.10320 --invalid self-intersecting
                                                          # INVALID demo: open, two-points,
                                                          # self-intersecting, bad-coords,
                                                          # too-small, too-large

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


INVALID_KINDS = ("open", "two-points", "self-intersecting", "bad-coords", "too-small", "too-large")


def make_invalid(kind, ring):
    """Break a valid closed ring the way --invalid KIND says, to rehearse the
    VALID / INVALID demonstration. The ground station REFUSES open, two-points,
    bad-coords and too-small (the 1 m inset collapses it); it ACCEPTS
    self-intersecting (reordered into a non-crossing polygon) and too-large
    (with a warning)."""
    pts = ring[:-1]
    if kind == "open":
        return pts                                       # last point != first
    if kind == "two-points":
        return [pts[0], pts[1], pts[0]]
    if kind == "self-intersecting":
        return [pts[0], pts[2], pts[1], pts[3], pts[0]]  # bow-tie
    if kind == "bad-coords":
        return [(91.0, pts[0][1])] + pts[1:] + [(91.0, pts[0][1])]
    lat0 = ring[0][0]
    scale = {"too-small": 0.05, "too-large": 100.0}[kind]   # 25 m -> ~1.2 m / ~2.5 km
    clat = sum(p[0] for p in pts) / len(pts)
    clon = sum(p[1] for p in pts) / len(pts)
    out = [(clat + (a - clat) * scale, clon + (b - clon) * scale) for a, b in pts]
    return out + out[:1]


def build_course(args, common_pb2, rx_course_pb2, Timestamp):
    if args.corner:
        corners = closed(args.corner)
    else:
        corners = rectangle(args.center, *args.size, args.rotation_deg)
    if args.invalid:
        corners = make_invalid(args.invalid, corners)
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
    g = ap.add_argument_group("serve: play RoboCommand's start sequence (Handbook 3.4.10)")
    g.add_argument("--serve", action="store_true",
                   help="After publishing the course, stay running as RoboCommand: check each "
                        "RunDeclaration (vehicles, tiers, UAV geofence vs this course, altitude) "
                        "and send RunStart once every declared vehicle reports STATE_AUTO")
    g.add_argument("--no-wait-auto", action="store_true",
                   help="With --serve: send RunStart --run-start-delay s after an accepted "
                        "declaration, without waiting for STATE_AUTO (props-off bench tests)")
    g.add_argument("--run-start-delay", type=float, default=3.0,
                   help="With --no-wait-auto: seconds before RunStart (default: 3)")
    g.add_argument("--max-alt-hae", type=float, default=50.0,
                   help="With --serve: competition maximum UAV altitude, m HAE (default: 50, "
                        "as the official test server)")
    g.add_argument("--team-id", default=os.getenv("TEAM_ID", "FLAU"),
                   help="With --serve: team whose requests/reports to watch (default: $TEAM_ID or FLAU)")

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
    g.add_argument("--invalid", choices=INVALID_KINDS,
                   help="Publish a deliberately broken course boundary (rehearses the "
                        "VALID/INVALID demonstration): open, two-points, bad-coords and "
                        "too-small are REFUSED; self-intersecting is reordered and accepted; "
                        "too-large is accepted with a warning. "
                        "An invalid altitude limit is set on the OCS side "
                        "(UAV_GEOFENCE_MAX_ALTITUDE_HAE_M), not in RxCourse.")
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
            if args.serve:
                serve(client, args, course, schemas)   # until stopped
                break
            if args.period <= 0:
                break
            time.sleep(args.period)
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        client.disconnect()


def serve(client, args, course, schemas):
    """Play RoboCommand's start sequence for this course (Handbook 3.4.10):
    check each RunDeclaration, then send RunStart when every declared vehicle
    reports STATE_AUTO (or after --run-start-delay with --no-wait-auto)."""
    import threading
    common_pb2, _, Timestamp = schemas
    from robotx import rx_commands_pb2, rx_reports_pb2, rx_requests_pb2
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # repo root, for utils/
    from utils.geofence_check import fence_problems, validate_altitude

    root = f"robocommand/robotx/{args.team_id}"
    corners = [(p.latitude, p.longitude) for p in course.corners]
    transit = [(p.latitude, p.longitude) for p in course.uav_transit_area] or None
    lock = threading.Lock()
    state = {"pending": None, "run_id": 0, "command_seq": 0, "start_at": None}

    def say(text):
        print(f"[mock RoboCommand] {text}", flush=True)

    def declaration_problems(d):
        problems = []
        if not d.vehicle_ids:
            problems.append("no vehicle_ids")
        for t in (1, 2, 3, 4):
            if getattr(d, f"task{t}_tier") == common_pb2.TIER_UNKNOWN:
                problems.append(f"task{t}_tier is TIER_UNKNOWN (use TIER_NONE)")
        if any(v.startswith("UAV") for v in d.vehicle_ids):
            fence = [(p.latitude, p.longitude) for p in d.uav_geofence]
            problems += fence_problems(fence, corners, transit) if fence else ["no uav_geofence"]
            problems += validate_altitude(d.uav_geofence_max_altitude_hae_m, args.max_alt_hae)
            if not d.uav_pilot_callsign:
                problems.append("no uav_pilot_callsign")
        return problems

    def send_run_start():
        p = state["pending"]
        state["run_id"] += 1
        state["command_seq"] += 1
        ts = Timestamp(); ts.GetCurrentTime()
        cmd = rx_commands_pb2.RxCommand(
            team_id=args.team_id, seq=state["command_seq"], sent_at=ts,
            run_start=rx_commands_pb2.RunStart(declaration_seq=p["seq"], run_id=state["run_id"]))
        client.publish(f"{root}/command", cmd.SerializeToString())
        p["started"] = True
        say(f"RunStart sent: declaration_seq={p['seq']} run_id={state['run_id']}")

    def on_message(c, userdata, msg):
        try:
            if msg.topic == f"{root}/request":
                r = rx_requests_pb2.RxRequest(); r.ParseFromString(msg.payload)
                if r.WhichOneof("body") != "run_declaration":
                    return
                d = r.run_declaration
                tiers = ", ".join(common_pb2.TaskTier.Name(getattr(d, f"task{t}_tier")) for t in (1, 2, 3, 4))
                problems = declaration_problems(d)
                with lock:
                    if problems:
                        state["pending"] = None
                        say(f"RunDeclaration seq={r.seq} REFUSED (RoboCommand sends nothing back): "
                            + "; ".join(problems))
                        return
                    state["pending"] = {"seq": r.seq, "vehicles": set(d.vehicle_ids), "auto": set(),
                                        "started": False}
                    say(f"RunDeclaration seq={r.seq} ACCEPTED: vehicles={sorted(d.vehicle_ids)} "
                        f"tiers=[{tiers}]")
                    if args.no_wait_auto:
                        state["start_at"] = time.time() + args.run_start_delay
                        say(f"--no-wait-auto: RunStart in {args.run_start_delay:g} s")
                    else:
                        say("waiting for every declared vehicle to report STATE_AUTO")
            elif msg.topic.startswith(f"{root}/") and msg.topic.endswith("/report"):
                r = rx_reports_pb2.RxReport(); r.ParseFromString(msg.payload)
                if r.WhichOneof("body") != "heartbeat":
                    return
                with lock:
                    p = state["pending"]
                    if p is None or p["started"] or args.no_wait_auto or r.vehicle_id not in p["vehicles"]:
                        return
                    auto = r.heartbeat.state == common_pb2.STATE_AUTO
                    if auto and r.vehicle_id not in p["auto"]:
                        say(f"{r.vehicle_id} reports STATE_AUTO")
                    (p["auto"].add if auto else p["auto"].discard)(r.vehicle_id)
                    if p["auto"] == p["vehicles"]:
                        send_run_start()
        except Exception as e:   # a malformed payload must not stop the mock
            say(f"ignored a message on {msg.topic}: {e}")

    client.on_message = on_message
    client.subscribe(f"{root}/request")
    client.subscribe(f"{root}/+/report")
    say(f"serving team {args.team_id}: waiting for a RunDeclaration "
        + ("(RunStart without waiting for AUTO)" if args.no_wait_auto else "(RunStart when all AUTO)"))
    while True:
        time.sleep(0.2)
        with lock:
            p = state["pending"]
            if state["start_at"] and p and not p["started"] and time.time() >= state["start_at"]:
                state["start_at"] = None
                send_run_start()


if __name__ == "__main__":
    main()
