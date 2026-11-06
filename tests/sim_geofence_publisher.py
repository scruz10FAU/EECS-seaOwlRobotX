#!/usr/bin/env python3
"""
sim_geofence_publisher.py — puts a test geofence on PX4 SITL (Gazebo) so
sweep_rrt_overfly_downward_center.py's SEARCH_AREA_SHAPE = "fence" can be
tested in sim, without the RoboCommand broker, seaowls_ocs or ELRS.

The sweep reads its fence back from PX4, so this uploads straight to PX4 with
the same uploader beacon_mavlink_ground.py uses on the real drone
(utils/px4_fence.py: inclusion polygon via the MAVLink mission protocol, then
GF_MAX_VER_DIST), confirms it, and exits.

Run it BEFORE the sweep: it binds the same SITL port (14540, PX4's offboard
link) and frees it on exit. In the sweep, for sim:
    MAVSDK_ADDRESS    = "udpin://0.0.0.0:14540"
    SEARCH_AREA_SHAPE = "fence"

By default the fence is a 25 x 25 m square centred on the simulated drone
(Gazebo's home position varies by world). The rectangle comes from
mock_robocommand_course.py, so shapes match the full-chain test.

Requires: pip install pymavlink

Usage (PX4 SITL + Gazebo running):
    python3 tests/sim_geofence_publisher.py                           # 25x25 m around the drone, 10 m ceiling
    python3 tests/sim_geofence_publisher.py --size 40,20 --rotation-deg 30
    python3 tests/sim_geofence_publisher.py --offset 30,0             # drone outside: sweep must ABORT
    python3 tests/sim_geofence_publisher.py --gf-action 1             # also enforce (1 = warning)
    python3 tests/sim_geofence_publisher.py --center 47.397742,8.545594 --ceiling-m 15
    python3 tests/sim_geofence_publisher.py --alt-hae 40              # real HAE conversion (needs alt_ellipsoid)
    python3 tests/sim_geofence_publisher.py --mavlink-endpoint udpin:0.0.0.0:14550   # QGC's port, QGC closed
    python3 tests/sim_geofence_publisher.py --dry-run
    python3 tests/sim_geofence_publisher.py --clear
"""

import argparse
import math
import os
import struct
import sys
import time
from pathlib import Path

# MAVLink 2 before pymavlink picks its dialect: the fence's mission_type is a
# MAVLink 2 field (utils/px4_fence.py refuses a MAVLink 1 connection).
os.environ.setdefault("MAVLINK20", "1")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # repo root, for utils/

from mock_robocommand_course import EARTH_RADIUS_M, _pair, closed, rectangle

DEFAULT_ENDPOINT = "udpin:0.0.0.0:14540"   # PX4 SITL offboard link (sweep's sim MAVSDK_ADDRESS)
POSITION_TIMEOUT_S = 15.0
UPLOAD_TIMEOUT_S = 30.0


def shifted(center, north_m, east_m):
    """center (lat, lon) moved north_m / east_m (flat earth)."""
    lat, lon = center
    return (lat + math.degrees(north_m / EARTH_RADIUS_M),
            lon + math.degrees(east_m / (EARTH_RADIUS_M * math.cos(math.radians(lat)))))


def wait_for(mav, types, timeout_s, accept=lambda m: True):
    """Next message of `types` passing `accept`, or None after timeout_s."""
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        msg = mav.recv_match(type=types, blocking=True, timeout=0.5)
        if msg is not None and accept(msg):
            return msg
    return None


def set_int_param(mav, target, name, value):
    """PX4 int param via PARAM_SET (bytewise encoding, as QGC). True if echoed."""
    from pymavlink import mavutil
    as_float = struct.unpack("<f", struct.pack("<i", value))[0]
    for _ in range(5):
        mav.mav.param_set_send(target, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, name.encode(),
                               as_float, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        echo = wait_for(mav, ["PARAM_VALUE"], 2.0, lambda m: m.param_id == name)
        if echo is not None and struct.unpack("<i", struct.pack("<f", echo.param_value))[0] == value:
            return True
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mavlink-endpoint", default=DEFAULT_ENDPOINT,
                    help=f"pymavlink connection string (default: {DEFAULT_ENDPOINT})")
    ap.add_argument("--dry-run", action="store_true",
                    help="Print the polygon instead of uploading (needs --center or --corner)")
    ap.add_argument("--clear", action="store_true", help="Remove PX4's fence and exit")

    g = ap.add_argument_group("fence")
    g.add_argument("--center", type=_pair, metavar="LAT,LON",
                   help="Rectangle centre (default: the drone's current position)")
    g.add_argument("--offset", type=_pair, default=(0.0, 0.0), metavar="N,E",
                   help="Shift the rectangle centre by this many metres north/east (default: 0,0)")
    g.add_argument("--size", type=_pair, default=(25.0, 25.0), metavar="W,L",
                   help="Rectangle width (east-west) and length (north-south) in m (default: 25,25)")
    g.add_argument("--rotation-deg", type=float, default=0.0,
                   help="Rotate the rectangle clockwise by this many degrees (default: 0)")
    g.add_argument("--corner", type=_pair, action="append", metavar="LAT,LON",
                   help="Explicit polygon vertex, in order (repeatable, >= 3; overrides the rectangle)")

    g = ap.add_argument_group("ceiling and enforcement")
    c = g.add_mutually_exclusive_group()
    c.add_argument("--ceiling-m", type=float, default=10.0,
                   help="GF_MAX_VER_DIST, metres above home (default: 10)")
    c.add_argument("--alt-hae", type=float,
                   help="Ceiling in metres HAE, converted like the real chain (needs HOME_POSITION "
                        "and GPS_RAW_INT alt_ellipsoid from SITL)")
    g.add_argument("--gf-action", type=int, choices=range(6), metavar="0-5",
                   help="Also set GF_ACTION (0 none, 1 warning, 2 hold, 3 return, 4 terminate, 5 land)")
    args = ap.parse_args()

    if args.corner is not None and len(set(args.corner)) < 3:
        ap.error("--corner needs at least 3 distinct points")
    if min(args.size) <= 0:
        ap.error("--size must be positive")

    def polygon(center):
        if args.corner:
            return closed(args.corner)
        return rectangle(shifted(center, *args.offset), *args.size, args.rotation_deg)

    if args.dry_run:
        if args.center is None and not args.corner:
            ap.error("--dry-run needs --center or --corner (it doesn't connect to PX4)")
        for lat, lon in polygon(args.center):
            print(f"{lat:.8f}, {lon:.8f}")
        return

    from pymavlink import mavutil
    from utils.px4_fence import FenceUploader, open_polygon

    print(f"[sim-fence] Connecting to {args.mavlink_endpoint} ...")
    mav = mavutil.mavlink_connection(args.mavlink_endpoint)
    hb = wait_for(mav, ["HEARTBEAT"], 20.0,
                  lambda m: m.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
                  and m.type != mavutil.mavlink.MAV_TYPE_GCS)
    if hb is None:
        sys.exit("[sim-fence] No PX4 heartbeat -- is SITL running, and is the sweep "
                 "(or QGC) holding this port?")
    target = hb.get_srcSystem()
    print(f"[sim-fence] PX4 system {target}")

    if args.clear:
        mav.mav.mission_clear_all_send(target, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                                       mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        ack = wait_for(mav, ["MISSION_ACK"], 5.0,
                       lambda m: m.mission_type == mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        ok = ack is not None and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED
        print("[sim-fence] ✓ Fence cleared" if ok else "[sim-fence] FAILED to clear the fence")
        sys.exit(0 if ok else 1)

    center = args.center
    if center is None and not args.corner:
        pos = wait_for(mav, ["GLOBAL_POSITION_INT"], POSITION_TIMEOUT_S,
                       lambda m: m.get_srcSystem() == target and (m.lat or m.lon))
        if pos is None:
            sys.exit("[sim-fence] No GLOBAL_POSITION_INT from PX4 -- pass --center LAT,LON")
        center = (pos.lat * 1e-7, pos.lon * 1e-7)
        print(f"[sim-fence] Drone at {center[0]:.7f}, {center[1]:.7f}")

    ring = polygon(center)
    print("[sim-fence] Fence polygon:")
    for lat, lon in ring:
        print(f"    {lat:.8f}, {lon:.8f}")

    fence = FenceUploader(mav, log=lambda s: print(f"[sim-fence] {s}"))
    ceiling = ({"max_alt_hae_m": args.alt_hae} if args.alt_hae is not None
               else {"ceiling_above_home_m": args.ceiling_m})
    fence.start(target, [open_polygon(*zip(*ring))], **ceiling)
    end = time.monotonic() + UPLOAD_TIMEOUT_S
    while fence.state not in ("done", "failed") and time.monotonic() < end:
        msg = mav.recv_match(blocking=True, timeout=0.2)
        if msg is not None:
            fence.handle(msg)
        fence.tick(time.monotonic())
    if fence.state != "done":
        if fence.state != "failed":
            print(f"[sim-fence] FAILED: not finished after {UPLOAD_TIMEOUT_S:.0f} s (state {fence.state})")
        sys.exit(1)

    if args.gf_action is not None:
        if not set_int_param(mav, target, "GF_ACTION", args.gf_action):
            sys.exit(f"[sim-fence] FAILED to set GF_ACTION = {args.gf_action}")
        print(f"[sim-fence] ✓ GF_ACTION = {args.gf_action}")
    print("[sim-fence] Done -- start the sweep with SEARCH_AREA_SHAPE = \"fence\"")


if __name__ == "__main__":
    main()
