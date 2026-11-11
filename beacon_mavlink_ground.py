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
script's design (nothing here needs ROS by default). --publish-ros also
publishes the UAV's status (seaowls_interfaces/msg/VehicleStatus, built from
HEARTBEAT + cached GLOBAL_POSITION_INT/GPS_RAW_INT/VFR_HUD/ATTITUDE/
EXTENDED_SYS_STATE) on --status-topic, so seaowls_interfaces must be sourced
too. current_task follows the drone's "SEABIRD_TASK <n>" STATUSTEXT (RxTask
value, sent by beacon_mavlink_bridge.py) and is TASK_NONE until one arrives.
--publish-ros also runs the RoboCommand geofence round trip for seaowls_ocs's
course (seaowls_interfaces/msg/UavGeofence on --geofence-topic), back over
this same connection:
  1. VALID/INVALID check of the course and altitude limit
     (utils/geofence_check.py); an invalid one is REFUSED and never uploaded;
  2. the boundary is inset --fence-inset-m (default 1 m) and the ceiling set
     --ceiling-margin-m (default 1 m) under the limit;
  3. both are uploaded to PX4 and READ BACK from it (utils/px4_fence.py);
  4. the readback is checked against the course as RoboCommand checks a
     RunDeclaration, and published -- what the UAV actually holds -- on
     --onboard-topic for seaowls_ocs to declare.
Progress / VALID / REFUSED with reasons goes to --fence-status-topic (JSON).
Both topics are latched (TRANSIENT_LOCAL) so late subscribers get the latest.

Usage:
IN WSL:
    python3 beacon_mavlink_ground.py --mavlink-endpoint udpin:127.0.0.1:14551 --baud 460800
    python3 beacon_mavlink_ground.py --mavlink-endpoint udpin:0.0.0.0:14550
    python3 beacon_mavlink_ground.py --mavlink-endpoint /dev/ttyUSB0 --baud 57600
    python3 beacon_mavlink_ground.py --mavlink-endpoint udpin:0.0.0.0:14550 --publish-ros
"""

import argparse
import csv
import json
import math
import os
import time
from datetime import datetime

# MAVLink 2 before pymavlink picks its dialect at import: fence upload needs
# mission_type and the HAE ceiling needs GPS_RAW_INT.alt_ellipsoid, both
# MAVLink 2 fields that pymavlink's default (MAVLink 1) drops.
os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil

from utils.beacon_mavlink_protocol import (
    unpack_value_array, BEACON_MAVLINK_ADDRESS, BEACON_MAVLINK_VERSION,
)
from utils.px4_fence import FenceUploader
from utils.geofence_check import (
    MAX_UAV_ALTITUDE_HAE_M, clip_transit, fence_problems, inset_polygon, merge_transit,
    min_clearance_m,
    validate_altitude, validate_course, course_warnings, simple_ring,
)

DEFAULT_ROS_TOPIC = "/seabird/beacon_info"
DEFAULT_STATUS_TOPIC = "/seaowls/uav1/status"
DEFAULT_SAFE_PASSAGE_TOPIC = "/seaowls/uav1/report/safe_passage"
DEFAULT_GEOFENCE_TOPIC = "/seaowls/uav1/mission/geofence"
DEFAULT_ONBOARD_TOPIC = "/seaowls/uav1/mission/geofence_onboard"
DEFAULT_FENCE_STATUS_TOPIC = "/seaowls/uav1/mission/geofence_status"

# No UAV heartbeat for this long -> publish STATE_UNKNOWN.
HEARTBEAT_STALE_S = 3.0
# Upper bound on VehicleStatus publish rate (heartbeat + position-driven).
STATUS_MAX_HZ = 5.0
# '[pos]' line interval while the UAV has a 3D GPS fix.
POSITION_PRINT_S = 5.0
# This script's MAVLink system ID -- not 255, which QGC (and the ELRS
# router's GCS heartbeat) use.
GROUND_MAVLINK_SYSID = 246

# PX4 modes (mavutil.mode_string_v10 names) reported as STATE_AUTO; any other
# mode is STATE_MANUAL. Arming doesn't affect the state -- only
# EMERGENCY/FLIGHT_TERMINATION system_status counts as STATE_KILLED.
_AUTO_MODES = {"OFFBOARD", "MISSION", "RTL", "LAND", "TAKEOFF", "LOITER",
               "FOLLOWME", "RTGS", "PRECLAND"}
_KILLED_SYSTEM_STATUS = {mavutil.mavlink.MAV_STATE_EMERGENCY,
                         mavutil.mavlink.MAV_STATE_FLIGHT_TERMINATION}

# robotx rx_common.proto FlightPhase.
_FLIGHT_PHASE_UNKNOWN, _FLIGHT_PHASE_GROUNDED, _FLIGHT_PHASE_AIRBORNE = 0, 1, 2

# STATUSTEXT "SEABIRD_TASK <n>" from the drone sets current_task to RxTask n.
# TASK_UNKNOWN (0) is never accepted -- RoboCommand mustn't see it as a change.
TASK_STATUSTEXT_PREFIX = "SEABIRD_TASK"
# RunStart relay to the UAV (Handbook 3.4.10: vehicles begin the run only after
# the OCS verifies RunStart). seaowls_ocs publishes the verified RunStart on
# RUN_START_TOPIC; it goes up ELRS as STATUSTEXT "SEABIRD_RUNSTART <run_id>",
# resent every RUNSTART_RESEND_S until the drone answers "SEABIRD_RUNSTART_ACK <run_id>".
RUN_START_TOPIC = "/seaowls/run_start"
RUNSTART_PREFIX = "SEABIRD_RUNSTART"
RUNSTART_ACK_PREFIX = "SEABIRD_RUNSTART_ACK"
RUNSTART_RESEND_S = 2.0
# Remote restart of the drone's seabird-mission (GUI "Restart drone mission").
# A String (request id) on RESTART_TOPIC goes up ELRS as STATUSTEXT
# "SEABIRD_RESTART_MISSION <id>", resent every RESTART_RESEND_S until
# voxl/seabird_remote_restart.py answers "SEABIRD_RESTART_ACK <id> OK|REFUSED <why>"
# or RESTART_GIVE_UP_S passes. Refused here while the UAV is armed (and again
# on the drone).
RESTART_TOPIC = "/seaowls/uav1/restart_mission"
RESTART_PREFIX = "SEABIRD_RESTART_MISSION"
RESTART_ACK_PREFIX = "SEABIRD_RESTART_ACK"
RESTART_RESEND_S = 2.0
RESTART_GIVE_UP_S = 30.0
_VALID_TASKS = {1, 2, 3, 4, 5}

# A msg_id seen again within this window is a repeated copy from the bridge
# (--repeat) and is dropped. Time-based rather than "ever seen" so a bridge
# restart, which starts msg_id over at 0, isn't mistaken for duplicates.
DEDUP_WINDOW_S = 10.0


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
        "tracking_id": beacon.tracking_id,
    }


def _is_uav_heartbeat(msg) -> bool:
    """HEARTBEAT from the UAV's autopilot -- not a GCS (e.g. the ELRS
    router's own sysid 255 heartbeat) or a companion component."""
    return (msg.type != mavutil.mavlink.MAV_TYPE_GCS
            and msg.get_srcComponent() == mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1)


def _parse_task_statustext(text):
    """RxTask value from a "SEABIRD_TASK <n>" STATUSTEXT, else None."""
    parts = text.strip("\x00 ").split()
    if len(parts) == 2 and parts[0] == TASK_STATUSTEXT_PREFIX and parts[1].isdigit():
        task = int(parts[1])
        if task in _VALID_TASKS:
            return task
    return None


def _flight_phase(heartbeat, ext_state):
    """FlightPhase from EXTENDED_SYS_STATE.landed_state, falling back to the
    armed flag when PX4 hasn't reported one. Stale link -> UNKNOWN."""
    if heartbeat is None:
        return _FLIGHT_PHASE_UNKNOWN
    if (ext_state is not None
            and ext_state.landed_state != mavutil.mavlink.MAV_LANDED_STATE_UNDEFINED):
        if ext_state.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
            return _FLIGHT_PHASE_GROUNDED
        return _FLIGHT_PHASE_AIRBORNE  # IN_AIR, TAKEOFF, LANDING
    if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        return _FLIGHT_PHASE_AIRBORNE
    return _FLIGHT_PHASE_GROUNDED


def _build_vehicle_status(VehicleStatus, stamp, vehicle_id, heartbeat, cache, current_task):
    """
    VehicleStatus from the latest UAV HEARTBEAT (None = link stale ->
    STATE_UNKNOWN) plus the most recent cached position/attitude messages.
    Fields with no data yet are NaN rather than 0, so a missing fix doesn't
    read as (0, 0).
    """
    nan = float("nan")
    out = VehicleStatus()
    out.header.stamp = stamp
    out.vehicle_id = vehicle_id
    out.vehicle_type = VehicleStatus.TYPE_UAV
    out.current_task = current_task
    out.flight_phase = _flight_phase(heartbeat, cache.get("EXTENDED_SYS_STATE"))
    out.depth_m = 0.0

    if heartbeat is None:
        out.state = VehicleStatus.STATE_UNKNOWN
    elif heartbeat.system_status in _KILLED_SYSTEM_STATUS:
        out.state = VehicleStatus.STATE_KILLED
    elif mavutil.mode_string_v10(heartbeat) in _AUTO_MODES:
        out.state = VehicleStatus.STATE_AUTO
    else:
        out.state = VehicleStatus.STATE_MANUAL

    # Each field falls back through whatever PX4 actually streams on this
    # link -- over ELRS, GLOBAL_POSITION_INT may not be sent at all.
    gpos = cache.get("GLOBAL_POSITION_INT")
    att = cache.get("ATTITUDE")
    gps = cache.get("GPS_RAW_INT")
    hud = cache.get("VFR_HUD")
    gps_fix = gps is not None and gps.fix_type >= 3

    if gpos:
        out.latitude, out.longitude = gpos.lat * 1e-7, gpos.lon * 1e-7
    elif gps_fix:
        out.latitude, out.longitude = gps.lat * 1e-7, gps.lon * 1e-7
    else:
        out.latitude = out.longitude = nan

    if gpos:
        out.speed_mps = math.hypot(gpos.vx, gpos.vy) / 100.0
    elif hud:
        out.speed_mps = float(hud.groundspeed)
    elif gps_fix and gps.vel != 65535:
        out.speed_mps = gps.vel / 100.0
    else:
        out.speed_mps = nan

    if gpos and gpos.hdg != 65535:
        out.heading_deg = gpos.hdg / 100.0
    elif hud:
        out.heading_deg = float(hud.heading % 360)
    elif att:
        out.heading_deg = math.degrees(att.yaw) % 360.0
    else:
        out.heading_deg = nan

    out.roll_deg = math.degrees(att.roll) if att else nan
    out.pitch_deg = math.degrees(att.pitch) if att else nan

    # Height above ellipsoid when GPS_RAW_INT carries it (MAVLink 2
    # extension, 0 if unset), else AMSL from GLOBAL_POSITION_INT / VFR_HUD.
    if gps_fix and getattr(gps, "alt_ellipsoid", 0):
        out.altitude_hae_m = gps.alt_ellipsoid / 1000.0
    elif gpos:
        out.altitude_hae_m = gpos.alt / 1000.0
    elif hud:
        out.altitude_hae_m = float(hud.alt)
    else:
        out.altitude_hae_m = nan
    return out


# (color, is_blinking) -> seaowls_interfaces/BuoyDetection beacon_state.
# Anything else (steady red/green, white, unknown blink) isn't reported --
# a guessed state costs more than a missing one. "off" = housing seen, LED
# never lit (see utils/downward_beacon_classifier.py).
_BEACON_STATES = {
    ("off", False): 1,    # OFF
    ("off", None): 1,     # OFF
    ("red", True): 2,     # FLASHING_RED
    ("green", True): 3,   # FLASHING_GREEN
    ("blue", True): 4,    # FLASHING_BLUE  -> entry
    ("blue", False): 5,   # STEADY_BLUE    -> exit
}
_STATE_ENTRY, _STATE_EXIT = 4, 5


def _distance_m(lat1, lon1, lat2, lon2) -> float:
    """Flat-earth distance -- fine at buoy-field scale."""
    north = math.radians(lat2 - lat1) * 6371000.0
    east = math.radians(lon2 - lon1) * 6371000.0 * math.cos(math.radians(lat1))
    return math.hypot(north, east)


class _BuoyField:
    """
    The full mapped buoy field for Task 1 SafePassage, which RoboCommand
    expects resent in full whenever it changes. Beacons merge by tracking_id
    when there is one (>= 0); otherwise (the downward classifier always sends
    -1) by distance to an existing buoy. A re-read replaces the old state, so
    colours that change mid-run (Disruptive tier) are tracked.
    """

    def __init__(self, merge_m: float):
        self.merge_m = merge_m
        self.buoys = []  # [{"tracking_id", "lat", "lon", "state", "t"}]

    def update(self, beacon, received: float) -> bool:
        """Merge one decoded beacon; True if the field changed."""
        state = _BEACON_STATES.get((beacon.color, beacon.is_blinking))
        if state is None:
            return False
        match = None
        if beacon.tracking_id >= 0:
            match = next((b for b in self.buoys if b["tracking_id"] == beacon.tracking_id), None)
        if match is None:
            near = [b for b in self.buoys
                    if _distance_m(b["lat"], b["lon"], beacon.latitude, beacon.longitude) <= self.merge_m]
            match = min(near, default=None, key=lambda b: _distance_m(
                b["lat"], b["lon"], beacon.latitude, beacon.longitude))
        if match is None:
            self.buoys.append({"tracking_id": beacon.tracking_id, "lat": beacon.latitude,
                               "lon": beacon.longitude, "state": state, "t": received})
            return True
        changed = (match["state"], match["lat"], match["lon"]) != (
            state, beacon.latitude, beacon.longitude)
        match.update(lat=beacon.latitude, lon=beacon.longitude, state=state, t=received)
        if beacon.tracking_id >= 0:
            match["tracking_id"] = beacon.tracking_id
        return changed

    def _latest(self, state):
        found = [b for b in self.buoys if b["state"] == state]
        return max(found, key=lambda b: b["t"]) if found else None

    def to_msg(self, SafePassage, BuoyDetection, stamp, vehicle_id):
        """seaowls_interfaces/SafePassage. entry/exit Point x=lat, y=lon (as
        seaowls_ocs reads them); left (0, 0) until that buoy is found."""
        out = SafePassage()
        out.header.stamp = stamp
        out.source_vehicle_id = vehicle_id
        for state, point in ((_STATE_ENTRY, out.entry_position), (_STATE_EXIT, out.exit_position)):
            b = self._latest(state)
            if b is not None:
                point.x, point.y = b["lat"], b["lon"]
        for b in self.buoys:
            d = BuoyDetection()
            d.latitude, d.longitude, d.beacon_state = b["lat"], b["lon"], b["state"]
            out.buoys.append(d)
        return out


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument(
        "--mavlink-endpoint", required=True, default="udpin:127.0.0.1:14551", 
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
    ap.add_argument(
        "--status-topic", default=DEFAULT_STATUS_TOPIC,
        help="VehicleStatus topic published once per UAV heartbeat when --publish-ros "
             f"is set (default: {DEFAULT_STATUS_TOPIC})",
    )
    ap.add_argument(
        "--vehicle-id", default="UAV1",
        help="VehicleStatus.vehicle_id -- seaowls_ocs only accepts the uppercase "
             "RoboCommand ID (default: UAV1)",
    )
    ap.add_argument(
        "--safe-passage-topic", default=DEFAULT_SAFE_PASSAGE_TOPIC,
        help="SafePassage topic (full buoy field, republished on every change) when "
             f"--publish-ros is set (default: {DEFAULT_SAFE_PASSAGE_TOPIC})",
    )
    ap.add_argument(
        "--buoy-merge-m", type=float, default=3.0,
        help="Beacons without a tracking_id within this distance (m) of a known buoy "
             "update it instead of adding a new one (default: 3.0)",
    )
    ap.add_argument(
        "--geofence-topic", default=DEFAULT_GEOFENCE_TOPIC,
        help="UavGeofence topic (from seaowls_ocs) uploaded to PX4 as its fence when "
             f"--publish-ros is set (default: {DEFAULT_GEOFENCE_TOPIC})",
    )
    ap.add_argument(
        "--fence-include-transit", action=argparse.BooleanOptionalAction, default=True,
        help="Also upload the UAV transit area as a second inclusion polygon "
             "(default: on; --no-fence-include-transit uploads the course boundary only)",
    )
    ap.add_argument(
        "--transit-fraction", type=float, default=0.7,
        help="Upload only the part of the transit area within this fraction of its depth "
             "from the course edge it adjoins (default: 0.7 -- the launch pad sits in the "
             "transit area and needs GPS margin; 1 = the whole transit area)",
    )
    ap.add_argument(
        "--onboard-topic", default=DEFAULT_ONBOARD_TOPIC,
        help="UavGeofence topic for the fence + ceiling READ BACK from PX4, for "
             f"seaowls_ocs to declare (default: {DEFAULT_ONBOARD_TOPIC})",
    )
    ap.add_argument(
        "--fence-status-topic", default=DEFAULT_FENCE_STATUS_TOPIC,
        help=f"Geofence round-trip status, std_msgs/String JSON (default: {DEFAULT_FENCE_STATUS_TOPIC})",
    )
    ap.add_argument(
        "--fence-inset-m", type=float, default=1.0,
        help="Upload the course boundary this far inside the published one (default: 1.0)",
    )
    ap.add_argument(
        "--ceiling-margin-m", type=float, default=1.0,
        help="Set PX4's ceiling this far under the altitude limit (default: 1.0)",
    )
    ap.add_argument(
        "--assume-home-hae-m", type=float, default=None,
        help="BENCH ONLY: home height above the ellipsoid (m) to assume while the UAV has no "
             "3D GPS fix / home position (indoors), so the round trip can finish. Never at "
             "the competition: the declared ceiling is only as right as this guess.",
    )
    ap.add_argument(
        "--max-alt-limit-hae-m", type=float, default=MAX_UAV_ALTITUDE_HAE_M,
        help="Competition maximum UAV altitude, m HAE: a higher limit is REFUSED "
             f"(default: {MAX_UAV_ALTITUDE_HAE_M:g})",
    )
    ap.add_argument(
        "--log", metavar="PATH", default=None,
        help="Append each decoded beacon and its receive time to PATH as CSV",
    )
    args = ap.parse_args()

    log_file = log_writer = None
    if args.log:
        log_file = open(args.log, "a", newline="")
        log_writer = csv.writer(log_file)
        if log_file.tell() == 0:
            log_writer.writerow(["iso_time", "epoch_time", "latitude", "longitude",
                                 "color", "is_blinking", "tracking_id", "msg_id"])
            log_file.flush()

    ros_publisher = None
    status_publisher = None
    safe_passage_publisher = None
    ros_node = None
    buoy_field = _BuoyField(args.buoy_merge_m)
    pending_fence = None   # latest UavGeofence not yet handed to the uploader
    run_start = None       # {run_id, acked, last_sent, sends} being relayed to the UAV
    restart_req = None     # {id, last_sent, started} remote restart being relayed
    uav_armed = None       # armed flag from the latest UAV heartbeat (None = none yet)
    onboard_publisher = fence_status_publisher = None
    if args.publish_ros:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from std_msgs.msg import String
        from seaowls_interfaces.msg import (
            VehicleStatus, SafePassage, BuoyDetection, UavGeofence,
        )
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        rclpy.init()
        ros_node = Node("beacon_mavlink_ground")
        ros_publisher = ros_node.create_publisher(String, args.ros_topic, 10)
        status_publisher = ros_node.create_publisher(VehicleStatus, args.status_topic, 10)
        safe_passage_publisher = ros_node.create_publisher(SafePassage, args.safe_passage_topic, 10)
        print(f"[beacon-ground] Publishing decoded beacons to ROS2 topic {args.ros_topic}")
        print(f"[beacon-ground] Publishing UAV status to ROS2 topic {args.status_topic}")
        print(f"[beacon-ground] Publishing SafePassage to ROS2 topic {args.safe_passage_topic}")

        def on_geofence(msg):
            nonlocal pending_fence
            pending_fence = msg

        # Latched (seaowls_ocs publishes it TRANSIENT_LOCAL): starting after the
        # OCS got the retained course still delivers it.
        ros_node.create_subscription(UavGeofence, args.geofence_topic, on_geofence, latched)

        def on_run_start(msg):
            nonlocal run_start
            try:
                run_id = int(json.loads(msg.data)["run_id"])
            except (ValueError, KeyError, TypeError):
                print(f"[beacon-ground] [runstart] ignored malformed RunStart: {msg.data!r}")
                return
            if run_start is None or run_start["run_id"] != run_id:
                run_start = dict(run_id=run_id, acked=False, last_sent=0.0, sends=0)
                print(f"[beacon-ground] [runstart] RunStart run_id={run_id} verified by the OCS "
                      f"-- relaying to the UAV")

        ros_node.create_subscription(String, RUN_START_TOPIC, on_run_start, latched)

        def on_restart(msg):
            nonlocal restart_req
            request_id = "".join(ch for ch in str(msg.data) if ch.isalnum())[:16]
            if not request_id:
                print(f"[beacon-ground] [restart] ignored request without an id: {msg.data!r}",
                      flush=True)
                return
            if uav_armed:
                print(f"[beacon-ground] [restart] REFUSED: {args.vehicle_id} is armed "
                      f"(request {request_id})", flush=True)
                return
            if uav_armed is None:
                print(f"[beacon-ground] [restart] no {args.vehicle_id} heartbeat yet -- sending "
                      f"anyway (the drone checks armed itself)", flush=True)
            restart_req = dict(id=request_id, last_sent=0.0, started=time.monotonic())
            print(f"[beacon-ground] [restart] request {request_id} sent to {args.vehicle_id}",
                  flush=True)

        # Volatile (not latched): an old request is never replayed on restart.
        ros_node.create_subscription(String, RESTART_TOPIC, on_restart, 10)
        onboard_publisher = ros_node.create_publisher(UavGeofence, args.onboard_topic, latched)
        fence_status_publisher = ros_node.create_publisher(String, args.fence_status_topic, latched)
        print(f"[beacon-ground] Uploading geofence from ROS2 topic {args.geofence_topic} to PX4"
              + (" (with transit area)" if args.fence_include_transit else "")
              + f", inset {args.fence_inset_m:g} m, ceiling {args.ceiling_margin_m:g} m under the limit;"
              f" readback on {args.onboard_topic}")

    def publish_fence_status(course_id, state, reasons=(), warnings=(), **extra):
        """Log a geofence round-trip step and publish it as JSON. Warnings
        (e.g. corners reordered, implausible size) don't block."""
        reasons, warnings = list(reasons), list(warnings)
        if state in ("uploading", "refused"):   # printed once; VALID carries them in the JSON
            for w in warnings:
                print(f"[beacon-ground] [fence] WARNING course {course_id!r}: {w}")
        line = f"[beacon-ground] [fence] {state.upper()} course {course_id!r}"
        print(line + (": " + "; ".join(reasons) if reasons else ""))
        if fence_status_publisher is not None:
            fence_status_publisher.publish(String(data=json.dumps(dict(
                course_id=course_id, state=state, reasons=reasons, warnings=warnings,
                time=time.time(), **extra))))

    def ring(lats, lons):
        return [(float(a), float(b)) for a, b in zip(lats, lons)]

    current_task = 1  # TASK_NONE until the drone reports a task

    def publish_status(heartbeat):
        if status_publisher is None:
            return
        status_publisher.publish(_build_vehicle_status(
            VehicleStatus, ros_node.get_clock().now().to_msg(),
            args.vehicle_id, heartbeat, cache, current_task,
        ))

    print(f"[beacon-ground] Connecting to {args.mavlink_endpoint} ...")
    # Own MAVLink system ID: QGC is also 255, and PX4 addresses mission
    # replies by system ID -- sharing it let QGC take our fence readback.
    mav = mavutil.mavlink_connection(args.mavlink_endpoint, baud=args.baud,
                                     source_system=GROUND_MAVLINK_SYSID)
    mav.wait_heartbeat()
    print(f"[beacon-ground] Heartbeat received from system {mav.target_system} "
          f"— listening for beacon messages")
    fence = FenceUploader(mav, log=lambda s: print(f"[beacon-ground] {s}"),
                          assume_home_hae_m=args.assume_home_hae_m)
    if args.assume_home_hae_m is not None:
        print(f"[beacon-ground] BENCH: assuming home {args.assume_home_hae_m:g} m HAE "
              f"until the UAV reports a GPS fix / home")
    last_fence_key = None  # skip re-checking/re-uploading an unchanged course
    fence_job = None       # the upload in progress: course, transit, course_id

    last_pos_print = 0.0
    had_fix = None

    def report_position(now):
        """'[pos]' line every POSITION_PRINT_S while there's a 3D GPS fix, and
        once when the fix is lost -- the GUI centres the mock course on it."""
        nonlocal last_pos_print, had_fix
        gps = cache.get("GPS_RAW_INT")
        fix = gps is not None and gps.fix_type >= 3
        if fix:
            gpos = cache.get("GLOBAL_POSITION_INT")
            lat, lon = (gpos.lat, gpos.lon) if gpos and (gpos.lat or gpos.lon) else (gps.lat, gps.lon)
            if not had_fix or now - last_pos_print >= POSITION_PRINT_S:
                print(f"[beacon-ground] [pos] 3D fix lat={lat * 1e-7:.7f} lon={lon * 1e-7:.7f} "
                      f"sats={gps.satellites_visible}", flush=True)
                last_pos_print = now
        elif had_fix is not False:
            print(f"[beacon-ground] [pos] no GPS fix (fix_type={gps.fix_type if gps else 'none'})",
                  flush=True)
        had_fix = fix

    seen = {}  # msg_id -> time first received
    unique = duplicates = 0
    cache = {}  # latest GLOBAL_POSITION_INT / ATTITUDE / GPS_RAW_INT / VFR_HUD / EXTENDED_SYS_STATE
    last_heartbeat_t = time.monotonic()  # wait_heartbeat() just saw one
    last_heartbeat = None  # latest UAV HEARTBEAT, reused for position-driven updates
    last_mode = None       # (PX4 mode, RoboCommand state, armed) last logged
    last_unknown_t = 0.0
    last_status_t = 0.0
    try:
        while True:
            if ros_node is not None:
                rclpy.spin_once(ros_node, timeout_sec=0)
            if pending_fence is not None and last_heartbeat is not None:
                course_id = pending_fence.course_id
                course = ring(pending_fence.boundary_latitude, pending_fence.boundary_longitude)
                transit = ring(pending_fence.transit_area_latitude,
                               pending_fence.transit_area_longitude) or None
                limit = pending_fence.max_altitude_hae_m
                pending_fence = None
                key = (tuple(course), tuple(transit or ()), limit)
                if key != last_fence_key or fence.state == "failed":
                    last_fence_key = key
                    fence_job = None
                    # 1. VALID / INVALID: the course and the altitude limit as received.
                    problems = [f"course boundary {p}" for p in validate_course(course)]
                    if args.fence_include_transit and transit:
                        problems += [f"transit area {p}" for p in validate_course(transit)]
                    problems += validate_altitude(limit, args.max_alt_limit_hae_m)
                    ceiling = limit - args.ceiling_margin_m
                    if not problems and ceiling <= 0:
                        problems.append(f"altitude limit {limit:g} m HAE minus the "
                                        f"{args.ceiling_margin_m:g} m margin is not above 0")
                    # 2. Corner order and size don't make a course INVALID: crossing
                    #    corners become a non-crossing polygon through the same
                    #    corners, and an implausible size is only a warning.
                    warnings = []
                    if not problems:
                        warnings += [f"course boundary {w}" for w in course_warnings(course)]
                        course, how = simple_ring(course)
                        if how == "convex hull":
                            warnings.append("course boundary: corners with equal bearings -- "
                                            "used their convex hull")
                        if transit and not validate_course(transit):
                            transit, _ = simple_ring(transit)
                            if args.fence_include_transit:
                                warnings += [f"transit area {w}" for w in course_warnings(transit)]
                    # 3. Inset (refused if it collapses the polygon).
                    polygons = []
                    if not problems:
                        # PX4 gets ONE polygon: the OCS declares the first read-back
                        # polygon to RoboCommand, so the drone's fence and the
                        # declared fence are the same outline. With the transit
                        # area, that is the course joined to the course-side part
                        # of the transit area (the full area is still what the
                        # readback is checked against; the joined one lies inside
                        # course + transit). If they can't be joined, course only.
                        outline = course
                        if args.fence_include_transit and transit:
                            try:
                                outline = merge_transit(course, clip_transit(
                                    course, transit, args.transit_fraction))
                            except ValueError as e:
                                warnings.append(f"transit area not uploaded ({e}) -- "
                                                "fence is the course boundary only")
                        try:
                            polygons.append(inset_polygon(outline, args.fence_inset_m)[:-1])
                        except ValueError as e:
                            problems.append(str(e))
                    if problems:
                        publish_fence_status(course_id, "refused", problems, warnings)
                    else:
                        # 4. Upload, then read back (FenceUploader).
                        publish_fence_status(course_id, "uploading", warnings=warnings,
                                             inset_m=args.fence_inset_m, ceiling_hae_m=ceiling)
                        fence.start(last_heartbeat.get_srcSystem(), polygons, ceiling)
                        # The readback is checked against the (repaired) course.
                        fence_job = dict(course_id=course_id, course=course, transit=transit,
                                         warnings=warnings)
            fence.tick(time.monotonic())
            if (run_start is not None and not run_start["acked"]
                    and time.monotonic() - run_start["last_sent"] >= RUNSTART_RESEND_S):
                mav.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO,
                                        f"{RUNSTART_PREFIX} {run_start['run_id']}".encode())
                run_start["last_sent"] = time.monotonic()
                run_start["sends"] += 1
                if run_start["sends"] % 15 == 0:
                    print(f"[beacon-ground] [runstart] still no acknowledgement from the UAV "
                          f"for run_id={run_start['run_id']} ({run_start['sends']} sends)")
            if restart_req is not None:
                if time.monotonic() - restart_req["started"] > RESTART_GIVE_UP_S:
                    print(f"[beacon-ground] [restart] no acknowledgement after "
                          f"{RESTART_GIVE_UP_S:.0f} s (request {restart_req['id']}): is "
                          f"seabird-remote running on the drone, and the ELRS bridge copying "
                          f"to 14571?", flush=True)
                    restart_req = None
                elif time.monotonic() - restart_req["last_sent"] >= RESTART_RESEND_S:
                    mav.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO,
                                            f"{RESTART_PREFIX} {restart_req['id']}".encode())
                    restart_req["last_sent"] = time.monotonic()
            if (fence_job is not None and fence.waiting_reason
                    and fence_job.get("waiting") != fence.waiting_reason):
                fence_job["waiting"] = fence.waiting_reason
                publish_fence_status(fence_job["course_id"], "waiting", [fence.waiting_reason])
            if fence_job is not None and fence.state in ("done", "failed"):
                job, fence_job = fence_job, None
                if fence.state == "failed":
                    publish_fence_status(job["course_id"], "failed", [fence.failure_reason])
                else:
                    # 5. Check what PX4 actually holds, then hand it to the OCS.
                    rb = fence.readback_polygons or []
                    hae = fence.readback_ceiling_hae_m
                    problems = []
                    if not rb:
                        problems.append("PX4 returned no inclusion polygon")
                    elif len(rb) > 1:
                        problems.append(f"PX4 holds {len(rb)} inclusion polygons; only the "
                                        "first would be declared to RoboCommand")
                    else:
                        onboard = rb[0] + rb[0][:1]
                        problems += [f"UAV readback: {p}" for p in
                                     fence_problems(onboard, job["course"], job["transit"])]
                    if hae is None:
                        problems.append("UAV readback: ceiling HAE unknown (no HOME_POSITION / "
                                        "GPS alt_ellipsoid)")
                    else:
                        problems += [f"UAV readback: {p}" for p in
                                     validate_altitude(hae, args.max_alt_limit_hae_m)]
                    if problems:
                        publish_fence_status(job["course_id"], "refused", problems)
                    else:
                        out = UavGeofence()
                        out.header.stamp = ros_node.get_clock().now().to_msg()
                        out.header.frame_id = "wgs84"
                        out.course_id = job["course_id"]
                        out.max_altitude_hae_m = hae
                        out.boundary_latitude = [p[0] for p in onboard]
                        out.boundary_longitude = [p[1] for p in onboard]
                        out.transit_area_latitude = [p[0] for p in job["transit"] or []]
                        out.transit_area_longitude = [p[1] for p in job["transit"] or []]
                        onboard_publisher.publish(out)
                        publish_fence_status(
                            job["course_id"], "valid", warnings=job["warnings"],
                            inset_m=args.fence_inset_m,
                            clearance_m=round(min_clearance_m(onboard, job["course"]), 2),
                            ceiling_hae_m=round(hae, 2), vertices=len(onboard) - 1,
                            home_hae_assumed=fence.home_hae_assumed)
                        if fence.home_hae_assumed:
                            print("[beacon-ground] [fence] NOTE: ceiling HAE uses the ASSUMED "
                                  "home height (bench) -- not valid for the competition")

            msg = mav.recv_match(
                type=["MEMORY_VECT", "HEARTBEAT", "GLOBAL_POSITION_INT",
                      "ATTITUDE", "GPS_RAW_INT", "VFR_HUD", "EXTENDED_SYS_STATE",
                      "STATUSTEXT", "MISSION_REQUEST_INT", "MISSION_REQUEST",
                      "MISSION_ACK", "MISSION_COUNT", "MISSION_ITEM_INT",
                      "PARAM_VALUE", "HOME_POSITION"],
                blocking=True, timeout=0.1 if ros_node is not None else 1.0,
            )
            now = time.monotonic()
            # Link stale: keep publishing STATE_UNKNOWN at ~1 Hz.
            if (now - last_heartbeat_t >= HEARTBEAT_STALE_S
                    and now - last_unknown_t >= 1.0):
                publish_status(None)
                last_unknown_t = now
            if msg is None:
                continue
            msg_type = msg.get_type()
            fence.handle(msg)
            if msg_type in ("MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK",
                            "MISSION_COUNT", "MISSION_ITEM_INT", "PARAM_VALUE",
                            "HOME_POSITION"):
                continue  # fence uploader only
            if msg_type == "HEARTBEAT":
                if _is_uav_heartbeat(msg):
                    last_heartbeat_t = now
                    last_heartbeat = msg
                    publish_status(msg)
                    last_status_t = now
                    # Log every flight-mode / RoboCommand-state change (same rule as
                    # _build_vehicle_status), e.g. to show the UAV AUTONOMOUS before RunStart.
                    mode = mavutil.mode_string_v10(msg)
                    state = ("STATE_KILLED" if msg.system_status in _KILLED_SYSTEM_STATUS
                             else "STATE_AUTO" if mode in _AUTO_MODES else "STATE_MANUAL")
                    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    uav_armed = armed
                    if (mode, state, armed) != last_mode:
                        last_mode = (mode, state, armed)
                        print(f"[beacon-ground] [mode] {args.vehicle_id} {mode} -> {state} "
                              f"({'armed' if armed else 'disarmed'})", flush=True)
                continue
            if msg_type == "STATUSTEXT":
                text = str(msg.text).strip("\x00 ")
                if text.startswith(RUNSTART_ACK_PREFIX + " ") and run_start is not None:
                    if text.split()[-1] == str(run_start["run_id"]) and not run_start["acked"]:
                        run_start["acked"] = True
                        print(f"[beacon-ground] [runstart] UAV acknowledged RunStart "
                              f"run_id={run_start['run_id']}")
                    continue
                if text.startswith(RESTART_ACK_PREFIX + " "):
                    parts = text.split(None, 2)   # prefix, id, result
                    if (restart_req is not None and len(parts) >= 3
                            and parts[1] == restart_req["id"]):
                        result = parts[2]
                        detail = ("seabird-mission restarting" if result == "OK"
                                  else "seabird-mission NOT restarted")
                        print(f"[beacon-ground] [restart] drone: {result} -- {detail} "
                              f"(request {restart_req['id']})", flush=True)
                        restart_req = None
                    continue
                task = _parse_task_statustext(msg.text)
                if task is not None and task != current_task:
                    print(f"[beacon-ground] current_task {current_task} -> {task}")
                    current_task = task
                    # Report the start/stand-down right away, not on the next heartbeat.
                    if last_heartbeat is not None and now - last_heartbeat_t < HEARTBEAT_STALE_S:
                        publish_status(last_heartbeat)
                        last_status_t = now
                continue
            if msg_type != "MEMORY_VECT":
                cache[msg_type] = msg
                if msg_type == "GPS_RAW_INT":
                    report_position(now)
                # 1 Hz heartbeats alone can trip seaowls_ocs's 2 s stale check
                # after one dropped packet -- also publish on ATTITUDE (the
                # fastest stream over ELRS), capped at STATUS_MAX_HZ, while the
                # heartbeat is still fresh.
                if (msg_type == "ATTITUDE" and last_heartbeat is not None
                        and now - last_heartbeat_t < HEARTBEAT_STALE_S
                        and now - last_status_t >= 1.0 / STATUS_MAX_HZ):
                    publish_status(last_heartbeat)
                    last_status_t = now
                continue
            if msg.address != BEACON_MAVLINK_ADDRESS:
                continue  # not ours — some other real memory-vector traffic on this link
            if msg.ver != BEACON_MAVLINK_VERSION:
                print(f"[beacon-ground] WARNING: received beacon message with unknown "
                      f"protocol version {msg.ver} (expected {BEACON_MAVLINK_VERSION}) — skipping")
                continue

            beacon = unpack_value_array(msg.value)
            received = time.time()
            if received - seen.get(beacon.msg_id, float("-inf")) < DEDUP_WINDOW_S:
                duplicates += 1
                continue
            seen[beacon.msg_id] = received
            if len(seen) > 1024:
                seen = {k: t for k, t in seen.items() if received - t < DEDUP_WINDOW_S}
            unique += 1

            ts = time.strftime("%H:%M:%S")
            blink_str = "unknown" if beacon.is_blinking is None else str(beacon.is_blinking)
            print(f"[{ts}] beacon: lat={beacon.latitude:.6f} lon={beacon.longitude:.6f} "
                  f"color={beacon.color} blinking={blink_str} tracking_id={beacon.tracking_id} "
                  f"msg_id={beacon.msg_id} (duplicates skipped: {duplicates})")

            if log_writer is not None:
                log_writer.writerow([
                    datetime.fromtimestamp(received).isoformat(timespec="milliseconds"),
                    f"{received:.6f}", beacon.latitude, beacon.longitude,
                    beacon.color, blink_str, beacon.tracking_id, beacon.msg_id,
                ])
                log_file.flush()

            if ros_publisher is not None:
                out = String()
                out.data = json.dumps(_build_detection_json(beacon))
                ros_publisher.publish(out)

            if buoy_field.update(beacon, received) and safe_passage_publisher is not None:
                safe_passage_publisher.publish(buoy_field.to_msg(
                    SafePassage, BuoyDetection, ros_node.get_clock().now().to_msg(),
                    args.vehicle_id,
                ))
                print(f"[beacon-ground] SafePassage published: {len(buoy_field.buoys)} buoys")
    except KeyboardInterrupt:
        pass
    finally:
        print(f"[beacon-ground] {unique} unique beacon messages, {duplicates} duplicates skipped")
        if log_file is not None:
            log_file.close()
        if ros_node is not None:
            ros_node.destroy_node()
            import rclpy
            rclpy.shutdown()


if __name__ == "__main__":
    main()
