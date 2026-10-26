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
HEARTBEAT + cached GLOBAL_POSITION_INT/GPS_RAW_INT/VFR_HUD/ATTITUDE) on
--status-topic, so seaowls_interfaces must be sourced too.

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
import time
from datetime import datetime

from pymavlink import mavutil

from utils.beacon_mavlink_protocol import (
    unpack_value_array, BEACON_MAVLINK_ADDRESS, BEACON_MAVLINK_VERSION,
)

DEFAULT_ROS_TOPIC = "/seabird/beacon_info"
DEFAULT_STATUS_TOPIC = "/seaowls/uav1/status"
DEFAULT_SAFE_PASSAGE_TOPIC = "/seaowls/uav1/report/safe_passage"

# No UAV heartbeat for this long -> publish STATE_UNKNOWN.
HEARTBEAT_STALE_S = 3.0
# Upper bound on VehicleStatus publish rate (heartbeat + position-driven).
STATUS_MAX_HZ = 5.0

# PX4 modes (mavutil.mode_string_v10 names) reported as STATE_AUTO; any other
# mode is STATE_MANUAL. Arming doesn't affect the state -- only
# EMERGENCY/FLIGHT_TERMINATION system_status counts as STATE_KILLED.
_AUTO_MODES = {"OFFBOARD", "MISSION", "RTL", "LAND", "TAKEOFF", "LOITER",
               "FOLLOWME", "RTGS", "PRECLAND"}
_KILLED_SYSTEM_STATUS = {mavutil.mavlink.MAV_STATE_EMERGENCY,
                         mavutil.mavlink.MAV_STATE_FLIGHT_TERMINATION}

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


def _build_vehicle_status(VehicleStatus, stamp, vehicle_id, heartbeat, cache):
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
    out.current_task = VehicleStatus.TASK_NONE
    out.flight_phase = 0
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
    if args.publish_ros:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
        from seaowls_interfaces.msg import VehicleStatus, SafePassage, BuoyDetection

        rclpy.init()
        ros_node = Node("beacon_mavlink_ground")
        ros_publisher = ros_node.create_publisher(String, args.ros_topic, 10)
        status_publisher = ros_node.create_publisher(VehicleStatus, args.status_topic, 10)
        safe_passage_publisher = ros_node.create_publisher(SafePassage, args.safe_passage_topic, 10)
        print(f"[beacon-ground] Publishing decoded beacons to ROS2 topic {args.ros_topic}")
        print(f"[beacon-ground] Publishing UAV status to ROS2 topic {args.status_topic}")
        print(f"[beacon-ground] Publishing SafePassage to ROS2 topic {args.safe_passage_topic}")

    def publish_status(heartbeat):
        if status_publisher is None:
            return
        status_publisher.publish(_build_vehicle_status(
            VehicleStatus, ros_node.get_clock().now().to_msg(),
            args.vehicle_id, heartbeat, cache,
        ))

    print(f"[beacon-ground] Connecting to {args.mavlink_endpoint} ...")
    mav = mavutil.mavlink_connection(args.mavlink_endpoint, baud=args.baud)
    mav.wait_heartbeat()
    print(f"[beacon-ground] Heartbeat received from system {mav.target_system} "
          f"— listening for beacon messages")

    seen = {}  # msg_id -> time first received
    unique = duplicates = 0
    cache = {}  # latest GLOBAL_POSITION_INT / ATTITUDE / GPS_RAW_INT
    last_heartbeat_t = time.monotonic()  # wait_heartbeat() just saw one
    last_heartbeat = None  # latest UAV HEARTBEAT, reused for position-driven updates
    last_unknown_t = 0.0
    last_status_t = 0.0
    try:
        while True:
            msg = mav.recv_match(
                type=["MEMORY_VECT", "HEARTBEAT", "GLOBAL_POSITION_INT",
                      "ATTITUDE", "GPS_RAW_INT", "VFR_HUD"],
                blocking=True, timeout=1.0,
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
            if msg_type == "HEARTBEAT":
                if _is_uav_heartbeat(msg):
                    last_heartbeat_t = now
                    last_heartbeat = msg
                    publish_status(msg)
                    last_status_t = now
                continue
            if msg_type != "MEMORY_VECT":
                cache[msg_type] = msg
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
