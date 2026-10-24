#!/usr/bin/env python3
"""
sweep_rrt.py
============
Beacon search using Rapidly-exploring Random Trees (RRT).

The drone grows an RRT from its takeoff position outward to MAX_SEARCH_RADIUS_M.

Two-stage beacon handling:
  Stage 1 (positions) — /seabird/beacon_map, published by beacon_overlay. Each
    message is the full fused map: beacons[] with a stable id and a position in
    PX4 local NED (same frame as MAVSDK), plus lat/lon and uncertainty. This is
    the ONLY position source; no world_position is computed here.
  Stage 2 (colour/blink) — the downward classifier run in-process
    (classify_beacon_downward_once) once the drone is hovering directly over a
    beacon. It reads the top face up close and returns colour + is_blinking,
    which map to the Task-1 state.

Beacon over-fly (BEACON_OVERFLY):
  The search diverts to fly DIRECTLY OVER the nearest mapped, unclassified
  beacon — it becomes an RRT goal and the tree is grown toward it one RRT_STEP_M
  leg at a time (each leg end is a real node), finishing with the drone
  overhead, where Stage 2 reads its state. Beacons are keyed by beacon-map id, so
  each physical beacon is classified once. See BEACON_POS_FRAME for the
  north/east convention (beacon_map is "ned").

Three RViz topics are published:
  /seabird/flight_path    — nav_msgs/Path of visited positions
  /seabird/path_marker    — LINE_STRIP of the same path
  /seabird/rrt_tree       — LINE_LIST of every RRT edge
  /seabird/rrt_nodes      — SPHERE_LIST of every RRT node

Pre-conditions:
  - MAVSDK must receive PX4 MAVLink on MAVSDK_ADDRESS.
  - beacon_overlay must be running and localising (outdoors, GPS 3D fix) so that
    /seabird/beacon_map carries beacons, on the same ROS_DOMAIN_ID.
  - configs/modal_config.json present for the downward classifier.
"""

import asyncio
import math
import json
import random
import threading
import sys
import time
import traceback
from typing import Optional


import rclpy
from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

# Positions come from /seabird/beacon_map (beacon_overlay, Stage 1). Colour +
# blink come from the downward classifier run overhead in-process (Stage 2).
import os
try:
    import beacon_detector_config as bdc
    from downward_beacon_classifier_node import classify_beacon_downward_once
except ImportError as e:
    print(f"[ERROR] downward-classifier modules not importable: {e}", flush=True)
    sys.exit(1)
_CFG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "configs", "modal_config.json")
try:
    cfg = bdc.load_config(_CFG_PATH)
except Exception as e:
    print(f"[ERROR] could not load downward-classifier config '{_CFG_PATH}': {e}",
          flush=True)
    sys.exit(1)

try:
    from mavsdk import System
    from mavsdk.offboard import OffboardError, PositionNedYaw
    from mavsdk.action import ActionError
except ImportError:
    print("[ERROR] mavsdk not installed. Run: pip install mavsdk --break-system-packages",
          flush=True)
    sys.exit(1)


# ─────────────────────────────────────────────────────────────────────────────
#  CONFIGURATION  — edit the constants in this block to tune the search
# ─────────────────────────────────────────────────────────────────────────────

# Takeoff and flight
TAKEOFF_ALT_M        = 5.0    # altitude in metres
WAYPOINT_TOL_M       = 0.5    # arrival tolerance
WAYPOINT_TIMEOUT     = 60.0   # seconds before a waypoint is skipped
HOVER_STABILIZE_S    = 8.0    # seconds to stabilise after takeoff

# ── Search area ───────────────────────────────────────────────────────────────
# The region the RRT is allowed to search. RobotX Task 1 courses are "on the
# order of 25 m x 25 m" (final size varies), so the default is a 25 x 25 box.
# Edit these for testing — e.g. shrink the box, or switch SEARCH_AREA_SHAPE to
# "disc" for the old circular behaviour.
SEARCH_AREA_SHAPE    = "disc"   # "rect" = W x L box (default), "disc" = circle
SEARCH_WIDTH_M       = 25.0     # rect: east–west extent of the box (m)
SEARCH_LENGTH_M      = 25.0     # rect: north–south extent of the box (m)
SEARCH_ORIGIN        = "center" # where the takeoff point sits in the box:
                                #   "center" = box centred on takeoff (±W/2, ±L/2)
                                #   "corner" = takeoff at the SW corner (+N, +E)
                                #   "edge"   = takeoff at the S-edge midpoint (+N, ±E/2)
MAX_SEARCH_RADIUS_M  = 10     # disc: radius from takeoff (m); used ONLY when
                                #       SEARCH_AREA_SHAPE == "disc"


def _search_bounds(origin_n: float, origin_e: float):
    """(n_min, n_max, e_min, e_max) of the rect box around a takeoff point."""
    W, L = SEARCH_WIDTH_M, SEARCH_LENGTH_M
    if SEARCH_ORIGIN == "corner":
        return origin_n, origin_n + L, origin_e, origin_e + W
    if SEARCH_ORIGIN == "edge":
        return origin_n, origin_n + L, origin_e - W / 2, origin_e + W / 2
    return origin_n - L / 2, origin_n + L / 2, origin_e - W / 2, origin_e + W / 2


# Longest straight-line distance across the area, and its area — used to size the
# node budget and the over-fly leg budget for either shape.
if SEARCH_AREA_SHAPE == "rect":
    _SEARCH_SPAN_M = math.hypot(SEARCH_WIDTH_M, SEARCH_LENGTH_M)
    _SEARCH_AREA_M2 = SEARCH_WIDTH_M * SEARCH_LENGTH_M
else:
    _SEARCH_SPAN_M = 2.0 * MAX_SEARCH_RADIUS_M
    _SEARCH_AREA_M2 = math.pi * MAX_SEARCH_RADIUS_M ** 2

# ── RRT parameters ────────────────────────────────────────────────────────────
RRT_STEP_M           = 1.5    # maximum edge length for each tree extension
RRT_GOAL_BIAS        = 0.08   # probability of biasing sample toward least-explored sector

# Reactive beacon attraction (Option A): when the detector has glimpsed a beacon
# that is NOT yet confirmed/classified, bias a fraction of search samples toward
# it so the drone drifts over to get a clean look. Attraction is suppressed once
# a buoy is confirmed at that spot, so the drone does not fixate on known beacons.
BEACON_ATTRACT_PROB  = 0.6    # chance a sample aims at an unconfirmed beacon
BEACON_ATTRACT_JITTER_M = 1.0 # Gaussian spread (m) around the beacon target

# Beacon over-fly: when the detector has given an unconfirmed beacon a world
# position, divert and fly DIRECTLY OVER it — inserting the legs to it (and the
# beacon itself) as real RRT nodes — then hover overhead to read its state.
# This is a stronger version of the attraction above: attraction only nudges the
# random sampling, overfly commits to the beacon as a goal. The two cooperate.
BEACON_OVERFLY          = True
BEACON_OVERFLY_TOL_M    = 0.6    # how close "directly overhead" must be (m)
BEACON_OVERFLY_OFFSET_M = 0.0    # standoff from the beacon along the approach
                                 # direction; 0.0 = dead overhead. Raise only if
                                 # a forward/angled camera loses the beacon when
                                 # directly above it.
BEACON_MAX_LEGS         = max(8, int(2.0 * _SEARCH_SPAN_M / RRT_STEP_M) + 4)
MAX_OVERFLY_ATTEMPTS    = 3       # give up flying over a spot after this many
                                  # passes that don't yield a confirmed state
                                  # (prevents thrashing on an unreadable beacon)
# Frame of the detection's "world_position". This file reads it as (north, east)
# = (wp[0], wp[1]) — "ned". beacon_detector builds world_position from MAVROS
# ENU pose ([east, north, up]); if your REAL detector publishes ENU, set this to
# "enu" and north/east are swapped consistently everywhere. Default "ned" keeps
# the exact behaviour you've been testing in sim.
BEACON_POS_FRAME        = "ned"

# Derived: enough nodes to cover the search area approximately twice over.
# Formula: 2 × (area) / (area per step cell). Scaled up by 2× because RRT
# coverage is less efficient than a grid. Clamped to a minimum of 20.
RRT_MAX_NODES = max(20, int(2.0 * _SEARCH_AREA_M2 / RRT_STEP_M ** 2))

# ── Beacon verification ───────────────────────────────────────────────────────
BLINK_VERIFY_TIMEOUT_S  = 20.0  # max hover time to confirm blink_is_blinking
BLINK_VERIFY_POLL_S     = 0.25  # poll interval during verification hover
BLINK_SETTLE_S          = 3.0   # stabilisation pause before polling — lets the drone
                                 # stop moving and the YOLO tracker lock a stable ID

# ── Mission targets ───────────────────────────────────────────────────────────
# RobotX 2026 Task 1 (Safe Passage): the UAV classifies each buoy's TOP beacon
# into one of five STATES. Each state has a fixed meaning to report to the USV /
# RoboCommand. States are keyed on (color, is_blinking):
#   flashing RED   -> STARBOARD marker (pass on USV starboard side)
#   flashing GREEN -> PORT marker      (pass on USV port side)
#   flashing BLUE  -> ENTRY point
#   solid    BLUE  -> EXIT point
#   OFF            -> inactive obstacle buoy
STATE_STARBOARD = "red_flash"
STATE_PORT      = "green_flash"
STATE_ENTRY     = "blue_flash"
STATE_EXIT      = "blue_solid"
STATE_OBSTACLE  = "off"

STATE_MEANING = {
    STATE_STARBOARD: "FLASHING RED   -> pass on USV starboard side",
    STATE_PORT:      "FLASHING GREEN -> pass on USV port side",
    STATE_ENTRY:     "FLASHING BLUE  -> safe ENTRY point",
    STATE_EXIT:      "SOLID BLUE     -> safe EXIT point",
    STATE_OBSTACLE:  "OFF/BLACK      -> inactive obstacle buoy",
}

# Survey the whole field, then report. The field has a known buoy count.
EXPECTED_BUOY_COUNT = 1      # Task 1 has ten buoys
BUOY_MERGE_DIST_M   = 2.0     # detections within this distance = same buoy

# ── MAVSDK ────────────────────────────────────────────────────────────────────
MAVSDK_ADDRESS              = "udpin://0.0.0.0:14551"
BENCH_TEST                  = False
MAVSDK_CONNECT_TIMEOUT_S    = 10.0
PX4_HEARTBEAT_TIMEOUT_S     = 20.0

# ── Safety ────────────────────────────────────────────────────────────────────
#BATTERY_RTL_PERCENT  = 0.30   # trigger RTL when battery.remaining_percent < this (0.0–1.0)
DETECTOR_TIMEOUT_S   = 30.0   # abort pre-flight if no message from beacon detector within this time

# ─────────────────────────────────────────────────────────────────────────────


# ── Logging ───────────────────────────────────────────────────────────────────

def log(msg: str) -> None:
    print(f"[{time.strftime('%F %T')}] {msg}", file=sys.stderr, flush=True)


# ── Shared detection state ────────────────────────────────────────────────────

_classified_buoys: list = []    # confirmed buoys (one per beacon-map id):
                                #   {"id","state","north","east","world","gps",
                                #    "lat","lon","sigma_m","color","is_blinking","blink_hz"}
_beacon_map: dict       = {}    # beacon-map id -> latest beacon entry from
                                #   /seabird/beacon_map: {"id","ned","lat","lon",
                                #   "sigma_m","n","outliers","age_s","seen"}
_pending_det: dict      = {}    # FALLBACK source: color -> latest detection from
                                #   /seabird/beacon_detections (beacon_detector_config),
                                #   used only for beacons the fused map hasn't got yet
_classified_ids: set    = set() # beacon-map ids already classified & recorded
_overfly_attempts: dict = {}    # target key (map id, or ("det",n,e)) -> passes tried
_lock = threading.Lock()
_battery_low: bool    = False   # set True by monitor_battery() when threshold crossed
_detector_alive       = threading.Event()  # set when first /seabird/beacon_map msg arrives


# ── ROS2 publishers (set in listener thread) ──────────────────────────────────

_path_pub       = None
_path_msg       = None
_marker_pub     = None
_marker_msg     = None
_rrt_edge_pub   = None
_rrt_edge_msg   = None
_rrt_node_pub   = None
_rrt_node_msg   = None
_rclpy_node     = None


# ── ROS2 listener thread ──────────────────────────────────────────────────────

def _rclpy_thread_fn() -> None:
    global _path_pub, _path_msg, _marker_pub, _marker_msg
    global _rrt_edge_pub, _rrt_edge_msg, _rrt_node_pub, _rrt_node_msg, _rclpy_node

    try:
        rclpy.init()
        node = rclpy.create_node("rrt_listener")
        _rclpy_node = node

        # ── Flight path (same topics as sweep_lawnmower) ──────────────────
        _path_pub = node.create_publisher(Path, "/seabird/flight_path", 10)
        _path_msg = Path()
        _path_msg.header.frame_id = "map"

        _marker_pub = node.create_publisher(Marker, "/seabird/path_marker", 10)
        _marker_msg = Marker()
        _marker_msg.header.frame_id = "map"
        _marker_msg.ns    = "flight_path"
        _marker_msg.id    = 0
        _marker_msg.type  = Marker.LINE_STRIP
        _marker_msg.action = Marker.ADD
        _marker_msg.scale.x = 0.20
        _marker_msg.color = ColorRGBA(r=0.0, g=0.8, b=1.0, a=1.0)
        _marker_msg.pose.orientation.w = 1.0

        # ── RRT tree edges — LINE_LIST (each pair = one edge) ─────────────
        _rrt_edge_pub = node.create_publisher(Marker, "/seabird/rrt_tree", 10)
        _rrt_edge_msg = Marker()
        _rrt_edge_msg.header.frame_id = "map"
        _rrt_edge_msg.ns    = "rrt_edges"
        _rrt_edge_msg.id    = 1
        _rrt_edge_msg.type  = Marker.LINE_LIST
        _rrt_edge_msg.action = Marker.ADD
        _rrt_edge_msg.scale.x = 0.08
        _rrt_edge_msg.color = ColorRGBA(r=1.0, g=0.85, b=0.0, a=0.75)
        _rrt_edge_msg.pose.orientation.w = 1.0

        # ── RRT nodes — SPHERE_LIST ───────────────────────────────────────
        _rrt_node_pub = node.create_publisher(Marker, "/seabird/rrt_nodes", 10)
        _rrt_node_msg = Marker()
        _rrt_node_msg.header.frame_id = "map"
        _rrt_node_msg.ns    = "rrt_nodes"
        _rrt_node_msg.id    = 2
        _rrt_node_msg.type  = Marker.SPHERE_LIST
        _rrt_node_msg.action = Marker.ADD
        _rrt_node_msg.scale.x = 0.25
        _rrt_node_msg.scale.y = 0.25
        _rrt_node_msg.scale.z = 0.25
        _rrt_node_msg.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.90)
        _rrt_node_msg.pose.orientation.w = 1.0

        # ── Beacon-map subscription (positions from beacon_overlay, Stage 1) ──
        # /seabird/beacon_map carries the full fused map each message:
        #   {"beacons":[{"id","ned":[N,E,D],"lat","lon","sigma_m","n",
        #                "outliers","age_s","seen"}], ...}
        # ned is PX4 local NED — the SAME frame as MAVSDK position_velocity_ned,
        # so ned[0]=north, ned[1]=east drop straight into the search with no
        # conversion. We rebuild _beacon_map from each message so beacons the
        # map drops also drop here.
        def _cb(msg: String) -> None:
            _detector_alive.set()   # signal that the beacon map is live
            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                return
            beacons = data.get("beacons") or []
            with _lock:
                _beacon_map.clear()
                for b in beacons:
                    bid = b.get("id")
                    if bid is None:
                        continue
                    _beacon_map[bid] = b

        node.create_subscription(String, "/seabird/beacon_map", _cb, 10)

        # ── Fallback detections (beacon_detector_config, Stage 1 alt) ─────────
        # /seabird/beacon_detections is one detection per message, keyed by
        # colour, carrying a world_position (NED). Used only as a fallback to
        # surface a beacon the fused map hasn't registered yet. Colour here is
        # NOT trusted for the state — that is read overhead by the downward
        # classifier; we keep only the latest frame per colour for its position.
        def _cb_det(msg: String) -> None:
            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                return
            color = data.get("color", "")
            if not color or color == "unknown":
                return
            if beacon_ne(data.get("world_position")) is None:
                return   # no usable position — nothing to fly to
            with _lock:
                _pending_det[color] = data

        node.create_subscription(String, "/seabird/beacon_detections", _cb_det, 10)
        node.get_logger().info(
            "[rrt_listener] Subscribed /seabird/beacon_map + /seabird/beacon_detections  |  "
            "Publishing /seabird/flight_path  /seabird/rrt_tree  /seabird/rrt_nodes"
        )
        rclpy.spin(node)

    except Exception:
        log("[rrt_listener] FATAL: unhandled exception")
        traceback.print_exc()


def start_rclpy_listener() -> threading.Thread:
    t = threading.Thread(target=_rclpy_thread_fn, daemon=True, name="rclpy_listener")
    t.start()
    return t


# ── Drone position state ──────────────────────────────────────────────────────

class _DroneState:
    north_m: float = 0.0
    east_m:  float = 0.0
    down_m:  float = 0.0

_state = _DroneState()


async def track_position(drone: System) -> None:
    try:
        async for pv in drone.telemetry.position_velocity_ned():
            _state.north_m = pv.position.north_m
            _state.east_m  = pv.position.east_m
            _state.down_m  = pv.position.down_m
    except Exception:
        log("[rrt] ERROR: track_position() failed")
        traceback.print_exc()


# ── Path / RRT publishing helpers ─────────────────────────────────────────────

def publish_path_point() -> None:
    if _path_pub is None or _rclpy_node is None:
        return
    now = _rclpy_node.get_clock().now().to_msg()
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp    = now
    pose.pose.position.x = _state.north_m
    pose.pose.position.y = _state.east_m
    pose.pose.position.z = -_state.down_m
    _path_msg.header.stamp = now
    _path_msg.poses.append(pose)
    _path_pub.publish(_path_msg)
    pt = Point(x=_state.north_m, y=_state.east_m, z=-_state.down_m)
    _marker_msg.header.stamp = now
    _marker_msg.points.append(pt)
    _marker_pub.publish(_marker_msg)


def publish_rrt_edge(pn: float, pe: float, cn: float, ce: float, alt: float) -> None:
    """Append a LINE_LIST pair (parent → child) for the new RRT edge."""
    if _rrt_edge_pub is None or _rclpy_node is None:
        return
    now = _rclpy_node.get_clock().now().to_msg()
    _rrt_edge_msg.header.stamp = now
    _rrt_edge_msg.points.append(Point(x=pn, y=pe, z=alt))
    _rrt_edge_msg.points.append(Point(x=cn, y=ce, z=alt))
    _rrt_edge_pub.publish(_rrt_edge_msg)


def publish_rrt_node(north: float, east: float, alt: float) -> None:
    """Append a sphere at (north, east) to the SPHERE_LIST marker."""
    if _rrt_node_pub is None or _rclpy_node is None:
        return
    now = _rclpy_node.get_clock().now().to_msg()
    _rrt_node_msg.header.stamp = now
    _rrt_node_msg.points.append(Point(x=north, y=east, z=alt))
    _rrt_node_pub.publish(_rrt_node_msg)


# ── Flight helpers ────────────────────────────────────────────────────────────

async def wait_for_ready_position(drone: System) -> None:
    log("[rrt] Waiting for PX4 position readiness...")
    while True:
        try:
            health = await asyncio.wait_for(_get_one_health(drone), timeout=5.0)
            log(
                f"[rrt] health: global={health.is_global_position_ok} "
                f"home={health.is_home_position_ok} "
                f"local={health.is_local_position_ok}"
            )
            if health.is_local_position_ok and health.is_home_position_ok:
                log("[rrt] ✓ Position ready")
                return
        except Exception:
            log("[rrt] health check failed, retrying...")
        await asyncio.sleep(1.0)


async def _get_one_health(drone: System):
    async for health in drone.telemetry.health():
        return health


async def fly_to(drone: System, north: float, east: float, down: float, yaw: float,
                 tol: float = WAYPOINT_TOL_M,
                 timeout: float = WAYPOINT_TIMEOUT) -> bool:
    """
    Fly to (north, east, down, yaw) in NED.  Returns True on arrival, False on timeout.
    Continuously re-sends the setpoint to keep offboard mode alive.
    """
    target = PositionNedYaw(north, east, down, yaw)
    await drone.offboard.set_position_ned(target)

    t0 = asyncio.get_event_loop().time()
    last_pub_s   = 0.0
    last_log_s   = -1

    while True:
        await drone.offboard.set_position_ned(target)
        await asyncio.sleep(0.1)

        dist = math.sqrt(
            (_state.north_m - north) ** 2
            + (_state.east_m - east) ** 2
            + (_state.down_m - down) ** 2
        )
        if dist < tol:
            publish_path_point()
            return True

        elapsed = asyncio.get_event_loop().time() - t0
        if elapsed > timeout:
            log(f"  [fly_to] ⚠ Timeout ({timeout:.0f}s) at N={north:.1f} E={east:.1f}")
            return False

        if elapsed - last_pub_s >= 0.5:
            publish_path_point()
            last_pub_s = elapsed

        s = int(elapsed)
        if s % 5 == 0 and s > 0 and s != last_log_s:
            last_log_s = s
            log(f"  [fly_to] N={_state.north_m:.1f}/{north:.1f}  "
                f"E={_state.east_m:.1f}/{east:.1f}  dist={dist:.1f}m  t={elapsed:.0f}s")


async def hold_position(drone: System, north: float, east: float,
                        down: float, yaw: float, duration_s: float) -> None:
    """Hold a fixed setpoint for duration_s, keeping offboard alive."""
    target = PositionNedYaw(north, east, down, yaw)
    deadline = asyncio.get_event_loop().time() + duration_s
    while asyncio.get_event_loop().time() < deadline:
        await drone.offboard.set_position_ned(target)
        await asyncio.sleep(0.1)


# ── Detection helpers ─────────────────────────────────────────────────────────

def state_from_downward(result: dict) -> tuple:
    """
    Map the downward classifier's result to a Task-1 state. Returns
    (state, color, is_blinking); state is "" if the blink didn't resolve.
    The result's 'blink' is a bare bool/None (or, defensively, a dict).
    """
    color = (result.get("color") or "").lower()
    raw = result.get("blink")
    is_blinking = raw.get("is_blinking") if isinstance(raw, dict) else raw
    if is_blinking is None:
        return "", color, is_blinking
    if color == "red"   and is_blinking:     return STATE_STARBOARD, color, is_blinking
    if color == "green" and is_blinking:     return STATE_PORT,      color, is_blinking
    if color == "blue"  and is_blinking:     return STATE_ENTRY,     color, is_blinking
    if color == "blue"  and not is_blinking: return STATE_EXIT,      color, is_blinking
    return STATE_OBSTACLE, color, is_blinking


async def classify_state(drone: System, hover_north: float, hover_east: float,
                         hover_down: float) -> tuple:
    """
    Overhead colour/blink read (Stage 2): run the downward classifier (~8 s)
    while HOLDING position — streaming offboard setpoints the whole time so PX4
    doesn't drop offboard during the blocking read. Returns
    (state, color, is_blinking, blink_hz); state is "" if not definitive / on
    failure.
    """
    hold = PositionNedYaw(hover_north, hover_east, hover_down, 0.0)
    task = asyncio.create_task(
        asyncio.to_thread(classify_beacon_downward_once, cfg, duration=8.0, tracking_id=-1)
    )
    while not task.done():
        await drone.offboard.set_position_ned(hold)
        await asyncio.sleep(0.1)
    try:
        result = await task
    except Exception as e:                       # camera error / timeout / bad keys
        log(f"[rrt] ⚠ downward classification failed: {e}")
        return "", None, None, None
    state, color, is_blinking = state_from_downward(result)
    blink_hz = (result.get("blink") or {}).get("blink_hz") if isinstance(
        result.get("blink"), dict) else result.get("blink_hz")
    return state, color, is_blinking, blink_hz


def _near(n: float, e: float, buoy: dict) -> bool:
    dn = n - buoy["north"]; de = e - buoy["east"]
    return (dn * dn + de * de) <= (BUOY_MERGE_DIST_M * BUOY_MERGE_DIST_M)


def record_buoy(key, state: str, north: float, east: float, b: dict,
                color=None, is_blinking=None, blink_hz=None) -> bool:
    """
    Add or update a classified buoy. `key` is the target key (a beacon-map id, or
    a ("det", n, e) tuple for a fallback detection). Deduplicates by matching key
    OR by location (within BUOY_MERGE_DIST_M), so the map and the fallback
    detector can't double-count the same physical beacon. Position/GPS come from
    `b`; colour/blink come from the downward classifier. Returns True if NEW.
    """
    lat, lon = b.get("lat"), b.get("lon")
    entry = {
        "id":          key,
        "state":       state,
        "north":       north,
        "east":        east,
        "color":       color,
        "is_blinking": is_blinking,
        "blink_hz":    blink_hz,
        "world":       b.get("ned"),
        "lat":         lat,
        "lon":         lon,
        "sigma_m":     b.get("sigma_m"),
        "gps":         ({"latitude": lat, "longitude": lon} if lat is not None else None),
    }
    with _lock:
        for existing in _classified_buoys:
            if existing.get("id") == key or _near(north, east, existing):
                keep_id = existing.get("id")     # don't let a fallback overwrite a map id
                existing.update(entry)
                existing["id"] = keep_id
                return False
        _classified_buoys.append(entry)
        return True


def classified_count() -> int:
    with _lock:
        return len(_classified_buoys)

def have_entry_and_exit() -> bool:
    """True once at least one ENTRY and one EXIT buoy have been classified."""
    with _lock:
        states = {b["state"] for b in _classified_buoys}
    return STATE_ENTRY in states and STATE_EXIT in states

def survey_complete() -> bool:
    """
    Field survey is 'done enough' to report when we have located all expected
    buoys AND found both the entry and exit markers.
    """
    return classified_count() >= EXPECTED_BUOY_COUNT and have_entry_and_exit()

def beacon_ne(wp) -> Optional[tuple]:
    """
    Convert a detection's world_position to (north, east) per BEACON_POS_FRAME,
    or None if there is no usable world position.
      "ned" (default): wp is [north, east, down]  -> (wp[0], wp[1])
      "enu":           wp is [east, north, up]    -> (wp[1], wp[0])
    """
    if not wp or len(wp) < 2:
        return None
    try:
        if BEACON_POS_FRAME == "enu":
            return (float(wp[1]), float(wp[0]))
        return (float(wp[0]), float(wp[1]))
    except (TypeError, ValueError):
        return None


# ── Target selection + attempt tracking ───────────────────────────────────────
# Candidates come from two sources, both giving (north, east):
#   PRIMARY  — /seabird/beacon_map (fused, stable id, lat/lon, uncertainty)
#   FALLBACK — /seabird/beacon_detections, only for a beacon the map hasn't got
# A candidate is ("map"|"det", key, north, east, entry). Colour is always read
# overhead by the downward classifier, never taken from these.

def _classified_points():
    with _lock:
        return [(b["north"], b["east"]) for b in _classified_buoys]


def _near_any(n: float, e: float, points, dist: float) -> bool:
    d2 = dist * dist
    return any((n - pn) ** 2 + (e - pe) ** 2 <= d2 for pn, pe in points)


def _map_candidates(classified_pts):
    """Unclassified mapped beacons under the attempt budget."""
    out = []
    with _lock:
        items = list(_beacon_map.items())
        classified = set(_classified_ids)
        attempts = dict(_overfly_attempts)
    for bid, b in items:
        if bid in classified:
            continue
        if attempts.get(bid, 0) >= MAX_OVERFLY_ATTEMPTS:
            continue
        ne = beacon_ne(b.get("ned"))
        if ne is None:
            continue
        if _near_any(ne[0], ne[1], classified_pts, BUOY_MERGE_DIST_M):
            continue
        out.append(("map", bid, ne[0], ne[1], b))
    return out


def _det_candidates(map_pts, classified_pts):
    """
    Fallback detections not already covered by a mapped beacon (map is primary)
    and not already classified. Keyed by rounded location.
    """
    out = []
    with _lock:
        dets = list(_pending_det.items())
        attempts = dict(_overfly_attempts)
    for _color, d in dets:
        ne = beacon_ne(d.get("world_position"))
        if ne is None:
            continue
        n, e = ne
        if _near_any(n, e, map_pts, BUOY_MERGE_DIST_M):
            continue                              # the map already has this one
        if _near_any(n, e, classified_pts, BUOY_MERGE_DIST_M):
            continue
        key = ("det", round(n, 1), round(e, 1))
        if attempts.get(key, 0) >= MAX_OVERFLY_ATTEMPTS:
            continue
        out.append(("det", key, n, e, d))
    return out


def _all_candidates():
    classified_pts = _classified_points()
    mapc = _map_candidates(classified_pts)
    map_pts = [(c[2], c[3]) for c in mapc]
    detc = _det_candidates(map_pts, classified_pts)
    return mapc + detc


def unconfirmed_beacon_location(near_n: float, near_e: float):
    """
    (north, east) of the nearest unclassified beacon (map or fallback), for the
    RRT's reactive attraction so the search drifts toward it. None if none.
    """
    cands = _all_candidates()
    if not cands:
        return None
    c = min(cands, key=lambda c: (c[2] - near_n) ** 2 + (c[3] - near_e) ** 2)
    return (c[2], c[3])


def _note_overfly_attempt(key) -> None:
    """Record one more over-fly pass at target `key` (map id or det tuple)."""
    with _lock:
        _overfly_attempts[key] = _overfly_attempts.get(key, 0) + 1


def next_overfly_target(near_n: float, near_e: float):
    """
    Nearest unclassified beacon from either source, under MAX_OVERFLY_ATTEMPTS.
    Returns (source, key, north, east, entry) or None.
    """
    cands = _all_candidates()
    if not cands:
        return None
    return min(cands, key=lambda c: (c[2] - near_n) ** 2 + (c[3] - near_e) ** 2)


def beacon_map_entry(bid):
    """Latest beacon-map entry for an id (snapshot), or {}."""
    with _lock:
        return dict(_beacon_map.get(bid) or {})


def _buoy_public(b: dict) -> dict:
    """Trim an internal buoy dict to the fields worth reporting."""
    return {
        "id":          b.get("id"),
        "state":       b["state"],
        "meaning":     STATE_MEANING.get(b["state"], b["state"]),
        "north":       round(b["north"], 2),
        "east":        round(b["east"], 2),
        "world":       b.get("world"),
        "gps":         b.get("gps"),
        "lat":         b.get("lat"),
        "lon":         b.get("lon"),
        "sigma_m":     b.get("sigma_m"),
        "color":       b.get("color"),
        "is_blinking": b.get("is_blinking"),
        "blink_hz":    b.get("blink_hz"),
    }


def build_passage_report(nodes_explored: int) -> dict:
    """
    Assemble the structured Safe-Passage report from the classified buoys:
    the ENTRY point, the EXIT point, the port (green) and starboard (red)
    markers, and the OFF obstacles. This is what the USV / RoboCommand needs.
    If several ENTRY or EXIT candidates were seen, the highest-blink-confidence
    is not tracked here, so the FIRST is used and the rest listed as extras.
    """
    with _lock:
        buoys = [dict(b) for b in _classified_buoys]

    def of(state): return [b for b in buoys if b["state"] == state]

    entries   = of(STATE_ENTRY)
    exits     = of(STATE_EXIT)
    starboard = of(STATE_STARBOARD)
    port      = of(STATE_PORT)
    obstacles = of(STATE_OBSTACLE)

    return {
        "task":       "safe_passage",
        "tier":       "advanced",
        "buoy_count": len(buoys),
        "expected":   EXPECTED_BUOY_COUNT,
        "entry":      _buoy_public(entries[0]) if entries else None,
        "exit":       _buoy_public(exits[0])   if exits   else None,
        "entry_extras": [_buoy_public(b) for b in entries[1:]],
        "exit_extras":  [_buoy_public(b) for b in exits[1:]],
        "starboard":  [_buoy_public(b) for b in starboard],
        "port":       [_buoy_public(b) for b in port],
        "obstacles":  [_buoy_public(b) for b in obstacles],
        "all_buoys":  [_buoy_public(b) for b in buoys],
        "nodes_explored": nodes_explored,
        "timestamp":  time.time(),
    }


def _fmt_pt(b) -> str:
    """One-line human string for a reported buoy point."""
    if not b:
        return "NOT FOUND"
    gps = b.get("gps")
    if gps:
        return (f"N={b['north']:+.2f} E={b['east']:+.2f}  "
                f"lat={gps['latitude']:.7f} lon={gps['longitude']:.7f}")
    return f"N={b['north']:+.2f} E={b['east']:+.2f}  gps=n/a"


# ── RRT class ─────────────────────────────────────────────────────────────────

class RRTNode:
    """Single node in the RRT tree."""
    __slots__ = ("north", "east", "parent")

    def __init__(self, north: float, east: float, parent: Optional["RRTNode"] = None):
        self.north  = north
        self.east   = east
        self.parent = parent


class RRT:
    """
    2-D Rapidly-exploring Random Tree constrained to the search area defined by
    SEARCH_AREA_SHAPE (a W x L box, default, or a disc) anchored to the takeoff
    position.

    Goal bias: with probability RRT_GOAL_BIAS the sample is nudged toward the
    least-explored direction (opposite of the current node centroid) so the tree
    spreads outward rather than clustering near the start.
    """

    def __init__(self, start_north: float, start_east: float) -> None:
        self._origin_north = start_north
        self._origin_east  = start_east
        # Rect bounds (always computed; ignored in disc mode).
        self._n_min, self._n_max, self._e_min, self._e_max = \
            _search_bounds(start_north, start_east)
        self.root  = RRTNode(start_north, start_east)
        self.nodes = [self.root]

    # ── Area helpers ────────────────────────────────────────────────────────────

    def _uniform_sample(self) -> tuple:
        """Uniform distribution over the search area (box or disc)."""
        if SEARCH_AREA_SHAPE == "rect":
            return (random.uniform(self._n_min, self._n_max),
                    random.uniform(self._e_min, self._e_max))
        angle = random.uniform(0.0, 2.0 * math.pi)
        r     = math.sqrt(random.random()) * MAX_SEARCH_RADIUS_M
        return (self._origin_north + r * math.cos(angle),
                self._origin_east  + r * math.sin(angle))

    def clamp(self, north: float, east: float) -> tuple:
        """Clamp a point into the search area (box clamp, or project onto disc)."""
        if SEARCH_AREA_SHAPE == "rect":
            return (min(max(north, self._n_min), self._n_max),
                    min(max(east,  self._e_min), self._e_max))
        dn = north - self._origin_north
        de = east  - self._origin_east
        d  = math.hypot(dn, de)
        if d <= MAX_SEARCH_RADIUS_M or d == 0.0:
            return north, east
        s = MAX_SEARCH_RADIUS_M / d
        return self._origin_north + dn * s, self._origin_east + de * s

    # ── Sampling ──────────────────────────────────────────────────────────────

    def _biased_sample(self) -> tuple:
        """
        Biased toward the direction opposite the current node centroid so the
        tree fills the search area rather than staying near the start.
        """
        # Option A — reactive beacon attraction:
        # if the detector has glimpsed an unconfirmed beacon, bias toward it so
        # the drone drifts over to confirm it (suppressed once confirmed).
        if random.random() < BEACON_ATTRACT_PROB:
            # use the newest tree tip as the "current" position reference
            cur = self.nodes[-1]
            loc = unconfirmed_beacon_location(cur.north, cur.east)
            if loc is not None:
                log(f"[rrt] ◉ biasing search toward unconfirmed beacon at "
                    f"N={loc[0]:+.1f} E={loc[1]:+.1f}")
                return (loc[0] + random.gauss(0.0, BEACON_ATTRACT_JITTER_M),
                        loc[1] + random.gauss(0.0, BEACON_ATTRACT_JITTER_M))

        if len(self.nodes) > 3 and random.random() < RRT_GOAL_BIAS:
            # centroid of current tree nodes
            cn = sum(n.north for n in self.nodes) / len(self.nodes)
            ce = sum(n.east  for n in self.nodes) / len(self.nodes)
            # direction from centroid away from origin → unexplored sector
            dn = self._origin_north - cn
            de = self._origin_east  - ce
            mag = math.sqrt(dn ** 2 + de ** 2) or 1.0
            reach = 0.45 * _SEARCH_SPAN_M
            target_n = self._origin_north + (dn / mag) * reach
            target_e = self._origin_east  + (de / mag) * reach
            # Gaussian jitter so repeated biased samples aren't identical
            target_n += random.gauss(0.0, 0.10 * _SEARCH_SPAN_M)
            target_e += random.gauss(0.0, 0.10 * _SEARCH_SPAN_M)
            return self.clamp(target_n, target_e)   # keep it inside the area
        return self._uniform_sample()

    # ── Core RRT operations ───────────────────────────────────────────────────

    def nearest(self, north: float, east: float) -> RRTNode:
        """Return the existing node closest to (north, east)."""
        return min(
            self.nodes,
            key=lambda n: (n.north - north) ** 2 + (n.east - east) ** 2,
        )

    def steer(self, from_node: RRTNode,
              target_n: float, target_e: float) -> tuple:
        """
        Step at most RRT_STEP_M from from_node toward (target_n, target_e).
        Returns the new (north, east) position.
        """
        dn   = target_n - from_node.north
        de   = target_e - from_node.east
        dist = math.sqrt(dn ** 2 + de ** 2)
        if dist <= RRT_STEP_M:
            return target_n, target_e
        scale = RRT_STEP_M / dist
        return from_node.north + dn * scale, from_node.east + de * scale

    def in_bounds(self, north: float, east: float) -> bool:
        """True if (north, east) lies inside the search area."""
        if SEARCH_AREA_SHAPE == "rect":
            return (self._n_min <= north <= self._n_max and
                    self._e_min <= east  <= self._e_max)
        dn = north - self._origin_north
        de = east  - self._origin_east
        return math.sqrt(dn ** 2 + de ** 2) <= MAX_SEARCH_RADIUS_M

    def add_node(self, north: float, east: float, parent: RRTNode) -> RRTNode:
        node = RRTNode(north, east, parent)
        self.nodes.append(node)
        return node

    def extend(self) -> tuple:
        """
        Sample, find nearest, steer.
        Returns (new_north, new_east, parent_node).
        The caller must call add_node() if the node is accepted.
        """
        q_rn, q_re = self._biased_sample()
        q_near      = self.nearest(q_rn, q_re)
        q_nn, q_ne  = self.steer(q_near, q_rn, q_re)
        return q_nn, q_ne, q_near

    @property
    def size(self) -> int:
        return len(self.nodes)


# ── Beacon over-fly flight ────────────────────────────────────────────────────

def _clamp_to_bounds(rrt: "RRT", north: float, east: float) -> tuple:
    """Clamp a point into the search area, so a beacon just past the boundary is
    still approached up to the edge. Delegates to the area-aware RRT.clamp()."""
    return rrt.clamp(north, east)


async def fly_over_beacon(drone: System, rrt: RRT,
                          bn: float, be: float, down_m: float) -> bool:
    """
    Grow the tree from its nearest node to the beacon at (bn, be) and fly there,
    one RRT_STEP_M leg at a time, so each leg end becomes a real RRT node and the
    drone finishes directly over the beacon. Returns True once within
    BEACON_OVERFLY_TOL_M, False if a leg times out or the budget is exhausted.
    """
    # Optional standoff so a forward/angled camera doesn't lose the beacon.
    gn, ge = bn, be
    if BEACON_OVERFLY_OFFSET_M > 0.0:
        adn = bn - _state.north_m
        ade = be - _state.east_m
        amag = math.hypot(adn, ade)
        if amag > 1e-6:
            gn = bn - (adn / amag) * BEACON_OVERFLY_OFFSET_M
            ge = be - (ade / amag) * BEACON_OVERFLY_OFFSET_M
    gn, ge = _clamp_to_bounds(rrt, gn, ge)

    for _ in range(BEACON_MAX_LEGS):
        if rrt.size >= RRT_MAX_NODES:
            return False
        node = rrt.nearest(gn, ge)
        if math.hypot(gn - node.north, ge - node.east) <= BEACON_OVERFLY_TOL_M:
            return True
        nn, ne = rrt.steer(node, gn, ge)
        yaw = math.degrees(math.atan2(ne - _state.east_m,
                                      nn - _state.north_m)) % 360.0
        log(f"[rrt]   ◎→ leg toward beacon: N={nn:+.2f} E={ne:+.2f} "
            f"(beacon N={bn:+.2f} E={be:+.2f})")
        reached = await fly_to(drone, nn, ne, down_m, yaw)
        if not reached:
            return False
        node_new = rrt.add_node(nn, ne, node)
        publish_rrt_node(nn, ne, TAKEOFF_ALT_M)
        publish_rrt_edge(node.north, node.east, nn, ne, TAKEOFF_ALT_M)
    return False


# ── Detection processing (called once overhead a mapped beacon) ───────────────

async def classify_and_record(drone: System, target, down_m: float) -> None:
    """
    The drone is directly over the beacon named by `target`
    (source, key, north, east, entry). Run the downward classifier (Stage 2) for
    colour+blink, then record the buoy — position/GPS from that beacon's entry
    (the beacon-map's NED+lat/lon, or the fallback detection's world_position +
    gps_position), colour/blink from the classifier. Marks it classified so the
    over-fly branch won't revisit it.
    """
    source, key, bn, be, extra = target
    label = f"B{key}" if source == "map" else "det-beacon"
    log(f"[rrt] ★ Overhead {label} at N={bn:+.2f} E={be:+.2f} "
        f"— reading colour/blink from the downward camera")

    state, color, is_blinking, blink_hz = await classify_state(drone, bn, be, down_m)
    if not state:
        log(f"[rrt]   … {label} state unconfirmed (blink unresolved), will retry")
        return   # not marked classified — a later pass can try again

    # Position/GPS source: the fused map entry (by id) if this is a map beacon,
    # else the fallback detection's own world_position + gps_position.
    if source == "map":
        b = beacon_map_entry(key)
    else:
        d = extra
        gps = d.get("gps_position") or {}
        b = {"ned": d.get("world_position"),
             "lat": gps.get("latitude"), "lon": gps.get("longitude"),
             "sigma_m": None}

    ne = beacon_ne(b.get("ned"))
    rn, re = ne if ne is not None else (bn, be)

    is_new = record_buoy(key, state, rn, re, b,
                         color=color, is_blinking=is_blinking, blink_hz=blink_hz)
    if source == "map":
        with _lock:
            _classified_ids.add(key)

    lat, lon = b.get("lat"), b.get("lon")
    gps_s = (f"lat={lat:.7f} lon={lon:.7f}" if lat is not None else "gps=n/a")
    tag = "NEW" if is_new else "upd"
    log(f"[rrt]   [{tag}] {label} {STATE_MEANING[state]}  "
        f"N={rn:+.2f} E={re:+.2f}  {gps_s}  "
        f"buoys={classified_count()}/{EXPECTED_BUOY_COUNT}")


# ── Safety monitors ──────────────────────────────────────────────────────────

#async def monitor_battery(drone: System) -> None:
    #"""Background coroutine — sets _battery_low and logs when battery drops below threshold."""
    #global _battery_low
    #try:
        #async for battery in drone.telemetry.battery():
            #if not _battery_low and battery.remaining_percent < BATTERY_RTL_PERCENT:
                #_battery_low = True
                #log(f"[rrt] ⚡ BATTERY LOW: {battery.remaining_percent * 100:.0f}% "
                    #f"(threshold {BATTERY_RTL_PERCENT * 100:.0f}%) — will RTL")
    #except Exception:
        #log("[rrt] ERROR: monitor_battery() failed")
        #traceback.print_exc()


#async def wait_for_detector() -> bool:
    #"""
    #Wait up to DETECTOR_TIMEOUT_S for at least one message on /seabird/beacon_detections.
    #Returns True if the detector is confirmed live, False on timeout.
    #"""
    #log(f"[rrt] Waiting for beacon detector on /seabird/beacon_detections "
        #f"({DETECTOR_TIMEOUT_S:.0f}s timeout)...")
    #t0 = asyncio.get_event_loop().time()
    #while asyncio.get_event_loop().time() - t0 < DETECTOR_TIMEOUT_S:
        #if _detector_alive.is_set():
            #log("[rrt] ✓ Beacon detector is publishing")
            #return True
        #await asyncio.sleep(0.5)
    #log("[rrt] ERROR: No message on /seabird/beacon_detections — "
        #"is beacon_detector_config.py running?")
   # return False


# ── Mission ───────────────────────────────────────────────────────────────────

async def run_mission() -> None:
    log("[rrt] run_mission() entered")
    log(f"[rrt] MAVSDK_ADDRESS={MAVSDK_ADDRESS}")

    drone = System()

    # ── Connect ───────────────────────────────────────────────────────────────
    try:
        log(f"[rrt] Connecting to {MAVSDK_ADDRESS}...")
        await asyncio.wait_for(
            drone.connect(system_address=MAVSDK_ADDRESS),
            timeout=MAVSDK_CONNECT_TIMEOUT_S,
        )
        log("[rrt] drone.connect() returned")
    except asyncio.TimeoutError:
        log(f"[rrt] ERROR: connect timed out after {MAVSDK_CONNECT_TIMEOUT_S:.0f}s")
        return
    except Exception:
        log("[rrt] ERROR: exception during connect()")
        traceback.print_exc()
        return

    async def _wait_connected() -> bool:
        async for state in drone.core.connection_state():
            log(f"[rrt] connection_state: is_connected={state.is_connected}")
            if state.is_connected:
                log("[rrt] ✓ Connected to PX4")
                return True
        return False

    try:
        ok = await asyncio.wait_for(_wait_connected(), timeout=PX4_HEARTBEAT_TIMEOUT_S)
        if not ok:
            log("[rrt] ERROR: connection_state stream ended before PX4 connected")
            return
    except asyncio.TimeoutError:
        log(f"[rrt] ERROR: PX4 heartbeat timed out after {PX4_HEARTBEAT_TIMEOUT_S:.0f}s")
        log(f"[rrt] Verify MAVLink is arriving on {MAVSDK_ADDRESS}")
        return
    except Exception:
        log("[rrt] ERROR: exception while waiting for PX4")
        traceback.print_exc()
        return

    if BENCH_TEST:
        log("*** BENCH_TEST — connection verified, not arming or taking off ***")
        return

    await wait_for_ready_position(drone)

    asyncio.ensure_future(track_position(drone))
    #asyncio.ensure_future(monitor_battery(drone))
    await asyncio.sleep(0.5)
    log(f"[rrt] Start position: N={_state.north_m:.2f}  E={_state.east_m:.2f}  "
        f"D={_state.down_m:.2f}")

    # ── Detector pre-flight check ─────────────────────────────────────────────
    #if not await wait_for_detector():
        #log("[rrt] ABORT: beacon detector not detected — not arming")
        #return

    # ── Arm ───────────────────────────────────────────────────────────────────
    log("[rrt] Arming...")
    try:
        await drone.action.arm()
    except ActionError as e:
        log(f"[rrt] Arm failed: {e}")
        return
    except Exception:
        log("[rrt] ERROR: arm() exception")
        traceback.print_exc()
        return
    log("[rrt] ✓ Armed")

    # ── Takeoff ───────────────────────────────────────────────────────────────
    log(f"[rrt] Taking off to {TAKEOFF_ALT_M}m...")
    try:
        await drone.action.set_takeoff_altitude(TAKEOFF_ALT_M)
        await drone.action.takeoff()
    except Exception:
        log("[rrt] ERROR: takeoff failed")
        traceback.print_exc()
        return

    log(f"[rrt] Stabilising for {HOVER_STABILIZE_S:.0f}s...")
    await asyncio.sleep(HOVER_STABILIZE_S)
    log(f"[rrt] ✓ Airborne — N={_state.north_m:.1f}  E={_state.east_m:.1f}  "
        f"Alt≈{-_state.down_m:.1f}m")

    # ── Start offboard mode ───────────────────────────────────────────────────
    log("[rrt] Priming offboard setpoint stream (3 s)...")
    hold = PositionNedYaw(_state.north_m, _state.east_m, -TAKEOFF_ALT_M, 0.0)
    try:
        for _ in range(30):                     # 30 × 0.1s = 3 s of priming
            await drone.offboard.set_position_ned(hold)
            await asyncio.sleep(0.1)

    
        await drone.offboard.set_position_ned(hold)
        log("[rrt] Switching to offboard mode...")
        try:
            await drone.offboard.start()
        except OffboardError as e:
            log(f"[rrt] First offboard start failed ({e}), retrying...")
            await drone.offboard.set_position_ned(hold)
            await asyncio.sleep(0.2)
            await drone.offboard.set_position_ned(hold)
            await drone.offboard.start()
    except OffboardError as e:
        log(f"[rrt] Offboard start failed: {e}")
        try:
            await drone.action.land()
        except Exception:
            pass
        return
    except Exception:
        log("[rrt] ERROR: offboard start exception")
        traceback.print_exc()
        try:
            await drone.action.land()
        except Exception:
            pass
        return
    log("[rrt] ✓ Offboard active")

    # ── RRT search ────────────────────────────────────────────────────────────
    origin_n = _state.north_m
    origin_e = _state.east_m
    down_m   = -TAKEOFF_ALT_M

    rrt = RRT(origin_n, origin_e)
    publish_rrt_node(origin_n, origin_e, TAKEOFF_ALT_M)

    log("")
    log("[rrt] ═══════════════════════════════════════════")
    log("[rrt]   RRT BEACON SEARCH")
    log("[rrt] ───────────────────────────────────────────")
    log(f"[rrt]   Origin          N={origin_n:+.2f}  E={origin_e:+.2f}")
    if SEARCH_AREA_SHAPE == "rect":
        log(f"[rrt]   Search area     {SEARCH_WIDTH_M:.0f} x {SEARCH_LENGTH_M:.0f} m "
            f"box ({SEARCH_ORIGIN})  N[{rrt._n_min:+.1f},{rrt._n_max:+.1f}] "
            f"E[{rrt._e_min:+.1f},{rrt._e_max:+.1f}]")
    else:
        log(f"[rrt]   Search area     disc, radius {MAX_SEARCH_RADIUS_M:.1f} m")
    log(f"[rrt]   Step length     {RRT_STEP_M:.1f} m")
    log(f"[rrt]   Max nodes       {RRT_MAX_NODES}")
    log(f"[rrt]   Verify timeout  {BLINK_VERIFY_TIMEOUT_S:.0f} s")
    log(f"[rrt]   Expected buoys  {EXPECTED_BUOY_COUNT}")
    log(f"[rrt]   Beacon overfly  {'ON' if BEACON_OVERFLY else 'OFF'} "
        f"(tol {BEACON_OVERFLY_TOL_M:.1f} m, frame {BEACON_POS_FRAME})")
    log("[rrt] ═══════════════════════════════════════════")
    log("")

    try:
        while rrt.size < RRT_MAX_NODES:

            # ── Battery failsafe ──────────────────────────────────────────────
            if _battery_low:
                log("[rrt] ⚡ Battery low — stopping search and returning to launch")
                break

            # ── Priority: fly directly over an unclassified beacon ────────────
            # Positions come from /seabird/beacon_map (primary, fused) or
            # /seabird/beacon_detections (fallback, for beacons the map hasn't got).
            # Divert and fly right over the nearest one (inserting the legs + beacon
            # as RRT nodes), then read its colour/blink from the downward camera
            # directly overhead.
            if BEACON_OVERFLY:
                ov = next_overfly_target(_state.north_m, _state.east_m)
                if ov is not None:
                    source, key, bn, be, _extra = ov
                    label = f"B{key}" if source == "map" else "det-beacon"
                    log(f"[rrt] ◎ {label} at N={bn:+.2f} E={be:+.2f} "
                        f"— flying directly over it")
                    arrived = await fly_over_beacon(drone, rrt, bn, be, down_m)
                    _note_overfly_attempt(key)
                    if arrived:
                        log("[rrt]   ✓ Overhead beacon — reading state")
                        await classify_and_record(drone, ov, down_m)
                    else:
                        log("[rrt]   ⚠ Could not reach the beacon overhead this pass")
                    if survey_complete():
                        log(f"[rrt] ✓ Survey complete: all {EXPECTED_BUOY_COUNT} buoys "
                            f"classified, entry & exit found — ending search to report.")
                        break
                    await asyncio.sleep(0.3)
                    continue

            # ── Extend the RRT ────────────────────────────────────────────────
            new_n, new_e, parent_node = rrt.extend()

            if not rrt.in_bounds(new_n, new_e):
                continue

            # Yaw faces direction of travel
            dn  = new_n - _state.north_m
            de  = new_e - _state.east_m
            yaw = math.degrees(math.atan2(de, dn)) % 360.0

            log(f"[rrt] → node {rrt.size + 1:3d}  "
                f"N={new_n:+.2f}  E={new_e:+.2f}  yaw={yaw:.0f}°  "
                f"buoys={classified_count()}/{EXPECTED_BUOY_COUNT}")

            reached = await fly_to(drone, new_n, new_e, down_m, yaw)

            if not reached:
                log("[rrt]   ⚠ Waypoint skipped (timeout)")
                continue

            # Node accepted — add to tree and update RViz
            new_node = rrt.add_node(new_n, new_e, parent_node)
            publish_rrt_node(new_n, new_e, TAKEOFF_ALT_M)
            publish_rrt_edge(parent_node.north, parent_node.east,
                             new_n, new_e, TAKEOFF_ALT_M)

            # Beacons are classified only from directly overhead, by the over-fly
            # branch at the top of the loop (positions come from /seabird/beacon_map).
            # There is no in-place read at an ordinary node any more — colour/blink
            # is a downward-camera read that is only valid over the beacon.

            # ── Stop early once the whole field is surveyed and entry+exit found ──
            if survey_complete():
                log(f"[rrt] ✓ Survey complete: all {EXPECTED_BUOY_COUNT} buoys classified, "
                    f"entry & exit found — ending search to report.")
                break


            # Brief pause before next extension
            await asyncio.sleep(0.3)

        log(f"[rrt] Search complete — {rrt.size} nodes explored, "
            f"{classified_count()} buoys classified")

    except Exception:
        # Any UNHANDLED error in the search loop lands here instead of killing
        # the process mid-air. Log it, then fall through to `finally` so the
        # drone still comes home on OUR command — not on PX4's offboard-loss
        # failsafe (which, with COM_OBL_RC_ACT=0, would just hand it to a pilot).
        log("[rrt] FATAL during search — forcing RTL")
        traceback.print_exc()

    finally:
        # Runs on EVERY way out of the search — normal finish, early break, OR a
        # crash caught above — so return-to-launch is guaranteed to be attempted.
        log("\n[rrt] Returning to launch...")
        try:
            await drone.offboard.stop()
        except Exception:
            pass
        try:
            await drone.action.return_to_launch()
        except Exception:
            # RTL itself failed (e.g. mode rejected) — fall back to a plain land
            # so the drone doesn't just hang on its last setpoint.
            log("[rrt] ERROR: return_to_launch() failed — trying land()")
            traceback.print_exc()
            try:
                await drone.action.land()
            except Exception:
                pass

    log("[rrt] Waiting for landing...")
    try:
        async for in_air in drone.telemetry.in_air():
            if not in_air:
                log("[rrt] ✓ Landed")
                break
            await asyncio.sleep(1.0)
    except Exception:
        log("[rrt] ERROR: exception while waiting for landing")
        traceback.print_exc()

    # ── Build and emit the Safe-Passage report ──────────────────────────────
    report = build_passage_report(rrt.size)
    log("")
    log("═" * 60)
    log("  SEABIRD — SAFE PASSAGE REPORT (RobotX 2026 Task 1)")
    log("═" * 60)
    log(f"  Nodes explored : {rrt.size}")
    log(f"  Buoys found    : {report['buoy_count']} / {EXPECTED_BUOY_COUNT}")
    entry = report["entry"]; exit_ = report["exit"]
    log(f"  ENTRY (flash blue) : {_fmt_pt(entry)}")
    log(f"  EXIT  (solid blue) : {_fmt_pt(exit_)}")
    log(f"  STARBOARD (flash red)  markers: {len(report['starboard'])}")
    for b in report["starboard"]:
        log(f"      {_fmt_pt(b)}")
    log(f"  PORT      (flash green) markers: {len(report['port'])}")
    for b in report["port"]:
        log(f"      {_fmt_pt(b)}")
    log(f"  OBSTACLES (off) : {len(report['obstacles'])}")
    log("═" * 60)

    # Machine-readable report for RoboCommand / the USV.
    # TODO(team): publish this dict (or your RoboCommand schema) on the
    #             agreed topic / link. It is emitted here as JSON for now.
    log("[rrt] REPORT_JSON " + json.dumps(report))
    log("")



# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    try:
        log("=" * 60)
        log("  SEABIRD — RRT Beacon Search")
        log("=" * 60)
        log(f"  MAVSDK_ADDRESS      = {MAVSDK_ADDRESS}")
        log(f"  MAX_SEARCH_RADIUS_M = {MAX_SEARCH_RADIUS_M} m")
        log(f"  RRT_STEP_M          = {RRT_STEP_M} m")
        log(f"  RRT_MAX_NODES       = {RRT_MAX_NODES}")
        log(f"  BENCH_TEST          = {BENCH_TEST}")
        log("=" * 60)
        log("")

        start_rclpy_listener()
        log("[rrt] ROS2 detection listener started")

        asyncio.run(run_mission())

    except KeyboardInterrupt:
        log("[rrt] Interrupted by user")
    except Exception:
        log("[rrt] FATAL: unhandled exception")
        traceback.print_exc()
        raise
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
