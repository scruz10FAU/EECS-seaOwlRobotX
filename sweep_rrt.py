#!/usr/bin/env python3
"""
sweep_rrt.py
============
Beacon search using Rapidly-exploring Random Trees (RRT).

The drone grows an RRT from its takeoff position outward to MAX_SEARCH_RADIUS_M.
At each tree extension the drone flies to the new node and checks whether any
new beacon colors have appeared on /seabird/beacon_detections.  When a new color
is seen the drone hovers in place until blink_is_blinking becomes True/False
(or BLINK_VERIFY_TIMEOUT_S elapses), then resumes the search.  Multiple beacons
of the same color may exist; the tree always grows to RRT_MAX_NODES before RTL.

Safety features:
  - Battery failsafe: triggers RTL when remaining < BATTERY_RTL_PERCENT.
  - Detector pre-flight check: aborts if /seabird/beacon_detections is silent
    for DETECTOR_TIMEOUT_S seconds before arming.

Start beacon_detector_config.py before this script so detections arrive on
/seabird/beacon_detections.

Three RViz topics are published:
  /seabird/flight_path    — nav_msgs/Path of visited positions
  /seabird/path_marker    — LINE_STRIP of the same path
  /seabird/rrt_tree       — LINE_LIST of every RRT edge
  /seabird/rrt_nodes      — SPHERE_LIST of every RRT node

Physical drone notes (same as sweep_lawnmower.py):
  - MAVSDK must receive PX4 MAVLink on MAVSDK_ADDRESS.
  - Start beacon_detector_config.py before this script so detections arrive
    on /seabird/beacon_detections.
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
MAX_SEARCH_RADIUS_M  = 10.0   # maximum distance from takeoff point; hard boundary

# ── RRT parameters ────────────────────────────────────────────────────────────
RRT_STEP_M           = 1.5    # maximum edge length for each tree extension
RRT_GOAL_BIAS        = 0.08   # probability of biasing sample toward least-explored sector

# Derived: enough nodes to cover the search disc approximately twice over.
# Formula: 2 × (area of disc) / (area per step cell).
# Scaled up by 2× because RRT coverage is less efficient than a grid.
# Clamped to a minimum of 20 so tiny radii still get a meaningful search.
RRT_MAX_NODES = max(20, int(2.0 * math.pi * MAX_SEARCH_RADIUS_M ** 2 / RRT_STEP_M ** 2))

# ── Beacon verification ───────────────────────────────────────────────────────
BLINK_VERIFY_TIMEOUT_S  = 15.0  # max hover time to confirm blink_is_blinking
BLINK_VERIFY_POLL_S     = 0.25  # poll interval during verification hover

# ── Mission targets ───────────────────────────────────────────────────────────
TARGET_COLORS        = {"red", "green", "blue"}

# ── MAVSDK ────────────────────────────────────────────────────────────────────
MAVSDK_ADDRESS              = "udpin://0.0.0.0:14551"
BENCH_TEST                  = False
MAVSDK_CONNECT_TIMEOUT_S    = 10.0
PX4_HEARTBEAT_TIMEOUT_S     = 20.0

# ── Safety ────────────────────────────────────────────────────────────────────
BATTERY_RTL_PERCENT  = 0.30   # trigger RTL when battery.remaining_percent < this (0.0–1.0)
DETECTOR_TIMEOUT_S   = 30.0   # abort pre-flight if no message from beacon detector within this time

# ─────────────────────────────────────────────────────────────────────────────


# ── Logging ───────────────────────────────────────────────────────────────────

def log(msg: str) -> None:
    print(f"[{time.strftime('%F %T')}] {msg}", file=sys.stderr, flush=True)


# ── Shared detection state ────────────────────────────────────────────────────

_detected_buoys: set  = set()   # colors whose blink status has been recorded
_pending_beacon: dict = {}      # color -> latest detection dict (from ROS cb)
_beacon_locations: dict = {}    # color -> (north, east) where first detected
_lock = threading.Lock()
_battery_low: bool    = False   # set True by monitor_battery() when threshold crossed
_detector_alive       = threading.Event()  # set when first /seabird/beacon_detections msg arrives


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

        # ── Detection subscription ────────────────────────────────────────
        def _cb(msg: String) -> None:
            _detector_alive.set()   # signal pre-flight check that detector is live
            try:
                data = json.loads(msg.data)
            except json.JSONDecodeError:
                return
            color = data.get("color", "")
            if not color or color == "unknown":
                return
            with _lock:
                if color not in _detected_buoys:
                    _pending_beacon[color] = data   # always update with latest frame

        node.create_subscription(String, "/seabird/beacon_detections", _cb, 10)
        node.get_logger().info(
            "[rrt_listener] Subscribed /seabird/beacon_detections  |  "
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

def all_found() -> bool:
    with _lock:
        return _detected_buoys >= TARGET_COLORS

def found_count() -> int:
    with _lock:
        return len(_detected_buoys)

def found_set() -> set:
    with _lock:
        return set(_detected_buoys)

def new_unverified_colors() -> list:
    """Return colors currently pending verification at this node."""
    with _lock:
        return list(_pending_beacon.keys())


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
    2-D Rapidly-exploring Random Tree constrained to a circle of
    MAX_SEARCH_RADIUS_M metres centred on the takeoff position.

    Goal bias: with probability RRT_GOAL_BIAS the sample is nudged toward the
    least-explored sector (opposite of the current node centroid) so the tree
    spreads outward rather than clustering near the start.
    """

    def __init__(self, start_north: float, start_east: float) -> None:
        self._origin_north = start_north
        self._origin_east  = start_east
        self.root  = RRTNode(start_north, start_east)
        self.nodes = [self.root]

    # ── Sampling ──────────────────────────────────────────────────────────────

    def _uniform_in_circle(self) -> tuple:
        """Uniform distribution over the search disc."""
        angle = random.uniform(0.0, 2.0 * math.pi)
        r     = math.sqrt(random.random()) * MAX_SEARCH_RADIUS_M
        return (
            self._origin_north + r * math.cos(angle),
            self._origin_east  + r * math.sin(angle),
        )

    def _biased_sample(self) -> tuple:
        """
        Biased toward the sector opposite the current node centroid so the tree
        fills the search area rather than staying near the start.
        """
        if len(self.nodes) > 3 and random.random() < RRT_GOAL_BIAS:
            # centroid of current tree nodes
            cn = sum(n.north for n in self.nodes) / len(self.nodes)
            ce = sum(n.east  for n in self.nodes) / len(self.nodes)
            # direction from centroid away from origin → unexplored sector
            dn = self._origin_north - cn
            de = self._origin_east  - ce
            mag = math.sqrt(dn ** 2 + de ** 2) or 1.0
            target_n = self._origin_north + (dn / mag) * MAX_SEARCH_RADIUS_M * 0.85
            target_e = self._origin_east  + (de / mag) * MAX_SEARCH_RADIUS_M * 0.85
            # Gaussian jitter so repeated biased samples aren't identical
            target_n += random.gauss(0.0, MAX_SEARCH_RADIUS_M * 0.15)
            target_e += random.gauss(0.0, MAX_SEARCH_RADIUS_M * 0.15)
            return target_n, target_e
        return self._uniform_in_circle()

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
        """True if (north, east) lies within MAX_SEARCH_RADIUS_M of origin."""
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


# ── Beacon verification ───────────────────────────────────────────────────────

async def verify_beacon(drone: System,
                        hover_north: float, hover_east: float,
                        hover_down: float, color: str) -> Optional[dict]:
    """
    Hover at the current position and wait until the blink detector reports
    a definitive blink_is_blinking value (True or False, not None) for `color`,
    or until BLINK_VERIFY_TIMEOUT_S elapses.

    Returns the latest detection dict (may have blink_is_blinking=None on
    timeout), or None if no detection was ever received for this color.
    """
    log(f"[rrt] ◉ Verifying blink for '{color}' — hovering up to {BLINK_VERIFY_TIMEOUT_S:.0f}s")

    # Try to face toward the beacon using its reported camera-frame offset.
    yaw = _bearing_to_beacon(color)
    target = PositionNedYaw(hover_north, hover_east, hover_down, yaw)

    t0 = asyncio.get_event_loop().time()
    while asyncio.get_event_loop().time() - t0 < BLINK_VERIFY_TIMEOUT_S:
        await drone.offboard.set_position_ned(target)
        await asyncio.sleep(BLINK_VERIFY_POLL_S)

        with _lock:
            det = _pending_beacon.get(color)

        if det is not None and (det.get("blink") or {}).get("is_blinking") is not None:
            blink_info = det.get("blink") or {}
            log(f"[rrt] ✓ '{color}' blink confirmed: "
                f"is_blinking={blink_info.get('is_blinking')}  "
                f"hz={blink_info.get('blink_hz', '?')}")
            return det

    elapsed = asyncio.get_event_loop().time() - t0
    log(f"[rrt] ⚠ Blink verification timed out for '{color}' after {elapsed:.1f}s")
    with _lock:
        return _pending_beacon.get(color)


def _bearing_to_beacon(color: str) -> float:
    """
    Rough yaw angle toward the beacon from the latest pos3d_x / pos3d_y reading.
    pos3d is in camera/body frame; treating x as lateral and y as forward gives
    a useful facing direction for a forward-mounted camera.
    Returns 0.0 (north) if no position data is available.
    """
    with _lock:
        det = _pending_beacon.get(color, {})
    try:
        pos3d = det.get("position_3d")
        if not pos3d or len(pos3d) < 3:
            return 0.0
        bx = float(pos3d[0])   # x = lateral in camera frame
        bz = float(pos3d[2])   # z = forward in camera frame
        if abs(bx) < 0.01 and abs(bz) < 0.01:
            return 0.0
        return math.degrees(math.atan2(bx, bz)) % 360.0
    except (TypeError, ValueError, IndexError):
        return 0.0


# ── Safety monitors ──────────────────────────────────────────────────────────

async def monitor_battery(drone: System) -> None:
    """Background coroutine — sets _battery_low and logs when battery drops below threshold."""
    global _battery_low
    try:
        async for battery in drone.telemetry.battery():
            if not _battery_low and battery.remaining_percent < BATTERY_RTL_PERCENT:
                _battery_low = True
                log(f"[rrt] ⚡ BATTERY LOW: {battery.remaining_percent * 100:.0f}% "
                    f"(threshold {BATTERY_RTL_PERCENT * 100:.0f}%) — will RTL")
    except Exception:
        log("[rrt] ERROR: monitor_battery() failed")
        traceback.print_exc()


async def wait_for_detector() -> bool:
    """
    Wait up to DETECTOR_TIMEOUT_S for at least one message on /seabird/beacon_detections.
    Returns True if the detector is confirmed live, False on timeout.
    """
    log(f"[rrt] Waiting for beacon detector on /seabird/beacon_detections "
        f"({DETECTOR_TIMEOUT_S:.0f}s timeout)...")
    t0 = asyncio.get_event_loop().time()
    while asyncio.get_event_loop().time() - t0 < DETECTOR_TIMEOUT_S:
        if _detector_alive.is_set():
            log("[rrt] ✓ Beacon detector is publishing")
            return True
        await asyncio.sleep(0.5)
    log("[rrt] ERROR: No message on /seabird/beacon_detections — "
        "is beacon_detector_config.py running?")
    return False


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
    asyncio.ensure_future(monitor_battery(drone))
    await asyncio.sleep(0.5)
    log(f"[rrt] Start position: N={_state.north_m:.2f}  E={_state.east_m:.2f}  "
        f"D={_state.down_m:.2f}")

    # ── Detector pre-flight check ─────────────────────────────────────────────
    if not await wait_for_detector():
        log("[rrt] ABORT: beacon detector not detected — not arming")
        return

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
        log("[rrt] Switching to offboard mode...")
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
    log(f"[rrt]   Max radius      {MAX_SEARCH_RADIUS_M:.1f} m")
    log(f"[rrt]   Step length     {RRT_STEP_M:.1f} m")
    log(f"[rrt]   Max nodes       {RRT_MAX_NODES}")
    log(f"[rrt]   Verify timeout  {BLINK_VERIFY_TIMEOUT_S:.0f} s")
    log(f"[rrt]   Targets         {TARGET_COLORS}")
    log("[rrt] ═══════════════════════════════════════════")
    log("")

    while rrt.size < RRT_MAX_NODES:

        # ── Battery failsafe ──────────────────────────────────────────────
        if _battery_low:
            log("[rrt] ⚡ Battery low — stopping search and returning to launch")
            break

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
            f"detections={found_count()}")

        reached = await fly_to(drone, new_n, new_e, down_m, yaw)

        if not reached:
            log("[rrt]   ⚠ Waypoint skipped (timeout)")
            continue

        # Node accepted — add to tree and update RViz
        new_node = rrt.add_node(new_n, new_e, parent_node)
        publish_rrt_node(new_n, new_e, TAKEOFF_ALT_M)
        publish_rrt_edge(parent_node.north, parent_node.east,
                         new_n, new_e, TAKEOFF_ALT_M)

        # ── Check for new beacon detections ───────────────────────────────
        unverified = new_unverified_colors()

        for color in unverified:
            # Record where this beacon was first spotted
            with _lock:
                if color not in _beacon_locations:
                    _beacon_locations[color] = (new_n, new_e)

            log(f"[rrt] ★ New beacon: '{color}'  at N={new_n:+.2f} E={new_e:+.2f}")

            # Hover and wait for definitive blink status
            det = await verify_beacon(drone, new_n, new_e, down_m, color)

            # Record result and clear pending so the same color can retrigger
            # at a new location (multiple beacons of the same color may exist).
            with _lock:
                _detected_buoys.add(color)
                _pending_beacon.pop(color, None)

            blink_info   = (det.get("blink") or {}) if det else {}
            blink_status = blink_info.get("is_blinking")
            blink_hz     = blink_info.get("blink_hz")
            log(f"[rrt]   Recorded '{color}': "
                f"is_blinking={blink_status}  hz={blink_hz}  "
                f"detections so far={found_count()}")

        # Brief pause before next extension
        await asyncio.sleep(0.3)

    log(f"[rrt] Search complete — {rrt.size} nodes explored, "
        f"{found_count()} beacon detection(s) recorded")

    # ── Return to launch ──────────────────────────────────────────────────────
    log("\n[rrt] Returning to launch...")
    try:
        await drone.offboard.stop()
    except Exception:
        pass

    try:
        await drone.action.return_to_launch()
    except Exception:
        log("[rrt] ERROR: return_to_launch() failed")
        traceback.print_exc()
        return

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

    # ── Final report ──────────────────────────────────────────────────────────
    log("")
    log("═" * 52)
    log("  SEABIRD RRT SEARCH — COMPLETE")
    log("═" * 52)
    log(f"  RRT nodes explored  : {rrt.size}")
    log(f"  Search radius       : {MAX_SEARCH_RADIUS_M:.1f} m")
    log(f"  Total detections    : {found_count()}")
    log(f"  Unique colors seen  : {sorted(found_set()) or 'none'}")
    for color, (fn, fe) in _beacon_locations.items():
        log(f"    {color:<8} first seen N={fn:+.2f}  E={fe:+.2f}")
    log("═" * 52)
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
