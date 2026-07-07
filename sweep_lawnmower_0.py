#!/usr/bin/env python3
"""
sweep_lawnmower.py
==================
Flies a boustrophedon (lawnmower) pattern over the course area.
Rows run east-west, advancing northward one row at a time.
Yaw tracks direction of travel (90° going east, 270° going west).
Stops early once all target buoy colors are detected.

Reads detections from /seabird/buoy_detections via rclpy in a background thread.
Publishes flight path on /seabird/flight_path (nav_msgs/Path) for RViz2.
Also publishes /seabird/path_markers (visualization_msgs/Marker) as a LINE_STRIP.

Prerequisites (all should already be running before this script):
  1. Isaac Sim + spawn_drone.py completed ("[init] Done")
  2. PX4 SITL connected ("Ready for takeoff!")
  3. buoy_detector.py running in another terminal

PX4 params to set once (in pxh, then param save):
  param set MPC_XY_CRUISE 2.0
  param set MPC_XY_VEL_MAX 3.0
  param set MPC_Z_VEL_MAX_UP 1.5
  param set MPC_Z_VEL_MAX_DN 1.0
  param set SYS_HAS_MAG 0
  param set COM_ARM_MAG_STR 0
  param set EKF2_ABL_LIM 5.0
  param save
"""

import asyncio
import math
import json
import threading
import sys

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
    print("[ERROR] mavsdk not installed. Run: pip install mavsdk --break-system-packages")
    sys.exit(1)


# ── Mission configuration ─────────────────────────────────────────────────────
TAKEOFF_ALT_M     = 5.0
WAYPOINT_TOL_M    = 2.5
WAYPOINT_TIMEOUT  = 60.0
HOVER_STABILIZE_S = 8.0

# Search box (metres from spawn origin, NED frame)
# 5ft x 5ft area: 5ft = 1.524m, centred at the spawn origin (±0.762m each axis)
LAWN_ROW_SPACING_M = 0.5     # gap between parallel E-W rows
LAWN_NORTH_M       =  0.762  # northern edge of search area
LAWN_SOUTH_M       = -0.762  # southern edge
LAWN_EAST_M        =  0.762  # eastern edge
LAWN_WEST_M        = -0.762  # western edge

TARGET_COLORS  = {"red", "green", "blue"}
MAVSDK_ADDRESS = "udp://:14540"

# ── Shared state ──────────────────────────────────────────────────────────────
_detected_buoys: set = set()
_lock = threading.Lock()

_path_pub   = None
_path_msg   = None
_marker_pub = None
_marker_msg = None
_rclpy_node = None


# ── ROS2 listener thread ──────────────────────────────────────────────────────

def _rclpy_thread_fn():
    global _path_pub, _path_msg, _marker_pub, _marker_msg, _rclpy_node
    rclpy.init()
    node = rclpy.create_node("sweep_listener")
    _rclpy_node = node

    _path_pub = node.create_publisher(Path, "/seabird/flight_path", 10)
    _path_msg = Path()
    _path_msg.header.frame_id = "map"

    _marker_pub = node.create_publisher(Marker, "/seabird/path_marker", 10)
    _marker_msg = Marker()
    _marker_msg.header.frame_id = "map"
    _marker_msg.ns = "flight_path"
    _marker_msg.id = 0
    _marker_msg.type = Marker.LINE_STRIP
    _marker_msg.action = Marker.ADD
    _marker_msg.scale.x = 0.3
    _marker_msg.color = ColorRGBA(r=0.0, g=1.0, b=0.5, a=1.0)
    _marker_msg.pose.orientation.w = 1.0

    def _cb(msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        color = data.get("color", "")
        if not color:
            return
        with _lock:
            if color not in _detected_buoys:
                _detected_buoys.add(color)
                node.get_logger().info(
                    f"[sweep_listener] ★ NEW: {color}   "
                    f"total={len(_detected_buoys)}/{len(TARGET_COLORS)}"
                )

    node.create_subscription(String, "/seabird/buoy_detections", _cb, 10)
    node.get_logger().info("[sweep_listener] Subscribed to /seabird/buoy_detections")
    node.get_logger().info("[sweep_listener] Publishing: /seabird/flight_path, /seabird/path_marker")
    rclpy.spin(node)


def start_rclpy_listener() -> threading.Thread:
    t = threading.Thread(target=_rclpy_thread_fn, daemon=True, name="rclpy_listener")
    t.start()
    return t


# ── Lawnmower waypoint generator ──────────────────────────────────────────────

def generate_lawnmower_waypoints(alt):
    """
    Boustrophedon pattern. Rows run E-W, advancing north by LAWN_ROW_SPACING_M.
    Each row contributes two waypoints: the near end and the far end.
    Yaw tracks direction of travel so the camera faces forward on each leg.
      even rows (0, 2, …): west → east  yaw=90°
      odd  rows (1, 3, …): east → west  yaw=270°
    """
    row_northings = []
    n = LAWN_SOUTH_M
    while n <= LAWN_NORTH_M + 0.01:
        row_northings.append(round(n, 2))
        n += LAWN_ROW_SPACING_M

    d = -alt
    wps = []
    for i, row_n in enumerate(row_northings):
        if i % 2 == 0:              # west → east
            wps.append((row_n, LAWN_WEST_M, d, 90.0))
            wps.append((row_n, LAWN_EAST_M, d, 90.0))
        else:                       # east → west
            wps.append((row_n, LAWN_EAST_M, d, 270.0))
            wps.append((row_n, LAWN_WEST_M, d, 270.0))
    return wps


# ── Drone state tracker ───────────────────────────────────────────────────────

class _DroneState:
    north_m: float = 0.0
    east_m:  float = 0.0
    down_m:  float = 0.0

_state = _DroneState()


async def track_position(drone: System):
    async for pv in drone.telemetry.position_velocity_ned():
        _state.north_m = pv.position.north_m
        _state.east_m  = pv.position.east_m
        _state.down_m  = pv.position.down_m


# ── Path publishing ───────────────────────────────────────────────────────────

def publish_path_point():
    if _path_pub is None or _rclpy_node is None:
        return

    now = _rclpy_node.get_clock().now().to_msg()

    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = now
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


# ── Flight helpers ────────────────────────────────────────────────────────────

async def fly_to(drone, north, east, down, yaw,
                 tol=WAYPOINT_TOL_M, timeout=WAYPOINT_TIMEOUT):
    await drone.offboard.set_position_ned(PositionNedYaw(north, east, down, yaw))
    t0 = asyncio.get_event_loop().time()
    last_pub = 0

    while True:
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
            print(f"  [fly_to] ⚠ Timeout ({timeout:.0f}s) at N={north:.1f} E={east:.1f}")
            return False

        if elapsed - last_pub >= 0.5:
            publish_path_point()
            last_pub = elapsed

        if int(elapsed) % 5 == 0 and int(elapsed) > 0:
            print(
                f"  [fly_to] → N={_state.north_m:.1f}/{north:.1f}  "
                f"E={_state.east_m:.1f}/{east:.1f}  "
                f"dist={dist:.1f}m  t={elapsed:.0f}s"
            )


def all_found():
    with _lock:
        return _detected_buoys >= TARGET_COLORS

def found_count():
    with _lock:
        return len(_detected_buoys)

def found_set():
    with _lock:
        return set(_detected_buoys)


# ── Mission ───────────────────────────────────────────────────────────────────

async def run_mission():

    drone = System()
    print(f"[sweep] Connecting to PX4 at {MAVSDK_ADDRESS}...")
    await drone.connect(system_address=MAVSDK_ADDRESS)

    print("[sweep] Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[sweep] ✓ Connected to PX4")
            break

    print("[sweep] Waiting for GPS fix and home position...")
    async for health in drone.telemetry.health():
        gps_ok  = health.is_global_position_ok
        home_ok = health.is_home_position_ok
        if gps_ok and home_ok:
            print("[sweep] ✓ GPS OK, home set")
            break
        if not gps_ok:
            print("  [sweep] ... waiting for global position estimate")
        await asyncio.sleep(1.0)

    asyncio.ensure_future(track_position(drone))
    await asyncio.sleep(0.5)
    print(f"[sweep] Drone at N={_state.north_m:.2f} E={_state.east_m:.2f} "
          f"D={_state.down_m:.2f}")

    print("[sweep] Arming...")
    try:
        await drone.action.arm()
    except ActionError as e:
        print(f"[sweep] Arm failed: {e} — is PX4 ready?")
        return
    print("[sweep] ✓ Armed")

    print(f"[sweep] Taking off to {TAKEOFF_ALT_M}m...")
    await drone.action.set_takeoff_altitude(TAKEOFF_ALT_M)
    await drone.action.takeoff()
    print(f"[sweep] Waiting {HOVER_STABILIZE_S:.0f}s to stabilize at altitude...")
    await asyncio.sleep(HOVER_STABILIZE_S)
    print(f"[sweep] ✓ Airborne — N={_state.north_m:.1f} E={_state.east_m:.1f} "
          f"Alt≈{-_state.down_m:.1f}m")

    print("[sweep] Switching to offboard mode...")
    await drone.offboard.set_position_ned(
        PositionNedYaw(_state.north_m, _state.east_m, -TAKEOFF_ALT_M, 90.0)
    )
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"[sweep] Offboard start failed: {e}")
        await drone.action.land()
        return
    print("[sweep] ✓ Offboard active")

    waypoints = generate_lawnmower_waypoints(alt=TAKEOFF_ALT_M)
    total_wps = len(waypoints)
    n_rows = total_wps // 2

    print(f"\n[sweep] Lawnmower: {n_rows} rows × 2 ends = {total_wps} waypoints")
    print(f"[sweep] Area: N={LAWN_SOUTH_M}→{LAWN_NORTH_M}m  "
          f"E={LAWN_WEST_M}→{LAWN_EAST_M}m  "
          f"Row spacing={LAWN_ROW_SPACING_M}m  Alt={TAKEOFF_ALT_M}m")
    print(f"[sweep] Target: {TARGET_COLORS}")
    print("[sweep] ─────────────────────────────────────")

    for i, (n, e, d, yaw) in enumerate(waypoints):
        row = i // 2 + 1
        leg = "start" if i % 2 == 0 else "end"
        direction = "→E" if yaw == 90.0 else "←W"

        if all_found():
            print(f"\n[sweep] ★★★ All {len(TARGET_COLORS)} buoys found! "
                  f"Exiting at WP {i+1}/{total_wps} (row {row})")
            break

        print(
            f"[sweep] WP {i+1:2d}/{total_wps}  row={row} {leg} {direction}  "
            f"N={n:+6.1f}  E={e:+6.1f}  "
            f"found={found_count()}/{len(TARGET_COLORS)} {found_set()}"
        )

        reached = await fly_to(drone, n, e, d, yaw)
        if not reached:
            print(f"  [sweep] Skipping to next waypoint")

        await asyncio.sleep(1.5)

    print("\n[sweep] Returning to launch...")
    try:
        await drone.offboard.stop()
    except Exception:
        pass
    await drone.action.return_to_launch()

    print("[sweep] Waiting for landing...")
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("[sweep] ✓ Landed")
            break
        await asyncio.sleep(1.0)

    found   = found_set()
    missing = TARGET_COLORS - found
    print("\n" + "═" * 50)
    print("  SEABIRD LAWNMOWER SWEEP COMPLETE")
    print("═" * 50)
    print(f"  Buoys detected : {sorted(found) or 'none'}")
    if missing:
        print(f"  NOT found      : {sorted(missing)}")
    else:
        print("  ✓ All target buoys accounted for")
    print("═" * 50 + "\n")


def main():
    print("=" * 60)
    print("  SEABIRD — Lawnmower Sweep")
    print("=" * 60)
    print("  Make sure these are already running:")
    print("  1. Isaac Sim + spawn_drone.py ('[init] Done')")
    print("  2. PX4 SITL ('Ready for takeoff!')")
    print("  3. python3 buoy_detector.py")
    print("=" * 60 + "\n")

    start_rclpy_listener()
    print("[sweep] ROS2 detection listener started\n")

    asyncio.run(run_mission())


if __name__ == "__main__":
    main()
