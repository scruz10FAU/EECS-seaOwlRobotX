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

Physical drone notes:
  - This script uses MAVSDK_ADDRESS below.
  - MAVSDK must receive PX4 MAVLink packets on that UDP port.
  - If another process, such as seabird_rc_trigger.py, already owns the same
    port, this script will fail to connect.
  - If MAVSDK binds successfully but no MAVLink heartbeat arrives, this script
    will now timeout and print useful diagnostics instead of waiting forever.

PX4 params to verify:
  param show COM_RC_OVERRIDE   # should be non-zero for RC override/takeover
"""

import asyncio
import math
import json
import threading
import sys
import time
import traceback

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
    print("[ERROR] mavsdk not installed. Run: pip install mavsdk --break-system-packages", flush=True)
    sys.exit(1)


# ── Logging / debug helpers ──────────────────────────────────────────────────

def log(msg):
    """Timestamped, flushed logging so output appears in launcher logs immediately."""
    #print(f"[{time.strftime('%F %T')}] {msg}", flush=True)
    print(f"[{time.strftime('%F %T')}] {msg}", file=sys.stderr, flush=True)


# ── Mission configuration ─────────────────────────────────────────────────────
TAKEOFF_ALT_M     = 5.0
WAYPOINT_TOL_M    = 0.5
WAYPOINT_TIMEOUT  = 60.0
HOVER_STABILIZE_S = 8.0

# Search box (metres from spawn origin, NED frame)
# 5ft x 5ft area: 5ft = 1.524m, centred at the spawn origin (±0.762m each axis)
LAWN_ROW_SPACING_M = 0.75     # gap between parallel E-W rows
LAWN_NORTH_M       =  2.0  # northern edge of search area
LAWN_SOUTH_M       = -2.0  # southern edge
LAWN_EAST_M        =  2.0  # eastern edge
LAWN_WEST_M        = -2.0  # western edge

TARGET_COLORS  = {"red", "green", "blue"}

# Use the port that actually receives MAVLink from VOXL/PX4.
# If seabird_rc_trigger.py owns 14551, either stop it, move it, or configure
# voxl-mavlink-server to provide a second MAVLink stream to another port.
#MAVSDK_ADDRESS = "udp://:14540"
MAVSDK_ADDRESS = "udpin://0.0.0.0:14551"

# Bench test skips GPS/home-position wait. Do NOT treat this as flightworthy.
BENCH_TEST = False

# Connection debug timeouts
MAVSDK_CONNECT_TIMEOUT_S = 10.0
PX4_HEARTBEAT_TIMEOUT_S = 20.0


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

    try:
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

    except Exception:
        log("[sweep_listener] FATAL: unhandled exception in ROS2 listener thread")
        traceback.print_exc()


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
    try:
        async for pv in drone.telemetry.position_velocity_ned():
            _state.north_m = pv.position.north_m
            _state.east_m  = pv.position.east_m
            _state.down_m  = pv.position.down_m
    except Exception:
        log("[sweep] ERROR: track_position() failed")
        traceback.print_exc()


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

async def wait_for_ready_position(drone: System):
    """
    Wait until PX4 reports enough position readiness for autonomous takeoff/offboard.
    This is required for real flight.
    """
    log("[sweep] Waiting for PX4 position readiness...")

    while True:
        try:
            health = await asyncio.wait_for(get_one_health(drone), timeout=5.0)

            log(
                f"[sweep] health: "
                f"global_position_ok={health.is_global_position_ok}, "
                f"home_position_ok={health.is_home_position_ok}, "
                f"local_position_ok={health.is_local_position_ok}"
            )

            if health.is_local_position_ok and health.is_home_position_ok:
                log("[sweep] ✓ Local position and home position ready")
                return

            if not health.is_local_position_ok:
                log("[sweep] waiting: local position not valid")

            if not health.is_home_position_ok:
                log("[sweep] waiting: home position not set")

        except Exception:
            log("[sweep] Could not read health while waiting for position")
            traceback.print_exc()

        await asyncio.sleep(1.0)


async def get_one_health(drone: System):
    async for health in drone.telemetry.health():
        return health

async def fly_to(drone, north, east, down, yaw,
                 tol=WAYPOINT_TOL_M, timeout=WAYPOINT_TIMEOUT):
    """
    await drone.offboard.set_position_ned(PositionNedYaw(north, east, down, yaw))
    t0 = asyncio.get_event_loop().time()
    last_pub = 0
    last_status_second = -1

    while True:
        await asyncio.sleep(0.1)
    """
    target_setpoint = PositionNedYaw(north, east, down, yaw)
    await drone.offboard.set_position_ned(target_setpoint)

    t0 = asyncio.get_event_loop().time()
    last_pub = 0
    last_status_second = -1

    while True:
        # Keep Offboard alive by continuously sending the active setpoint.
        await drone.offboard.set_position_ned(target_setpoint)
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

        if elapsed - last_pub >= 0.5:
            publish_path_point()
            last_pub = elapsed

        current_second = int(elapsed)
        if current_second % 5 == 0 and current_second > 0 and current_second != last_status_second:
            last_status_second = current_second
            log(
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

async def slow_yaw_turn(drone, north, east, down, start_yaw, end_yaw, duration_s=4.0):
    """
    Slowly rotate yaw while holding position.
    This slows only the turn/yaw behavior, not the straight-line flight speed.
    """
    steps = int(duration_s / 0.1)

    # Pick shortest yaw direction
    yaw_delta = ((end_yaw - start_yaw + 540.0) % 360.0) - 180.0

    for i in range(steps + 1):
        frac = i / steps
        yaw = (start_yaw + yaw_delta * frac) % 360.0

        await drone.offboard.set_position_ned(
            PositionNedYaw(north, east, down, yaw)
        )
        await asyncio.sleep(0.1)


# ── Mission ───────────────────────────────────────────────────────────────────

async def run_mission():
    log("[sweep] run_mission() entered")
    log(f"[sweep] MAVSDK_ADDRESS={MAVSDK_ADDRESS}")

    drone = System()

    try:
        log(f"[sweep] Calling drone.connect({MAVSDK_ADDRESS})...")
        await asyncio.wait_for(
            drone.connect(system_address=MAVSDK_ADDRESS),
            timeout=MAVSDK_CONNECT_TIMEOUT_S
        )
        log("[sweep] drone.connect() returned")
    except asyncio.TimeoutError:
        log(f"[sweep] ERROR: drone.connect() timed out after {MAVSDK_CONNECT_TIMEOUT_S:.0f} seconds")
        log("[sweep] Check whether the MAVSDK UDP port is already in use or blocked.")
        return
    except Exception:
        log("[sweep] ERROR: exception during drone.connect()")
        traceback.print_exc()
        return

    log("[sweep] Waiting for PX4 connection heartbeat...")

    async def wait_for_px4_connection():
        async for state in drone.core.connection_state():
            log(f"[sweep] connection_state: is_connected={state.is_connected}")
            if state.is_connected:
                log("[sweep] Connected to PX4")
                return True

        return False

    try:
        connected = await asyncio.wait_for(
            wait_for_px4_connection(),
            timeout=PX4_HEARTBEAT_TIMEOUT_S
        )

        if not connected:
            log("[sweep] ERROR: connection_state stream ended before PX4 connected")
            return

    except asyncio.TimeoutError:
        log(f"[sweep] ERROR: Timed out after {PX4_HEARTBEAT_TIMEOUT_S:.0f}s waiting for PX4 heartbeat")
        log("[sweep] MAVSDK is listening, but no MAVLink packets are arriving on this UDP port.")
        log(f"[sweep] Verify with: timeout 5 tcpdump -ni lo udp port {MAVSDK_ADDRESS.rsplit(':', 1)[-1]}")
        return

    except Exception:
        log("[sweep] ERROR: exception while waiting for PX4 connection")
        traceback.print_exc()
        return

    log("[sweep] BENCH_TEST checkpoint reached")
    log(f"[sweep] BENCH_TEST={BENCH_TEST}")

    if BENCH_TEST:
        log("*** BENCH_TEST MODE - connection test only ***")
        log("*** BENCH_TEST MODE - not arming, not taking off, not starting offboard ***")
        log("[sweep] MAVSDK connection, PX4 heartbeat, and ROS2 listener verified.")
        return

    await wait_for_ready_position(drone)

    asyncio.ensure_future(track_position(drone))
    await asyncio.sleep(0.5)
    log(f"[sweep] Drone at N={_state.north_m:.2f} E={_state.east_m:.2f} "
        f"D={_state.down_m:.2f}")

    log("[sweep] Arming...")
    try:
        await drone.action.arm()
    except ActionError as e:
        log(f"[sweep] Arm failed: {e} — is PX4 ready?")
        return
    except Exception:
        log("[sweep] ERROR: unexpected exception during arm()")
        traceback.print_exc()
        return
    log("[sweep] ✓ Armed")

    log(f"[sweep] Taking off to {TAKEOFF_ALT_M}m...")
    try:
        await drone.action.set_takeoff_altitude(TAKEOFF_ALT_M)
        await drone.action.takeoff()
    except Exception:
        log("[sweep] ERROR: takeoff failed")
        traceback.print_exc()
        return

    log(f"[sweep] Waiting {HOVER_STABILIZE_S:.0f}s to stabilize at altitude...")
    await asyncio.sleep(HOVER_STABILIZE_S)
    log(f"[sweep] ✓ Airborne — N={_state.north_m:.1f} E={_state.east_m:.1f} "
        f"Alt≈{-_state.down_m:.1f}m")
    """
    log("[sweep] Switching to offboard mode...")
    try:
        await drone.offboard.set_position_ned(
            PositionNedYaw(_state.north_m, _state.east_m, -TAKEOFF_ALT_M, 90.0)
        )
        await drone.offboard.start()
    """
    log("[sweep] Starting offboard setpoint stream before mode switch...")

    hold_setpoint = PositionNedYaw(
        _state.north_m,
        _state.east_m,
        -TAKEOFF_ALT_M,
        90.0
    )

    try:
        # PX4 needs to see offboard setpoints BEFORE accepting Offboard mode.
        # Send a steady hold-position stream first.
        for _ in range(30):  # 30 x 0.1s = 3 seconds
            await drone.offboard.set_position_ned(hold_setpoint)
            await asyncio.sleep(0.1)

        log("[sweep] Switching to offboard mode...")
        await drone.offboard.start()

    except OffboardError as e:
        log(f"[sweep] Offboard start failed: {e}")
        try:
            await drone.action.land()
        except Exception:
            log("[sweep] ERROR: landing command after offboard failure also failed")
            traceback.print_exc()
        return
    except Exception:
        log("[sweep] ERROR: unexpected exception while starting offboard")
        traceback.print_exc()
        try:
            await drone.action.land()
        except Exception:
            pass
        return
    log("[sweep] ✓ Offboard active")

    waypoints = generate_lawnmower_waypoints(alt=TAKEOFF_ALT_M)
    total_wps = len(waypoints)
    n_rows = total_wps // 2

    log(f"\n[sweep] Lawnmower: {n_rows} rows × 2 ends = {total_wps} waypoints")
    log(f"[sweep] Area: N={LAWN_SOUTH_M}→{LAWN_NORTH_M}m  "
        f"E={LAWN_WEST_M}→{LAWN_EAST_M}m  "
        f"Row spacing={LAWN_ROW_SPACING_M}m  Alt={TAKEOFF_ALT_M}m")
    log(f"[sweep] Target: {TARGET_COLORS}")
    log("[sweep] ─────────────────────────────────────")

    for i, (n, e, d, yaw) in enumerate(waypoints):
        row = i // 2 + 1
        leg = "start" if i % 2 == 0 else "end"
        direction = "→E" if yaw == 90.0 else "←W"

        if all_found():
            log(f"\n[sweep] ★★★ All {len(TARGET_COLORS)} buoys found! "
                f"Exiting at WP {i+1}/{total_wps} (row {row})")
            break

        log(
            f"[sweep] WP {i+1:2d}/{total_wps}  row={row} {leg} {direction}  "
            f"N={n:+6.1f}  E={e:+6.1f}  "
            f"found={found_count()}/{len(TARGET_COLORS)} {found_set()}"
        )
        """
        reached = await fly_to(drone, n, e, d, yaw)
        if not reached:
            log("  [sweep] Skipping to next waypoint")

        await asyncio.sleep(1.5)
        """
        reached = await fly_to(drone, n, e, d, yaw)
        if not reached:
            log("  [sweep] Skipping to next waypoint")

        # If the next waypoint has a different yaw, slow down only the turn.
        if i + 1 < total_wps:
            next_n, next_e, next_d, next_yaw = waypoints[i + 1]

            if abs(((next_yaw - yaw + 540.0) % 360.0) - 180.0) > 45.0:
                log(f"  [sweep] Slow yaw turn: {yaw:.0f}° → {next_yaw:.0f}°")
                await slow_yaw_turn(
                    drone,
                    _state.north_m,
                    _state.east_m,
                    d,
                    yaw,
                    next_yaw,
                    duration_s=4.0
                )
            else:
                await asyncio.sleep(1.0)
        else:
            await asyncio.sleep(1.0)

    log("\n[sweep] Returning to launch...")
    try:
        await drone.offboard.stop()
    except Exception:
        pass

    try:
        await drone.action.return_to_launch()
    except Exception:
        log("[sweep] ERROR: return_to_launch() failed")
        traceback.print_exc()
        return

    log("[sweep] Waiting for landing...")
    try:
        async for in_air in drone.telemetry.in_air():
            if not in_air:
                log("[sweep] ✓ Landed")
                break
            await asyncio.sleep(1.0)
    except Exception:
        log("[sweep] ERROR: exception while waiting for landing")
        traceback.print_exc()

    found   = found_set()
    missing = TARGET_COLORS - found
    log("\n" + "═" * 50)
    log("  SEABIRD LAWNMOWER SWEEP COMPLETE")
    log("═" * 50)
    log(f"  Buoys detected : {sorted(found) or 'none'}")
    if missing:
        log(f"  NOT found      : {sorted(missing)}")
    else:
        log("  ✓ All target buoys accounted for")
    log("═" * 50 + "\n")


def main():
    try:
        log("=" * 60)
        log("  SEABIRD — Lawnmower Sweep")
        log("=" * 60)
        log(f"  MAVSDK_ADDRESS={MAVSDK_ADDRESS}")
        log("  Physical drone mode: verify PX4/voxl-mavlink-server is sending MAVLink to this port")
        log("  Required before flight: verify COM_RC_OVERRIDE is non-zero for RC takeover")
        log("=" * 60 + "\n")

        start_rclpy_listener()
        log("[sweep] ROS2 detection listener started")

        asyncio.run(run_mission())

    except KeyboardInterrupt:
        log("[sweep] Interrupted by user")
    except Exception:
        log("[sweep] FATAL: unhandled exception")
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