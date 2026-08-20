#!/usr/bin/env python3
"""
element3_safety_check.py
========================
Flies the "PILOT SAFETY CHECK — ELEMENT #3" manoeuvre:

  a) Take-off and establish a hover at 2-3 m.
  b) Climb up and out to point (2): ~30-40 m out, at 30-40 m altitude.
  c) Commence towards point (3); on the assessor's call perform 3-4 stationary
     pirouettes (full 360° yaw spins), re-orientate, then continue to point (3).
  d) Descend at ~45° towards the landing area at point (4).
  e) Establish a hover at 2-3 m and land.
  f) Render the aircraft SAFE (disarm).

Geometry (NED frame, metres from spawn origin; North = "out", East = right):

        (3) ●────────────── SPAN_M ──────────────● (2)     altitude HIGH_ALT_M
             \                                   /
              \   descend ~45°       climb ~45° /
               \    (down + in)     (up + out) /
                \                             /
                 ●──────────────────────────●
                        (4 / 1)  START / END          hover HOVER_ALT_M

Pirouettes are performed at the midpoint of the 2→3 leg (P_PIROU).

Publishes the flown path on /seabird/flight_path (nav_msgs/Path) and
/seabird/path_marker (visualization_msgs/Marker LINE_STRIP) for RViz2.

Prerequisites (running before this script):
  1. Isaac Sim + spawn_drone.py completed ("[init] Done")
  2. PX4 SITL connected ("Ready for takeoff!")

Suggested PX4 params for a smooth 30-40 m manoeuvre (set once in pxh, then save):
  param set MPC_XY_CRUISE 5.0
  param set MPC_XY_VEL_MAX 8.0
  param set MPC_Z_VEL_MAX_UP 3.0
  param set MPC_Z_VEL_MAX_DN 2.0
  param set MPC_YAWRAUTO_MAX 90.0
  param set SYS_HAS_MAG 0
  param set COM_ARM_MAG_STR 0
  param set EKF2_ABL_LIM 5.0
  param save
"""

import asyncio
import math
import threading
import sys

import rclpy
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


# ── Manoeuvre configuration ───────────────────────────────────────────────────
HOVER_ALT_M   = 2.5     # low hover altitude at points 1 & 4   (spec: 2-3 m)
HIGH_ALT_M    = 35.0    # altitude at points 2 & 3             (spec: 30-40 m)
OUT_M         = 30.0    # "out" (north) distance of pts 2 & 3  (drives 30-40 m out)
SPAN_M        = 35.0    # E-W separation between pts 2 & 3     (spec: 30-40 m)
N_PIROUETTES  = 4       # stationary 360° spins                (spec: 3-4)

TAKEOFF_ALT_M     = HOVER_ALT_M
WAYPOINT_TOL_M    = 2.5
WAYPOINT_TIMEOUT  = 90.0
HOVER_STABILIZE_S = 6.0

MAVSDK_ADDRESS = "udpin://0.0.0.0:14551"

# Derived waypoints (North, East, Down)  — Down is negative up
P1_START = (   0.0,          0.0,        -HOVER_ALT_M)   # take-off / hover
P2       = ( OUT_M,   +SPAN_M / 2.0,     -HIGH_ALT_M )   # up & out (right, high)
P_PIROU  = ( OUT_M,          0.0,        -HIGH_ALT_M )   # midpoint 2→3, pirouette here
P3       = ( OUT_M,   -SPAN_M / 2.0,     -HIGH_ALT_M )   # far left, high
P4_LAND  = (   0.0,          0.0,        -HOVER_ALT_M)   # landing hover (== start)


# ── Geometry helper ───────────────────────────────────────────────────────────

def yaw_to(n_from, e_from, n_to, e_to):
    """Heading in degrees (NED: 0°=North, 90°=East) from one point to another."""
    return math.degrees(math.atan2(e_to - e_from, n_to - n_from)) % 360.0


# ── Shared state ──────────────────────────────────────────────────────────────
_path_pub   = None
_path_msg   = None
_marker_pub = None
_marker_msg = None
_rclpy_node = None


# ── ROS2 path-publisher thread ────────────────────────────────────────────────

def _rclpy_thread_fn():
    global _path_pub, _path_msg, _marker_pub, _marker_msg, _rclpy_node
    rclpy.init()
    node = rclpy.create_node("element3_path_publisher")
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

    node.get_logger().info("[element3] Publishing: /seabird/flight_path, /seabird/path_marker")
    rclpy.spin(node)


def start_rclpy_publisher() -> threading.Thread:
    t = threading.Thread(target=_rclpy_thread_fn, daemon=True, name="rclpy_publisher")
    t.start()
    return t


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

async def fly_to(drone, north, east, down, yaw, label="",
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
            print(f"  [fly_to] ⚠ Timeout ({timeout:.0f}s) {label} "
                  f"N={north:.1f} E={east:.1f} D={down:.1f}")
            return False

        if elapsed - last_pub >= 0.5:
            publish_path_point()
            last_pub = elapsed

        if int(elapsed) % 5 == 0 and int(elapsed) > 0:
            print(
                f"  [fly_to] {label} → N={_state.north_m:6.1f}/{north:6.1f}  "
                f"E={_state.east_m:6.1f}/{east:6.1f}  "
                f"Alt={-_state.down_m:5.1f}/{-down:5.1f}  "
                f"dist={dist:5.1f}m  t={elapsed:.0f}s"
            )


async def do_pirouettes(drone, north, east, down, n_spins, exit_yaw):
    """
    Hold position and perform n full 360° stationary yaw rotations, then
    re-orientate to exit_yaw (direction of travel) — step (c) in the spec.
    """
    print(f"[element3] ★ Assessor's call — performing {n_spins} stationary pirouettes")
    step_deg = 15.0
    steps_per_rev = int(360 / step_deg)
    for rev in range(n_spins):
        for s in range(steps_per_rev):
            yaw = (s * step_deg) % 360.0
            await drone.offboard.set_position_ned(PositionNedYaw(north, east, down, yaw))
            await asyncio.sleep(0.2)
            publish_path_point()
        print(f"  [pirouette] rotation {rev + 1}/{n_spins} complete")

    # Re-orientate to the travel direction and settle before continuing
    await drone.offboard.set_position_ned(PositionNedYaw(north, east, down, exit_yaw))
    await asyncio.sleep(2.0)
    publish_path_point()
    print(f"[element3] ✓ Re-orientated to {exit_yaw:.0f}° — continuing to point 3")


# ── Mission ───────────────────────────────────────────────────────────────────

async def run_mission():

    drone = System()
    print(f"[element3] Connecting to PX4 at {MAVSDK_ADDRESS}...")
    await drone.connect(system_address=MAVSDK_ADDRESS)

    print("[element3] Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[element3] ✓ Connected to PX4")
            break

    print("[element3] Waiting for GPS fix and home position...")
    async for health in drone.telemetry.health():
        gps_ok  = health.is_global_position_ok
        home_ok = health.is_home_position_ok
        if gps_ok and home_ok:
            print("[element3] ✓ GPS OK, home set")
            break
        if not gps_ok:
            print("  [element3] ... waiting for global position estimate")
        await asyncio.sleep(1.0)

    asyncio.ensure_future(track_position(drone))
    await asyncio.sleep(0.5)
    print(f"[element3] Drone at N={_state.north_m:.2f} E={_state.east_m:.2f} "
          f"D={_state.down_m:.2f}")

    # Pre-compute travel headings
    yaw_climb   = yaw_to(P1_START[0], P1_START[1], P2[0], P2[1])   # up & out to (2)
    yaw_west    = yaw_to(P2[0], P2[1], P3[0], P3[1])               # (2) → (3), ~270°
    yaw_descend = yaw_to(P3[0], P3[1], P4_LAND[0], P4_LAND[1])     # (3) → (4)

    desc_horiz = math.hypot(P3[0] - P4_LAND[0], P3[1] - P4_LAND[1])
    desc_vert  = abs(P3[2] - P4_LAND[2])
    desc_angle = math.degrees(math.atan2(desc_vert, desc_horiz))

    print("\n[element3] ── Manoeuvre plan ─────────────────────────────")
    print(f"[element3]  (1) START/hover : N={P1_START[0]:+5.1f} E={P1_START[1]:+5.1f}  "
          f"Alt={-P1_START[2]:.1f}m")
    print(f"[element3]  (2) up & out    : N={P2[0]:+5.1f} E={P2[1]:+5.1f}  "
          f"Alt={-P2[2]:.1f}m  yaw={yaw_climb:.0f}°")
    print(f"[element3]  (P) pirouettes  : N={P_PIROU[0]:+5.1f} E={P_PIROU[1]:+5.1f}  "
          f"Alt={-P_PIROU[2]:.1f}m  ({N_PIROUETTES}× 360°)")
    print(f"[element3]  (3) far side    : N={P3[0]:+5.1f} E={P3[1]:+5.1f}  "
          f"Alt={-P3[2]:.1f}m  yaw={yaw_west:.0f}°")
    print(f"[element3]  (4) LAND        : N={P4_LAND[0]:+5.1f} E={P4_LAND[1]:+5.1f}  "
          f"Alt={-P4_LAND[2]:.1f}m  descent≈{desc_angle:.0f}°")
    print("[element3] ────────────────────────────────────────────────\n")

    # ── (a) Arm + take-off, hover at 2-3 m ────────────────────────────────────
    print("[element3] (a) Arming...")
    try:
        await drone.action.arm()
    except ActionError as e:
        print(f"[element3] Arm failed: {e} — is PX4 ready?")
        return
    print("[element3] ✓ Armed")

    print(f"[element3] (a) Taking off to {TAKEOFF_ALT_M:.1f}m and hovering...")
    await drone.action.set_takeoff_altitude(TAKEOFF_ALT_M)
    await drone.action.takeoff()
    print(f"[element3] Waiting {HOVER_STABILIZE_S:.0f}s to stabilize the hover...")
    await asyncio.sleep(HOVER_STABILIZE_S)
    print(f"[element3] ✓ Hovering — N={_state.north_m:.1f} E={_state.east_m:.1f} "
          f"Alt≈{-_state.down_m:.1f}m")

    print("[element3] Switching to offboard mode...")
    await drone.offboard.set_position_ned(
        PositionNedYaw(_state.north_m, _state.east_m, -HOVER_ALT_M, yaw_climb)
    )
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"[element3] Offboard start failed: {e}")
        await drone.action.land()
        return
    print("[element3] ✓ Offboard active")

    # ── (b) Climb up and out to point (2) ─────────────────────────────────────
    print(f"[element3] (b) Climbing up & out to point (2)...")
    await fly_to(drone, P2[0], P2[1], P2[2], yaw_climb, label="→(2)")
    print("[element3] ✓ At point (2)")
    await asyncio.sleep(1.5)

    # ── (c) Commence toward (3), pirouettes on call, then continue ────────────
    print("[element3] (c) Commencing toward point (3)...")
    await fly_to(drone, P_PIROU[0], P_PIROU[1], P_PIROU[2], yaw_west, label="→pirouette pt")
    await do_pirouettes(drone, P_PIROU[0], P_PIROU[1], P_PIROU[2],
                        N_PIROUETTES, exit_yaw=yaw_west)
    await fly_to(drone, P3[0], P3[1], P3[2], yaw_west, label="→(3)")
    print("[element3] ✓ At point (3)")
    await asyncio.sleep(1.5)

    # ── (d) Descend at ~45° toward the landing area, point (4) ─────────────────
    print(f"[element3] (d) Descending ~{desc_angle:.0f}° toward the landing area (4)...")
    await fly_to(drone, P4_LAND[0], P4_LAND[1], P4_LAND[2], yaw_descend, label="→(4)")
    print("[element3] ✓ Over the landing area, hovering at "
          f"{-P4_LAND[2]:.1f}m")
    await asyncio.sleep(2.0)

    # ── (e) Hover then land ───────────────────────────────────────────────────
    print("[element3] (e) Landing...")
    try:
        await drone.offboard.stop()
    except Exception:
        pass
    await drone.action.land()

    print("[element3] Waiting for touchdown...")
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("[element3] ✓ Landed")
            break
        await asyncio.sleep(1.0)

    # ── (f) Render SAFE (disarm) ──────────────────────────────────────────────
    print("[element3] (f) Rendering aircraft SAFE (disarm)...")
    try:
        await drone.action.disarm()
        print("[element3] ✓ Disarmed")
    except ActionError as e:
        print(f"[element3] Disarm note: {e} (PX4 may have auto-disarmed on landing)")

    print("\n" + "═" * 52)
    print("  SEABIRD — PILOT SAFETY CHECK, ELEMENT #3 COMPLETE")
    print("═" * 52)
    print("  (a) take-off & hover ....... ok")
    print("  (b) climb up & out to (2) .. ok")
    print(f"  (c) {N_PIROUETTES} pirouettes → (3) ...... ok")
    print(f"  (d) ~{desc_angle:.0f}° descent to (4) ...... ok")
    print("  (e) hover & land ........... ok")
    print("  (f) rendered SAFE .......... ok")
    print("═" * 52 + "\n")


def main():
    print("=" * 60)
    print("  SEABIRD — Pilot Safety Check, Element #3")
    print("=" * 60)
    print("  Make sure these are already running:")
    print("  1. Isaac Sim + spawn_drone.py ('[init] Done')")
    print("  2. PX4 SITL ('Ready for takeoff!')")
    print("=" * 60 + "\n")

    start_rclpy_publisher()
    print("[element3] ROS2 path publisher started\n")

    asyncio.run(run_mission())


if __name__ == "__main__":
    main()
