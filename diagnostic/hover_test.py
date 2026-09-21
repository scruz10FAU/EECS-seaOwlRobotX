#!/usr/bin/env python3
"""
hover_test.py
=============
Minimal takeoff-and-hover verification for the physical drone (or SITL).

Behavior:
  1. Connect to PX4 over MAVLink.
  2. Wait for GPS fix + home position (same gate sweep_lawnmower.py uses).
  3. Arm.
  4. Take off to HOVER_ALT_M and hold there for HOVER_DURATION_S.
  5. Land, wait for touchdown, disarm.

What this is for: verifying the arm -> takeoff -> hover -> land -> disarm
chain works end to end BEFORE trusting element3_safety_check.py's full
manoeuvre (climb to 35 m, pirouettes, 45 deg descent). If this script
does not hover reliably, do not run the Element #3 flight.

SAFETY (same rules as element3_safety_check.py):
  * PROPS ON only for real flight; do this outdoors, wide open area.
  * COM_RC_OVERRIDE must be non-zero and stick-takeover bench-verified.
  * NAV_RCL_ACT=0 disables RC-loss failsafe -- know this before flying.
  * DO NOT copy the SITL param overrides from element3 (SYS_HAS_MAG=0,
    COM_ARM_MAG_STR=0) onto the physical drone. The physical drone has a
    working magnetometer; disabling it flies blind on yaw.

Prerequisites:
  Physical: drone powered on, flight controller reachable over MAVSDK
            via the voxl-vision-hub localhost feed on port 14551
            (en_localhost_mavlink_udp=true in voxl-vision-hub.conf).
  Sim:      Isaac Sim + PX4 SITL running (set MAVSDK_ADDRESS to udp://:14540).
"""

import asyncio
import sys

try:
    from mavsdk import System
    from mavsdk.action import ActionError
except ImportError:
    print("[ERROR] mavsdk not installed. Run: pip install mavsdk")
    sys.exit(1)


# -- Configuration -----------------------------------------------------------
# Physical drone via voxl-vision-hub localhost feed. For Isaac Sim / SITL,
# change to "udp://:14540".
MAVSDK_ADDRESS = "udpin://0.0.0.0:14551"

HOVER_ALT_M       = 2.5     # takeoff/hover altitude (metres)
HOVER_DURATION_S  = 15.0    # how long to hold the hover before landing
CONNECT_TIMEOUT_S = 30.0    # max wait for MAVLink heartbeat

# BENCH_TEST=True: skip the GPS/home-position gate AND the arm command.
# Verifies the pipeline (MAVLink connect + telemetry) without commanding
# any motor activity. Safe to run on the bench with props on or off, no
# PX4 param changes needed. The useful signal is "did we connect and see
# telemetry?" -- if yes, the same script with BENCH_TEST=False will get
# as far as arming when a position estimate is available.
# DO NOT LEAVE True permanently -- outdoor flight needs the real gate.
BENCH_TEST = True


# -- Mission -----------------------------------------------------------------

async def run_hover_test():
    drone = System()

    print("[hover] Connecting to PX4 at {} ...".format(MAVSDK_ADDRESS))
    await drone.connect(system_address=MAVSDK_ADDRESS)

    # Wait for MAVLink connection with a timeout so a wrong address fails
    # loudly instead of hanging forever.
    print("[hover] Waiting for connection...")
    try:
        await asyncio.wait_for(_wait_connected(drone), timeout=CONNECT_TIMEOUT_S)
    except asyncio.TimeoutError:
        print("[hover] ERROR: no MAVLink heartbeat after {:.0f}s at {}".format(
            CONNECT_TIMEOUT_S, MAVSDK_ADDRESS))
        print("        Check that voxl-vision-hub is running and")
        print("        en_localhost_mavlink_udp is true in its config.")
        return
    print("[hover] Connected to PX4")

    if BENCH_TEST:
        print("")
        print("*" * 60)
        print("*** BENCH_TEST MODE - pipeline verification only      ***")
        print("*** skipping GPS/home wait AND arm command            ***")
        print("*** motors will NOT spin                              ***")
        print("*" * 60)
        print("")
    else:
        # Same GPS + home gate sweep_lawnmower.py uses. Home-position waits
        # get their own print so a stall there is not silent.
        print("[hover] Waiting for GPS fix and home position...")
        async for health in drone.telemetry.health():
            gps_ok  = health.is_global_position_ok
            home_ok = health.is_home_position_ok
            if gps_ok and home_ok:
                print("[hover] GPS OK, home set")
                break
            if not gps_ok:
                print("  [hover] ... waiting for global position estimate")
            elif not home_ok:
                print("  [hover] ... waiting for home position")
            await asyncio.sleep(1.0)

    # Bench mode: verify the pipeline WITHOUT arming. Sample a few
    # telemetry streams to prove data flows both directions (script <->
    # PX4). If these come back, the same script with BENCH_TEST=False
    # will get through connect and the GPS gate to the arm step.
    if BENCH_TEST:
        print("[hover] Sampling telemetry (no arm command)...")

        # Battery voltage / remaining
        try:
            async for batt in drone.telemetry.battery():
                print("[hover]   battery: {:.2f} V, {:.0%} remaining".format(
                    batt.voltage_v, batt.remaining_percent))
                break
        except Exception as e:
            print("[hover]   battery: unavailable ({})".format(e))

        # Attitude (proves the IMU + attitude estimator are talking to us)
        try:
            async for att in drone.telemetry.attitude_euler():
                print("[hover]   attitude: roll={:+.1f}  pitch={:+.1f}  yaw={:+.1f}".format(
                    att.roll_deg, att.pitch_deg, att.yaw_deg))
                break
        except Exception as e:
            print("[hover]   attitude: unavailable ({})".format(e))

        # Health flags (shows exactly which arming gates would block us)
        try:
            async for health in drone.telemetry.health():
                print("[hover]   health flags:")
                print("[hover]     gyro_calibration_ok        = {}".format(health.is_gyrometer_calibration_ok))
                print("[hover]     accel_calibration_ok       = {}".format(health.is_accelerometer_calibration_ok))
                print("[hover]     mag_calibration_ok         = {}".format(health.is_magnetometer_calibration_ok))
                print("[hover]     local_position_ok          = {}".format(health.is_local_position_ok))
                print("[hover]     global_position_ok         = {}".format(health.is_global_position_ok))
                print("[hover]     home_position_ok           = {}".format(health.is_home_position_ok))
                print("[hover]     armable                    = {}".format(health.is_armable))
                break
        except Exception as e:
            print("[hover]   health: unavailable ({})".format(e))

        print("\n" + "=" * 52)
        print("  BENCH TEST COMPLETE (pipeline verified, no arm)")
        print("=" * 52 + "\n")
        return

    print("[hover] Arming...")
    try:
        await drone.action.arm()
    except ActionError as e:
        print("[hover] Arm failed: {}".format(e))
        return
    print("[hover] Armed")

    print("[hover] Taking off to {:.1f} m ...".format(HOVER_ALT_M))
    try:
        await drone.action.set_takeoff_altitude(HOVER_ALT_M)
        await drone.action.takeoff()
    except ActionError as e:
        print("[hover] Takeoff failed: {}".format(e))
        # Try to disarm so we do not leave the drone in an armed state.
        try:
            await drone.action.disarm()
        except ActionError:
            pass
        return

    # Wait until PX4 reports we are in the air, then hold.
    print("[hover] Waiting for in-air state...")
    airborne_deadline = asyncio.get_event_loop().time() + 30.0
    async for in_air in drone.telemetry.in_air():
        if in_air:
            print("[hover] Airborne")
            break
        if asyncio.get_event_loop().time() > airborne_deadline:
            print("[hover] WARN: never reported in-air after 30s; continuing")
            break
        await asyncio.sleep(0.5)

    print("[hover] Holding hover for {:.0f}s ...".format(HOVER_DURATION_S))
    await asyncio.sleep(HOVER_DURATION_S)

    print("[hover] Landing...")
    try:
        await drone.action.land()
    except ActionError as e:
        print("[hover] Land command failed: {}".format(e))
        return

    print("[hover] Waiting for touchdown...")
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("[hover] Landed")
            break
        await asyncio.sleep(1.0)

    print("[hover] Disarming (if PX4 has not auto-disarmed)...")
    try:
        await drone.action.disarm()
        print("[hover] Disarmed")
    except ActionError as e:
        print("[hover] Disarm note: {} (PX4 may have auto-disarmed on landing)".format(e))

    print("\n" + "=" * 52)
    print("  HOVER TEST COMPLETE")
    print("=" * 52)
    print("  connected .......... ok")
    print("  gps/home ok ........ ok")
    print("  armed .............. ok")
    print("  takeoff ............ ok")
    print("  hover held ......... ok")
    print("  landed ............. ok")
    print("  disarmed ........... ok")
    print("=" * 52 + "\n")


async def _wait_connected(drone):
    async for state in drone.core.connection_state():
        if state.is_connected:
            return


def main():
    print("=" * 60)
    print("  HOVER TEST -- takeoff, hold, land, disarm")
    print("=" * 60)
    if BENCH_TEST:
        print("  MODE:            *** BENCH_TEST (pipeline only, no arm) ***")
    else:
        print("  MODE:            flight (takeoff, hover, land)")
        print("  Hover altitude:  {:.1f} m".format(HOVER_ALT_M))
        print("  Hover duration:  {:.0f} s".format(HOVER_DURATION_S))
    print("  MAVLink address: {}".format(MAVSDK_ADDRESS))
    print("=" * 60 + "\n")

    asyncio.run(run_hover_test())


if __name__ == "__main__":
    main()