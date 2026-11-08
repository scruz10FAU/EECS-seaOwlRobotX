#!/usr/bin/env python3
"""
flight_validation_hover.py
==========================
RobotX 2026 UAV Flight Validation: show the UAV in AUTONOMOUS mode, then
perform an autonomous hover only after RoboCommand's RunStart.

Team Handbook 3.4.10: "All vehicles should position-hold after switching into
Autonomous Mode. Only when the OCS receives and verifies the RunStart should
any vehicle begin their run."

Behavior:
  1. Connect to PX4 and wait for a global position + home.
  2. Go AUTONOMOUS on the ground: put PX4 in Hold (AUTO.LOITER) while still
     disarmed (AUTONOMOUS_MODE_SOURCE = "script"), or wait for the pilot to
     switch to an autonomous mode ("pilot"). beacon_mavlink_ground.py reports
     an autonomous PX4 mode as STATE_AUTO in the RoboCommand heartbeat --
     RoboCommand's condition for RunStart.
  3. Wait for RunStart on /seabird/run_start (verified by the OCS, relayed
     ground -> ELRS -> beacon_mavlink_bridge.py). A RunStart received before
     this script started (latched from an earlier run) is ignored.
  4. After RunStart: arm, autonomous takeoff to HOVER_ALT_M, autonomous hover
     in Hold for HOVER_DURATION_S, autonomous land, disarm. Every phase is an
     autonomous PX4 mode (TAKEOFF / HOLD / LAND), so the heartbeat stays
     STATE_AUTO.

The flight mode is logged on every change, for the record. The pilot can
take over with the sticks at any time (needs COM_RC_OVERRIDE != 0); the
script then stops commanding.

SAFETY:
  * Props on only for real flight, outdoors, in the validation area.
  * COM_RC_OVERRIDE must be non-zero and stick takeover bench-verified.
  * The PX4 geofence from RoboCommand's course should be VALID (on the UAV)
    before flying -- see tests/ROBOCOMMAND_GEOFENCE_TEST.md.

Run on the drone (same place/environment as the sweep script):
    python3.10 flight_validation_hover.py
"""

import asyncio
import json
import sys
import threading
import time

try:
    from mavsdk import System
    from mavsdk.action import ActionError
    from mavsdk.telemetry import FlightMode
except ImportError:
    print("[ERROR] mavsdk not installed. Run: pip install mavsdk")
    sys.exit(1)

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


# -- Configuration -----------------------------------------------------------
MAVSDK_ADDRESS = "udpin://0.0.0.0:14551"   # physical drone; "udpin://0.0.0.0:14540" for SITL

HOVER_ALT_M        = 3.0     # autonomous takeoff/hover altitude above home (m)
HOVER_DURATION_S   = 20.0    # how long to hold the autonomous hover
CONNECT_TIMEOUT_S  = 30.0
POSITION_TIMEOUT_S = 180.0   # give up if PX4 has no global position/home by then
POSITION_REPORT_S  = 5.0     # how often to log what's still missing
RUNSTART_WAIT_MAX_S = 1800.0  # give up (stay on the ground) if no RunStart by then
TAKEOFF_TIMEOUT_S  = 45.0    # to reach the hover altitude
LAND_TIMEOUT_S     = 90.0

# How the UAV gets into autonomous mode before RunStart:
#   "script" -- this script commands Hold (AUTO.LOITER) on the ground, disarmed
#   "pilot"  -- the pilot switches to an autonomous mode (Hold, etc.) via RC/QGC
AUTONOMOUS_MODE_SOURCE = "script"

# BENCH_TEST=True: connect, go autonomous (Hold, disarmed), wait for RunStart,
# then only LOG what the flight would do -- never arms. Safe with props on.
BENCH_TEST = True

RUN_START_TOPIC = "/seabird/run_start"

# PX4 modes beacon_mavlink_ground.py reports as STATE_AUTO (its _AUTO_MODES).
# getattr: not every MAVSDK version has every member (e.g. PRECISION_LAND).
AUTONOMOUS_MODES = {m for m in (getattr(FlightMode, name, None) for name in (
    "HOLD", "MISSION", "OFFBOARD", "TAKEOFF", "LAND", "RETURN_TO_LAUNCH", "FOLLOW_ME",
    "PRECISION_LAND")) if m is not None}


def log(text):
    print(f"[{time.strftime('%H:%M:%S')}] [validation] {text}", flush=True)


# -- RunStart listener (ROS, own context in a thread, like the sweep) ---------

class RunStartListener:
    """Latches the first RunStart received AFTER this script started."""

    def __init__(self):
        self.started_at = time.time()
        self.event = threading.Event()
        self.run_id = None
        self._ctx = rclpy.Context()
        rclpy.init(context=self._ctx)
        self._node = rclpy.create_node("flight_validation_hover", context=self._ctx)
        self._node.create_subscription(
            String, RUN_START_TOPIC, self._on_run_start,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._running = True
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        from rclpy.executors import SingleThreadedExecutor
        ex = SingleThreadedExecutor(context=self._ctx)
        ex.add_node(self._node)
        while self._running and rclpy.ok(context=self._ctx):
            ex.spin_once(timeout_sec=0.1)
        ex.shutdown()

    def close(self):
        """Stop the ROS thread cleanly (exiting with it still spinning aborts the process)."""
        self._running = False
        self._thread.join(timeout=2.0)
        self._node.destroy_node()
        rclpy.shutdown(context=self._ctx)

    def _on_run_start(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        received = float(data.get("received", 0.0))
        if received and received < self.started_at:
            log(f"ignoring RunStart run_id={data.get('run_id')} from before this script started "
                f"(latched from an earlier run)")
            return
        if not self.event.is_set():
            self.run_id = data.get("run_id")
            self.event.set()


# -- Flight ------------------------------------------------------------------

class ModeWatcher:
    """Tracks PX4's flight mode and logs every change."""

    def __init__(self, drone):
        self.mode = None
        self._drone = drone

    async def run(self):
        async for mode in self._drone.telemetry.flight_mode():
            if mode != self.mode:
                tag = "AUTONOMOUS" if mode in AUTONOMOUS_MODES else "not autonomous"
                log(f"flight mode {self.mode.name if self.mode else '-'} -> {mode.name} ({tag})")
                self.mode = mode

    @property
    def autonomous(self):
        return self.mode in AUTONOMOUS_MODES


async def wait_for(predicate, timeout_s, poll_s=0.2):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if predicate():
            return True
        await asyncio.sleep(poll_s)
    return predicate()


async def _wait_connected(drone):
    async for state in drone.core.connection_state():
        if state.is_connected:
            return


async def run_validation(runstart: RunStartListener):
    drone = System()
    log(f"connecting to PX4 at {MAVSDK_ADDRESS} ...")
    await drone.connect(system_address=MAVSDK_ADDRESS)
    try:
        await asyncio.wait_for(_wait_connected(drone), timeout=CONNECT_TIMEOUT_S)
    except asyncio.TimeoutError:
        log(f"ERROR: no MAVLink heartbeat after {CONNECT_TIMEOUT_S:.0f} s at {MAVSDK_ADDRESS}")
        return
    log("connected to PX4")

    modes = ModeWatcher(drone)
    asyncio.ensure_future(modes.run())
    altitude = {"rel": 0.0}

    async def track_altitude():
        async for pos in drone.telemetry.position():
            altitude["rel"] = pos.relative_altitude_m
    asyncio.ensure_future(track_altitude())
    in_air = {"v": False}

    async def track_in_air():
        async for v in drone.telemetry.in_air():
            in_air["v"] = v
    asyncio.ensure_future(track_in_air())

    # 1. Position + home (needed for Hold and for takeoff).
    log("waiting for a global position estimate and home...")
    t0, last_report, ready = time.monotonic(), 0.0, False
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            log("global position OK, home set")
            ready = True
            break
        elapsed = time.monotonic() - t0
        if elapsed - last_report >= POSITION_REPORT_S:
            missing = [name for name, ok in (("global position", health.is_global_position_ok),
                                              ("home", health.is_home_position_ok)) if not ok]
            log(f"still waiting ({elapsed:.0f} s): missing {', '.join(missing)} -- PX4 refuses "
                f"Hold without them (check the GPS fix)")
            last_report = elapsed
        if elapsed > POSITION_TIMEOUT_S:
            break
        await asyncio.sleep(1.0)
    if not ready:
        log(f"ERROR: no global position/home after {POSITION_TIMEOUT_S:.0f} s -- can't go "
            f"autonomous; stopping (still on the ground)")
        return

    # 2. Autonomous on the ground.
    if AUTONOMOUS_MODE_SOURCE == "script":
        log("switching PX4 to Hold (AUTO.LOITER) on the ground -- the UAV is now AUTONOMOUS")
        try:
            await drone.action.hold()
        except ActionError as e:
            log(f"PX4 refused Hold on the ground ({e}): ask the pilot to switch to an autonomous "
                f"mode (e.g. Hold) via RC/QGC -- waiting for it")
    else:
        log("waiting for the pilot to switch to an autonomous mode (e.g. Hold)...")
    if not await wait_for(lambda: modes.autonomous, 120.0):
        log("ERROR: the UAV isn't in an autonomous mode after 120 s -- stopping (still on the ground)")
        return
    log(f"UAV AUTONOMOUS in {modes.mode.name}: RoboCommand heartbeat reports STATE_AUTO -- "
        f"holding until RunStart")

    # 3. Wait for RunStart, holding position (on the ground) meanwhile.
    t0, warned = time.monotonic(), False
    while not runstart.event.is_set():
        if time.monotonic() - t0 > RUNSTART_WAIT_MAX_S:
            log(f"no RunStart after {RUNSTART_WAIT_MAX_S:.0f} s -- not flying")
            return
        if not modes.autonomous and not warned:
            log(f"WARNING: left autonomous mode ({modes.mode.name}) before RunStart -- RoboCommand "
                f"won't start the run until the UAV is autonomous again")
            warned = True
        elif modes.autonomous:
            warned = False
        await asyncio.sleep(0.2)
    log(f"RunStart received (run_id={runstart.run_id}) -- beginning the run: autonomous hover")

    # 4. The run: arm, takeoff, hover, land -- all in autonomous PX4 modes.
    if BENCH_TEST:
        log(f"BENCH_TEST: would arm, take off to {HOVER_ALT_M:g} m, hover {HOVER_DURATION_S:g} s "
            f"in Hold, land and disarm -- not arming")
        return
    try:
        log("arming")
        await drone.action.arm()
        await drone.action.set_takeoff_altitude(HOVER_ALT_M)
        log(f"autonomous takeoff to {HOVER_ALT_M:g} m")
        await drone.action.takeoff()
    except ActionError as e:
        log(f"ERROR: arm/takeoff refused: {e} -- staying on the ground")
        return
    reached = await wait_for(lambda: altitude["rel"] >= 0.9 * HOVER_ALT_M, TAKEOFF_TIMEOUT_S)
    if not reached:
        log(f"WARNING: hover altitude not reached in {TAKEOFF_TIMEOUT_S:.0f} s "
            f"(at {altitude['rel']:.1f} m) -- hovering where it is")
    # PX4 goes TAKEOFF -> HOLD by itself at the takeoff altitude; make sure it holds.
    await wait_for(lambda: modes.mode == FlightMode.HOLD, 10.0)
    if modes.mode != FlightMode.HOLD:
        try:
            await drone.action.hold()
        except ActionError as e:
            log(f"WARNING: Hold refused in the air ({e})")
    log(f"autonomous hover in {modes.mode.name} at {altitude['rel']:.1f} m for {HOVER_DURATION_S:g} s")
    end = time.monotonic() + HOVER_DURATION_S
    while time.monotonic() < end:
        if not modes.autonomous:
            log(f"pilot took over ({modes.mode.name}) -- the script stops commanding")
            return
        await asyncio.sleep(0.5)
    log("hover complete -- autonomous landing")
    try:
        await drone.action.land()
    except ActionError as e:
        log(f"ERROR: land refused: {e} -- pilot, land manually")
        return
    if await wait_for(lambda: not in_air["v"], LAND_TIMEOUT_S, poll_s=0.5):
        log("landed")
    else:
        log(f"WARNING: still in the air after {LAND_TIMEOUT_S:.0f} s -- pilot, take over")
        return
    try:
        await drone.action.disarm()
    except ActionError:
        pass   # PX4 usually auto-disarms after landing
    log("disarmed -- validation hover complete")


def main():
    log("=" * 60)
    log("RobotX UAV Flight Validation: autonomous mode -> RunStart -> autonomous hover")
    log(f"  MAVSDK_ADDRESS={MAVSDK_ADDRESS}  HOVER_ALT_M={HOVER_ALT_M}  HOVER_DURATION_S={HOVER_DURATION_S}")
    log(f"  AUTONOMOUS_MODE_SOURCE={AUTONOMOUS_MODE_SOURCE}  BENCH_TEST={BENCH_TEST}")
    log("=" * 60)
    runstart = RunStartListener()
    log(f"listening for RunStart on {RUN_START_TOPIC}")
    try:
        asyncio.run(run_validation(runstart))
    except KeyboardInterrupt:
        log("interrupted")
    finally:
        runstart.close()


if __name__ == "__main__":
    main()
