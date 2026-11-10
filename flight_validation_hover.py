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
  1-5. mission_gate.run_gate() -- every vehicle-side RoboCommand step before
     the run (same directory, shared with the sweep): PX4 + GPS/home, the PX4
     geofence present with the UAV inside it, current_task = TASK_NONE (the
     validation hover is not a task attempt), AUTONOMOUS on the ground while
     disarmed (MISSION with this hover uploaded, or Hold), and a fresh
     RunStart. Ground-mode / fence settings live in mission_gate.py.
  6. After RunStart: arm. If PX4 isn't ready (e.g. GPS lost: arm / MISSION /
     Hold refused), retry every READY_RETRY_S until it is, for up to
     RUNSTART_READY_WAIT_S, and take off as soon as it is -- once.
       MISSION: start the mission once armed; PX4 flies it (takeoff to
                HOVER_ALT_M, hover HOVER_DURATION_S, land).
       Hold:    autonomous takeoff, hover in Hold, autonomous land.
     Then disarm. Every phase is an autonomous PX4 mode, so the heartbeat
     stays STATE_AUTO.

The flight mode and PX4's own messages are logged. The pilot can take over
with the sticks at any time (needs COM_RC_OVERRIDE != 0); the script then
stops commanding.

SAFETY:
  * Props on only for real flight, outdoors, in the validation area.
  * COM_RC_OVERRIDE must be non-zero and stick takeover bench-verified.
  * After RunStart the UAV may arm and take off as soon as PX4 is ready
    (up to RUNSTART_READY_WAIT_S later): keep clear.

Run on the drone (same place/environment as the sweep script), with
mission_gate.py next to it:
    python3.10 flight_validation_hover.py
"""

import asyncio
import os
import sys
import time

try:
    from mavsdk import System
    from mavsdk.action import ActionError
    from mavsdk.telemetry import FlightMode
except ImportError:
    print("[ERROR] mavsdk not installed. Run: pip install mavsdk")
    sys.exit(1)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mission_gate import TASK_NONE, run_gate, wait_for  # noqa: E402


# -- Configuration -----------------------------------------------------------
MAVSDK_ADDRESS = "udpin://0.0.0.0:14551"   # physical drone; "udpin://0.0.0.0:14540" for SITL

HOVER_ALT_M        = 3.0     # autonomous takeoff/hover altitude above home (m)
HOVER_DURATION_S   = 20.0    # how long to hold the autonomous hover
TAKEOFF_TIMEOUT_S  = 45.0    # to be airborne after arming
LAND_TIMEOUT_S     = 90.0
# After RunStart, if PX4 isn't ready (e.g. GPS lost: arming / MISSION / Hold
# refused), keep retrying every READY_RETRY_S until it is, for up to
# RUNSTART_READY_WAIT_S -- then take off as soon as it's ready. Never retries
# once airborne.
READY_RETRY_S         = 5.0
RUNSTART_READY_WAIT_S = 600.0

# BENCH_TEST=True: run the gate (autonomous, RunStart), then only LOG what the
# flight would do -- never arms. Safe with props on.
BENCH_TEST = False


def log(text):
    print(f"[{time.strftime('%H:%M:%S')}] [validation] {text}", flush=True)


# -- Flight ------------------------------------------------------------------

async def safe_disarm(drone):
    """Disarm on the ground; kill (motors off) if PX4 refuses. Only call on the ground."""
    try:
        await drone.action.disarm()
        log("disarmed")
    except ActionError as e:
        log(f"disarm refused ({e}) -- killing motors (on the ground)")
        try:
            await drone.action.kill()
        except ActionError as e2:
            log(f"ERROR: kill refused too ({e2}) -- pilot, disarm now")


async def mission_takeoff(gate, in_air):
    """One attempt: (re)enter MISSION, arm, start the mission, wait to be airborne.
    'airborne' | 'retry' (PX4 not ready, nothing armed) | 'failed' (armed, didn't lift off)."""
    drone, modes = gate.drone, gate.modes
    if modes.mode != FlightMode.MISSION:
        log(f"not in MISSION ({modes.name}) -- re-entering it")
        if not await gate.enter_mission(HOVER_ALT_M, HOVER_DURATION_S):
            return "retry"
    try:
        log("arming -- PX4 flies the mission (takeoff, hover, land)")
        await drone.action.arm()
    except ActionError as e:
        log(f"arm refused: {e} -- will retry")
        return "retry"
    # This PX4 doesn't start the mission on arming in MISSION mode: (re)start it
    # now that we're armed, or the UAV sits armed on the ground.
    try:
        await drone.mission.start_mission()
        log("mission started (armed) -- taking off")
    except Exception as e:   # MissionError
        log(f"mission start refused after arming ({type(e).__name__}: {e}) -- disarming, will retry")
        await safe_disarm(drone)
        return "retry"
    if not await wait_for(lambda: in_air["v"], TAKEOFF_TIMEOUT_S):
        log(f"ERROR: armed but not airborne after {TAKEOFF_TIMEOUT_S:.0f} s "
            f"(mode {modes.name}) -- disarming, not retrying")
        await safe_disarm(drone)
        return "failed"
    return "airborne"


async def hold_takeoff(gate, in_air):
    """One attempt: arm, autonomous takeoff, wait to be airborne. Same results as mission_takeoff."""
    drone = gate.drone
    try:
        log("arming")
        await drone.action.arm()
    except ActionError as e:
        log(f"arm refused: {e} -- will retry")
        return "retry"
    try:
        await drone.action.set_takeoff_altitude(HOVER_ALT_M)
        log(f"autonomous takeoff to {HOVER_ALT_M:g} m")
        await drone.action.takeoff()
    except ActionError as e:
        log(f"takeoff refused: {e} -- disarming, will retry")
        await safe_disarm(drone)
        return "retry"
    if not await wait_for(lambda: in_air["v"], TAKEOFF_TIMEOUT_S):
        log(f"ERROR: armed but not airborne after {TAKEOFF_TIMEOUT_S:.0f} s "
            f"(mode {gate.modes.name}) -- disarming, not retrying")
        await safe_disarm(drone)
        return "failed"
    return "airborne"


async def finish_mission(gate, in_air):
    """Airborne in MISSION: let PX4 fly the rest (hover, land), then disarm."""
    drone, modes = gate.drone, gate.modes

    async def log_progress():
        async for p in drone.mission.mission_progress():
            log(f"mission item {p.current}/{p.total}")
    progress = asyncio.ensure_future(log_progress())
    try:
        log("airborne -- mission running")
        end = time.monotonic() + TAKEOFF_TIMEOUT_S + HOVER_DURATION_S + LAND_TIMEOUT_S
        while in_air["v"]:
            if not modes.autonomous:
                log(f"pilot took over ({modes.name}) -- the script stops")
                return
            if time.monotonic() > end:
                log("WARNING: mission still in the air past its expected time -- pilot, take over")
                return
            await asyncio.sleep(0.5)
        log("landed")
        try:
            await drone.action.disarm()
        except ActionError:
            pass   # PX4 usually auto-disarms after landing
        log("disarmed -- validation hover complete")
    finally:
        progress.cancel()


async def finish_hold(gate, in_air, altitude):
    """Airborne after a Hold-path takeoff: hover in Hold, land, disarm."""
    drone, modes = gate.drone, gate.modes
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
    log(f"autonomous hover in {modes.name} at {altitude['rel']:.1f} m for {HOVER_DURATION_S:g} s")
    end = time.monotonic() + HOVER_DURATION_S
    while time.monotonic() < end:
        if not modes.autonomous:
            log(f"pilot took over ({modes.name}) -- the script stops commanding")
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


async def run_validation():
    drone = System()
    log(f"connecting to PX4 at {MAVSDK_ADDRESS} ...")
    await drone.connect(system_address=MAVSDK_ADDRESS)

    altitude = {"rel": 0.0}
    in_air = {"v": False}

    async def track_altitude():
        async for pos in drone.telemetry.position():
            altitude["rel"] = pos.relative_altitude_m
    asyncio.ensure_future(track_altitude())

    async def track_in_air():
        async for v in drone.telemetry.in_air():
            in_air["v"] = v
    asyncio.ensure_future(track_in_air())

    # 1-5: every RoboCommand step before the run (validation hover: no task).
    gate = await run_gate(drone, run_task=TASK_NONE,
                          mission_alt_m=HOVER_ALT_M, mission_hover_s=HOVER_DURATION_S)
    if gate is None:
        log("start gate failed (see [gate] above) -- not flying")
        return
    try:
        await fly(gate, in_air, altitude)
    finally:
        gate.close()


async def fly(gate, in_air, altitude):
    """6: the run -- arm, take off, hover, land; all in autonomous PX4 modes."""
    use_mission = gate.in_mission
    log(f"RunStart {gate.run_id}: beginning the run -- autonomous hover "
        f"({'MISSION' if use_mission else 'Hold'})")
    if BENCH_TEST:
        how = "PX4 flies the MISSION" if use_mission else "script commands takeoff/Hold/land"
        log(f"BENCH_TEST: would arm, take off to {HOVER_ALT_M:g} m, hover {HOVER_DURATION_S:g} s, "
            f"land and disarm ({how}) -- not arming")
        return
    # Retry until PX4 is ready (e.g. GPS back after a dropout), then take off once.
    deadline = time.monotonic() + RUNSTART_READY_WAIT_S
    attempt = 0
    while True:
        if not await gate.wait_ready(deadline):
            log(f"ERROR: PX4 not ready {RUNSTART_READY_WAIT_S:.0f} s after RunStart -- "
                f"stopping (still on the ground)")
            return
        attempt += 1
        log(f"takeoff attempt {attempt}")
        if use_mission:
            result = await mission_takeoff(gate, in_air)
        else:
            result = await hold_takeoff(gate, in_air)
        if result == "airborne":
            break
        if result == "failed":
            return
        if time.monotonic() + READY_RETRY_S > deadline:
            log(f"ERROR: PX4 still refusing {RUNSTART_READY_WAIT_S:.0f} s after RunStart -- "
                f"stopping (still on the ground)")
            return
        await asyncio.sleep(READY_RETRY_S)
    if use_mission:
        await finish_mission(gate, in_air)
    else:
        await finish_hold(gate, in_air, altitude)


def main():
    log("=" * 60)
    log("RobotX UAV Flight Validation: autonomous mode -> RunStart -> autonomous hover")
    log(f"  MAVSDK_ADDRESS={MAVSDK_ADDRESS}  HOVER_ALT_M={HOVER_ALT_M}  "
        f"HOVER_DURATION_S={HOVER_DURATION_S}  BENCH_TEST={BENCH_TEST}")
    log("=" * 60)
    try:
        asyncio.run(run_validation())
    except KeyboardInterrupt:
        log("interrupted")


if __name__ == "__main__":
    main()
