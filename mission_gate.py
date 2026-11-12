#!/usr/bin/env python3
"""
mission_gate.py
===============
RobotX 2026 UAV start gate -- every vehicle-side RoboCommand step before a
run, shared by the flight scripts (flight_validation_hover.py,
sweep_rrt_overfly_downward_center.py, ...). Self-contained (MAVSDK + rclpy).

Library use (in-process, on the flight script's own MAVSDK connection):

    from mission_gate import run_gate, TASK_SAFE_PASSAGE
    gate = await run_gate(drone, run_task=TASK_SAFE_PASSAGE)
    if gate is None:
        return                      # a step failed: stay on the ground
    ... arm + fly (gate.wait_ready() for GPS-dropout retries) ...
    gate.set_task(TASK_NONE)        # attempt over
    gate.close()

Wrapper use (for scripts that don't call run_gate themselves):

    python3 mission_gate.py <flight_script.py> [its args...]

  runs the gate on its own connection, then frees UDP 14551, stops its ROS
  node and execs the script with SEABIRD_RUNSTART_ID / SEABIRD_RUNSTART_RECEIVED
  set (that script must treat RunStart as received and publish current_task
  itself -- the gate is gone after exec).

Steps (Team Handbook 3.4.10 / 3.4.14), each logged; a failure returns None:
  1. PX4 connected; global position + home OK. PX4's own messages are logged.
  2. Geofence ON THE UAV: PX4 holds an inclusion polygon (read back from
     PX4), the UAV is inside it, and GF_MAX_VER_DIST (if set) is above the
     flight altitude. REQUIRE_FENCE=False only warns (bench without a course).
  3. current_task: TASK_NONE (1) on /seabird/current_task (1 Hz, relayed to
     RoboCommand by beacon_mavlink_bridge.py) until RunStart, then run_task.
  4. AUTONOMOUS on the ground, disarmed: MISSION (a hover mission is uploaded
     at the UAV's position; the caller decides whether to fly it), falling
     back to Hold. beacon_mavlink_ground.py reports STATE_AUTO.
  5. A fresh RunStart on /seabird/run_start (latched ones from before the gate
     started are ignored; SEABIRD_RUNSTART_ID counts as received).
The gate never arms.
"""

import asyncio
import json
import math
import os
import socket
import sys
import threading
import time

try:
    from mavsdk import System
    from mavsdk.action import ActionError
    from mavsdk.telemetry import FlightMode
    from mavsdk.mission import MissionItem, MissionPlan
except ImportError:
    print("[ERROR] mavsdk not installed. Run: pip install mavsdk")
    sys.exit(1)

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String, UInt8


# -- Settings ----------------------------------------------------------------
MAVSDK_ADDRESS = "udpin://0.0.0.0:14551"   # wrapper mode; SITL: "udpin://0.0.0.0:14540"

# Autonomous mode on the ground before RunStart:
#   "script": the gate commands it -- GROUND_AUTONOMOUS_MODE "mission" (falls
#             back to Hold) or "hold";  "pilot": wait for the pilot to.
AUTONOMOUS_MODE_SOURCE = "script"
GROUND_AUTONOMOUS_MODE = "mission"

# Geofence on the UAV must be present and contain the UAV (competition).
# False: missing fence / outside it are warnings only (bench, no course).
REQUIRE_FENCE = True

CONNECT_TIMEOUT_S    = 30.0
POSITION_TIMEOUT_S   = 180.0    # global position + home
POSITION_REPORT_S    = 5.0
FENCE_TIMEOUT_S      = 30.0     # geofence readback from PX4
AUTONOMOUS_TIMEOUT_S = 120.0
RUNSTART_WAIT_MAX_S  = 1800.0
TASK_REPUBLISH_S     = 1.0

RUN_START_TOPIC    = "/seabird/run_start"
CURRENT_TASK_TOPIC = "/seabird/current_task"
HANDOFF_PORT       = 14551      # wrapper mode: the next script's MAVSDK port

# rx_common.proto TaskType values used on /seabird/current_task.
TASK_NONE         = 1
TASK_SAFE_PASSAGE = 2

# PX4 modes beacon_mavlink_ground.py reports as STATE_AUTO.
AUTONOMOUS_MODES = {m for m in (getattr(FlightMode, n, None) for n in (
    "HOLD", "MISSION", "OFFBOARD", "TAKEOFF", "LAND", "RETURN_TO_LAUNCH", "FOLLOW_ME",
    "PRECISION_LAND")) if m is not None}

FENCE_INCLUSION_VERTEX = 5001   # MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
_GATE_STARTED = time.time()     # RunStarts received before this need runstart_accept()'s checks

# A RunStart can reach the UAV before the flight script starts (e.g. while the
# launcher waits for the RC trigger). It is still accepted if no earlier attempt
# has ARMED on that run_id (the ledger below) and it is at most
# RUNSTART_EARLY_MAX_S old -- so an early RunStart for this attempt starts the
# run, but a RunStart a previous attempt already flew on never does.
RUNSTART_EARLY_MAX_S = 600.0
RUNSTART_LEDGER = os.environ.get("SEABIRD_RUNSTART_LEDGER", os.path.join("logs", "runstart_used.txt"))


def runstart_used_ids():
    """run_ids some attempt already armed on (from RUNSTART_LEDGER)."""
    try:
        with open(RUNSTART_LEDGER) as f:
            return {line.split()[0] for line in f if line.strip()}
    except OSError:
        return set()


def mark_runstart_used(run_id):
    """Record that the UAV armed on this run_id: it won't start another attempt."""
    if run_id is None:
        return
    try:
        os.makedirs(os.path.dirname(RUNSTART_LEDGER) or ".", exist_ok=True)
        with open(RUNSTART_LEDGER, "a") as f:
            f.write(f"{run_id} armed {time.strftime('%Y-%m-%dT%H:%M:%S')}\n")
    except OSError as e:
        log(f"WARNING: couldn't record RunStart {run_id} as used in {RUNSTART_LEDGER} ({e}) -- "
            f"only the {RUNSTART_EARLY_MAX_S:.0f} s age limit protects against reusing it")


def runstart_accept(data, started):
    """(accept, reason) for a RunStart message (dict with run_id, received)
    seen by a script that started at `started` (time.time())."""
    run_id = data.get("run_id")
    received = float(data.get("received", 0.0) or 0.0)
    if not received or received >= started:
        return True, "received after this script started"
    if str(run_id) in runstart_used_ids():
        return False, f"run_id={run_id} was already used to arm (in {RUNSTART_LEDGER})"
    age = time.time() - received
    if age > RUNSTART_EARLY_MAX_S:
        return False, f"run_id={run_id} is {age:.0f} s old (> {RUNSTART_EARLY_MAX_S:.0f} s)"
    return True, f"received {age:.0f} s before this script started, not yet used"


last_failure = None   # text of the last "FAIL step N: ..." -- why run_gate returned None


def log(text):
    global last_failure
    if text.startswith("FAIL"):
        last_failure = text
    print(f"[{time.strftime('%H:%M:%S')}] [gate] {text}", flush=True)


async def wait_for(predicate, timeout_s, poll_s=0.2):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if predicate():
            return True
        await asyncio.sleep(poll_s)
    return predicate()


# -- ROS: RunStart in, current_task out (own context, own thread) ------------

class GateRos:
    def __init__(self, run_task):
        self.run_task = run_task
        self.task = TASK_NONE
        self.event = threading.Event()
        self.run_id = None
        env_id = os.environ.get("SEABIRD_RUNSTART_ID")
        if env_id:   # handed off by mission_gate.py's wrapper mode
            self.run_id = env_id
            self.event.set()
            log(f"RunStart {env_id} already received (SEABIRD_RUNSTART_ID, from the gate)")
        self._ctx = rclpy.Context()
        rclpy.init(context=self._ctx)
        self._node = rclpy.create_node("mission_gate", context=self._ctx)
        self._node.create_subscription(
            String, RUN_START_TOPIC, self._on_run_start,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._task_pub = self._node.create_publisher(UInt8, CURRENT_TASK_TOPIC, 10)
        self._node.create_timer(TASK_REPUBLISH_S, self._publish_task)
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

    def _publish_task(self):
        self._task_pub.publish(UInt8(data=self.task))

    def set_task(self, task):
        if task != self.task:
            log(f"current_task {self.task} -> {task}")
        self.task = task
        try:
            self._publish_task()
        except Exception:
            pass   # already closed

    def _on_run_start(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        ok, why = runstart_accept(data, _GATE_STARTED)
        if not ok:
            log(f"ignoring RunStart run_id={data.get('run_id')}: {why}")
            return
        if not self.event.is_set():
            log(f"accepting RunStart run_id={data.get('run_id')}: {why}")
            self.run_id = data.get("run_id")
            self.event.set()

    def close(self):
        if not self._running:
            return
        self._running = False
        self._thread.join(timeout=2.0)
        self._node.destroy_node()
        rclpy.shutdown(context=self._ctx)


# -- PX4 helpers -------------------------------------------------------------

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

    @property
    def name(self):
        return self.mode.name if self.mode else "-"


def _mission_item(lat, lon, rel_alt_m, loiter_s, action_name):
    nan = float("nan")
    return MissionItem(
        latitude_deg=lat, longitude_deg=lon, relative_altitude_m=rel_alt_m,
        speed_m_s=nan, is_fly_through=False, gimbal_pitch_deg=nan, gimbal_yaw_deg=nan,
        camera_action=MissionItem.CameraAction.NONE, loiter_time_s=loiter_s,
        camera_photo_interval_s=nan, acceptance_radius_m=1.0, yaw_deg=nan,
        camera_photo_distance_m=nan,
        vehicle_action=getattr(MissionItem.VehicleAction, action_name))


async def enter_mission_mode(drone, lat, lon, modes, alt_m, hover_s):
    """Upload a hover mission (takeoff alt_m, loiter hover_s, land) at lat/lon
    and switch PX4 to MISSION. True = PX4 reports MISSION."""
    try:
        items = [_mission_item(lat, lon, alt_m, 0.0, "TAKEOFF"),
                 _mission_item(lat, lon, alt_m, hover_s, "NONE"),
                 _mission_item(lat, lon, 0.0, 0.0, "LAND")]
        log(f"uploading hover mission at {lat:.7f}, {lon:.7f}: takeoff {alt_m:g} m, "
            f"hover {hover_s:g} s, land")
        await drone.mission.set_return_to_launch_after_mission(False)
        await drone.mission.upload_mission(MissionPlan(items))
        log("switching PX4 to MISSION (disarmed)")
        await drone.mission.start_mission()
    except Exception as e:   # MissionError, or TypeError from an older MAVSDK
        log(f"MISSION mode not available ({type(e).__name__}: {e})")
        return False
    if await wait_for(lambda: modes.mode == FlightMode.MISSION, 10.0):
        return True
    log(f"PX4 didn't switch to MISSION (now {modes.name})")
    return False


def _local_xy(lat, lon, lat0, lon0):
    k = 111_320.0
    return (lon - lon0) * k * math.cos(math.radians(lat0)), (lat - lat0) * k


def point_in_fence(lat, lon, polygons):
    """(inside any polygon, distance in m to the nearest fence edge)."""
    inside, best = False, float("inf")
    for poly in polygons:
        pts = [_local_xy(a, b, lat, lon) for a, b in poly]   # the UAV at (0, 0)
        hit = False
        for i in range(len(pts)):
            (x1, y1), (x2, y2) = pts[i - 1], pts[i]
            if (y1 > 0) != (y2 > 0) and 0 < (x2 - x1) * (0 - y1) / (y2 - y1) + x1:
                hit = not hit
            dx, dy = x2 - x1, y2 - y1
            t = max(0.0, min(1.0, -(x1 * dx + y1 * dy) / (dx * dx + dy * dy or 1.0)))
            best = min(best, math.hypot(x1 + t * dx, y1 + t * dy))
        inside = inside or hit
    return inside, best


async def read_fence(drone):
    """PX4's inclusion polygons [[(lat, lon), ...], ...], or None if unreadable."""
    if not hasattr(drone.mission_raw, "download_geofence"):
        log("ERROR: this MAVSDK has no mission_raw.download_geofence()")
        return None
    try:
        items = await asyncio.wait_for(drone.mission_raw.download_geofence(), FENCE_TIMEOUT_S)
    except Exception as e:
        log(f"ERROR: geofence readback failed: {type(e).__name__}: {e}")
        return None
    polygons, current = [], []
    for item in sorted(items, key=lambda it: it.seq):
        if item.command != FENCE_INCLUSION_VERTEX:
            continue
        current.append((item.x * 1e-7, item.y * 1e-7))
        if len(current) >= int(item.param1):
            if len(current) >= 3:
                polygons.append(current)
            current = []
    return polygons


# -- The gate ----------------------------------------------------------------

class GateResult:
    """What run_gate() hands the flight script."""

    def __init__(self, drone, ros, modes, position, px4_ready, fence, in_mission):
        self.drone = drone
        self.run_id = ros.run_id
        self.modes = modes              # .mode / .autonomous, kept up to date
        self.position = position        # {"lat", "lon", "abs_alt"}, kept up to date
        self.px4_ready = px4_ready      # {"gpos", "home"}, kept up to date
        self.fence = fence              # inclusion polygons [(lat, lon), ...] or []
        self.in_mission = in_mission    # PX4 in MISSION with the hover mission loaded
        self._ros = ros

    def set_task(self, task):
        """Publish current_task (e.g. TASK_NONE when the attempt is over)."""
        self._ros.set_task(task)

    async def wait_ready(self, deadline):
        """Wait until PX4 has global position + home again (GPS dropout after
        RunStart). False = deadline (time.monotonic()) passed."""
        last = 0.0
        while not (self.px4_ready["gpos"] and self.px4_ready["home"]):
            now = time.monotonic()
            if now > deadline:
                return False
            if now - last >= POSITION_REPORT_S:
                missing = [n for n, ok in (("global position", self.px4_ready["gpos"]),
                                           ("home", self.px4_ready["home"])) if not ok]
                log(f"waiting for PX4 to be ready (missing {', '.join(missing)}); "
                    f"{deadline - now:.0f} s left")
                last = now
            await asyncio.sleep(0.5)
        return True

    async def enter_mission(self, alt_m, hover_s):
        """(Re-)upload the hover mission at the current position and switch to MISSION."""
        self.in_mission = await enter_mission_mode(
            self.drone, self.position["lat"], self.position["lon"], self.modes, alt_m, hover_s)
        return self.in_mission

    def close(self):
        """Stop the ROS node (current_task stops being published)."""
        self._ros.close()


async def run_gate(drone, run_task=TASK_NONE, connect=False, mission_alt_m=3.0,
                   mission_hover_s=20.0, min_ceiling_m=None):
    """
    Run steps 1-5 on `drone` (an mavsdk.System). connect=True connects it to
    MAVSDK_ADDRESS first; otherwise the caller already did.
      run_task        -- current_task published once RunStart arrives
      mission_alt_m / mission_hover_s -- the hover mission uploaded for MISSION
      min_ceiling_m   -- GF_MAX_VER_DIST must be above this (default mission_alt_m)
    Returns a GateResult, or None if a step failed (nothing armed).
    """
    min_ceiling_m = mission_alt_m if min_ceiling_m is None else min_ceiling_m
    log("=" * 60)
    log(f"RobotX UAV start gate  (ground mode {GROUND_AUTONOMOUS_MODE}, "
        f"source {AUTONOMOUS_MODE_SOURCE}, REQUIRE_FENCE={REQUIRE_FENCE}, run_task={run_task})")
    log("=" * 60)
    ros = GateRos(run_task)
    ros.set_task(TASK_NONE)
    log(f"current_task {TASK_NONE} on {CURRENT_TASK_TOPIC}; RunStart on {RUN_START_TOPIC}")
    ok = False
    try:
        result = await _steps(drone, ros, run_task, connect, mission_alt_m, mission_hover_s,
                              min_ceiling_m)
        ok = result is not None
        return result
    finally:
        if not ok:
            ros.close()


async def _steps(drone, ros, run_task, connect, mission_alt_m, mission_hover_s, min_ceiling_m):
    # 1. PX4 link, position, home.
    if connect:
        log(f"connecting to PX4 at {MAVSDK_ADDRESS} ...")
        await drone.connect(system_address=MAVSDK_ADDRESS)

    async def connected():
        async for state in drone.core.connection_state():
            if state.is_connected:
                return
    try:
        await asyncio.wait_for(connected(), timeout=CONNECT_TIMEOUT_S)
    except asyncio.TimeoutError:
        log(f"FAIL step 1: no PX4 heartbeat after {CONNECT_TIMEOUT_S:.0f} s")
        return None
    log("step 1: PX4 connected")

    modes = ModeWatcher(drone)
    asyncio.ensure_future(modes.run())
    position = {"lat": None, "lon": None, "abs_alt": None}
    px4_ready = {"gpos": False, "home": False}

    async def track_position():
        async for p in drone.telemetry.position():
            position["lat"], position["lon"] = p.latitude_deg, p.longitude_deg
            position["abs_alt"] = p.absolute_altitude_m
    asyncio.ensure_future(track_position())

    async def track_health():
        async for h in drone.telemetry.health():
            px4_ready["gpos"], px4_ready["home"] = h.is_global_position_ok, h.is_home_position_ok
    asyncio.ensure_future(track_health())

    async def px4_text():
        try:
            async for st in drone.telemetry.status_text():
                log(f"PX4 [{getattr(st.type, 'name', st.type)}]: {st.text}")
        except Exception as e:
            log(f"(PX4 message log stopped: {type(e).__name__}: {e})")
    asyncio.ensure_future(px4_text())

    t0, last = time.monotonic(), -POSITION_REPORT_S
    while not (px4_ready["gpos"] and px4_ready["home"] and position["lat"] is not None):
        elapsed = time.monotonic() - t0
        if elapsed > POSITION_TIMEOUT_S:
            log(f"FAIL step 1: no global position/home after {POSITION_TIMEOUT_S:.0f} s "
                f"(check the GPS fix)")
            return None
        if elapsed - last >= POSITION_REPORT_S:
            missing = [n for n, ok in (("global position", px4_ready["gpos"]),
                                       ("home", px4_ready["home"])) if not ok] or ["position"]
            log(f"step 1: waiting ({elapsed:.0f} s): missing {', '.join(missing)}")
            last = elapsed
        await asyncio.sleep(0.5)
    log(f"step 1: global position OK, home set ({position['lat']:.7f}, {position['lon']:.7f})")

    # 2. Geofence on the UAV.
    def fence_problem(text):
        log(("FAIL step 2: " if REQUIRE_FENCE else "WARNING step 2 (REQUIRE_FENCE=False): ") + text)
        return REQUIRE_FENCE

    fence = await read_fence(drone)
    if fence is None:
        if fence_problem("couldn't read the geofence back from PX4"):
            return None
        fence = []
    elif not fence:
        if fence_problem("PX4 has no inclusion-polygon geofence (is the course fence VALID "
                         "on the UAV? -- GUI Geofence row)"):
            return None
    else:
        inside, edge_m = point_in_fence(position["lat"], position["lon"], fence)
        if not inside:
            if fence_problem(f"the UAV is OUTSIDE the PX4 geofence ({edge_m:.1f} m from it)"):
                return None
        else:
            log(f"step 2: PX4 geofence OK: {len(fence)} polygon(s), UAV inside, "
                f"{edge_m:.1f} m from the nearest edge")
    try:
        ceiling = await drone.param.get_param_float("GF_MAX_VER_DIST")
        if ceiling > 0 and ceiling <= min_ceiling_m:
            if fence_problem(f"fence ceiling GF_MAX_VER_DIST={ceiling:.1f} m is not above the "
                             f"flight altitude {min_ceiling_m:g} m"):
                return None
        else:
            log(f"step 2: fence ceiling GF_MAX_VER_DIST="
                f"{'off' if ceiling <= 0 else f'{ceiling:.1f} m above home'}")
    except Exception as e:
        log(f"step 2: couldn't read GF_MAX_VER_DIST ({type(e).__name__}) -- not checked")

    # 3. current_task is already TASK_NONE (published since the gate started).
    log(f"step 3: current_task {TASK_NONE} (none) until RunStart, then {run_task}")

    # 4. Autonomous on the ground, disarmed.
    in_mission = False
    if AUTONOMOUS_MODE_SOURCE == "script":
        if GROUND_AUTONOMOUS_MODE == "mission":
            in_mission = await enter_mission_mode(drone, position["lat"], position["lon"], modes,
                                                  mission_alt_m, mission_hover_s)
        if not in_mission:
            log("switching PX4 to Hold (disarmed)")
            try:
                await drone.action.hold()
            except ActionError as e:
                log(f"PX4 refused Hold ({e}) -- pilot, switch to an autonomous mode")
    else:
        log("waiting for the pilot to switch to an autonomous mode...")
    if not await wait_for(lambda: modes.autonomous, AUTONOMOUS_TIMEOUT_S):
        log(f"FAIL step 4: not in an autonomous mode after {AUTONOMOUS_TIMEOUT_S:.0f} s")
        return None
    log(f"step 4: UAV AUTONOMOUS in {modes.name} -- RoboCommand heartbeat STATE_AUTO")

    # 5. RunStart.
    log("step 5: holding for RoboCommand's RunStart...")
    t0, warned = time.monotonic(), False
    while not ros.event.is_set():
        if time.monotonic() - t0 > RUNSTART_WAIT_MAX_S:
            log(f"FAIL step 5: no RunStart after {RUNSTART_WAIT_MAX_S:.0f} s")
            return None
        if not modes.autonomous and not warned:
            log(f"WARNING: left autonomous mode ({modes.name}) before RunStart -- RoboCommand "
                f"won't start the run until it is autonomous again")
            warned = True
        elif modes.autonomous:
            warned = False
        await asyncio.sleep(0.2)
    log(f"step 5: RunStart received (run_id={ros.run_id}) in {modes.name} -- gate open")
    ros.set_task(run_task)
    return GateResult(drone, ros, modes, position, px4_ready, fence,
                      in_mission and modes.mode == FlightMode.MISSION)


# -- Wrapper mode: gate, then exec another flight script ----------------------

def _stop_mavsdk_server(drone):
    stop = getattr(drone, "_stop_mavsdk_server", None)
    try:
        if callable(stop):
            stop()
        else:
            proc = getattr(drone, "_server_process", None)
            if proc is not None:
                proc.kill()
                proc.wait(timeout=5)
    except Exception as e:
        log(f"WARNING: stopping mavsdk_server: {type(e).__name__}: {e}")


def _port_free(port, timeout_s=10.0):
    end = time.monotonic() + timeout_s
    while True:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.bind(("0.0.0.0", port))
            return True
        except OSError:
            if time.monotonic() > end:
                return False
            time.sleep(0.2)
        finally:
            s.close()


def main():
    if len(sys.argv) < 2:
        print("usage: python3 mission_gate.py <flight_script.py> [its args...]", file=sys.stderr)
        sys.exit(2)
    script = sys.argv[1]
    if not os.path.isabs(script) and not os.path.exists(script):
        script = os.path.join(os.path.dirname(os.path.abspath(__file__)), script)
    if not os.path.isfile(script):
        log(f"ERROR: flight script not found: {sys.argv[1]}")
        sys.exit(2)

    drone = System()
    holder = {}

    async def go():
        holder["gate"] = await run_gate(drone, connect=True)
        return holder["gate"]
    try:
        gate = asyncio.run(go())
    except KeyboardInterrupt:
        log("interrupted -- not handing off")
        gate = None
    if gate is not None:
        gate.close()
    _stop_mavsdk_server(drone)
    if gate is None:
        sys.exit(1)
    if not _port_free(HANDOFF_PORT):
        log(f"ERROR: UDP {HANDOFF_PORT} still in use after stopping mavsdk_server -- not "
            f"handing off (the next script couldn't reach PX4)")
        sys.exit(1)
    env = dict(os.environ, SEABIRD_RUNSTART_ID=str(gate.run_id),
               SEABIRD_RUNSTART_RECEIVED=f"{time.time():.3f}")
    argv = [sys.executable, "-u", script, *sys.argv[2:]]
    log(f"handing off: {' '.join(argv[2:])}  (SEABIRD_RUNSTART_ID={gate.run_id})")
    os.execve(sys.executable, argv, env)


if __name__ == "__main__":
    main()
