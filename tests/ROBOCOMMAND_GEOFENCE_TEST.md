# RoboCommand geofence + UAV ground station

How to run the UAV ground station on the bench (**Test**) and at Ground /
Flight Validation and the competition (**Competition**). It covers the
geofence round trip that RoboNation checks, the VALID / INVALID
demonstration, Task 1 reporting, and the ground-station GUI that runs all of
it.

---

## 1. What the system does

### Geofence round trip (what Validation checks)

```
RoboCommand ──RxCourse (MQTT, retained)──► seaowls_ocs (WSL)
   ▲                                          │ /seaowls/uav1/mission/geofence
   │                                          ▼
   │                         beacon_mavlink_ground.py (WSL)
   │                           1. VALID / INVALID check of the course + altitude limit
   │                              (invalid → REFUSED with reason, nothing uploaded)
   │                           2. inset the boundary 1 m, ceiling 1 m under the limit
   │                           3. upload to PX4 over ELRS ─────────────► UAV
   │                           4. READ BACK fence + ceiling from PX4 ◄── UAV
   │                           5. check the readback against the course
   │                                          │ /seaowls/uav1/mission/geofence_onboard
   │                                          ▼
   └──RunDeclaration (uav_geofence = the fence READ BACK from the UAV)── seaowls_ocs
                                              re-checks it, then declares
```

- **The OCS never declares the raw course corners**, only what the UAV
  actually holds. Without the drone link there is no RunDeclaration and no
  RunStart.
- **INVALID (REFUSED with the reason, not uploaded, not declared):**
  - the course isn't a closed ring, has fewer than 3 distinct corners, has
    all corners on a line, or has invalid coordinates;
  - it's too small for the 1 m inset (under ~2.5 m across);
  - the altitude limit is 0 or less, above the competition maximum (50 m HAE
    by default), or not above home;
  - the readback leaves the course + transit area by more than 1 m (the same
    rule as RoboCommand's own checker).
- **Not invalid:**
  - **Corner order.** If the corners arrive in an order whose edges cross,
    they're reordered into a non-crossing polygon through the same corners
    (by angle around their centre), and that is what's inset, uploaded and
    checked. A course whose edges don't cross is used exactly as received.
  - **Size.** Under 5 m or over 1 km across is a **warning** ("check the
    coordinates"), not a refusal.
  - Warnings show under the GUI's Geofence status and in the Ground log as
    `[fence] WARNING …`.
  - RoboCommand checks the declared fence against its *own* published
    course. If RoboCommand itself publishes crossing corners, its checker may
    still refuse the declaration; that's on their side.
- The fence is inset **1 m** inside the published boundary (RoboNation's
  advice), and PX4's ceiling is set **1 m** under the altitude limit.

### Heartbeats and Task 1

- The OCS sends a RoboCommand heartbeat for UAV1 at 2 Hz. It includes
  `uav_registration` and `flight_phase` (1 GROUNDED / 2 AIRBORNE).
  `current_task` is 2 while the sweep (or the mock beacon publisher) runs
  Task 1, and 1 after it.
- `uav_pilot_callsign` goes in the RunDeclaration.

### UAV identity formats (the OCS refuses to declare otherwise)

| Field | Format | Example |
|---|---|---|
| `uav_pilot_callsign` | `<pilot first name>-<UAPL ID>`, or just `<pilot first name>` during Ground Validation (no UAPL ID yet) | `Samantha` / `Samantha-UA0042` |
| `uav_registration` | `<team code>-<CAAS sticker number>`, **without** the sticker's `AA` | `FLAU-12345` |

The GUI builds both from separate fields and strips the `AA` for you.

---

## 2. Network (Team Handbook 3.4.3)

The RoboCommand-facing Ethernet interface **must use DHCP**. Never configure
a static address.

| Network | Your laptop (DHCP) | RoboCommand |
|---|---|---|
| Ground / Flight Validation | `192.168.69.15–254` | `192.168.69.2–6` (the instance you're cabled to) |
| Course Alpha / Bravo / Charlie / Delta | `192.168.65–68.15–254` | `192.168.65–68.2` |
| Team Village demo | DHCP | given with the demo instructions |

**The RoboCommand address is your DHCP default gateway.** Leave the GUI's
"RoboCommand host" blank and it uses the gateway automatically. Also on that
interface:
- no network bridge;
- no Internet Connection Sharing;
- no IP forwarding: the ELRS and Wi-Fi links must not be routed through it.

Course networks have **no Internet**, so don't use web access as a test.
MQTT is TCP **1883**. The GUI's Competition preflight checks all of this.

---

## 3. One-time setup

Do these once per laptop / drone, not every run.

1. **WSL mirrored networking**, so the OCS in WSL can use the Windows
   Ethernet adapter and the router uplink. `%UserProfile%\.wslconfig` must
   contain:
   ```
   [wsl2]
   networkingMode=mirrored
   ```
   Then run `wsl --shutdown` and reopen WSL.
2. **Build the OCS.** Rebuild after any change to `seaowls-robotx-robocommand`,
   including a new file:
   ```bash
   cd "/mnt/c/Users/Precision 3571/seaowls-robotx-robocommand/ros2_ws"
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install --packages-select seaowls_robocommand
   ```
3. **Python on Windows:** `py -3.10` with `pyserial` and `pymavlink` (for the
   router). tkinter comes with Python. Git for Windows is used by the
   protobuf check.
4. **PX4 parameters** (QGC → Parameters):

   | Parameter | Value |
   |---|---|
   | `GF_ACTION` | `1` (warning) on the bench, `2`/`3` (hold/return) for flight. At 0 the fence isn't enforced. |
   | `COM_RC_OVERRIDE` | non-zero, so the sticks can take over from OFFBOARD |

   `GF_MAX_VER_DIST` is set automatically. Don't set it by hand.
5. **MAVSDK on the drone** (only for the sweep's fence mode), inside the
   container that runs the sweep:
   ```bash
   python3.10 -c "import mavsdk.mission_raw as m; print(hasattr(m.MissionRaw, 'download_geofence'))"
   ```
   It must print `True`.

---

## 4. Starting the ground-station GUI

The GUI replaces `start_seabird_ground.ps1`. On Windows, in PowerShell:

```powershell
cd "C:\Users\Precision 3571\EECS-seaOwlRobotX\elrs_windows_routing"
py -3.10 seabird_ground_gui.py
```

### The window

- **Profile** at the top: **Competition** or **Test**. Each keeps its own
  settings, saved in `seabird_ground_gui_settings.json` next to the script
  when you press **Save settings** (also on Start / Preflight / quit).
- **Settings (both profiles):**
  - ELRS COM port and baud (Device Manager shows the port; default 460800);
  - Team ID, Vehicles (`UAV1`, or `USV1,UAV1` with the boat);
  - pilot first name, UAPL ID (blank for Ground Validation), team code,
    CAAS sticker number;
  - altitude limit (m HAE, what the OCS sends) and competition maximum
    (default 50);
  - fence inset and ceiling margin (default 1 m each);
  - WSL distro and RoboCommand repo path.

  The line under the fields shows the exact `uav_pilot_callsign` and
  `uav_registration` that will be sent.
- **Send RunDeclaration automatically when the UAV fence is VALID**
  (checkbox, **unticked by default**). Unticked, the OCS prepares and
  checks the RunDeclaration but only sends it when you press **Send
  RunDeclaration** on the Declaration row, so you decide when RoboCommand
  gets it, before going autonomous. Ticked, it's sent as soon as it's ready.
  Outside the GUI the OCS defaults to automatic (`OCS_DECLARE_MODE=auto`;
  `manual` waits for `std_msgs/Empty` on `/seaowls/ocs/send_run_declaration`).
- **Competition box:** the RoboCommand-facing adapter (↻ refreshes the list),
  and the RoboCommand host. Leave the host blank to use the DHCP gateway.
- **Test box:**
  - **Centre the course on the drone's GPS position** (ticked by default).
    The course is centred on the drone's latest 3D GPS fix, which the Ground
    process reports every 5 s and the "Drone position" row shows with its
    age.
  - **Manual centre `LAT,LON`:** used automatically whenever there's no fix
    in the last 30 s (indoors), or always if the box above is unticked.
  - The course size, and its kind. Each entry names its expected outcome: `VALID course`, `VALID after repair: corners in crossing order`, `VALID + warning: too large (> 1 km)`, or one of the `INVALID: …` entries.
  - **Publish mock course** and **Clear mock course**.
- **Home HAE when there's no GPS fix (m)** (Test box, default `0`): indoors the
  drone has no 3D GPS fix or home, so the ceiling can't be converted to PX4's
  above-home value. This height is used instead, so the round trip (ceiling,
  readback, declaration) still finishes. A real fix that arrives first takes
  priority. When it's used, the Geofence row shows
  `[ASSUMED home HAE: bench only]`. Blank means 0. The GUI never passes this
  in Competition.
- **Run preflight:** a table of ✓ OK / ⚠ WARN / ✗ FAIL checks:
  - the protobuf is the latest RoboCommand version (needs Internet; on a
    course network it warns instead);
  - callsign and registration formats, and the altitude limit;
  - WSL mirrored networking;
  - Competition: DHCP, address in the course subnet, gateway = RoboCommand,
    no bridge / ICS / forwarding, MQTT 1883 reachable;
  - Test: local broker running, mock centre set.
- **Start all / Stop all**, plus Start / Stop per process:
  - **Router:** `elrs_windows_router.py` on Windows;
  - **Broker:** Mosquitto in WSL, Test only;
  - **OCS:** `seaowls_ocs` in WSL, with the settings above;
  - **Ground:** `beacon_mavlink_ground.py --publish-ros` in WSL.

  Start all asks before starting with an invalid callsign, registration or
  altitude, which is what you want for the INVALID altitude demo.
- **Status:**
  - Router: radio bytes coming in;
  - OCS: MQTT connected;
  - Ground: receiving UAV MAVLink;
  - **Heartbeats:** UAV1 → RoboCommand at ~2 Hz;
  - **Geofence:** VALID / REFUSED / FAILED + reason;
  - **Declaration:** `READY` (waiting for you), `sent seq N` with the fence and altitude, or why
    not. The **Send RunDeclaration** button on this row sends it (see below);
  - **RunStart.**
- **Log tabs** per process. Each start's logs are also kept in
  `logs/ground_gui/<timestamp>/`, never overwritten.

> **Give it about a minute after Start all.** ROS in WSL can be slow to
> start: on this laptop a node took 6–11 s to create and the OCS ~50 s to
> connect. The OCS status turns green when MQTT connects.

---

## 5. Bench test (Test profile, props off)

### 5.1 Drone (VOXL SSH)

```bash
systemctl status voxl-px4 voxl-mavlink-server elrs-mavlink     # all active
cd /home/voxluser/seabirdDocker/EECS-seaOwlRobotX
python3 beacon_mavlink_bridge.py --mavlink-endpoint udpout:10.200.0.2:14550
```

**One-time deployment for the RunStart relay.** RoboCommand's RunStart
reaches the sweep through the drone's ELRS bridge, which now also copies
ground STATUSTEXT starting `SEABIRD_` to `127.0.0.1:14570`, where
`beacon_mavlink_bridge.py` listens. Copy the updated
`voxl/elrs_mavlink_bridge_ratelimited.py` over the drone's
`/usr/local/bin/elrs_mavlink_bridge.py` (the `elrs-mavlink` service), then
`systemctl restart elrs-mavlink`. Its startup log should include
`Ground STATUSTEXT 'SEABIRD_*' also copied onboard to 127.0.0.1:14570`.

The ELRS bridge runs in the `rfdns` network namespace. If
`beacon_mavlink_bridge.py` runs outside it (it reaches the bridge at
`10.200.0.2`), the copy must go to an address it can reach. Start the ELRS
bridge with `--onboard-ip`, and `beacon_mavlink_bridge.py` with
`--onboard-listen udpin:<that ip>:14570`. Check with the bench test below:
the GUI's RunStart row must reach **UAV acknowledged**.

### 5.2 GUI

1. Select **Test**, fill in the settings, then **Run preflight**.
2. Leave **Centre the course on the drone's GPS position** ticked and the kind
   at `VALID course`. For indoors, also fill in **Manual centre** (used when
   there's no fix). **Home HAE when there's no GPS fix** defaults to `0`.
3. **Start all.** Wait for the **Drone position** row to show a 3D fix, and for:
   - Router: radio bytes > 0;
   - OCS: connected;
   - Ground: receiving UAV MAVLink.
4. **Publish mock course.** Expect:
   - **Geofence:** `UPLOADING`, then `VALID — course MOCK (on the UAV, read back)`;
   - **Declaration:** `READY — course MOCK, UAV1 fence 5 points 39.0m HAE: press Send RunDeclaration…`;
   - **Heartbeats:** green.
5. **Send RunDeclaration** (Declaration row). Expect
   `sent seq 1, course MOCK uav_geofence=5 points (read back from UAV1) altitude=39.0m HAE callsign=…`.
   It takes a few seconds (the button starts a short ROS process in WSL). Pressing it again sends
   a fresh declaration (next seq).
6. In QGC, under Plan → Fence → Download, the fence should sit 1 m inside
   the course.

**RunStart on the bench.** After publishing, the mock keeps running as
RoboCommand (`--serve`), with its log in the Mock tab:
- It checks your RunDeclaration as RoboCommand does: vehicles listed, no
  `TIER_UNKNOWN`, a callsign, the UAV fence within the mock course (same 1 m
  rule as RoboNation's checker), and the altitude no higher than the GUI's
  **Competition maximum**. It logs `ACCEPTED`, or `REFUSED` with the reason;
  like real RoboCommand it sends nothing back for a refusal.
- It then sends **RunStart** once every declared vehicle's heartbeat reports
  `STATE_AUTO`. With **"Mock RoboCommand sends RunStart without waiting for
  AUTO"** ticked (the default, for props-off bench work), it sends RunStart
  3 s after an accepted declaration instead.
- The GUI shows **RunStart** `verified: run_id N — relaying to the UAV…` →
  `UAV acknowledged, the run may begin`, and **Commands** `last received:
  run_start`. The sweep's hold is released (with `RUNSTART_GATE = True`).

Untick the box to rehearse the real condition: the drone must be in an AUTO
mode (OFFBOARD, Hold, Mission…) before RunStart arrives. **Clear mock
course** stops the mock RoboCommand. Publishing again restarts it, and each
new declaration needs a new RunStart.

### 5.3 Rehearse the INVALID demonstration

| Demonstration | How (Test profile) | Expect |
|---|---|---|
| INVALID boundary | Course kind → one of the `INVALID: …` entries (open ring, only two corners, bad coordinates, too small for the inset), then **Publish mock course** | **Geofence: REFUSED** + reason (e.g. "course boundary not closed…"). Nothing uploaded to the UAV, no new declaration. |
| Crossing corner order | Course kind → `VALID after repair: corners in crossing order` | **VALID**, with `⚠ … reordered into a non-crossing polygon…` under it |
| Implausible size | Course kind → `VALID + warning: too large (> 1 km)` | **VALID**, with `⚠ … 3532 m across … check the coordinates` |
| INVALID altitude | Stop all → set altitude limit to `0` or `60` → Start all (confirm the prompt) → Publish a `VALID course` | **Geofence: REFUSED**, e.g. "altitude limit 60 m HAE is above the maximum 50 m HAE" |
| VALID again | Kind `VALID course`, altitude `40`, Publish, then **Send RunDeclaration** | **VALID**, then `sent seq N` |

### 5.4 Task 1 reporting check

On the VOXL, with `beacon_mavlink_bridge.py` running:

```bash
python3 tests/mock_beacon_publisher.py --field --lat <lat> --lon <lon>
```

This reports Task 1 started (`current_task` 2) while it runs and stood down
(1) when it ends, including on Ctrl-C. Its beacons produce SafePassage
reports, which the OCS logs as `SafePassage UAV1 -> RoboCommand`. Use
`--no-task` when the sweep is running.

### 5.5 Task 4 (Dynamic Incident Response) rehearsal

The OCS handles all three Task 4 chains (Handbook 3.4.12) and routes each
command to the targeted vehicle(s). A command addressed by type goes to every
vehicle of that type in **Vehicles**; one with a `vehicle_id` goes only to
that vehicle:

| Tier | RoboCommand → | OCS / vehicle response |
|---|---|---|
| Core | AssistanceRequest | IncidentAck, then ReadinessReport when the vehicle is on station; RoboCommand answers ReadinessConfirm |
| Advanced | KeepOutZone, then AllClear | IncidentAck to each |
| Disruptive | MovingObjectAlert | none (routed to the vehicles only) |

**In the GUI (Test profile):**
1. Put the target vehicle in **Vehicles** (e.g. `USV1,UAV1`) and Start all.
2. Choose the vehicle type next to **Rehearse Task 4** and press it. It plays
   RoboCommand's side (`tests/mock_robocommand_task4.py`).
3. Watch the **Task 4** row: AssistanceRequest → IncidentAck sent → (waiting
   for the vehicle). When the vehicle is on station, press **Send
   ReadinessReport** (or let the vehicle's own code report it, below). Then
   it moves on: ReadinessConfirm "chain complete" → KeepOutZone/AllClear
   acks → MovingObjectAlert routed.
4. The Task 4 log tab ends with `RESULT: PASS -- every expected response arrived`.

**Settings:** "Task 4: vehicles send their own IncidentAck" (unticked by
default: the OCS acknowledges as soon as it has routed the command). At the
competition, Task 4 commands arrive from RoboCommand and the same Task 4 row
and **Send ReadinessReport** button apply.

**For the vehicle software teams (ROS interface):**
- Commands arrive as `seaowls_interfaces/Task4Command` on
  `/seaowls/<vehicle lowercase>/task4/command` (`kind`, `command_seq`,
  position, `radius_m`, heading/speed for moving objects, `assistance_seq`
  for READINESS_CONFIRM).
- On station after an ASSISTANCE_REQUEST: publish `std_msgs/UInt32` (the
  `command_seq`) on `/seaowls/<vehicle>/task4/ready`, and the OCS sends the
  ReadinessReport.
- Only with "vehicles send their own IncidentAck" ticked: acknowledge with
  `std_msgs/UInt32` on `/seaowls/<vehicle>/task4/ack`.
- `seaowls_interfaces` gained `Task4Command.msg` and `PipelineSurvey.msg`:
  rebuild the workspace
  (`colcon build --packages-select seaowls_interfaces seaowls_robocommand`).

### 5.6 Tasks 2 and 3 reports

| Report (Handbook 3.4.12) | Sent as | How it reaches the OCS |
|---|---|---|
| Task 2 `PipelineSurveyReport` | UUV1 | The UUV publishes `seaowls_interfaces/PipelineSurvey` on `/seaowls/uuv1/report/pipeline_survey`: active (GREEN) buoy position and `segments` (0 unknown / 1 intact / 2 damaged, from the active buoy end), once, after the survey and any repair. Needs `UUV1` in **Vehicles**. |
| Tasks 2/3 `ResourceDeliveryRequest` (detecting vehicle) | USV1 | The USV publishes on `/seaowls/usv1/report/resource_delivery` (unchanged). The OCS also passes it to the UAV on `/seaowls/uav1/task/resource_delivery`. |
| Tasks 2/3 `ResourceDeliveryRequest` echo when delivery begins (Advanced/Disruptive) | UAV1 | The UAV publishes the same request on `/seaowls/uav1/report/resource_delivery`, or the operator presses **Echo delivery request (UAV)** on the GUI's Reports row when the UAV starts the delivery. |
| Task 3 `DockingReport`, `FirefightingReport` | USV1 | Unchanged |

The GUI's **Reports** row shows the last report sent (vehicle and report
seq), a delivery request waiting for the UAV's echo, and any report the OCS
refused, e.g. an invalid segment value or `UUV1` not in Vehicles.

---

## 6. Ground / Flight Validation and competition (Competition profile)

**Before leaving for the course (no Internet there):**
- **Sync the laptop clock.** `sent_at` on every message is the OCS clock
  (Handbook 3.4.8). In Windows Settings → Time & language → Date & time,
  turn on "Set time automatically" and press **Sync now**, then run
  `wsl --shutdown` so WSL takes the corrected time. Preflight's **WSL
  clock** and **Windows clock sync** rows must be ✓. On 2026-11-07 this
  laptop had the Windows Time service stopped and WSL 2 hours off from
  Windows.
- **Confirm the protobuf is the latest** (preflight ✓ while you still have
  Internet).

**During a run, never restart the OCS.** Each vehicle's report sequence
counter lives in the OCS and must not reset during an active run (3.4.15).
After RunStart, the GUI asks before stopping the OCS (Stop, Stop all, Clear
mock course, closing the window).

**Rate limit (3.4.5, at most 5 messages/s per system):** UAV1 sends 2
heartbeats/s, and SafePassage updates are merged to at most one per second
(the latest buoy field is always sent).

1. Plug the provided RJ-45 cable into the laptop's Ethernet port. Check that
   the adapter is on DHCP (Windows Settings → Network → Ethernet → IP
   assignment: Automatic).
2. In the GUI select **Competition**, choose that **Ethernet** adapter, and
   leave **RoboCommand host blank**.
3. **Callsign:** at Ground Validation, pilot first name only (no UAPL ID).
   Once you have a UAPL ID, add it.
4. **Run preflight.** Everything should be ✓, except a protobuf ⚠ if there's
   no Internet. Confirm the protobuf before leaving the pits.
5. Start the drone side (5.1), then **Start all**.
6. RoboCommand publishes the course and the round trip runs by itself.
   Watch for:
   - Geofence: `VALID — course <id> (on the UAV, read back)`;
   - Declaration: `READY — course <id> …`;
   - Heartbeats: green.
7. **Before starting the autonomous mission**, press **Send RunDeclaration**.
   Expect `Declaration: sent seq N … read back from UAV1`. RoboCommand
   doesn't reply to a declaration: **RunStart** arrives once every declared
   vehicle reports AUTO. No RunStart after going AUTO means the declaration
   was refused or a vehicle isn't AUTO.

What to show the judges:
- **heartbeats:** the Heartbeats row and OCS log, complete at 2 Hz;
- **retrieve → send to UAV → retrieve from UAV → relay:** the Ground log's
  `Fence accepted by PX4` → `Read back from PX4` → `VALID`, then the OCS's
  RunDeclaration line;
- **VALID / INVALID boundaries and altitudes:** whatever RoboCommand sends is
  judged and shown as VALID, or REFUSED with the reason.

---

## 7. Flight test (sweep fence mode)

In `sweep_rrt_overfly_downward_center.py`:

```python
SEARCH_AREA_SHAPE    = "fence"    # or "disc" for the 10 m circle
FENCE_MARGIN_M       = 2.0        # search stays this far inside the fence
BENCH_TEST           = False      # BENCH_TEST returns before the fence is read
RUNSTART_GATE        = True       # hold in OFFBOARD until RoboCommand's RunStart
RUNSTART_WAIT_MAX_S  = 600.0      # no RunStart by then -> return to launch
```

**RunStart gate (Handbook 3.4.10: "only when the OCS receives and verifies
the RunStart should any vehicle begin their run").** After takeoff and
OFFBOARD, the sweep holds position, which counts as AUTO for RoboCommand,
until RunStart arrives on `/seabird/run_start`. Only then does it set
`current_task` 2 and start searching. The full chain:
1. OCS verifies RunStart (GUI RunStart row: `verified: run_id N — relaying to the UAV…`);
2. ground relays it up ELRS;
3. the drone's ELRS bridge copies it onboard;
4. `beacon_mavlink_bridge.py` publishes it and acknowledges (GUI: `UAV acknowledged, the run may begin`).

The sweep logs `Holding position in OFFBOARD until RoboCommand's RunStart
arrives…`, then `✓ RunStart received (run_id=N) — beginning the run`.
Without a RunStart in `RUNSTART_WAIT_MAX_S` it doesn't search and returns
to launch. Set `RUNSTART_GATE = False` for bench and sim tests without
RoboCommand.

The fence must be **VALID (on the UAV)** before the sweep starts. The sweep
reads it once, before arming. In the drone's container:

```bash
python3.10 sweep_rrt_overfly_downward_center.py
```

Expect `[rrt] ✓ PX4 geofence: 1 inclusion polygon(s), … m², margin 2.0 m`.
The sweep aborts before arming if there's no usable fence, or if takeoff is
outside it. `current_task` goes 1 → 2 when the run begins (after RunStart)
and 2 → 1 after landing. For Gazebo, see [SIM_GEOFENCE_TEST.md](SIM_GEOFENCE_TEST.md).

---

## 8. Without the GUI (manual commands)

The GUI runs exactly these. In WSL, each in its own terminal, after
`source /opt/ros/humble/setup.bash` and the robocommand
`ros2_ws/install/setup.bash`:

```bash
# Test only: broker + mock course
mosquitto -p 1883 -v
python3 tests/mock_robocommand_course.py --center <lat>,<lon> [--invalid self-intersecting]

# OCS
export ROBOCOMMAND_HOST=<gateway or localhost> ROBOCOMMAND_PORT=1883 TEAM_ID=FLAU VEHICLES=UAV1
export UAV_GEOFENCE_MAX_ALTITUDE_HAE_M=40 UAV_MAX_ALTITUDE_HAE_M=50
export UAV_PILOT_CALLSIGN=Samantha UAV_REGISTRATION=FLAU-12345
ros2 run seaowls_robocommand seaowls_ocs

# Ground
python3 beacon_mavlink_ground.py --mavlink-endpoint udpin:0.0.0.0:14551 --publish-ros \
    --fence-inset-m 1 --ceiling-margin-m 1 --max-alt-limit-hae-m 50
```

On Windows: `py -3.10 elrs_windows_router.py --com COM6`.

---

## 9. Troubleshooting

| Symptom | Likely cause |
|---|---|
| Router: "nothing from the radio" | Wrong COM port or baud |
| OCS never "connected" | Broker not running (Test) / wrong host, cable or subnet (Competition, see preflight). Allow ~1 min for ROS startup. |
| Ground never "receiving UAV MAVLink" | Router not running, drone side down, or WSL not mirrored |
| Declaration: `waiting for UAV1's read-back geofence` | The ground script hasn't finished the round trip. Check the Geofence row and the Ground log. |
| Declaration: `RunDeclaration NOT sent: UAV_PILOT_CALLSIGN …` | Fix the callsign / registration fields, then restart the OCS |
| Geofence: `FAILED: no reply from PX4 …` | Uplink from WSL to the radio not working (router, mirrored networking), or QGC busy with its own mission transfer |
| Geofence: `FAILED … ceiling … is not above home` | Altitude limit below the drone's home height above the ellipsoid |
| Geofence: `WAITING for GPS — … no home position … no 3D GPS fix …` | The fence is already on PX4, and the ceiling step needs a GPS fix and home. Go outside until QGC shows a 3D lock and it continues by itself, or set **Bench: assume home HAE** (Test only). |
| Fence "not in QGC" | QGC doesn't refresh a fence another program uploaded: Plan → Fence → **Download**. Also check where the course is: RoboNation's `test_server.py` course ALPHA is in **Singapore**, so use the GUI's mock course (centred on the drone) for bench tests. A fence far away also blocks arming: clear it (Plan → Fence → Clear → Upload). |
| Heartbeats red: `none for N s` | Drone link stale: the OCS suppresses stale telemetry rather than send bad heartbeats |
| `ros2` commands hang | Run `ros2 daemon start` (can take ~20 s) or use `--no-daemon` |
| `current_task` stuck at 2 | Publish `ros2 topic pub --once /seabird/current_task std_msgs/msg/UInt8 "{data: 1}"` on the VOXL, or restart `beacon_mavlink_bridge.py` |

---

## 10. Cleanup

1. **Stop all.**
2. Test: **Clear mock course** (it offers to stop the OCS first, because the
   OCS would read the empty clear message as an empty course).
3. Clear the fence from PX4 if the next flight shouldn't use it: QGC → Plan
   → Fence → Clear, then upload.
