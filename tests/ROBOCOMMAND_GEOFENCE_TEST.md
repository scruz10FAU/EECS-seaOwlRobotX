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
- **VALID / INVALID:**
  - The course must be a closed ring, with at least 3 distinct corners, no
    crossing edges, valid coordinates, and 5 m–1 km across.
  - The altitude limit must be above 0, at most the competition maximum
    (50 m HAE by default), and above home.
  - The readback must lie within the course (1 m tolerance, the same rule as
    RoboCommand's own checker).
  - Anything else is **REFUSED with the reason**: not uploaded, not declared.
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
- **Competition box:** the RoboCommand-facing adapter (↻ refreshes the list),
  and the RoboCommand host. Leave the host blank to use the DHCP gateway.
- **Test box:** the mock course centre (`LAT,LON`, the drone's position from
  QGC), its size, and its kind: `(valid)` or one of the INVALID kinds.
  **Publish mock course** and **Clear mock course** are here.
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
  - **Declaration:** seq, fence and altitude sent, or why not;
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

### 5.2 GUI

1. Select **Test**, fill in the settings, then **Run preflight**.
2. Read the drone's latitude/longitude in QGC and put them in **Course
   centre**. Leave the kind at `(valid)`.
3. **Start all.** Wait for:
   - Router: radio bytes > 0;
   - OCS: connected;
   - Ground: receiving UAV MAVLink.
4. **Publish mock course.** Expect:
   - **Geofence:** `UPLOADING`, then `VALID — course MOCK (on the UAV, read back)`;
   - **Declaration:** `sent seq 1, course MOCK uav_geofence=5 points (read back from UAV1) altitude=39.0m HAE callsign=…`;
   - **Heartbeats:** green.
5. In QGC, under Plan → Fence → Download, the fence should sit 1 m inside
   the course.

The mock doesn't answer the RunDeclaration, so **RunStart** stays empty on
the bench. Only real RoboCommand (or RoboNation's `test_server.py`, whose
course is fixed in Singapore) sends it.

### 5.3 Rehearse the INVALID demonstration

| Demonstration | How (Test profile) | Expect |
|---|---|---|
| INVALID boundary | Course kind → `open`, `two-points`, `self-intersecting`, `bad-coords`, `too-small` or `too-large`, then **Publish mock course** | **Geofence: REFUSED** + reason (e.g. "course boundary edges cross each other"). Nothing uploaded to the UAV, no new declaration. |
| INVALID altitude | Stop all → set altitude limit to `0` or `60` → Start all (confirm the prompt) → Publish a `(valid)` course | **Geofence: REFUSED**, e.g. "altitude limit 60 m HAE is above the maximum 50 m HAE" |
| VALID again | Kind `(valid)`, altitude `40`, Publish | **VALID**, new declaration seq |

### 5.4 Task 1 reporting check

On the VOXL, with `beacon_mavlink_bridge.py` running:

```bash
python3 tests/mock_beacon_publisher.py --field --lat <lat> --lon <lon>
```

This reports Task 1 started (`current_task` 2) while it runs and stood down
(1) when it ends, including on Ctrl-C. Its beacons produce SafePassage
reports, which the OCS logs as `SafePassage UAV1 -> RoboCommand`. Use
`--no-task` when the sweep is running.

---

## 6. Ground / Flight Validation and competition (Competition profile)

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
   - Declaration: `sent seq N … read back from UAV1`;
   - Heartbeats: green;
   - RunStart: received once every declared vehicle reports AUTO.

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
```

The fence must be **VALID (on the UAV)** before the sweep starts. The sweep
reads it once, before arming. In the drone's container:

```bash
python3.10 sweep_rrt_overfly_downward_center.py
```

Expect `[rrt] ✓ PX4 geofence: 1 inclusion polygon(s), … m², margin 2.0 m`.
The sweep aborts before arming if there's no usable fence, or if takeoff is
outside it. `current_task` goes 1 → 2 at offboard start and 2 → 1 after
landing. For Gazebo, see [SIM_GEOFENCE_TEST.md](SIM_GEOFENCE_TEST.md).

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
| Geofence stuck `UPLOADING` / `Waiting for HOME_POSITION …` | No 3D GPS fix or home yet. It continues by itself. |
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
