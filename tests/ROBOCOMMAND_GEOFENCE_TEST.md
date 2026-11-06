# RoboCommand geofence + Task 1 reporting test

End-to-end test of the RoboCommand ↔ UAV path without the competition
server, using a mock course at our own test site.

## What is being tested

**Course → PX4 fence → search area**

```
tests/mock_robocommand_course.py   RxCourse protobuf, MQTT robocommand/robotx/course (retained)
  → seaowls_ocs (WSL)              /seaowls/uav1/mission/geofence  (seaowls_interfaces/UavGeofence)
  → beacon_mavlink_ground.py (WSL) uploads PX4 fence + GF_MAX_VER_DIST over MAVLink
  → elrs_windows_router.py → ELRS → PX4
  → sweep_rrt_overfly_downward_center.py   SEARCH_AREA_SHAPE = "fence": reads the fence back, searches inside it
```

**Task 1 start/end → RoboCommand heartbeat**

```
sweep script      /seabird/current_task (std_msgs/UInt8): 2 at offboard start, 1 after landing
  → beacon_mavlink_bridge.py (VOXL)  STATUSTEXT "SEABIRD_TASK <n>", resent every 2 s
  → ELRS → beacon_mavlink_ground.py  VehicleStatus.current_task on /seaowls/uav1/status
  → seaowls_ocs                      RoboCommand Heartbeat.current_task
```

`VehicleStatus.flight_phase` is also checked: 1 = GROUNDED, 2 = AIRBORNE
(from PX4 `EXTENDED_SYS_STATE`), 0 = UNKNOWN when the link is stale.

The fence that goes to PX4 is the course boundary (`RxCourse.corners`), the
same polygon the OCS declares to RoboCommand. The transit area is only added
with `beacon_mavlink_ground.py --fence-include-transit`.

---

## 1. One-time checks

Do these once per setup, not every run.

**MAVSDK on the drone has `download_geofence`.** Run inside the Docker
container that runs the sweep script:

```bash
python3.10 -c "import mavsdk.mission_raw as m; print(hasattr(m.MissionRaw, 'download_geofence'))"
```

It must print `True`. If it prints `False`, fence mode refuses to arm. Use
`SEARCH_AREA_SHAPE = "disc"` until MAVSDK is upgraded. The rest of this test
(the fence upload and task reporting) still works.

**WSL can send to Windows.** The fence upload goes WSL → Windows router →
radio. That needs WSL mirrored networking. In WSL:

```bash
wslinfo --networking-mode      # want: mirrored
```

If the command doesn't exist, check `%UserProfile%\.wslconfig` on Windows for
`networkingMode=mirrored` under `[wsl2]`. Then run `wsl --shutdown` and reopen
WSL.

**PX4 parameters** (QGC → Vehicle Setup → Parameters):

| Parameter | Value | Why |
|---|---|---|
| `GF_ACTION` | `1` (Warning) for bench, `2`/`3` (Hold/Return) for flight | What PX4 does on a breach. At 0 the fence is not enforced. |
| `COM_RC_OVERRIDE` | non-zero (3) | Sticks must be able to take over from OFFBOARD |

`GF_MAX_VER_DIST` is set automatically by the ground script. Don't set it by
hand.

**OCS built** with `UavGeofence`:

```bash
cd "/mnt/c/Users/Precision 3571/seaowls-robotx-robocommand/ros2_ws"
source /opt/ros/humble/setup.bash && colcon build --symlink-install
```

---

## 2. Bench test (props off)

Start in this order. Each step lists the line that shows it worked.

### 2.1 Drone (VOXL SSH)

```bash
systemctl status voxl-px4 voxl-mavlink-server elrs-mavlink     # all active
cd /home/voxluser/seabirdDocker/EECS-seaOwlRobotX
python3 beacon_mavlink_bridge.py --mavlink-endpoint udpout:10.200.0.2:14550
```

✅ `Subscribed to /seabird/current_task — relaying as STATUSTEXT 'SEABIRD_TASK <n>' every 2 s`

### 2.2 Windows

```powershell
cd "C:\Users\Precision 3571\EECS-seaOwlRobotX\elrs_windows_routing"
py -3.10 .\elrs_windows_router.py --com COM6     # use the ELRS port shown in Device Manager
```

Open QGC and connect `ModalELRS_UDP`. Wait for GPS lock and note the drone's
latitude/longitude. You need them for the course centre.

### 2.3 WSL: broker and mock course

```bash
sudo service mosquitto start
sudo ss -ltnp | grep ':1883'          # broker listening

cd "/mnt/c/Users/Precision 3571/EECS-seaOwlRobotX"
python3 tests/mock_robocommand_course.py --center <drone_lat>,<drone_lon>
```

✅ `Published RxCourse (retained) -> robocommand/robotx/course on localhost:1883`

This makes a 25 × 25 m course centred on the drone. Use `--size W,L`,
`--rotation-deg` or repeated `--corner LAT,LON` for other shapes, and
`--dry-run` to preview without publishing. **Don't also run RoboNation's
`test_server.py`.** Its Course Alpha in Singapore would replace this one.

### 2.4 WSL: OCS and ground script

Either run `elrs_windows_routing\start_seabird_ground.ps1`, which starts the
router, OCS and ground script (check its `$ComPort` first), or start them by
hand in two WSL terminals:

```bash
# terminal A — OCS
source /opt/ros/humble/setup.bash
source "/mnt/c/Users/Precision 3571/seaowls-robotx-robocommand/ros2_ws/install/setup.bash"
export ROBOCOMMAND_HOST=localhost ROBOCOMMAND_PORT=1883 TEAM_ID=FLAU VEHICLES=UAV1
export UAV_GEOFENCE_MAX_ALTITUDE_HAE_M=40
ros2 run seaowls_robocommand seaowls_ocs
```

✅ `[SEAOWLS OCS] Published UAV geofence course=MOCK boundary_points=5 transit_points=0 altitude=40.0m`

```bash
# terminal B — ground script
source /opt/ros/humble/setup.bash
source "/mnt/c/Users/Precision 3571/seaowls-robotx-robocommand/ros2_ws/install/setup.bash"
cd "/mnt/c/Users/Precision 3571/EECS-seaOwlRobotX"
python3 beacon_mavlink_ground.py --mavlink-endpoint udpin:0.0.0.0:14551 --publish-ros
```

✅ in order (the upload starts once the drone's heartbeat has been seen):

```
[beacon-ground] Geofence for course 'MOCK'
[beacon-ground] [fence] Uploading 1 inclusion polygon(s), 4 vertices, to system 1
[beacon-ground] [fence] ✓ Fence accepted by PX4
[beacon-ground] [fence] ✓ GF_MAX_VER_DIST = <h> m above home (40.0 m HAE)
```

The OCS will keep waiting for `RunStart`, because the mock doesn't answer its
`RunDeclaration`. That's expected and doesn't block the fence.

### 2.5 Checks

| Check | How | Expect |
|---|---|---|
| Fence on PX4 | QGC → Plan → Fence → Download | The 25 × 25 m polygon around the drone |
| Ceiling | QGC parameter `GF_MAX_VER_DIST` | ≈ 40 m minus the home altitude above the ellipsoid |
| Status topic | `ros2 topic echo /seaowls/uav1/status` (WSL) | `vehicle_id: UAV1`, `current_task: 1`, `flight_phase: 1` (disarmed on the ground) |
| Task relay | on the VOXL: `ros2 topic pub -r 1 /seabird/current_task std_msgs/msg/UInt8 "{data: 2}"` | Within ~2 s the ground prints `current_task 1 -> 2` and the status shows `current_task: 2` |
| Task reset | Ctrl-C the pub, then `ros2 topic pub --once /seabird/current_task std_msgs/msg/UInt8 "{data: 1}"` | Ground prints `current_task 2 -> 1`. **Always reset after this check:** the bridge keeps resending its last value. |
| Fence change | Re-run the mock with `--size 30,30` | Ground uploads again. Re-publishing an identical course does **not** re-upload. |
| Arming outside the fence | Re-run the mock with `--center` ~100 m away, try to arm | PX4 refuses with a geofence message (needs `GF_ACTION` ≠ 0). Then re-publish the real centre. |

---

## 3. Flight test

In `sweep_rrt_overfly_downward_center.py`:

```python
SEARCH_AREA_SHAPE    = "fence"
FENCE_MARGIN_M       = 2.0      # search stays this far inside the fence edges
BENCH_TEST           = False    # BENCH_TEST returns before the fence is read
```

Steps 2.1–2.4 must be running and the fence accepted **before** the sweep
starts. The sweep reads the fence once, before arming. Then run the sweep in
the drone's Docker container, from the repo directory:

```bash
python3.10 sweep_rrt_overfly_downward_center.py
```

✅ before arming:

```
[rrt] ✓ PX4 geofence: 1 inclusion polygon(s), 625 m², span 35.4 m, margin 2.0 m
```

✅ during the run (ground script and `/seaowls/uav1/status`):

| Moment | Sweep log | Ground / status |
|---|---|---|
| Takeoff | | `flight_phase: 2` |
| Offboard active, search starts | `[rrt] current_task 1 -> 2` | `current_task 1 -> 2` → RoboCommand sees the Task 1 attempt start |
| Search | `Search area     PX4 geofence, 1 polygon(s), 625 m², margin 2.0 m`; every RRT node stays ≥ 2 m inside the fence | |
| Landed | `[rrt] current_task 2 -> 1` | `current_task 2 -> 1` (stand-down), `flight_phase: 1` |

If the sweep script crashes or you press Ctrl-C, it still publishes
`current_task 1` on the way out.

To fly the old 10 m circle instead, set `SEARCH_AREA_SHAPE = "disc"`. Nothing
else changes, and the fence is still enforced by PX4 if it was uploaded.

---

## 4. Troubleshooting

| Symptom | Likely cause |
|---|---|
| OCS never logs `Published UAV geofence` | Broker not running, or the mock published to a different host/port than `ROBOCOMMAND_HOST`/`ROBOCOMMAND_PORT` |
| Ground never prints `Geofence for course` | No UAV heartbeat yet, or ground started without `--publish-ros`, or not sourced with `seaowls_interfaces` |
| `[fence] FAILED: no reply from PX4 to fence upload` | WSL → Windows path not working (see the mirrored-networking check), or the router isn't running |
| `[fence] FAILED: PX4 rejected the fence: …` | Bad polygon (self-intersecting `--corner` order) or PX4 busy with another mission transfer (QGC uploading). Re-publish the course to retry. |
| Stuck at `Waiting for HOME_POSITION and a GPS_RAW_INT alt_ellipsoid` | No 3D fix or home not set yet. It continues by itself once they arrive. |
| `[fence] FAILED: ceiling … is not above home` | `UAV_GEOFENCE_MAX_ALTITUDE_HAE_M` below the home altitude. Raise it (RoboCommand allows up to 50). |
| Sweep: `ABORT: no usable PX4 geofence` | Fence not uploaded yet, or MAVSDK lacks `download_geofence` (see one-time checks) |
| Sweep: `ABORT: takeoff position is outside the PX4 geofence` | Course `--center` isn't where the drone is |
| `current_task` stuck at 2 | Bridge still holds the last value after a test or crash. Publish `{data: 1}` or restart `beacon_mavlink_bridge.py`. |
| `current_task` never changes | Bridge not running, or not on the same ROS domain as the sweep container |

---

## 5. Cleanup

1. Stop `seaowls_ocs` **first**. It would read the empty clear message as an
   empty course.
2. Remove the retained mock course from the broker:
   ```bash
   python3 tests/mock_robocommand_course.py --clear
   ```
3. Clear the fence from PX4: QGC → Plan → Fence → Clear, then upload.
4. Set `SEARCH_AREA_SHAPE` back to `"disc"` if the next flight shouldn't use
   the fence.
