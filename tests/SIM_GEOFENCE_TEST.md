# Geofence in sim (PX4 SITL + Gazebo)

How to test `sweep_rrt_overfly_downward_center.py`'s fence mode
(`SEARCH_AREA_SHAPE = "fence"`) in Gazebo with the mock publisher
`tests/sim_geofence_publisher.py`.

## What it does

In fence mode, the sweep reads its search area **from PX4's geofence**
before arming. On the real drone that fence comes from RoboCommand through
the OCS, the ground station and ELRS (see
[ROBOCOMMAND_GEOFENCE_TEST.md](ROBOCOMMAND_GEOFENCE_TEST.md)). In sim,
`sim_geofence_publisher.py` uploads a test fence straight into PX4 SITL
instead:

```
sim_geofence_publisher.py --(MAVLink, port 14540)--> PX4 SITL fence + GF_MAX_VER_DIST
sweep_rrt_overfly_downward_center.py --(MAVSDK)--> reads the fence back, searches inside it
```

It uploads with the same code the ground station uses on the real drone
(`utils/px4_fence.py`), so the sim exercises the real upload path. It
uploads, confirms and exits. It does not keep running.

## Requirements

- PX4 SITL running with Gazebo, for example `make px4_sitl gz_x500`
  (Gazebo) or `make px4_sitl gazebo-classic` (Gazebo Classic).
- `pymavlink` wherever the publisher runs.
- MAVSDK with `download_geofence` wherever the sweep runs. Check with:
  ```bash
  python3 -c "import mavsdk.mission_raw as m; print(hasattr(m.MissionRaw, 'download_geofence'))"
  ```
  This must print `True`. If it prints `False`, fence mode can't be used:
  the sweep will abort before arming.

---

## Quick start

**1. Start PX4 SITL + Gazebo** and wait until PX4 has a position (`pxh>`
prompt, `home set` in the PX4 console).

**2. Upload the fence:**

```bash
cd EECS-seaOwlRobotX
python3 tests/sim_geofence_publisher.py --gf-action 1
```

Expected:

```
[sim-fence] Connecting to udpin:0.0.0.0:14540 ...
[sim-fence] PX4 system 1
[sim-fence] Drone at <lat>, <lon>
[sim-fence] Fence polygon:
    ... 5 points (closed square) ...
[sim-fence] [fence] Uploading 1 inclusion polygon(s), 4 vertices, to system 1
[sim-fence] [fence] ✓ Fence accepted by PX4
[sim-fence] [fence] ✓ GF_MAX_VER_DIST = 10.0 m above home
[sim-fence] ✓ GF_ACTION = 1
[sim-fence] Done -- start the sweep with SEARCH_AREA_SHAPE = "fence"
```

By default the fence is a 25 × 25 m square centred on the simulated drone,
with a 10 m ceiling above home. `--gf-action 1` (warning) makes PX4 actually
enforce it. At `GF_ACTION = 0` the fence is stored but ignored.

**3. Set the sweep for sim.** In `sweep_rrt_overfly_downward_center.py`:

```python
SEARCH_AREA_SHAPE    = "fence"
MAVSDK_ADDRESS       = "udpin://0.0.0.0:14540"   # drone value: "udpin://0.0.0.0:14551"
BENCH_TEST           = False                     # BENCH_TEST returns before the fence is read
RUNSTART_GATE        = False                     # no RoboCommand in sim: don't wait for RunStart
```

**4. Run the sweep:**

```bash
python3 sweep_rrt_overfly_downward_center.py
```

Expected before arming:

```
[rrt] ✓ PX4 geofence: 1 inclusion polygon(s), 625 m², span 35.4 m, margin 2.0 m
```

and in the search banner:

```
[rrt]   Search area     PX4 geofence, 1 polygon(s), 625 m², margin 2.0 m
```

Every RRT node then stays at least `FENCE_MARGIN_M` (2 m) inside the fence.

> **Order matters:** run the publisher **first**. It and the sweep both use
> UDP port 14540, and the publisher frees it when it exits. Re-running the
> publisher while the sweep is connected fails with "No PX4 heartbeat". To
> change the fence mid-flight, use QGC's port instead (see scenario 6).

---

## Options

| Option | Default | What it does |
|---|---|---|
| `--size W,L` | `25,25` | Rectangle width (east-west) and length (north-south), m |
| `--rotation-deg D` | `0` | Rotate the rectangle clockwise |
| `--corner LAT,LON` (repeat) | | Exact polygon, in order, ≥ 3 points. Overrides the rectangle. |
| `--center LAT,LON` | drone's position | Rectangle centre |
| `--offset N,E` | `0,0` | Shift the rectangle centre by metres north/east |
| `--ceiling-m H` | `10` | `GF_MAX_VER_DIST`, metres above home |
| `--alt-hae H` | | Ceiling in metres HAE, converted like the real chain. Needs SITL to report GPS ellipsoid height (often it doesn't; see troubleshooting). |
| `--gf-action N` | unchanged | `GF_ACTION`: 0 none, 1 warning, 2 hold, 3 return, 4 terminate, 5 land |
| `--mavlink-endpoint` | `udpin:0.0.0.0:14540` | PX4 SITL's offboard port. `udpin:0.0.0.0:14550` is QGC's (close QGC first). |
| `--dry-run` | | Print the polygon, don't connect. Needs `--center` or `--corner`. |
| `--clear` | | Remove PX4's fence and exit |

---

## Test scenarios

| # | Scenario | Command | Expect |
|---|---|---|---|
| 1 | Normal search | `python3 tests/sim_geofence_publisher.py --gf-action 1` | Sweep loads the fence and searches only inside it, ≥ 2 m from the edges |
| 2 | Rotated / non-square fence | `... --size 40,15 --rotation-deg 30 --gf-action 1` | RRT tree fills the rotated strip (check RViz `/seabird/rrt_tree`) |
| 3 | Takeoff outside the fence | `... --offset 30,0 --gf-action 1` | Sweep: `ABORT: takeoff position is outside the PX4 geofence — not arming`. The sweep stops before asking PX4 to arm; with `GF_ACTION` ≠ 0, PX4 would also refuse. |
| 4 | No fence | `python3 tests/sim_geofence_publisher.py --clear` | Sweep: `ERROR: PX4 geofence has no inclusion polygon` → `ABORT: no usable PX4 geofence — not arming` |
| 5 | Fence changed between runs | Re-run with a different `--size` / `--offset` | Next sweep run reports the new area. The sweep reads the fence once, at startup. |
| 6 | PX4 breach action | Start a normal run (1) with `--gf-action 2`. While it's searching, close QGC and run `... --offset 20,0 --mavlink-endpoint udpin:0.0.0.0:14550` | The drone is now outside the new fence: PX4 switches to Hold. This tests PX4's enforcement, not the sweep. |
| 7 | Ceiling | `... --ceiling-m 1.5 --gf-action 1` (sweep `TAKEOFF_ALT_M = 2.0`) | PX4 geofence warning when climbing through 1.5 m |

---

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `No PX4 heartbeat -- is SITL running, and is the sweep (or QGC) holding this port?` | SITL not up yet, or another process is bound to the port. Stop the sweep (or use `--mavlink-endpoint udpin:0.0.0.0:14550` with QGC closed). |
| `No GLOBAL_POSITION_INT from PX4 -- pass --center LAT,LON` | PX4 has no position yet. Wait for `home set`, or pass `--center`. |
| `[fence] FAILED: MAVLink 1 connection …` | pymavlink was loaded as MAVLink 1. The script sets `MAVLINK20=1` itself, so this means something imported pymavlink first. Run it as shown, not imported from another script. |
| `--alt-hae` stuck on `Waiting for HOME_POSITION and a GPS_RAW_INT alt_ellipsoid …` | SITL's simulated GPS doesn't report ellipsoid height. Use `--ceiling-m` in sim. |
| `[fence] FAILED: PX4 rejected the fence: …` | Usually a self-intersecting `--corner` order. List corners going around the polygon. |
| Sweep: `this MAVSDK has no mission_raw.download_geofence()` | MAVSDK too old (see Requirements). Use `SEARCH_AREA_SHAPE = "disc"` for now. |
| Sweep: `geofence download failed` | The sweep isn't connected to the same SITL instance. Check `MAVSDK_ADDRESS`. |

---

## Cleanup

1. Remove the fence so later sim runs start clean:
   ```bash
   python3 tests/sim_geofence_publisher.py --clear
   ```
   (Or restart SITL. Whether the fence survives a restart depends on your
   SITL setup.)
2. Before flying the real drone, put the sweep back:
   ```python
   MAVSDK_ADDRESS       = "udpin://0.0.0.0:14551"
   SEARCH_AREA_SHAPE    = "fence"   # or "disc" for the 10 m circle
   ```
