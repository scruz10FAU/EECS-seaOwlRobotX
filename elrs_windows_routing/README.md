# ELRS Windows Routing

Ground-station side of the ELRS MAVLink link. The drone's telemetry and beacon detections arrive at a Windows laptop through an ELRS transmitter on a serial (COM) port. `elrs_windows_router.py` reads them and passes them on to QGroundControl, plus to WSL, where `beacon_mavlink_ground.py` decodes the beacons.

> **Bench-test tool, not flight-qualified.** Check QGC commands and telemetry on the bench before relying on it.

---

## Folder Overview

| File | Role |
|---|---|
| `elrs_windows_router.py` | Serial ⇄ UDP router. The **only** program that may open the radio COM port. |
| `start_seabird_ground.ps1` | Launcher: starts the router in a new PowerShell window, then the WSL beacon receiver. |
| `setupforelrs.txt` | Original step-by-step setup checklist (drone and ground). |
| `elrs_testing/` | Logs and an evaluation script for link tests (delivery, latency, update rate). See [elrs_testing/README.md](elrs_testing/README.md). |

The other scripts in the chain live in the repo root: `beacon_mavlink_bridge.py` (drone side), `beacon_mavlink_ground.py` (WSL side), and `utils/beacon_mavlink_protocol.py` (the shared wire format).

---

## How Data Flows

```
 VOXL2 (drone)                               Windows laptop                                 WSL (Ubuntu-22.04)
 ─────────────                               ──────────────                                 ──────────────────
 PX4 telemetry ─┐                                                     ┌─► UDP 127.0.0.1:14550 ─► QGroundControl
                ├─► elrs-mavlink ─► ELRS ─► COM1 ─► elrs_windows_router ┤
 beacon_mavlink_bridge.py ─┘   (10.200.0.2:14550)                     └─► UDP 127.0.0.1:14551 ─► beacon_mavlink_ground.py
   (MEMORY_VECT beacons)                                                                          └─► ROS 2 /seabird/beacon_info
                                  ◄──────────── COM1 ◄── UDP 127.0.0.1:14600 ◄── QGroundControl (commands)
```

**Radio → ground.** The router reads raw bytes from the COM port and parses them into complete MAVLink frames, checking each frame's CRC with `pymavlink`. Every valid frame goes to both QGC and WSL as the **original, unmodified bytes**, so MAVLink 2 signatures survive. Corrupt bytes are counted and dropped.

**QGC → radio.** Anything QGC sends back reaches the router's UDP port 14600 and is written to the COM port unchanged.

**Keep-alive.** Once a second the router sends a GCS `HEARTBEAT` (sysid 255, compid 190) out over the radio. This keeps a standalone ELRS transmitter in MAVLink mode even when QGC isn't connected.

**Beacons.** Beacon detections travel as standard `MEMORY_VECT` messages with address `0xBEAC` (see [utils/README.md](../utils/README.md)). If the router can import `utils.beacon_mavlink_protocol`, it prints each beacon's `tracking_id`; without it, it still forwards and counts them.

---

## Prerequisites

| Where | Needs |
|---|---|
| Windows | Python 3.10 (`py -3.10`), `pip install pyserial pymavlink`; the ELRS transmitter on **COM1**; QGroundControl |
| WSL | `Ubuntu-22.04`, ROS 2 Humble (`/opt/ros/humble`), `pip install pymavlink`; this repo at `/mnt/c/Users/Precision 3571/EECS-seaOwlRobotX` |
| Drone | `voxl-px4`, `voxl-mavlink-server`, `elrs-mavlink` running; the Seabird Docker container |

Find the radio's COM port in Device Manager → *Ports (COM & LPT)*. If it isn't COM1, pass `--com COMx` and change `$ComPort` in `start_seabird_ground.ps1`.

---

## Setup

### 1. Drone (SSH into the VOXL2)

```bash
systemctl status voxl-px4 voxl-mavlink-server voxl-microdds-agent   # PX4 + MAVLink services up
systemctl status elrs-mavlink                                        # ELRS bridge up
ip netns exec rfdns ss -ulnp | grep 14550                            # ELRS bridge listening on 10.200.0.2:14550
systemctl status voxl-mpa-ros2                                       # ROS 2 bridge running

cd /home/voxluser && sudo ./run_seabird.sh                           # start the Seabird container
```

Inside the container, start a beacon source:

```bash
python3 beacon_detector_config.py      # real detector
# or
python3 mock_beacon_publisher.py       # synthetic detections for testing
```

In a new SSH terminal, check `ros2 topic list` shows the detections topic. If it doesn't, restart `voxl-mpa-ros2`. Then start the bridge, which sends detections straight to the ELRS bridge:

```bash
cd /home/voxluser/seabirdDocker/EECS-seaOwlRobotX
python3 beacon_mavlink_bridge.py --mavlink-endpoint udpout:10.200.0.2:14550
```

By default the bridge reads `/seabird/beacon_detections_downward` and sends every detection twice, 0.5 s apart (`--repeat`, `--repeat-gap`). The ground receiver drops the duplicate copy.

### 2. Windows: the router

```powershell
cd "C:\Users\Precision 3571\EECS-seaOwlRobotX"
$env:PYTHONPATH = (Get-Location).Path      # lets the router import utils/ for beacon tracking IDs
py -3.10 .\elrs_windows_routing\elrs_windows_router.py --com COM1
```

Expect `Connected to COM1`, then a stats line every 5 s (see [Console output](#console-output)). Leave this window open.

### 3. QGroundControl

Connect the UDP comm link (`ModalELRS_UDP`), which listens on port **14550**. Telemetry should appear within a few seconds. QGC replies to the address the packets came from, so its commands find the router's port 14600 without extra setup.

### 4. WSL: the beacon receiver

```powershell
wsl -d Ubuntu-22.04
```

```bash
cd "/mnt/c/Users/Precision 3571/EECS-seaOwlRobotX"
source /opt/ros/humble/setup.bash
python3 beacon_mavlink_ground.py --mavlink-endpoint udpin:0.0.0.0:14551 --publish-ros
```

Each decoded beacon prints with its `tracking_id` and `msg_id`. With `--publish-ros` it is also published as JSON on `/seabird/beacon_info` (`--ros-topic` to change). Add `--log beacons.csv` to record each beacon with its receive time.

### Quick start: the launcher

Steps 2 and 4 in one go. Run it once the drone side is up and the transmitter is plugged in:

```powershell
powershell -ExecutionPolicy Bypass -File .\elrs_windows_routing\start_seabird_ground.ps1
```

It opens the router in a new window on `$ComPort` (COM1), then runs the WSL receiver in the current window. You still connect QGC yourself. See [Known issues](#known-issues): the launcher doesn't set `PYTHONPATH` yet.

---

## Router Options

```
py -3.10 elrs_windows_router.py [--com COM1] [--baud 460800] [--qgc HOST:PORT] [--wsl HOST:PORT]
                                [--listen PORT] [--wsl-log CSV] [--stats-period S] [--heartbeat-period S]
```

| Option | Default | Meaning |
|---|---|---|
| `--com` | `COM6` | Radio serial port. **Pass `--com COM1`** for this setup. |
| `--baud` | `460800` | Serial baud rate; must match the ELRS transmitter |
| `--qgc` | `127.0.0.1:14550` | Where radio frames go for QGroundControl |
| `--wsl` | `127.0.0.1:14551` | Where radio frames go for `beacon_mavlink_ground.py` |
| `--listen` | `14600` | Local UDP port for QGC → radio traffic |
| `--wsl-log CSV` | off | Append one row per frame sent to WSL (see [Frame log](#frame-log)) |
| `--stats-period` | `5.0` | Seconds between stats lines |
| `--heartbeat-period` | `1.0` | Seconds between GCS heartbeats sent to the radio |

If the COM port disappears (e.g. the transmitter is unplugged), the router keeps retrying every few seconds and reconnects on its own. Ctrl+C stops it and prints running totals.

### Console output

```
Last interval: serial_bytes=5012 valid_frames=140 memory_vect=3 beacons=3 bad_data=0 parser_errors=0 wsl_sent=140 wsl_failed=0 qgc_sent=140 qgc_failed=0 qgc_to_serial_bytes=420
BEACON Windows RX tracking_id=42 MAVLink_seq=17
```

| Counter | Meaning |
|---|---|
| `serial_bytes` | Raw bytes read from the radio |
| `valid_frames` | Frames that passed the CRC check, all forwarded |
| `memory_vect` / `beacons` | `MEMORY_VECT` frames / those that are beacon messages (address `0xBEAC`, current version) |
| `bad_data`, `parser_errors` | Corrupt or unparseable bytes, dropped |
| `wsl_sent`/`wsl_failed`, `qgc_sent`/`qgc_failed` | UDP sends to each destination |
| `qgc_to_serial_bytes` | QGC → radio bytes written |

`serial_bytes` > 0 with `valid_frames` = 0 usually means the baud rate is wrong.

### Frame log

`--wsl-log frames.csv` appends one row per frame forwarded to WSL. A header is written only if the file is new or empty, so separate runs pile up in the same file; use a new file name per test.

| Column | Contents |
|---|---|
| `iso_time`, `epoch_time` | When the frame was forwarded (Windows clock) |
| `status` | `SENT` or `SEND_FAILED` |
| `bytes`, `msg_type` | Frame size and MAVLink message name (`UNKNOWN_<id>` = corrupt bytes that happened to parse) |
| `src_system`, `src_component`, `mav_seq` | MAVLink header. Sequence gaps per sender = frames lost over the radio |
| `tracking_id` | Beacon tracking ID for beacon frames, else empty |
| `hex_payload` | The full frame as hex |

These are the `elrs_to_wsl.csv` files in `elrs_testing/`.

---

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `Unable to open COM1 ... Access is denied` | Another program holds the port (QGC auto-connecting to serial, another router, a serial monitor). In QGC, turn off serial auto-connect under *Application Settings → General*. |
| `Unable to open COM1 ... cannot find the file` | Wrong port; check Device Manager and pass `--com`. |
| `serial_bytes=0` | Radio not linked to the drone, or `elrs-mavlink` not running on the VOXL2 (setup step 1). |
| `valid_frames=0` but `serial_bytes` > 0 | Baud mismatch; set `--baud` to the transmitter's MAVLink baud. |
| `beacons=0` while `memory_vect` > 0 | Beacon wire-format version mismatch: update the drone's and this repo's `utils/beacon_mavlink_protocol.py` to the same version. |
| `NOTE: utils.beacon_mavlink_protocol not importable` | `PYTHONPATH` doesn't include the repo root. Beacons are still forwarded and counted, but no tracking IDs are printed or logged. Set it as in setup step 2. |
| QGC shows nothing | QGC isn't listening on UDP 14550, or a second QGC/router is bound to it. |
| WSL receiver prints nothing while `wsl_sent` rises | Frames from Windows' `127.0.0.1` aren't reaching WSL. With WSL 2's default NAT networking, set `networkingMode=mirrored` in `%UserProfile%\.wslconfig` (then `wsl --shutdown`), or send to WSL's own address: `--wsl <output of "wsl hostname -I">:14551`. |
| Many `UNKNOWN_*` frames / high `bad_data` | Radio link is overloaded or weak. Lower the PX4 stream rates sent over ELRS; see the results in `elrs_testing/`. |

---

## Known Issues

- **The launcher runs the router without `PYTHONPATH`.** `start_seabird_ground.ps1` starts the router from inside `elrs_windows_routing\`, so `utils/` isn't importable and tracking IDs are missing. Workaround: start the router by hand as in setup step 2.
- **`--com` defaults to `COM6`** in the router, while this setup and the launcher use COM1. Always pass `--com COM1`.
- **`setupforelrs.txt` is out of date.** It runs `.\elrs_windows_router.py` from the repo root and uses COM6; the router now lives in `elrs_windows_routing\`. The steps above are current.
