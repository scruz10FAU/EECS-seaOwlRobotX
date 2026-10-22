# beacon-overlay container: deployment

Runs `beacon_overlay.py` on a VOXL2: YOLO11n-pose beacon/diamond detection on the
Adreno GPU (TFLite 2.19 built into the image), beacon localisation from PX4, the
annotated MJPEG stream on port 8091, and the beacon map on ROS 2
`/seabird/beacon_map`. Background and measurements: `voxl/NOTES.md`.

Everything below runs **on the drone** as root unless marked *dev machine*.
`run.sh` wraps all docker commands; its settings are environment variables
(see [Settings](#settings)).

## Quick start: live camera -> `/seabird/beacon_map`

The flight drone (`192.168.50.121`, ROS domain 0, `BASE=voxl2-humble`). For the
bench board add `ROS_DOMAIN_ID=8` to `start` and `BASE=seabird` to `build`.

**On the dev machine**, from the repo root, push the code and model (skip the
model if it's already on the drone):

```bash
DRONE=root@192.168.50.121
rsync -a --delete --exclude .git --exclude seabird_dataset --exclude models \
  --exclude 'modal*' --exclude results --exclude '*.npy' --exclude __pycache__ \
  --exclude '*_saved_model' --exclude '*.pt' --exclude '*.onnx' \
  ./ $DRONE:/home/root/seaowl/
ssh $DRONE mkdir -p /data/models_eval
scp voxl/YOLOv11n_pose_diamond_beacon_saved_model/YOLOv11n_pose_diamond_beacon_float16.tflite \
  $DRONE:/data/models_eval/
```

The fp16 `.tflite` is committed to the repo, so a fresh clone can deploy as is.
Re-export it only after retraining ([step 1](#1-export-the-model-dev-machine)).

**On the drone:**

```bash
cd /home/root/seaowl

# Once, and again only after Dockerfile / voxl/beacon_gpu changes (~15 min):
voxl/beacon_overlay/run.sh build

# Run. --water-z: water surface below PX4's arming point, metres (0 if it armed
# at water level). RESTART=unless-stopped keeps it running across reboots.
RESTART=unless-stopped voxl/beacon_overlay/run.sh start --water-z 0

voxl/beacon_overlay/run.sh logs        # Ctrl-C to leave; the container keeps running
```

**Before expecting the topic**, the logs should show, in this order:

1. `Created 1 GPU delegate kernels` and `Backend: gpu (OpenCL)`: detection on the GPU.
2. `subscribed to PX4 ...`, `publishing beacon map on /seabird/beacon_map`,
   `subscribed to /tracking_front`.
3. `[overlay] ... fps ...` lines every 5 s: camera frames are arriving.
4. `pose @capture N` rising: PX4 has a valid position. This needs a GPS 3D fix and
   PX4's EKF reporting `xy_valid`, so **outdoors only**. While `none` rises
   instead, nothing is published and the stream shows
   `no valid PX4 pose: not localising`.

Then check the topic from the drone:

```bash
docker exec beacon_overlay bash -lc \
  "source /opt/ros/humble/setup.bash; timeout 10 ros2 topic hz /seabird/beacon_map; ros2 topic echo /seabird/beacon_map"
```

(`docker exec` inherits the container's `ROS_DOMAIN_ID`.) It publishes once per
processed frame (~8 Hz) while localising, with or without
a beacon in view. `beacons` holds the map (entries with at least `--min-fixes`
fixes) and `fixes` this frame's individual fixes. Watch the annotated stream at
`http://192.168.50.121:8091/`.

**Reading it from another node** (same `ROS_DOMAIN_ID`):

```python
import json
import rclpy
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node("beacon_map_listener")

def on_map(msg):
    m = json.loads(msg.data)
    for b in m["beacons"]:      # lat/lon are None until PX4 has a global reference
        print(f"B{b['id']}: {b['lat']}, {b['lon']}  +/-{b['sigma_m']} m  "
              f"n={b['n']}  seen now={b['seen']}  last seen {b['age_s']} s ago")

node.create_subscription(String, "/seabird/beacon_map", on_map, 10)
rclpy.spin(node)
```

Message fields (all positions in PX4 local NED, metres):

| field | |
|---|---|
| `stamp` | camera capture time of the frame (Unix s) |
| `source` | `"live"` (or `"replay"`) |
| `drone` | `ned`, `lat`, `lon` of the drone at capture |
| `water_z_ned` | the `--water-z` in use |
| `beacons[]` | `id` (stable for the run), `ned`, `lat`, `lon`, `sigma_m` (horizontal 1-sigma, grows while unseen), `n` fixes, `outliers`, `age_s` since last fix, `seen` this frame |
| `fixes[]` | `method` (`pnp` diamond pose / `box` beacon box / `ray` diamond ray), `ned`, `lat`, `lon`, `sigma_m`, `range_m` slant, `horiz_m`, `rms_px` (pnp), `n_diamonds`, `n_boxes` |

The map lives in memory: restarting the container starts an empty map with ids
from 0. To stop: `voxl/beacon_overlay/run.sh stop`.

## Boards

| | flight drone | bench board (damaged VOXL2) |
|---|---|---|
| address | `root@192.168.50.121` | `root@192.168.50.146` |
| `BASE` (ROS 2 Humble image to build on) | `voxl2-humble` (default) | `seabird` |
| `ROS_DOMAIN_ID` (must match `voxl_mpa_to_ros2` and PX4) | `0` (default) | `8` |
| host TFLite (not used by the container) | qrb5165-tflite 2.17 | 2.8 |

Find a board's domain with `tr '\0' '\n' < /proc/$(pgrep -f voxl_mpa_to_ros2_node | head -1)/environ | grep ROS_DOMAIN_ID`
(nothing printed = 0).

## Prerequisites on the drone

- Docker and the `BASE` image (`docker images`). It needs python3, numpy, cv2,
  rclpy and px4_msgs. On the flight drone `voxl2-humble` is built from
  `/home/root/humble_docker/`.
- Internet access for the first build (TensorFlow source + CMake dependencies).
- `/dev/kgsl-3d0`, `/dev/ion` and the Adreno libraries in `/usr/lib`
  (`libOpenCL.so libCB.so libgsl.so libllvm-qcom.so libcutils.so.0 liblog.so.0
  libsync.so.0 libion.so.0`). Present on stock VOXL2 system images.
- `/data/modalai/opencv_tracking_front_intrinsics.yml` and
  `/etc/modalai/extrinsics.conf` (the drone's own calibration; mounted read-only).
- PX4 publishing `/fmu/out/vehicle_local_position`, `vehicle_attitude` and
  `vehicle_gps_position` over microDDS on the same domain as the camera (needed for
  localisation, not for detection).

## 1. Export the model (*dev machine*; only after retraining)

`voxl/YOLOv11n_pose_diamond_beacon_saved_model/YOLOv11n_pose_diamond_beacon_float16.tflite`
is tracked in git (the rest of the `*_saved_model/` export is ignored). Regenerate
it from the `.pt` when the model changes, and commit the new `.tflite`:

```bash
source ~/.venvs/yolo-pose/bin/activate     # ultralytics 8.4.55, tensorflow 2.19.0, onnx==1.17.0
cd voxl
yolo export model=YOLOv11n_pose_diamond_beacon.pt format=tflite imgsz=640 half=True
# -> YOLOv11n_pose_diamond_beacon_saved_model/YOLOv11n_pose_diamond_beacon_float16.tflite
```

Export with TensorFlow 2.19 to match the runtime in the image (`TF_VERSION`); a
newer exporter can emit op versions the runtime doesn't know. `onnx` must be
pinned to 1.17.0 (newer versions need a protobuf that TF 2.19 rejects).

The model was trained on 1280x800 frames **stretched** to 640x640, which is what
`beacon_overlay.py --resize stretch` (the default) does. Don't switch to
`letterbox` for this model.

## 2. Copy the code and model to the drone (*dev machine*)

```bash
DRONE=root@192.168.50.121
rsync -a --delete --exclude .git --exclude seabird_dataset --exclude models \
  --exclude 'modal*' --exclude results --exclude '*.npy' --exclude __pycache__ \
  --exclude '*_saved_model' --exclude '*.pt' --exclude '*.onnx' \
  ./ $DRONE:/home/root/seaowl/
ssh $DRONE mkdir -p /data/models_eval
scp voxl/YOLOv11n_pose_diamond_beacon_saved_model/YOLOv11n_pose_diamond_beacon_float16.tflite \
  $DRONE:/data/models_eval/
```

The repo is mounted into the container read-only at run time, so later code
changes only need this rsync and a restart (step 4), not a rebuild.

## 3. Build the image

```bash
cd /home/root/seaowl
voxl/beacon_overlay/run.sh build                 # flight drone
BASE=seabird voxl/beacon_overlay/run.sh build    # bench board
```

About 15 minutes on the VOXL2 the first time (TFLite compiles with `JOBS=4`;
more jobs risks running out of the 7.5 GB RAM). The build ends by listing the 8
exported `bg_*` symbols of `libtflite_gpu_shim.so`, then
`Successfully tagged beacon-overlay:latest`. TFLite is built in its own layer, so
rebuilding after an edit to `voxl/beacon_gpu/tflite_gpu_shim.cc` only relinks.

Rebuild only when `voxl/beacon_overlay/Dockerfile`, `voxl/beacon_gpu/*`,
`BASE` or `TF_VERSION` change.

## 4. Start

Live camera, with localisation and the ROS topic:

```bash
cd /home/root/seaowl
voxl/beacon_overlay/run.sh start --water-z <m>                    # flight drone
ROS_DOMAIN_ID=8 voxl/beacon_overlay/run.sh start --water-z <m>    # bench board
```

`start` replaces any running `beacon_overlay` container. Arguments after `start`
go to `beacon_overlay.py`; the ones that matter in the field:

| argument | default | meaning |
|---|---|---|
| `--water-z M` | `0` | water surface in PX4 local NED z (m, down positive) relative to where PX4 armed; 1.6 for the 2026-10-15 pool recording. Wrong values reject most beacon-box fixes. |
| `--conf C` | `0.5` | detection confidence threshold |
| `--image-time-offset MS` | `0` | added to the camera stamp before looking up the PX4 pose; not yet fitted |
| `--drift V` | `0.15` | beacon random walk in the map (m/sqrt(s)); 0 = static beacons |
| `--map-floor M` | `0.5` | per-fix noise floor when fusing (covers drone GPS error) |
| `--min-fixes N` | `3` | fixes before a beacon enters the map / topic |
| `--map-topic T` | `/seabird/beacon_map` | ROS topic for the map |
| `--no-publish` / `--no-locate` | | no topic / no localisation (camera-only ranges) |
| `--out-width W`, `--max-fps F`, `--quality Q` | | stream size / rate / JPEG quality |

Run `python3 beacon_overlay.py --help` inside the container for the rest.

To start it on every boot, start it once with `RESTART=unless-stopped`.

### Replay a recording instead of the camera

A `telemetry_viewer.py` recording (`<ts>.mkv` + `<ts>.csv`) plays on a loop with
the same localisation and topic (`"source": "replay"`):

```bash
mkdir -p /data/replay    # copy <ts>.mkv and <ts>.csv here
REPLAY_DIR=/data/replay ROS_DOMAIN_ID=8 voxl/beacon_overlay/run.sh start \
  --video /replay/<ts>.mkv --telemetry /replay/<ts>.csv --water-z 1.6
```

The replay uses about 2.5 CPU cores. On the bench board that starves the camera's
ROS bridge (`/tracking_front` drops below 1 Hz), so don't run a replay alongside
live tests there.

## 5. Check it

```bash
voxl/beacon_overlay/run.sh logs
```

Expect, within about 10 s:

```
INFO: Created 1 GPU delegate kernels.            <- whole model on the GPU
[tflite-hexagon] Backend: gpu (OpenCL)
[overlay] subscribed to PX4 local position / attitude / GPS
[overlay] publishing beacon map on /seabird/beacon_map
[overlay] subscribed to /tracking_front
[overlay]  8.2 fps | gpu (OpenCL)   55 ms | diamonds 1 | beacons 1 | frame 412 | ... | frames 380 | pose @capture 380, latest 0, none 0
```

- Stream: `http://<drone-ip>:8091/` in a browser (`/stream.mjpg`,
  `/snapshot.jpg`). Snapshot returns 503 until the first frame arrives.
- `pose @capture` should rise. `none` rising means PX4 has no valid position
  (normal indoors); `latest` means the camera and PX4 stamps didn't match (see
  Troubleshooting).
- Topic:

  ```bash
  docker exec beacon_overlay bash -lc \
    "source /opt/ros/humble/setup.bash; ros2 topic echo /seabird/beacon_map"
  ```

  It publishes only while localising (live with a valid PX4 position, or
  replay). The JSON schema is in the `beacon_overlay.py` docstring.

## 6. Stop and clean up

```bash
voxl/beacon_overlay/run.sh stop               # remove the container
docker rmi beacon-overlay                     # remove the image
docker builder prune                          # drop build cache (forces a ~15 min rebuild)
```

## Settings

Environment variables read by `run.sh`:

| variable | default | |
|---|---|---|
| `REPO` | `/home/root/seaowl` | repo on the drone; mounted at `/workspace` |
| `MODEL_DIR` | `/data/models_eval` | mounted at `/models` |
| `MODEL` | `YOLOv11n_pose_diamond_beacon_float16.tflite` | file in `MODEL_DIR` |
| `CLASSES` | `beacon,diamond` | model class order |
| `PORT` | `8091` | HTTP stream port |
| `GPU` | `1` | `0` = same TFLite on the CPU (XNNPACK, ~0.35-0.65 s per frame) |
| `ROS_DOMAIN_ID` | `0` | see [Boards](#boards) |
| `REPLAY_DIR` | unset | mounted at `/replay` |
| `RESTART` | `no` | docker restart policy, e.g. `unless-stopped` |
| `BASE` | `voxl2-humble` | build only |
| `TF_VERSION` | `2.19.0` | build only; change together with the exporter's TensorFlow |

## Troubleshooting

| symptom | cause / fix |
|---|---|
| Page loads, `/snapshot.jpg` gives 503, log never shows `fps` lines | No camera frames: wrong `ROS_DOMAIN_ID`. Check with `ros2 topic hz /tracking_front` inside the container under each domain. |
| `Backend: cpu (GPU fallback)` | GPU delegate didn't load: `/dev/kgsl-3d0` / `/dev/ion` or one of the `/usr/lib` Adreno libraries is missing, or `GPU=0` was set. |
| `no valid PX4 pose: not localising` on the stream | PX4 reports `xy_valid`/`z_valid` false (indoors, no GPS) or its topics aren't reaching the container (domain). |
| `WARNING: image stamp ... vs PX4 ...: clocks differ` | Camera and PX4 timestamps are on different clocks; it falls back to the latest pose (~35 ms lag). Check the microDDS agent's time sync. |
| Most beacon boxes give no fix | `--water-z` is wrong for this site; box fixes are rejected when size-range and water-ray disagree. |
| Build fails downloading KleidiAI / with undefined `absl::...LogMessage` | Both are handled in the Dockerfile / `CMakeLists.txt`; if they return, the Dockerfile or `voxl/beacon_gpu/CMakeLists.txt` lost `-DXNNPACK_ENABLE_KLEIDIAI=OFF` or the `absl::log` link. |
| `Address already in use` | Another overlay (or a test instance) still holds the port: `run.sh stop`, or `ss -ltnp \| grep 8091`. |
| Stream far below the camera rate | Expected: the stream runs at inference + drawing rate (~8 fps), not 30 fps. |
