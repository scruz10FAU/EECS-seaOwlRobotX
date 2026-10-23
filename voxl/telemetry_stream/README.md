# telemetry-stream container: collecting images and video

Streams the drone's `tracking_front` camera together with PX4 telemetry (GPS,
altitude / local position, attitude, IMU) over HTTP on port 8092. On a laptop,
`telemetry_viewer.py` shows the stream and saves single frames (JPEG) or
recordings (MKV), each with a CSV of the telemetry for every frame. This is how
we collect training data and the recordings `beacon_overlay` replays.

The frames have no detections drawn on them, unlike the `beacon_overlay` stream on 8091.

Code: `telemetry_stream.py` (drone side) and `telemetry_viewer.py` (laptop side),
both at the repo root. `run.sh` here wraps the docker commands.

## Quick start

**On the dev machine**, push the repo to the drone (same rsync as
`voxl/beacon_overlay/README.md`; skip it if the code there is current):

```bash
DRONE=root@192.168.50.121
rsync -a --delete --exclude .git --exclude seabird_dataset --exclude models \
  --exclude 'modal*' --exclude results --exclude '*.npy' --exclude __pycache__ \
  --exclude '*_saved_model' --exclude '*.pt' --exclude '*.onnx' \
  ./ $DRONE:/home/root/seaowl/
```

**On the drone** (as root):

```bash
cd /home/root/seaowl
voxl/telemetry_stream/run.sh start     # prints http://<drone-ip>:8092/
voxl/telemetry_stream/run.sh logs      # Ctrl-C to leave; the container keeps running
```

There is nothing to build: it runs the existing `voxl2-humble` image. Within a few
seconds the log should show:

```
[telemetry] subscribed to /tracking_front
[telemetry] subscribed to PX4 gps / local position / attitude / sensor_combined
[telemetry] serving http://0.0.0.0:8092/  (stream: /stream.mjpg)
[telemetry] frames 50 | 1280x800 96 KB | telemetry: gps, local, attitude, imu
```

`telemetry: none` (or a missing group) means PX4 messages aren't arriving; frames
are still streamed and saved, with empty telemetry columns.

**On the laptop:**

```bash
pip install pillow                      # tkinter ships with Python; ffmpeg needed for video
python3 telemetry_viewer.py --host 192.168.50.121 --out-dir ~/captures
```

Depending on device, may need to modify python call and ip address. Sam's device is:
```bash
py -3.10 .\telemetry_viewer.py --host 192.168.1.168 --out-dir ~/captures
```

## Collecting data

The viewer window shows the live frame, the telemetry panels (red = no data, or
older than 1 s) and the stream rate.

- **Single images:** press **Space** (or **Save frame**). It saves the frame on
  screen exactly as received (no re-encode).
- **Video:** **Record** starts, **Stop** ends. Every received frame goes into the
  file, not just the ones drawn on screen. Needs `ffmpeg` on `PATH`. Closing the
  window while recording stops and saves the recording.

Output under `--out-dir` (default `./captures`):

```
captures/
  images/20261023_141502_123456.jpg     # one per Save frame
  captures.csv                          # one row per saved image, appended across sessions
  videos/20261023_141530_654321.mkv     # one per Record/Stop
  videos/20261023_141530_654321.csv     # one row per video frame
```

File names are the laptop's local time. A recording that received no frames is
deleted on Stop.

### Exposure

The **Exposure** slider (0.05-30 ms, log scale) and **gain** box set the camera's
manual exposure through `voxl-camera-server`. Moving the slider sends the command
once it settles; **Auto** goes back to auto exposure. The line under the slider shows
the command sent or the error. The camera keeps its current exposure until you
touch these controls. Manual exposure stays set after the viewer closes, so press
**Auto** before you leave if other users expect auto exposure.

### Telemetry CSV columns

`captures.csv` starts with `filename`; a video's CSV starts with `frame` (0-based
index into the `.mkv`). The other columns are the same:

| columns | |
|---|---|
| `laptop_time_iso` | laptop time when the frame was saved / received |
| `frame_stamp` | camera capture time (Unix s) |
| `gps_lat`, `gps_lon`, `gps_alt_msl_m`, `gps_fix_type`, `gps_sats`, `gps_eph`, `gps_epv` | `vehicle_gps_position`; `fix_type` < 3 = no 3D fix |
| `height_agl_m`, `local_x`, `local_y`, `local_z`, `heading_deg` | `vehicle_local_position`, NED metres, interpolated to `frame_stamp` |
| `roll_deg`, `pitch_deg`, `yaw_deg`, `qw`..`qz` | `vehicle_attitude` (FRD body -> NED), interpolated to `frame_stamp` |
| `gyro_*`, `accel_*` | `sensor_combined`, body FRD (rad/s, m/s^2) |
| `gps_age_s`, `local_age_s`, `att_age_s`, `imu_age_s` | seconds since that group's last message |

Empty cells mean that group had no data yet.

### Turning a recording into images

The `.mkv` holds the stream's JPEGs unchanged, so they can be extracted without
re-encoding. Frame `N` in the output matches row `frame = N` in the CSV:

```bash
mkdir frames
ffmpeg -i videos/<ts>.mkv -c:v copy -fps_mode passthrough -start_number 0 frames/%05d.jpg
# ffmpeg older than 5.1: replace -fps_mode passthrough with -vsync 0
```

To thin it out for labelling, add `-vf fps=2` and drop `-c:v copy` (this
re-encodes, and the frame numbers no longer match the CSV rows).

### Replaying a recording

`<ts>.mkv` + `<ts>.csv` replay through `beacon_overlay` with the full
localisation. See "Replay a recording instead of the camera" in
`voxl/beacon_overlay/README.md`.

### From images to a training set

Label the images in RoboFlow and export as COCO keypoints. Then
`voxl/prep_diamond_dataset.py <export.zip>` square-crops, resizes to 640 and
greyscales them. Check `voxl/NOTES.md` for how the current model's input was
prepared before mixing datasets: the deployed model was trained on frames
stretched to 640x640, not cropped.

## Stream options

Arguments after `start` go to `telemetry_stream.py`:

| argument | default | meaning |
|---|---|---|
| `--max-fps F` | `10` | cap the stream rate; `0` = every camera frame |
| `--quality Q` | `85` | JPEG quality of the stream (and therefore of saved frames) |
| `--out-width W` | `0` | downscale to this width; `0` = full 1280x800 |
| `--image-time-offset MS` | `0` | added to the camera stamp before interpolating the pose |
| `--topic T` | `/tracking_front` | ROS image topic |

For training data, `run.sh start --quality 95` keeps compression artefacts down.
Higher `--max-fps` and quality cost Wi-Fi bandwidth; check the viewer's fps reading.

Environment variables read by `run.sh`:

| variable | default | |
|---|---|---|
| `REPO` | `/home/root/seaowl` | repo on the drone; mounted read-only at `/workspace` |
| `PORT` | `8092` | HTTP port |
| `RESTART` | `no` | docker restart policy, e.g. `unless-stopped` to start on boot |
| `ROS_DOMAIN_ID` | `8` | must match the host's `voxl_mpa_to_ros2` and PX4; e.g. `ROS_DOMAIN_ID=0 run.sh start` |

## Without the viewer

From a browser or `curl`:

| URL | |
|---|---|
| `http://<drone-ip>:8092/` | live view (image only) |
| `/snapshot.jpg` | latest frame; 503 until the first frame arrives |
| `/telemetry.json` | latest telemetry |
| `/stream.mjpg` | MJPEG; each part has an `X-Telemetry: <json>` header for that frame |
| `POST /camera?exp_ms=2.0&gain=100` / `POST /camera?ae=1` | manual / auto exposure |

```bash
curl -o frame.jpg http://192.168.50.121:8092/snapshot.jpg
```

## Stop

```bash
voxl/telemetry_stream/run.sh stop
```

## Troubleshooting

| symptom | cause / fix |
|---|---|
| Viewer says `disconnected ... retrying` | Container not running (`docker ps`), wrong `--host`, or laptop not on the drone's network. |
| Log never shows `frames` lines; `/snapshot.jpg` gives 503; viewer stays blank | No camera frames on `/tracking_front`: usually `ROS_DOMAIN_ID` doesn't match `voxl_mpa_to_ros2`. Find the host's with `tr '\0' '\n' < /proc/$(pgrep -f voxl_mpa_to_ros2_node \| head -1)/environ \| grep ROS_DOMAIN_ID` (nothing printed = 0) and restart with that value. Also check the camera itself with `voxl-inspect-cam tracking_front`. |
| Telemetry panels say `no data` | PX4 topics aren't reaching the container (microDDS agent down, or domain mismatch as above). |
| `ffmpeg not found on PATH` | Install ffmpeg on the laptop; single frames still work without it. |
| Exposure `failed: ... /run/mpa/tracking_front/control` | `voxl-camera-server` isn't running, or the camera name differs (`--camera`). |
| Stream well below 10 fps | Wi-Fi bandwidth: lower `--quality` or set `--out-width`. |
