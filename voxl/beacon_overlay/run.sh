#!/bin/bash
# Build / start / stop the beacon_overlay container on the drone.
#
#   run.sh build                 docker build -t beacon-overlay (context: voxl/; builds
#                                TFLite $TF_VERSION from source, ~15 min on the VOXL2)
#   run.sh start [extra args]    (re)start the stream; extra args go to beacon_overlay.py
#   run.sh stop | logs
#
# Then open http://<drone-ip>:$PORT/ (stream: /stream.mjpg).
#
# Host network + IPC like humble_stack, so DDS discovery/shared memory reach the
# /tracking_front publisher. Everything is mounted read-only. The container does
# not start on boot unless RESTART says so.
#
# Inference goes through the image's libtflite_gpu_shim.so (TFLite built in the
# Dockerfile). GPU=1 (default): on the Adreno; the container gets only the
# kgsl/ion device nodes and the host driver libraries libOpenCL.so loads (traced
# with LD_DEBUG). GPU=0: the same TFLite on the CPU (XNNPACK, ~4 cores).
#
# BASE: the ROS 2 Humble image to build on (voxl2-humble on the flight drone,
# seabird on the bench board). ROS_DOMAIN_ID must match the host's
# voxl_mpa_to_ros2: 0 on the flight drone, 8 on the bench board (.146).
#
# Replay a telemetry_viewer.py recording instead of the live camera:
#   REPLAY_DIR=/data/replay run.sh start --video /replay/<ts>.mkv \
#       --telemetry /replay/<ts>.csv [--water-z <m>]
set -euo pipefail

REPO=${REPO:-/home/root/seaowl}
MODEL_DIR=${MODEL_DIR:-/data/models_eval}
MODEL=${MODEL:-YOLOv11n_pose_diamond_beacon_float16.tflite}
CLASSES=${CLASSES:-beacon,diamond}
PORT=${PORT:-8091}
RESTART=${RESTART:-no}
REPLAY_DIR=${REPLAY_DIR:-}
GPU=${GPU:-1}
BASE=${BASE:-voxl2-humble}
TF_VERSION=${TF_VERSION:-2.19.0}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
IMAGE=beacon-overlay
NAME=beacon_overlay

GPU_LIBS="libOpenCL.so libCB.so libgsl.so libllvm-qcom.so libcutils.so.0 liblog.so.0 libsync.so.0 libion.so.0"
GPU_ARGS=()
APP_ARGS=(--gpu-lib /opt/beacon_gpu/libtflite_gpu_shim.so)
if [ "$GPU" = 1 ]; then
  GPU_ARGS+=(--device /dev/kgsl-3d0 --device /dev/ion -e LD_LIBRARY_PATH=/opt/adreno)
  for f in $GPU_LIBS; do
    GPU_ARGS+=(-v "/usr/lib/$f:/opt/adreno/$f:ro")
  done
else
  APP_ARGS+=(--cpu)
fi
MOUNTS=(-v /etc/modalai/extrinsics.conf:/etc/modalai/extrinsics.conf:ro)
if [ -n "$REPLAY_DIR" ]; then
  MOUNTS+=(-v "$REPLAY_DIR":/replay:ro)
fi

cmd=${1:-}
shift || true

case "$cmd" in
  build)
    docker build -t "$IMAGE" -f "$REPO/voxl/beacon_overlay/Dockerfile" \
      --build-arg BASE="$BASE" --build-arg TF_VERSION="$TF_VERSION" "$REPO/voxl"
    ;;
  start)
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    docker run -d --name "$NAME" \
      --network host --ipc host --restart "$RESTART" \
      -e PYTHONDONTWRITEBYTECODE=1 -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
      -v "$REPO":/workspace:ro \
      -v /data/modalai:/data/modalai:ro \
      -v "$MODEL_DIR":/models:ro \
      "${MOUNTS[@]}" \
      "${GPU_ARGS[@]}" \
      "$IMAGE" \
      /bin/bash -c '
        source /opt/ros/humble/setup.bash
        source /opt/px4_ws/install/setup.bash
        exec python3 -u beacon_overlay.py "$@"
      ' bash \
      --model "/models/$MODEL" \
      --classes "$CLASSES" \
      --port "$PORT" \
      "${APP_ARGS[@]}" "$@"

    echo "started $NAME: http://$(hostname -I | awk '{print $1}'):$PORT/"
    ;;
  stop)
    docker rm -f "$NAME"
    ;;
  logs)
    docker logs -f --tail 50 "$NAME"
    ;;
  *)
    echo "usage: $0 {build|start [args]|stop|logs}" >&2
    exit 1
    ;;
esac
