#!/bin/bash
# Start / stop the telemetry_stream container on the drone.
#
#   run.sh start [extra args]    (re)start the stream; extra args go to telemetry_stream.py
#   run.sh stop | logs
#
# Then on the laptop: python3 telemetry_viewer.py --host <drone-ip>
# (or a browser at http://<drone-ip>:$PORT/).
#
# Runs humble_stack's own image (rclpy, px4_msgs, cv2), so there is nothing to
# build. Host network + IPC like humble_stack, so DDS discovery/shared memory
# reach the /tracking_front and /fmu/out/* publishers. The repo is mounted
# read-only; /run/mpa read-write so exposure commands reach voxl-camera-server's
# control pipe. The container does not start on boot unless RESTART says so.
set -euo pipefail

REPO=${REPO:-/home/root/seaowl}
PORT=${PORT:-8092}
RESTART=${RESTART:-no}
IMAGE=voxl2-humble
NAME=telemetry_stream

cmd=${1:-}
shift || true

case "$cmd" in
  start)
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    docker run -d --name "$NAME" \
      --network host --ipc host --restart "$RESTART" \
      -e PYTHONDONTWRITEBYTECODE=1 \
      -v "$REPO":/workspace:ro \
      -v /run/mpa:/run/mpa \
      -w /workspace \
      "$IMAGE" \
      bash -c 'source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && exec python3 -u telemetry_stream.py "$@"' \
      _ --port "$PORT" "$@"
    echo "started $NAME: http://$(hostname -I | awk '{print $NF}'):$PORT/"
    ;;
  stop)
    docker rm -f "$NAME"
    ;;
  logs)
    docker logs -f --tail 50 "$NAME"
    ;;
  *)
    echo "usage: $0 {start [args]|stop|logs}" >&2
    exit 1
    ;;
esac
