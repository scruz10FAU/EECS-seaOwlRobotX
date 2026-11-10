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
# ROS_DOMAIN_ID must match the host's voxl_mpa_to_ros2 (default 8).
set -euo pipefail

REPO=${REPO:-/home/root/seaowl}
PORT=${PORT:-8092}
RESTART=${RESTART:-no}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-8}
# Local saving on the drone: offline = only while no viewer is receiving the
# stream (e.g. no Wi-Fi), always, or off. Saved to RECORD_DIR/<start time>/.
RECORD=${RECORD:-offline}
RECORD_DIR=${RECORD_DIR:-/data/telemetry_stream}
IMAGE=voxl2-humble
NAME=telemetry_stream

cmd=${1:-}
shift || true

case "$cmd" in
  start)
    rec_mount=() rec_args=()
    case "$RECORD" in
      off) ;;
      offline|always)
        rec_mount=(-v "$RECORD_DIR":/recordings)
        rec_args=(--record-dir /recordings --record-when "$RECORD") ;;
      *) echo "RECORD must be offline, always or off (got '$RECORD')" >&2; exit 1 ;;
    esac
    docker rm -f "$NAME" >/dev/null 2>&1 || true
    docker run -d --name "$NAME" \
      --network host --ipc host --restart "$RESTART" \
      -e PYTHONDONTWRITEBYTECODE=1 -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
      -v "$REPO":/workspace:ro \
      -v /run/mpa:/run/mpa \
      ${rec_mount[@]+"${rec_mount[@]}"} \
      -w /workspace \
      "$IMAGE" \
      bash -c 'source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash && exec python3 -u telemetry_stream.py "$@"' \
      _ --port "$PORT" ${rec_args[@]+"${rec_args[@]}"} "$@"
    echo "started $NAME: http://$(hostname -I | awk '{print $1}'):$PORT/"
    [ "$RECORD" = off ] || echo "local saving ($RECORD) -> $RECORD_DIR/<start time>/"
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
