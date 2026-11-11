#!/bin/bash
# Start / stop exposure_controller.py (tracking_front exposure for the beacons)
# in a ROS 2 Humble container on the VOXL host.
#
#   ./run_exposure_controller.sh start [extra args]   e.g. start --dry-run
#   ./run_exposure_controller.sh stop | logs | status
#
# Host network + IPC (sees /tracking_front like beacon_overlay / telemetry_stream);
# the repo is mounted read-only; /run/mpa read-write so the exposure commands reach
# voxl-camera-server's control pipe. stop uses SIGTERM (docker stop), never
# SIGKILL, so the controller puts the camera back on auto exposure ("start_ae").
# Its output is kept: before a container is replaced its log is saved to
# $LOG_DIR/<start time>.log.
set -euo pipefail

REPO=${REPO:-$(cd "$(dirname "$0")" && pwd)}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-8}
IMAGE=${IMAGE:-voxl2-humble}
NAME=exposure_controller
LOG_DIR=${LOG_DIR:-/data/exposure_controller_logs}

save_log() {   # keep the old container's output before it is removed
    if docker inspect "$NAME" >/dev/null 2>&1; then
        mkdir -p "$LOG_DIR"
        local started out
        started=$(docker inspect -f '{{.State.StartedAt}}' "$NAME" | cut -c1-19 | tr -d ':-' | tr 'T' '_')
        out="$LOG_DIR/${started:-unknown}.log"
        [ -e "$out" ] && out="$LOG_DIR/${started:-unknown}_$(date +%s).log"
        docker logs "$NAME" >"$out" 2>&1 || true
        echo "saved previous log: $out"
    fi
}

stop_container() {
    if docker inspect "$NAME" >/dev/null 2>&1; then
        docker stop -t 10 "$NAME" >/dev/null || true   # SIGTERM -> start_ae
        save_log
        docker rm "$NAME" >/dev/null 2>&1 || true
    fi
}

cmd=${1:-}
shift || true

case "$cmd" in
  start)
    stop_container
    docker run -d --name "$NAME" --init \
      --network host --ipc host --restart no \
      -e PYTHONDONTWRITEBYTECODE=1 -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
      -v "$REPO":/workspace:ro \
      -v /run/mpa:/run/mpa \
      -w /workspace \
      "$IMAGE" \
      bash -c 'source /opt/ros/humble/setup.bash && exec python3 -u exposure_controller.py "$@"' \
      _ "$@"
    echo "started $NAME (logs: $0 logs)"
    ;;
  stop)
    stop_container
    echo "stopped $NAME (camera back on auto exposure)"
    ;;
  logs)
    docker logs -f --tail 50 "$NAME"
    ;;
  status)
    docker ps -a --filter "name=^${NAME}$"
    ;;
  *)
    echo "usage: $0 {start [args]|stop|logs|status}" >&2
    exit 1
    ;;
esac
