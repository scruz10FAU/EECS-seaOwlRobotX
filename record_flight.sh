#!/bin/bash
# record_flight.sh -- on the VOXL2 host, record a flight's beacon detections
# (ROS 2 bag) and every MAVLink message handed to / received from the ELRS
# radio (packet capture), into a new folder per run. Nothing is overwritten.
#
#   sudo ./record_flight.sh                 # record until Ctrl-C
#   sudo DURATION=900 ./record_flight.sh    # or stop by itself after 900 s
#   ./record_flight.sh decode <dir>/elrs.pcap [out.csv]   # MAVLink CSV (drone or laptop)
#
# What is captured
#   bag/       ros2 bag of $TOPICS (beacon detections, beacon map, current_task,
#              run_start, exposure_controller.py's /seabird/exposure).
#   elrs.pcap  UDP to/from the drone's ELRS bridge (elrs-mavlink) inside the
#              $NETNS namespace: port 14550 = everything sent towards the ground
#              (PX4 telemetry from voxl-mavlink-server + beacon_mavlink_bridge.py's
#              MEMORY_VECT beacons, SEABIRD_TASK / RUNSTART_ACK STATUSTEXT) and
#              what came up from the ground; 14570/14571 = ground SEABIRD_
#              STATUSTEXT copied on board (RunStart, restart requests).
#              This is what was HANDED to the ELRS bridge; its rate limiter may
#              still drop low-priority telemetry (never STATUSTEXT/MEMORY_VECT).
#   record.log what this script did, ros2 bag output, tcpdump output.
#
# Settings (environment variables):
#   OUT_ROOT       /data/seabird_flight_logs   (a new <YYYYmmdd_HHMMSS>/ per run)
#   TOPICS         the ROS 2 topics to bag (space-separated)
#   ROS_DOMAIN_ID  8 -- must match the nodes publishing the beacon topics
#   NETNS          rfdns -- namespace the ELRS bridge runs in ("" = host namespace)
#   PORTS          "14550 14570 14571"
#   DURATION       stop after this many seconds (default: until Ctrl-C)
#   MIN_FREE_MB    1024 -- don't start, or stop cleanly, below this much free
#                  space on $OUT_ROOT's disk (leaves room for PX4's own flight log)
#   ROS_IMAGE      voxl2-humble -- Docker image with ROS 2 Humble; ros2 bag runs
#                  in a container of it (--network host --ipc host, like
#                  voxl/telemetry_stream/run.sh), since the VOXL host has no Humble
#
# Run on the VOXL HOST (not inside a container), as root: tcpdump + ip netns
# need the host. Needs tcpdump, docker and the $ROS_IMAGE image.

set -u

decode() {
    local pcap=${1:?usage: $0 decode <file.pcap> [out.csv]}
    local out=${2:-${pcap%.pcap}_mavlink.csv}
    if [ -e "$out" ]; then
        echo "refusing to overwrite $out -- pass another output name" >&2
        exit 1
    fi
    # Beacon payloads are decoded when this repo's utils/ is importable.
    local repo
    repo=$(cd "$(dirname "$0")" && pwd)   # this script sits at the repo root
    PYTHONPATH="$repo${PYTHONPATH:+:$PYTHONPATH}" python3 - "$pcap" "$out" <<'PY'
import csv, struct, sys
from datetime import datetime, timezone
from pymavlink.dialects.v20 import common as mavlink2
try:
    from utils.beacon_mavlink_protocol import BEACON_MAVLINK_ADDRESS, unpack_value_array
except ImportError:
    BEACON_MAVLINK_ADDRESS, unpack_value_array = 0xBEAC, None

pcap_path, out_path = sys.argv[1], sys.argv[2]
LINK_HDR = {1: 14, 113: 16, 276: 20, 101: 0, 228: 0}   # ethernet, SLL, SLL2, raw IPv4

with open(pcap_path, "rb") as f:
    data = f.read()
magic = data[:4]
endian = {b"\xd4\xc3\xb2\xa1": "<", b"\xa1\xb2\xc3\xd4": ">",
          b"\x4d\x3c\xb2\xa1": "<", b"\xa1\xb2\x3c\x4d": ">"}.get(magic)
if endian is None:
    sys.exit("not a classic pcap file (pcapng is not supported; capture with tcpdump -w)")
nano = magic in (b"\x4d\x3c\xb2\xa1", b"\xa1\xb2\x3c\x4d")
linktype = struct.unpack(endian + "I", data[20:24])[0]
if linktype not in LINK_HDR:
    sys.exit(f"unsupported pcap link type {linktype}")

parsers, rows, bad = {}, 0, 0
with open(out_path, "x", newline="") as out:
    w = csv.writer(out)
    w.writerow(["time_utc", "unix_s", "src", "dst", "direction", "msg_id", "msg_type",
                "sysid", "compid", "fields"])
    pos = 24
    while pos + 16 <= len(data):
        ts_s, ts_frac, incl, _ = struct.unpack(endian + "IIII", data[pos:pos + 16])
        pkt = data[pos + 16:pos + 16 + incl]
        pos += 16 + incl
        ip = pkt[LINK_HDR[linktype]:]
        if len(ip) < 28 or ip[0] >> 4 != 4 or ip[9] != 17:
            continue                       # IPv4 UDP only
        ihl = (ip[0] & 0x0F) * 4
        sport, dport = struct.unpack(">HH", ip[ihl:ihl + 4])
        payload = ip[ihl + 8:]
        src = f"{'.'.join(map(str, ip[12:16]))}:{sport}"
        dst = f"{'.'.join(map(str, ip[16:20]))}:{dport}"
        t = ts_s + ts_frac / (1e9 if nano else 1e6)
        direction = "to_elrs" if dport == 14550 else ("from_elrs" if sport == 14550 else "onboard_copy")
        p = parsers.get((src, dst))
        if p is None:
            p = parsers[(src, dst)] = mavlink2.MAVLink(None)
            p.robust_parsing = True
        try:
            msgs = p.parse_buffer(payload) or []
        except Exception:
            bad += 1
            continue
        for m in msgs:
            if m.get_type() == "BAD_DATA":
                bad += 1
                continue
            d = m.to_dict()
            d.pop("mavpackettype", None)
            if (m.get_type() == "MEMORY_VECT" and d.get("address") == BEACON_MAVLINK_ADDRESS
                    and unpack_value_array is not None):
                try:
                    d = {"beacon": str(unpack_value_array(m.value))}
                except Exception as e:
                    d["beacon_decode_error"] = str(e)
            w.writerow([datetime.fromtimestamp(t, timezone.utc).isoformat(), f"{t:.6f}",
                        src, dst, direction, m.get_msgId(), m.get_type(),
                        m.get_srcSystem(), m.get_srcComponent(), d])
            rows += 1
print(f"{rows} MAVLink messages -> {out_path} ({bad} unparseable chunks)")
PY
}

if [ "${1:-}" = decode ]; then
    shift
    decode "$@"
    exit $?
fi

OUT_ROOT=${OUT_ROOT:-/data/seabird_flight_logs}
TOPICS=${TOPICS:-"/seabird/beacon_detections_downward /seabird/beacon_detections /seabird/beacon_map /seabird/current_task /seabird/run_start /seabird/exposure"}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-8}
NETNS=${NETNS-rfdns}
PORTS=${PORTS:-"14550 14570 14571"}
DURATION=${DURATION:-}
MIN_FREE_MB=${MIN_FREE_MB:-1024}
ROS_IMAGE=${ROS_IMAGE:-voxl2-humble}
BAG_CONTAINER=seabird_record_bag
free_mb() { df -Pm "$1" 2>/dev/null | awk 'NR==2 {print $4}'; }
# ros2 in a Humble container on the host network (sees the same DDS topics).
ros2_docker() {   # ros2_docker [docker run options...] -- <ros2 args...>
    local opts=()
    while [ "$1" != "--" ]; do opts+=("$1"); shift; done
    shift
    # --init: ros2 isn't PID 1, so the SIGINT from stop() reaches it.
    docker run --rm --init --network host --ipc host -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
        "${opts[@]}" "$ROS_IMAGE" \
        bash -c 'source /opt/ros/humble/setup.bash && exec ros2 "$@"' _ "$@"
}

if [ "$(id -u)" -ne 0 ]; then
    echo "run as root (sudo): tcpdump and ip netns need it" >&2
    exit 1
fi
command -v tcpdump >/dev/null || { echo "tcpdump not found (apt install tcpdump)" >&2; exit 1; }
if [ -n "$NETNS" ] && ! ip netns list | grep -qw "$NETNS"; then
    echo "network namespace '$NETNS' not found (is elrs-mavlink running? NETNS= for host)" >&2
    exit 1
fi
command -v docker >/dev/null || { echo "docker not found" >&2; exit 1; }
docker image inspect "$ROS_IMAGE" >/dev/null 2>&1 || {
    echo "Docker image '$ROS_IMAGE' not found (ROS_IMAGE= to use another Humble image)" >&2
    exit 1
}
docker rm -f "$BAG_CONTAINER" >/dev/null 2>&1 || true   # a bag container left from a crash

# New folder per run; never reuse an existing one.
base="$OUT_ROOT/$(date +%Y%m%d_%H%M%S)"
OUT=$base n=1
while [ -e "$OUT" ]; do OUT="${base}_$n"; n=$((n + 1)); done
mkdir -p "$OUT_ROOT" || exit 1
avail=$(free_mb "$OUT_ROOT")
if [ -n "$avail" ] && [ "$avail" -lt "$MIN_FREE_MB" ]; then
    echo "only ${avail} MB free on $OUT_ROOT (< MIN_FREE_MB=$MIN_FREE_MB) -- not recording;" \
         "copy old runs off the drone first" >&2
    exit 1
fi
mkdir -p "$OUT" || exit 1
LOG="$OUT/record.log"
say() { echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }
say "${avail:-?} MB free on $OUT_ROOT; stopping below $MIN_FREE_MB MB"

say "recording to $OUT (ROS_DOMAIN_ID=$ROS_DOMAIN_ID, netns=${NETNS:-host}, ports $PORTS)"
present=$(timeout 30 ros2_docker -- topic list 2>/dev/null)
for t in $TOPICS; do
    grep -qx "$t" <<<"$present" || say "WARNING: $t not visible yet -- it is recorded if it appears later"
done

# ELRS packet capture (-U: write each packet at once, so a power cut keeps the file).
filter="udp and ($(printf 'port %s or ' $PORTS | sed 's/ or $//'))"
netns_cmd=()
[ -n "$NETNS" ] && netns_cmd=(ip netns exec "$NETNS")
"${netns_cmd[@]}" tcpdump -i any -U -n -w "$OUT/elrs.pcap" "$filter" >>"$LOG" 2>&1 &
TCPDUMP_PID=$!

# ros2 bag in a Humble container; the run folder is mounted at /rec.
# shellcheck disable=SC2086
ros2_docker --name "$BAG_CONTAINER" -v "$OUT":/rec -- bag record -o /rec/bag $TOPICS \
    >>"$LOG" 2>&1 &
BAG_PID=$!   # the docker client: exits when the container does

stop() {
    trap - INT TERM
    say "stopping..."
    # SIGINT to ros2 bag inside the container: it finalises metadata.yaml.
    docker kill --signal=INT "$BAG_CONTAINER" >/dev/null 2>&1
    kill -INT "$TCPDUMP_PID" 2>/dev/null
    for _ in $(seq 15); do
        kill -0 "$BAG_PID" 2>/dev/null || break
        sleep 1
    done
    if kill -0 "$BAG_PID" 2>/dev/null; then
        say "WARNING: ros2 bag did not stop in 15 s -- stopping its container"
        docker stop -t 5 "$BAG_CONTAINER" >/dev/null 2>&1
    fi
    wait "$BAG_PID" "$TCPDUMP_PID" 2>/dev/null
    say "done: $OUT"
    ls -la "$OUT" "$OUT/bag" 2>/dev/null | tee -a "$LOG"
    say "decode the ELRS capture with: $0 decode $OUT/elrs.pcap"
    exit 0
}
trap stop INT TERM

sleep 3
kill -0 "$TCPDUMP_PID" 2>/dev/null || say "ERROR: tcpdump exited -- see $LOG"
kill -0 "$BAG_PID" 2>/dev/null || say "ERROR: ros2 bag exited -- see $LOG"
say "recording (Ctrl-C to stop${DURATION:+, or after ${DURATION} s})"

# Watch free space (and DURATION) every 5 s; Ctrl-C still stops at once.
started=$SECONDS
while kill -0 "$BAG_PID" 2>/dev/null || kill -0 "$TCPDUMP_PID" 2>/dev/null; do
    avail=$(free_mb "$OUT")
    if [ -n "$avail" ] && [ "$avail" -lt "$MIN_FREE_MB" ]; then
        say "WARNING: only ${avail} MB free (< $MIN_FREE_MB MB) -- stopping the recording"
        stop
    fi
    if [ -n "$DURATION" ] && [ $((SECONDS - started)) -ge "$DURATION" ]; then
        stop
    fi
    sleep 5 & wait $!
done
say "ERROR: both recorders exited on their own -- see $LOG"
stop
