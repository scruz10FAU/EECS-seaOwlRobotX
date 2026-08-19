#!/bin/bash
# start_seabird_beacon.sh — Seabird mission launcher.
#
# Launches ONLY the Element #3 pilot safety-check flight
# (element3_safety_check.py). The detector and recorder are intentionally
# NOT started: the Element #3 manoeuvre is a pure flight pattern
# (take-off → climb up/out → pirouettes → 45° descent → land → disarm)
# and does not use vision, so no beacon detection or dataset recording is
# needed.
#
# Run this script from the directory containing element3_safety_check.py.
#
# Usage:
#   ./start_seabird_beacon.sh                          # run the Element #3 mission
#   ELEMENT3_ARGS="..." ./start_seabird_beacon.sh      # forward extra flags to the script
#   MUTE=ELEMENT3 ./start_seabird_beacon.sh            # hide ELEMENT3 output on this terminal
#
# !! SAFETY WARNING — verify before EVERY flight !!
#   RC takeover requires COM_RC_OVERRIDE to be non-zero. If it is 0, moving
#   the RC sticks will NOT interrupt the autonomous mission.
#   Check on the PX4 shell before arming:
#     pxh> param show COM_RC_OVERRIDE   # must be non-zero (3 = offboard + auto)
#   Do a bench test: arm, let the script enter OFFBOARD, move sticks, confirm
#   QGroundControl shows the mode switch to Position/Stabilized.
#   Also note: NAV_RCL_ACT=0 disables the RC-loss failsafe — the mission will
#   continue uninterrupted if the RC signal drops. Ctrl-C this terminal to abort.
#
# Pre-conditions (this script does NOT start them — start them manually):
#   Sim:      Isaac Sim running with marina scene + drone spawned (init_scene.py)
#             PX4 SITL running (make px4_sitl none_iris)
#   Physical: drone hardware on + flight controller reachable over MAVSDK
#
# What runs:
#   element3_safety_check.py [ELEMENT3, magenta] Element #3 manoeuvre:
#       take-off & hover → climb up/out to (2) → 3-4 stationary pirouettes →
#       ~45° descent to (4) → land → render SAFE. Publishes
#       /seabird/flight_path and /seabird/path_marker for RViz2.
#
# Output:
#   - Tagged, colored, interleaved lines on this terminal
#   - Plain per-component logs at logs/<timestamp>/<component>.log
#     plus <component>.stderr and a launch.log start/exit ledger
#
# Stop:
#   Ctrl-C in this terminal. The trap kills all children and waits for
#   them to exit before the script returns.

# ── ROS2 — must be sourced BEFORE `set -u` (ROS setup scripts are not -u safe) ──
if [ -z "${ROS_DISTRO:-}" ]; then
    if [ -f /opt/ros/humble/setup.bash ]; then
        # shellcheck disable=SC1091
        source /opt/ros/humble/setup.bash
        echo "[start_seabird] sourced ROS2 Humble"
    else
        echo "[start_seabird] WARNING: /opt/ros/humble/setup.bash not found —"
        echo "                rclpy-based nodes will fail. Source ROS2 manually."
    fi
fi

set -u

LOG_ROOT="logs"
TS=$(date +%Y%m%d_%H%M%S)
LOG_DIR="${LOG_ROOT}/${TS}"
mkdir -p "${LOG_DIR}"

# Launch/exit ledger — the "tombstone" file. One line per component start,
# one per component death with its exit code (or signal number). Written
# the instant a child exits, so a node that dies mid-flight leaves a record.
LAUNCH_LOG="${LOG_DIR}/launch.log"
: > "${LAUNCH_LOG}"

# ── Components: name | script | color (ANSI) ─────────────────────────────────
# Only the Element #3 flight runs. The script takes no required arguments;
# forward extras at run time with ELEMENT3_ARGS="...".
declare -a COMPONENTS=(
    "ELEMENT3|element3_safety_check.py|35"  # magenta — Element #3 pilot safety-check flight
)

# Inter-component startup delay (seconds) — lets each node set up its
# subscriptions before the next one starts. (Harmless with a single node.)
START_DELAY=1.5

# Parse MUTE env var ("TAG,TAG") into a lookup of muted tags.
MUTE="${MUTE:-}"
declare -A MUTED
if [ -n "${MUTE}" ]; then
    IFS=',' read -ra MUTE_LIST <<< "${MUTE}"
    for m in "${MUTE_LIST[@]}"; do
        clean=$(echo "${m}" | tr -d ' ' | tr '[:lower:]' '[:upper:]')
        MUTED["${clean}"]=1
    done
fi

# ── Banner ───────────────────────────────────────────────────────────
echo "══════════════════════════════════════════════════════════════"
echo "  Seabird Mission Launcher — Element #3 only"
echo "  Log dir:    ${LOG_DIR}"
echo "  Launch log: ${LAUNCH_LOG}"
if [ -n "${MUTE}" ]; then
    echo "  Muted:      ${MUTE}"
fi
# Echo any <TAG>_ARGS overrides so the operator sees what got forwarded.
for entry in "${COMPONENTS[@]}"; do
    IFS='|' read -r _tag _script _color <<< "${entry}"
    _tag_trimmed=$(echo "${_tag}" | tr -d ' ')
    _args_var="${_tag_trimmed}_ARGS"
    _args_val="${!_args_var:-}"
    if [ -n "${_args_val}" ]; then
        printf "  %-14s %s\n" "${_args_var}:" "${_args_val}"
    fi
done
echo "══════════════════════════════════════════════════════════════"

# ── Kill zombies ─────────────────────────────────────────────────────
# Before launching, terminate any stale instances of the component
# scripts so we never end up with two of the same node publishing at
# once. Derived from COMPONENTS, so newly added nodes are covered too.
kill_zombies() {
    echo "  checking for zombie nodes..."
    local found=0 entry tag script color base pids

    # mavsdk_server holds the UDP port between runs — always kill it first.
    pids=$(pgrep -f -- "mavsdk_server" 2>/dev/null || true)
    if [ -n "${pids}" ]; then
        echo "    zombie mavsdk_server: SIGTERM pids ${pids}"
        # shellcheck disable=SC2086
        kill -TERM ${pids} 2>/dev/null || true
        found=1
    fi

    for entry in "${COMPONENTS[@]}"; do
        IFS='|' read -r tag script color <<< "${entry}"
        base=$(basename "${script}" | tr -d ' ')
        pids=$(pgrep -f -- "python3.*${base}\|python3.10.*${base}" 2>/dev/null || true)
        if [ -n "${pids}" ]; then
            echo "    zombie ${base}: SIGTERM pids ${pids}"
            # shellcheck disable=SC2086
            kill -TERM ${pids} 2>/dev/null || true
            found=1
        fi
    done
    if [ "${found}" -eq 1 ]; then
        sleep 2
        pids=$(pgrep -f -- "mavsdk_server" 2>/dev/null || true)
        if [ -n "${pids}" ]; then
            echo "    zombie mavsdk_server: SIGKILL pids ${pids}"
            # shellcheck disable=SC2086
            kill -KILL ${pids} 2>/dev/null || true
        fi
        for entry in "${COMPONENTS[@]}"; do
            IFS='|' read -r tag script color <<< "${entry}"
            base=$(basename "${script}" | tr -d ' ')
            pids=$(pgrep -f -- "python3.*${base}\|python3.10.*${base}" 2>/dev/null || true)
            if [ -n "${pids}" ]; then
                echo "    zombie ${base}: SIGKILL pids ${pids}"
                # shellcheck disable=SC2086
                kill -KILL ${pids} 2>/dev/null || true
            fi
        done
    else
        echo "    none found"
    fi
}

# ── Spawn one tagged child ───────────────────────────────────────────
# Args: tag(padded) script color_code log_file stderr_file
#
# PYTHONUNBUFFERED=1 + python3 -u force line-buffered stdout (otherwise
# Python buffers prints when stdout is a pipe and you see nothing until
# the child exits). stdout+stderr go through awk, which prefixes each
# line with the colored tag (terminal) and appends the raw line to the
# log file. stderr is also tee'd to a dedicated .stderr file so a
# signal/segfault death leaves a raw record even if awk's tail is lost.
spawn_child() {
    local tag="$1"
    local script="$2"
    local color="$3"
    local log_file="$4"
    local stderr_file="$5"
    local tag_trimmed
    tag_trimmed=$(echo "${tag}" | tr -d ' ')

    local is_muted=0
    if [ -n "${MUTED[${tag_trimmed}]:-}" ]; then
        is_muted=1
    fi

    # Per-component extra args via <TAG>_ARGS env var (e.g. ELEMENT3_ARGS).
    # Indirect expansion (${!var}) with :- guards against set -u when unset.
    # Left unquoted in the python invocation so flags word-split normally.
    local args_var="${tag_trimmed}_ARGS"
    local extra_args="${!args_var:-}"
    if [ -n "${extra_args}" ]; then
        echo "$(date '+%F %T') ${tag_trimmed} ARGS ${extra_args}" >> "${LAUNCH_LOG}"
    fi

    (
        {
            # shellcheck disable=SC2086
            PYTHONUNBUFFERED=1 python3.10 -u "./${script}" ${extra_args} \
                2> >(tee -a "${stderr_file}" >&2)
        } 2>&1 \
        | awk -v tag="${tag}" -v color="${color}" -v muted="${is_muted}" \
              -v logfile="${log_file}" '
            {
                print $0 >> logfile
                fflush(logfile)
                if (muted == 0) {
                    printf "\033[%sm[%s]\033[0m %s\n", color, tag, $0
                    fflush()
                }
            }
            END { close(logfile) }
        '
        # PIPESTATUS[0] is the python stage (awk is [1]). A non-zero code,
        # or a signal (rc > 128), is the tombstone we want on disk.
        rc=${PIPESTATUS[0]}
        if [ "${rc}" -gt 128 ]; then
            echo "$(date '+%F %T') ${tag_trimmed} EXITED rc=${rc} (killed by signal $((rc - 128)))" >> "${LAUNCH_LOG}"
        else
            echo "$(date '+%F %T') ${tag_trimmed} EXITED rc=${rc}" >> "${LAUNCH_LOG}"
        fi
    ) &
    CHILD_PIDS+=("$!")
    echo "$(date '+%F %T') ${tag_trimmed} STARTED subshell_pid=$! script=${script}" >> "${LAUNCH_LOG}"
}

# ── Cleanup on Ctrl-C ────────────────────────────────────────────────
CHILD_PIDS=()
cleanup() {
    echo ""
    echo "──────────────────────────────────────────────────────────"
    echo "  Shutting down Seabird nodes..."
    echo "──────────────────────────────────────────────────────────"

    for pid in "${CHILD_PIDS[@]}"; do
        if kill -0 "${pid}" 2>/dev/null; then
            kill -TERM "${pid}" 2>/dev/null || true
        fi
    done

    sleep 3

    for pid in "${CHILD_PIDS[@]}"; do
        if kill -0 "${pid}" 2>/dev/null; then
            kill -KILL "${pid}" 2>/dev/null || true
        fi
    done

    # Belt-and-suspenders: catch python children that escaped the subshell.
    # Derived from COMPONENTS so added nodes are covered automatically.
    local entry tag script color base pids
    for entry in "${COMPONENTS[@]}"; do
        IFS='|' read -r tag script color <<< "${entry}"
        base=$(basename "${script}" | tr -d ' ')
        pids=$(pgrep -f -- "python3.*${base}\|python3.10.*${base}" 2>/dev/null || true)
        if [ -n "${pids}" ]; then
            # shellcheck disable=SC2086
            kill -TERM ${pids} 2>/dev/null || true
        fi
    done

    echo "  done. Logs preserved at ${LOG_DIR}"
    exit 0
}
trap cleanup INT TERM

# ── Pre-flight: clear zombies ────────────────────────────────────────
kill_zombies

# ── Start each component in order ────────────────────────────────────
echo "──────────────────────────────────────────────────────────"
for entry in "${COMPONENTS[@]}"; do
    IFS='|' read -r tag script color <<< "${entry}"
    tag_trimmed=$(echo "${tag}" | tr -d ' ')
    script_trimmed=$(echo "${script}" | tr -d ' ')
    log_file="${LOG_DIR}/${tag_trimmed}.log"
    stderr_file="${LOG_DIR}/${tag_trimmed}.stderr"

    if [ ! -f "${script_trimmed}" ]; then
        echo "  [SKIP] ${tag_trimmed}: ${script_trimmed} not found in $(pwd)"
        continue
    fi

    : > "${log_file}"
    : > "${stderr_file}"

    echo "  starting ${tag_trimmed} → ${log_file}"
    spawn_child "${tag}" "${script_trimmed}" "${color}" "${log_file}" "${stderr_file}"
    sleep "${START_DELAY}"
done

echo "──────────────────────────────────────────────────────────"
echo "  Node started. Streaming output (Ctrl-C to stop)."
echo "──────────────────────────────────────────────────────────"
echo ""

wait
