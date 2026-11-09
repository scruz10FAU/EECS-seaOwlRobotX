#!/usr/bin/env python3
"""
seabird_remote_restart.py -- restart seabird-mission from the ground station.

Runs on the VOXL host as root (voxl/seabird-remote.service), independent of
seabird-mission, so it still works when the mission (and the
beacon_mavlink_bridge.py inside it) is down.

    GUI "Restart drone mission"
      -> beacon_mavlink_ground.py: STATUSTEXT "SEABIRD_RESTART_MISSION <id>"
      -> ELRS -> elrs_mavlink_bridge.py copies "SEABIRD_*" to
         --onboard-ip:--onboard-port (10.200.0.1:14570,14571)
      -> this listener (14571):
           armed?  -> "SEABIRD_RESTART_ACK <id> REFUSED armed"
           else    -> systemctl restart --no-block seabird-mission
                      "SEABIRD_RESTART_ACK <id> OK"
      -> back over ELRS to the ground (via the bridge at 10.200.0.2:14550).

Each <id> is acted on once; the ground repeats the request until it sees an
ACK, and repeats get the same answer again without a second restart.

After a restart the launcher waits for the CH7 RC trigger again, so a
restart never starts a flight. The armed check fails safe: if PX4's state
can't be read, the restart is refused.

    sudo python3 seabird_remote_restart.py \
        --listen udpin:10.200.0.1:14571 --reply udpout:10.200.0.2:14550
"""

import argparse
import os
import re
import shutil
import subprocess
import sys
import time

os.environ.setdefault("MAVLINK20", "1")   # before importing pymavlink
from pymavlink import mavutil  # noqa: E402

REQUEST_PREFIX = "SEABIRD_RESTART_MISSION"
ACK_PREFIX = "SEABIRD_RESTART_ACK"
STATUSTEXT_MAX = 50
PX4_LISTENER_CANDIDATES = ("px4-listener", "/usr/bin/px4-listener", "/usr/local/bin/px4-listener")
ARMING_STATE_ARMED = 2   # vehicle_status.arming_state (PX4 v1.14 and v1.15)


def log(text):
    print(f"[remote-restart] {text}", flush=True)


def px4_armed():
    """(armed, detail). armed is None when PX4's state can't be read."""
    exe = next((p for p in (shutil.which(c) or c for c in PX4_LISTENER_CANDIDATES)
                if os.path.isfile(p) and os.access(p, os.X_OK)), None)
    if not exe:
        return None, "px4-listener not found"
    try:
        # Python 3.6 on the VOXL: no capture_output/text (3.7+).
        out = subprocess.run([exe, "vehicle_status"], stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, universal_newlines=True,
                             timeout=5.0).stdout
    except (OSError, subprocess.TimeoutExpired) as e:
        return None, f"px4-listener failed: {e}"
    m = re.search(r"arming_state:\s*(\d+)", out)
    if not m:
        return None, "no arming_state in px4-listener output"
    state = int(m.group(1))
    return state == ARMING_STATE_ARMED, f"arming_state={state}"


def restart(unit):
    """(ok, detail) for 'systemctl restart --no-block <unit>'."""
    try:
        r = subprocess.run(["systemctl", "restart", "--no-block", unit],
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                           universal_newlines=True, timeout=10.0)
    except (OSError, subprocess.TimeoutExpired) as e:
        return False, f"systemctl failed: {e}"
    if r.returncode != 0:
        return False, (r.stderr or r.stdout or f"exit {r.returncode}").strip()
    return True, "restart queued"


def decide(request_id, unit):
    """Check armed, restart if safe. Returns the ACK suffix ('OK' / 'REFUSED ...')."""
    armed, detail = px4_armed()
    if armed is None:
        log(f"request {request_id}: REFUSED -- can't read PX4 armed state ({detail})")
        return "REFUSED no-armed-state"
    if armed:
        log(f"request {request_id}: REFUSED -- UAV is armed ({detail})")
        return "REFUSED armed"
    ok, why = restart(unit)
    if not ok:
        log(f"request {request_id}: FAILED -- {why}")
        return "REFUSED systemctl-failed"
    log(f"request {request_id}: OK -- {unit} restarting (disarmed, {detail})")
    return "OK"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--listen", default="udpin:10.200.0.1:14571",
                    help="where the ELRS bridge copies 'SEABIRD_*' ground STATUSTEXT")
    ap.add_argument("--reply", default="udpout:10.200.0.2:14550",
                    help="the ELRS bridge's UDP port, for the ACK back to the ground")
    ap.add_argument("--unit", default="seabird-mission", help="systemd unit to restart")
    ap.add_argument("--sysid", type=int, default=1, help="MAVLink source system for the ACK")
    ap.add_argument("--compid", type=int, default=192, help="MAVLink source component for the ACK")
    args = ap.parse_args()

    if os.geteuid() != 0:
        log("WARNING: not running as root -- systemctl restart will probably fail")

    listen = mavutil.mavlink_connection(args.listen)
    reply = mavutil.mavlink_connection(args.reply, source_system=args.sysid,
                                       source_component=args.compid)
    log(f"listening on {args.listen} for '{REQUEST_PREFIX} <id>'; replies to {args.reply}; "
        f"unit {args.unit}")

    answers = {}   # request id -> ACK suffix (each id acted on once)
    while True:
        msg = listen.recv_match(type="STATUSTEXT", blocking=True, timeout=1.0)
        if msg is None:
            continue
        text = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="replace")
        text = text.rstrip("\x00").strip()
        if not text.startswith(REQUEST_PREFIX + " "):
            continue
        request_id = text[len(REQUEST_PREFIX):].strip()[:16]
        if not request_id:
            continue
        if request_id not in answers:
            log(f"request {request_id} received")
            try:
                answers[request_id] = decide(request_id, args.unit)
            except Exception as e:   # never die on a request: answer it instead
                log(f"request {request_id}: REFUSED -- internal error: {type(e).__name__}: {e}")
                answers[request_id] = "REFUSED internal-error"
            if len(answers) > 100:
                answers.pop(next(iter(answers)))
        ack = f"{ACK_PREFIX} {request_id} {answers[request_id]}"[:STATUSTEXT_MAX]
        reply.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, ack.encode())


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
