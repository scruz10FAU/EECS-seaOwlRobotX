#!/usr/bin/env python3
"""
beacon_mavlink_ground.py — ground-station decoder for beacon detections
relayed over MAVLink by beacon_mavlink_bridge.py (drone side).

Listens on a MAVLink connection and decodes MEMORY_VECT messages whose
`address` matches BEACON_MAVLINK_ADDRESS (see
utils/beacon_mavlink_protocol.py for the wire format), printing GPS
position, color, and blink status for each one. Messages with a different
`address` are ignored — they're not ours.

This is meant to run on the GROUND STATION machine, separate from the
drone. Copy this file AND the utils/ folder (for
utils/beacon_mavlink_protocol.py) over together, so the import path below
still resolves.

Requires: pip install pymavlink

Usage:
    python3 beacon_mavlink_ground.py --mavlink-endpoint udpin:0.0.0.0:14550
    python3 beacon_mavlink_ground.py --mavlink-endpoint /dev/ttyUSB0 --baud 57600
"""

import argparse
import time

from pymavlink import mavutil

from utils.beacon_mavlink_protocol import (
    unpack_value_array, BEACON_MAVLINK_ADDRESS, BEACON_MAVLINK_VERSION,
)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument(
        "--mavlink-endpoint", required=True,
        help="pymavlink connection string, e.g. udpin:0.0.0.0:14550 or /dev/ttyUSB0",
    )
    ap.add_argument(
        "--baud", type=int, default=57600,
        help="Baud rate, only used for serial endpoints (default: 57600)",
    )
    args = ap.parse_args()

    print(f"[beacon-ground] Connecting to {args.mavlink_endpoint} ...")
    mav = mavutil.mavlink_connection(args.mavlink_endpoint, baud=args.baud)
    mav.wait_heartbeat()
    print(f"[beacon-ground] Heartbeat received from system {mav.target_system} "
          f"— listening for beacon messages")

    while True:
        msg = mav.recv_match(type="MEMORY_VECT", blocking=True, timeout=5.0)
        if msg is None:
            continue
        if msg.address != BEACON_MAVLINK_ADDRESS:
            continue  # not ours — some other real memory-vector traffic on this link
        if msg.ver != BEACON_MAVLINK_VERSION:
            print(f"[beacon-ground] WARNING: received beacon message with unknown "
                  f"protocol version {msg.ver} (expected {BEACON_MAVLINK_VERSION}) — skipping")
            continue

        beacon = unpack_value_array(msg.value)
        ts = time.strftime("%H:%M:%S")
        blink_str = "unknown" if beacon.is_blinking is None else str(beacon.is_blinking)
        print(f"[{ts}] beacon: lat={beacon.latitude:.6f} lon={beacon.longitude:.6f} "
              f"color={beacon.color} blinking={blink_str}")


if __name__ == "__main__":
    main()
