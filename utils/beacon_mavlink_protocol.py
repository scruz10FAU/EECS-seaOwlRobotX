"""
beacon_mavlink_protocol.py — binary wire format for relaying beacon
detections to the ground station over the existing MAVLink link.

Packs (latitude, longitude, color, blink status) into the `value` byte
array of a standard MAVLink MEMORY_VECT message (common.xml, msg id 249)
instead of defining a custom dialect — both ends only need stock
pymavlink, no mavgen/code-generation step. `address` is set to
BEACON_MAVLINK_ADDRESS, a sentinel so the ground-station decoder can pick
our traffic out from any other real memory-vector use on the same link.

This module has no pymavlink/ROS dependency of its own (just `struct`), so
it's imported identically by beacon_mavlink_bridge.py (drone side, ROS2)
and beacon_mavlink_ground.py (ground station, no ROS) — copy the whole
`utils/` folder to the ground station machine so the import path matches.
"""

import struct
from typing import NamedTuple, Optional

BEACON_MAVLINK_ADDRESS = 0xBEAC   # MEMORY_VECT.address sentinel identifying our messages
BEACON_MAVLINK_VERSION = 1        # MEMORY_VECT.ver — bump if the payload layout changes

COLOR_CODES = {"unknown": 0, "red": 1, "green": 2, "blue": 3, "white": 4}
COLOR_NAMES = {v: k for k, v in COLOR_CODES.items()}

BLINK_CODES = {None: 0, False: 1, True: 2}
BLINK_NAMES = {v: k for k, v in BLINK_CODES.items()}

_PAYLOAD_FMT = "<iiBB"                        # lat_e7: int32, lon_e7: int32, color: uint8, blink: uint8
_PAYLOAD_SIZE = struct.calcsize(_PAYLOAD_FMT)  # 10 bytes
_VALUE_ARRAY_LEN = 32                          # MEMORY_VECT.value is a fixed int8[32]


class BeaconMavlinkMessage(NamedTuple):
    latitude: float
    longitude: float
    color: str
    is_blinking: Optional[bool]


def pack_value_array(latitude: float, longitude: float, color: str,
                      is_blinking: Optional[bool]) -> list:
    """
    Build the 32-entry signed-int8 list for MEMORY_VECT.value.

    lat/lon are scaled ×1e7 and stored as int32 (same convention MAVLink's
    own GLOBAL_POSITION_INT uses), matching this repo's existing GPS
    ground-truth fields (see gps_ground_truth in beacon_detector_config.py).
    """
    lat_e7 = int(round(latitude * 1e7))
    lon_e7 = int(round(longitude * 1e7))
    color_code = COLOR_CODES.get(color, COLOR_CODES["unknown"])
    blink_code = BLINK_CODES.get(is_blinking, BLINK_CODES[None])

    payload = struct.pack(_PAYLOAD_FMT, lat_e7, lon_e7, color_code, blink_code)
    padded = payload.ljust(_VALUE_ARRAY_LEN, b"\x00")
    return list(struct.unpack(f"{_VALUE_ARRAY_LEN}b", padded))  # raw bytes -> signed int8 list


def unpack_value_array(value) -> BeaconMavlinkMessage:
    """Inverse of pack_value_array — value is MEMORY_VECT.value (32 signed int8s)."""
    raw = struct.pack(f"{_VALUE_ARRAY_LEN}b", *value)
    lat_e7, lon_e7, color_code, blink_code = struct.unpack(_PAYLOAD_FMT, raw[:_PAYLOAD_SIZE])
    return BeaconMavlinkMessage(
        latitude=lat_e7 / 1e7,
        longitude=lon_e7 / 1e7,
        color=COLOR_NAMES.get(color_code, "unknown"),
        is_blinking=BLINK_NAMES.get(blink_code, None),
    )
