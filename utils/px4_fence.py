"""
px4_fence.py — uploads a polygon geofence and an altitude ceiling to PX4
over an existing pymavlink connection, without blocking the caller's loop.

Used by beacon_mavlink_ground.py to put the RoboCommand course geofence
(seaowls_interfaces/UavGeofence from seaowls_ocs) on the drone:

  - each polygon becomes one PX4 inclusion polygon, uploaded with the
    MAVLink mission protocol (MAV_MISSION_TYPE_FENCE): MISSION_COUNT ->
    MISSION_REQUEST_INT per vertex -> MISSION_ACK;
  - the ceiling (metres HAE, as RoboCommand declares it) becomes
    GF_MAX_VER_DIST, which PX4 measures above home. Converted with home's
    AMSL altitude (HOME_POSITION) plus the local geoid separation
    (GPS_RAW_INT alt_ellipsoid - alt), and confirmed by the PARAM_VALUE echo.

What PX4 does on a breach is GF_ACTION -- not touched here.

Usage (caller owns the connection and the receive loop):
    fence = FenceUploader(mav)
    fence.start(target_system, [[(lat, lon), ...]], max_alt_hae_m)
    ... for every received message: fence.handle(msg)
    ... every loop iteration:       fence.tick(time.monotonic())
"""

import time

from pymavlink import mavutil

# No reply for this long -> resend the last step; give up after MAX_RETRIES
# resends in a row.
REPLY_TIMEOUT_S = 3.0
MAX_RETRIES = 5
CEILING_PARAM = "GF_MAX_VER_DIST"

_FENCE = mavutil.mavlink.MAV_MISSION_TYPE_FENCE


def open_polygon(lats, lons):
    """[(lat, lon), ...] with RoboCommand's closing point (== first) dropped --
    PX4 closes fence polygons itself."""
    points = list(zip(lats, lons))
    if len(points) > 1 and points[0] == points[-1]:
        points.pop()
    return points


class FenceUploader:
    """One fence + ceiling upload at a time; start() again to replace it."""

    def __init__(self, mav, log=print):
        self.mav = mav
        self.log = log
        self.state = "idle"   # idle | fence | ceiling | done | failed
        self._home_amsl_m = None
        self._geoid_sep_m = None

    def start(self, target_system, polygons, max_alt_hae_m):
        """Upload `polygons` (lists of (lat, lon), >= 3 points each) as
        inclusion polygons, then set the ceiling from `max_alt_hae_m`."""
        polygons = [p for p in polygons if len(p) >= 3]
        if not polygons:
            self._fail("no polygon with at least 3 vertices")
            return
        self._target = target_system
        self._items = [(lat, lon, len(p)) for p in polygons for lat, lon in p]
        self._max_alt_hae_m = max_alt_hae_m
        self._last_seq = None
        self._waiting_logged = False
        self.state = "fence"
        self.log(f"[fence] Uploading {len(polygons)} inclusion polygon(s), "
                 f"{len(self._items)} vertices, to system {target_system}")
        self._retries = 0
        self._send_count()

    # ── incoming messages ────────────────────────────────────────────────

    def handle(self, msg):
        msg_type = msg.get_type()
        if msg_type == "HOME_POSITION":
            self._home_amsl_m = msg.altitude / 1000.0
        elif msg_type == "GPS_RAW_INT":
            if msg.fix_type >= 3 and getattr(msg, "alt_ellipsoid", 0):
                self._geoid_sep_m = (msg.alt_ellipsoid - msg.alt) / 1000.0
        elif self.state == "fence" and self._for_us(msg):
            if msg_type in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
                if msg.mission_type == _FENCE and 0 <= msg.seq < len(self._items):
                    self._retries = 0
                    self._send_item(msg.seq)
            elif msg_type == "MISSION_ACK" and msg.mission_type == _FENCE:
                if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    self.log("[fence] ✓ Fence accepted by PX4")
                    self.state = "ceiling"
                    self._sent_ceiling = None
                    self._retries = 0
                    self.tick(time.monotonic())
                else:
                    name = mavutil.mavlink.enums["MAV_MISSION_RESULT"][msg.type].name
                    self._fail(f"PX4 rejected the fence: {name}")
        elif (self.state == "ceiling" and msg_type == "PARAM_VALUE"
              and msg.get_srcSystem() == self._target
              and msg.param_id == CEILING_PARAM and self._sent_ceiling is not None):
            if abs(msg.param_value - self._sent_ceiling) < 0.01:
                self.log(f"[fence] ✓ {CEILING_PARAM} = {msg.param_value:.1f} m above home "
                         f"({self._max_alt_hae_m:.1f} m HAE)")
                self.state = "done"

    def _for_us(self, msg):
        return (msg.get_srcSystem() == self._target
                and msg.target_system == self.mav.source_system
                and msg.target_component == self.mav.source_component)

    # ── timeouts / pending work ─────────────────────────────────────────

    def tick(self, now):
        if self.state == "ceiling" and self._sent_ceiling is None:
            if self._home_amsl_m is None or self._geoid_sep_m is None:
                if not self._waiting_logged:
                    self.log("[fence] Waiting for HOME_POSITION and a GPS_RAW_INT "
                             "alt_ellipsoid to convert the HAE ceiling")
                    self._waiting_logged = True
                return
            ceiling = self._max_alt_hae_m - (self._home_amsl_m + self._geoid_sep_m)
            if ceiling <= 0:
                self._fail(f"ceiling {self._max_alt_hae_m:.1f} m HAE is not above home "
                           f"({self._home_amsl_m + self._geoid_sep_m:.1f} m HAE)")
                return
            self._sent_ceiling = round(ceiling, 2)
            self._send_ceiling()
            return
        if self.state in ("fence", "ceiling") and now - self._sent_t >= REPLY_TIMEOUT_S:
            self._retries += 1
            if self._retries > MAX_RETRIES:
                step = "fence upload" if self.state == "fence" else CEILING_PARAM
                self._fail(f"no reply from PX4 to {step} after {MAX_RETRIES} retries")
            elif self.state == "ceiling":
                self._send_ceiling()
            elif self._last_seq is None:
                self._send_count()
            else:
                self._send_item(self._last_seq)

    # ── outgoing messages ───────────────────────────────────────────────

    def _send_count(self):
        self.mav.mav.mission_count_send(
            self._target, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            len(self._items), _FENCE,
        )
        self._sent_t = time.monotonic()

    def _send_item(self, seq):
        lat, lon, vertex_count = self._items[seq]
        self.mav.mav.mission_item_int_send(
            self._target, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1, seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
            0, 0,                         # current, autocontinue
            vertex_count, 0, 0, 0,        # param1 = vertices in this polygon
            int(round(lat * 1e7)), int(round(lon * 1e7)), 0.0,
            _FENCE,
        )
        self._last_seq = seq
        self._sent_t = time.monotonic()

    def _send_ceiling(self):
        self.mav.mav.param_set_send(
            self._target, mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            CEILING_PARAM.encode(), self._sent_ceiling,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        self._sent_t = time.monotonic()

    def _fail(self, reason):
        self.log(f"[fence] FAILED: {reason}")
        self.state = "failed"
