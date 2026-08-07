#!/usr/bin/env python3
"""
blink_detector.py — Rolling-window blink frequency estimator for beacon lights.

Imported by beacon_detector.py.  No ROS or OpenCV dependency.
"""

from collections import deque, Counter

_BLINK_WINDOW_SEC          = 12.0  # rolling window length for blink estimation (seconds)
_BLINK_MIN_DATA_SEC        = 4.0   # return "unknown" until this many seconds of samples are in the window
_BLINK_INTENSITY_MIN_SWING = 0.05  # blue beacon: min peak-to-peak intensity swing to qualify as blinking
_BLINK_HZ_RANGE            = (0.12, 2.0)  # valid blink frequency range
_BLINK_MIN_EDGES           = 3     # rising edges needed for blue beacon (2 complete periods)
_BLINK_MIN_EDGE_GAP        = 0.20  # debounce: ignore edges closer than this (filters threshold chatter)
_BLINK_MAX_IOI_SEC         = 5.0   # blue beacon: max IOI (= max period for 0.2 Hz)
_BLINK_MAX_IOI_SEC_COLOR   = 8.0   # color beacons: slack for long on-periods between blinks
_BLINK_COLOR_CONF_MIN      = 0.001 # min color_confidence to count a non-blue reading as signal
_BLINK_GAP_OFF_SEC         = 5.0   # red/green only: gap longer than this means beacon was off
_BLINK_CC_ON_THRESHOLD     = 0.15  # blue beacon: color_confidence above this = LED on, below = LED off/dim
_BLINK_MAX_IOI_RATIO       = None  # if set, reject blink if max(IOIs)/min(IOIs) exceeds this ratio (None = disabled)

# Number of recent samples used to decide whether the beacon is blue (housing always
# visible) vs red/green (YOLO loses it entirely when off).
_BLUE_DETECT_WINDOW        = 6


class BlinkDetector:
    """
    Estimates blink frequency from a rolling window of (timestamp, color, intensity,
    color_conf) samples.

    Blue beacons: on/off signal comes from color_confidence oscillation and
      actual "unknown" detections.  Gap injection is skipped — the beacon housing
      is always visible so inter-frame gaps are not off-periods.

    Red/green beacons: rising edge = transition from "blue" (off, housing visible)
      to signal color (on).  Gap injection is used because YOLO loses the beacon
      entirely when the LED is off.

    Returns a dict: {is_blinking, blink_color, blink_hz, phase}.
    """

    def __init__(self):
        self._samples: deque = deque()  # (timestamp, color, intensity, color_conf)
        self._finalised = False  # set by finalise() to skip warm-up guards (burst mode)

    def finalise(self) -> None:
        """Signal that all data has been collected (burst mode).
        Subsequent _estimate() calls will skip the minimum-data-span guards
        and treat partial windows as definitive rather than 'still accumulating'."""
        self._finalised = True

    def _is_blue_beacon(self) -> bool:
        """True when recent samples are predominantly blue/unknown (housing always visible)."""
        if not self._samples:
            return False
        recent = list(self._samples)[-_BLUE_DETECT_WINDOW:]
        blue_like = sum(1 for s in recent if s[1] in ("blue", "unknown"))
        return blue_like > len(recent) // 2

    def update(self, ts: float, color: str, intensity: float, color_conf: float = 1.0) -> dict:
        # For red/green beacons, a gap means the beacon was off and YOLO missed it.
        # Inject a synthetic off-marker so rising-edge detection sees the transition.
        # Skip this for blue beacons — the housing is always detectable, so gaps are
        # just inter-frame intervals, not genuine off-periods.
        if not self._is_blue_beacon() and self._samples and \
                (ts - self._samples[-1][0]) > _BLINK_GAP_OFF_SEC:
            off_ts = self._samples[-1][0] + _BLINK_GAP_OFF_SEC / 2
            self._samples.append((off_ts, "_off_", 0.0, 0.0))
        self._samples.append((ts, color, intensity, color_conf))
        cutoff = ts - _BLINK_WINDOW_SEC
        while self._samples and self._samples[0][0] < cutoff:
            self._samples.popleft()
        return self._estimate()

    def _estimate(self) -> dict:
        if len(self._samples) < 4:
            return {"is_blinking": None, "blink_color": "unknown", "blink_hz": None, "phase": "unknown"}

        timestamps  = [s[0] for s in self._samples]
        colors      = [s[1] for s in self._samples]
        intensities = [s[2] for s in self._samples]
        color_confs = [s[3] if len(s) > 3 else 1.0 for s in self._samples]

        data_span = timestamps[-1] - timestamps[0]

        if data_span < _BLINK_MIN_DATA_SEC:
            return {"is_blinking": None, "blink_color": "unknown", "blink_hz": None, "phase": "unknown"}

        # Determine signal type from dominant non-blue color in window.
        # Require minimum color_confidence to avoid red/green noise forcing the wrong mode.
        color_counts = Counter(
            c for c, cc in zip(colors, color_confs)
            if c not in ("blue", "unknown", "_off_") and cc >= _BLINK_COLOR_CONF_MIN
        )
        if color_counts:
            blink_color = color_counts.most_common(1)[0][0]
            on_flags = [c == blink_color for c in colors]
        else:
            blink_color = "blue"
            # On = LED lit (color_conf above threshold); Off = dim/unknown/injected.
            # color_confidence cleanly separates the LED-on state (~0.4+) from the
            # housing-only state (~0.02-0.05) without relying on gap injection.
            on_flags = [
                c not in ("_off_", "unknown") and cc >= _BLINK_CC_ON_THRESHOLD
                for c, cc in zip(colors, color_confs)
            ]

            # Fallback: if every sample passes the confidence threshold, no off-states
            # exist via color_conf. Try intensity oscillation as a secondary signal.
            if all(on_flags):
                live_intensities = [i for c, i in zip(colors, intensities)
                                    if c not in ("_off_", "unknown")]
                if not live_intensities:
                    return {"is_blinking": False, "blink_color": "blue",
                            "blink_hz": None, "phase": "unknown"}
                swing = max(live_intensities) - min(live_intensities)
                if swing < _BLINK_INTENSITY_MIN_SWING:
                    return {"is_blinking": False, "blink_color": "blue",
                            "blink_hz": None, "phase": "on"}
                mean_intensity = sum(live_intensities) / len(live_intensities)
                on_flags = [i >= mean_intensity if c not in ("_off_", "unknown") else False
                            for c, i in zip(colors, intensities)]

        phase = "on" if on_flags[-1] else "off"

        # Rising edges (off→on), debounced to suppress threshold chatter.
        raw_edges = [
            timestamps[i]
            for i in range(1, len(on_flags))
            if not on_flags[i - 1] and on_flags[i]
        ]
        rising_edges: list = []
        last_edge = -1.0
        for t in raw_edges:
            if t - last_edge >= _BLINK_MIN_EDGE_GAP:
                rising_edges.append(t)
                last_edge = t

        # Blue beacons require 3 edges (2 complete periods) for confidence.
        # Red/green only need 2 edges (1 complete period) since color transitions
        # are unambiguous.
        min_edges = _BLINK_MIN_EDGES if blink_color == "blue" else 2
        if len(rising_edges) < min_edges:
            still_accumulating = not self._finalised and data_span < (_BLINK_MIN_DATA_SEC + 1.0)
            return {
                "is_blinking": None if still_accumulating else False,
                "blink_color": blink_color if not still_accumulating else "unknown",
                "blink_hz": None,
                "phase": phase if not still_accumulating else "unknown",
            }

        # Duty-cycle guard for the 2-edge case on non-blue beacons.
        if blink_color != "blue" and len(rising_edges) == 2:
            on_fraction = sum(1 for f in on_flags if f) / len(on_flags)
            if on_fraction > 0.80:
                return {"is_blinking": False, "blink_color": blink_color, "blink_hz": None, "phase": phase}

        iois = [rising_edges[i + 1] - rising_edges[i] for i in range(len(rising_edges) - 1)]
        mean_ioi = sum(iois) / len(iois)
        if blink_color == "blue" and _BLINK_MAX_IOI_RATIO is not None and len(iois) >= 2:
            if max(iois) / min(iois) > _BLINK_MAX_IOI_RATIO:
                return {"is_blinking": False, "blink_color": blink_color, "blink_hz": None, "phase": phase}
        if mean_ioi <= 0:
            return {"is_blinking": False, "blink_color": blink_color, "blink_hz": None, "phase": phase}

        max_ioi_limit = _BLINK_MAX_IOI_SEC if blink_color == "blue" else _BLINK_MAX_IOI_SEC_COLOR
        if max(iois) > max_ioi_limit:
            return {"is_blinking": False, "blink_color": blink_color, "blink_hz": None, "phase": phase}

        hz = 1.0 / mean_ioi
        lo, hi = _BLINK_HZ_RANGE
        is_blinking = lo <= hz <= hi
        return {
            "is_blinking": is_blinking,
            "blink_color": blink_color,
            "blink_hz":    round(hz, 2) if is_blinking else None,
            "phase":       phase,
        }


_blink_detectors: dict = {}


def _get_blink_detector(tracking_id: int) -> BlinkDetector:
    if tracking_id not in _blink_detectors:
        _blink_detectors[tracking_id] = BlinkDetector()
    return _blink_detectors[tracking_id]
