#!/usr/bin/env python3
"""
blink_detector.py — Rolling-window blink frequency estimator for beacon lights.

Imported by beacon_detector.py.  No ROS or OpenCV dependency.
"""

from collections import deque, Counter

_BLINK_WINDOW_SEC          = 4.0   # rolling window length for blink estimation (seconds)
_BLINK_MIN_DATA_SEC        = 2.0   # return "unknown" until this many seconds of samples are in the window
_BLINK_INTENSITY_MIN_SWING = 0.05  # blue beacon: min peak-to-peak intensity swing to qualify as blinking
_BLINK_HZ_RANGE            = (0.5, 2.0)  # valid blink frequency range — centered on 1 Hz target
_BLINK_MIN_EDGES           = 3     # rising edges needed (= 2 complete periods in the window)
_BLINK_MIN_EDGE_GAP        = 0.20  # debounce: ignore edges closer than this (filters threshold chatter)
_BLINK_MAX_OFF_SEC         = 1.0   # blue beacon: max consecutive off-state duration; longer = slow drift
_BLINK_MAX_IOI_SEC         = 2.0   # blue beacon: max IOI (= max period for 0.5 Hz, the lowest valid freq)
_BLINK_MAX_IOI_SEC_COLOR   = 2.5   # color beacons: extra 0.5 s slack absorbs YOLO detection gaps that can
                                   # make one IOI appear as ~2 periods (e.g. gap swallows one green cycle)
_BLINK_MIN_ON_OFF_GAP      = 0.40  # blue beacon: on/off mean separation must be ≥ 40% of total swing


class BlinkDetector:
    """
    Estimates blink frequency from a rolling window of (timestamp, color, intensity) samples.

    Red/green beacons: rising edge = transition from "blue" (off) to signal color (on).
    Blue beacon:       rising edge = intensity crossing _BLUE_ON_INTENSITY upward.
    Returns a dict: {is_blinking, blink_color, blink_hz, phase}.
    """

    def __init__(self):
        self._samples: deque = deque()  # (timestamp, color, intensity)

    def update(self, ts: float, color: str, intensity: float) -> dict:
        self._samples.append((ts, color, intensity))
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

        data_span = timestamps[-1] - timestamps[0]

        # Not enough history yet — is_blinking=None signals "still deciding",
        # distinct from False which means "confirmed not blinking".
        if data_span < _BLINK_MIN_DATA_SEC:
            return {"is_blinking": None, "blink_color": "unknown", "blink_hz": None, "phase": "unknown"}

        # Determine signal type from dominant non-blue color in window
        color_counts = Counter(c for c in colors if c not in ("blue", "unknown"))
        if color_counts:
            blink_color = color_counts.most_common(1)[0][0]
            on_flags = [c == blink_color for c in colors]
        else:
            # All blue — detect oscillations relative to the window mean.
            # Requires a minimum peak-to-peak swing so flat/steady signals don't
            # generate spurious rising edges.
            blink_color = "blue"
            swing = max(intensities) - min(intensities)
            if swing < _BLINK_INTENSITY_MIN_SWING:
                phase = "on" if intensities[-1] >= (sum(intensities) / len(intensities)) else "off"
                return {"is_blinking": False, "blink_color": "blue", "blink_hz": None, "phase": phase}
            mean_intensity = sum(intensities) / len(intensities)
            on_flags = [i >= mean_intensity for i in intensities]

            # Reject slow intensity drifts that stay below the mean for longer than
            # the maximum half-period of a valid blink (1 s at the 0.5 Hz floor).
            # Camera-motion noise produces sustained "off" runs of 1-2 s; true 1 Hz
            # blinks produce off runs of ~0.5 s.
            _off_dur, _off_t0 = 0.0, None
            for _t, _f in zip(timestamps, on_flags):
                if not _f:
                    if _off_t0 is None:
                        _off_t0 = _t
                    _off_dur = max(_off_dur, _t - _off_t0)
                else:
                    _off_t0 = None
            if _off_dur > _BLINK_MAX_OFF_SEC:
                return {"is_blinking": False, "blink_color": "blue", "blink_hz": None,
                        "phase": "on" if on_flags[-1] else "off"}

            # The mean "on" intensity and mean "off" intensity must be well-separated.
            # If the split at the dynamic mean divides near-constant noise, the on/off
            # means are almost identical — not a real blink signal.
            _n_on  = sum(1 for f in on_flags if f)
            _n_off = sum(1 for f in on_flags if not f)
            if _n_on and _n_off:
                _on_mean  = sum(intensities[j] for j, f in enumerate(on_flags) if     f) / _n_on
                _off_mean = sum(intensities[j] for j, f in enumerate(on_flags) if not f) / _n_off
                if (_on_mean - _off_mean) < _BLINK_MIN_ON_OFF_GAP * swing:
                    return {"is_blinking": False, "blink_color": "blue", "blink_hz": None,
                            "phase": "on" if on_flags[-1] else "off"}

        phase = "on" if on_flags[-1] else "off"

        # Rising edges (off→on), debounced to suppress threshold chatter.
        # Frames near the mean crossing can flip rapidly, producing fake edges
        # with sub-frame IOIs that would corrupt the frequency estimate.
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

        # Blue beacons use intensity oscillations which are prone to noise — require
        # 3 edges (2 complete periods) for confidence.  Color-transition beacons
        # (red/green) produce rising edges only on genuine color changes, so 2 edges
        # (1 complete period, 1 IOI) is sufficient and avoids false negatives caused
        # by YOLO occasionally missing frames at a transition point.
        min_edges = _BLINK_MIN_EDGES if blink_color == "blue" else 2
        if len(rising_edges) < min_edges:
            still_accumulating = data_span < (_BLINK_MIN_DATA_SEC + 1.0)
            return {
                "is_blinking": None if still_accumulating else False,
                "blink_color": blink_color if not still_accumulating else "unknown",
                "blink_hz": None,
                "phase": phase if not still_accumulating else "unknown",
            }

        # Duty-cycle guard for the 2-edge case on non-blue beacons.
        # A solid beacon with occasional color-classification noise can produce exactly
        # 2 rising edges while its on-fraction stays high (≥ 65%) because it's nearly
        # always the signal color.  A genuinely blinking beacon has an on-fraction near
        # 50%.  Skip this check when ≥ 3 edges are present — the IOI consistency test
        # below is a stronger filter at that point.
        if blink_color != "blue" and len(rising_edges) == 2:
            on_fraction = sum(1 for f in on_flags if f) / len(on_flags)
            if on_fraction > 0.65:
                return {"is_blinking": False, "blink_color": blink_color, "blink_hz": None, "phase": phase}

        iois = [rising_edges[i + 1] - rising_edges[i] for i in range(len(rising_edges) - 1)]
        mean_ioi = sum(iois) / len(iois)
        if mean_ioi <= 0:
            return {"is_blinking": False, "blink_color": blink_color, "blink_hz": None, "phase": phase}
        # A single IOI longer than the maximum valid period means two edges span a
        # skipped cycle — the mean would pass the Hz check but the pattern is not a
        # stable blink.  Color beacons get extra slack because a YOLO detection gap
        # can swallow one full green/red cycle, making one IOI appear as ~2 periods.
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
