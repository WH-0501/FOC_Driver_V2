"""Bounded NumPy ring. GUI consumer owns writes; snapshots are independent arrays."""
import numpy as np
from core.waveform_curves import CURVE_KEYS

WAVEFORM_WINDOW_SECONDS = 6.0
WAVEFORM_MAX_POINTS = 120_000
NOMINAL_REPORT_PERIOD = 0.001

class WaveformBuffer:
    def __init__(self, max_points=WAVEFORM_MAX_POINTS, window_seconds=120.0):
        if max_points < 2 or window_seconds <= 0:
            raise ValueError("Invalid buffer capacity")
        self.max_points = int(max_points)
        self.window_seconds = float(window_seconds)
        self._data = np.empty((len(CURVE_KEYS) + 1, self.max_points), dtype=np.float64)
        self._head = self._size = 0
        self._last_raw_ts = None
        self._time = 0.0
        self._last_target = dict(pos=0.0, vel=0.0, iq=0.0)
        self.generation = self.total_samples = 0

    def __len__(self):
        return self._size

    @property
    def latest_time(self):
        return self._time if self._size else None

    @property
    def times(self):
        return self.snapshot(keys=())["times"]

    def set_target(self, pos=None, vel=None, iq=None):
        for key, value in (("pos", pos), ("vel", vel), ("iq", iq)):
            if value is not None:
                self._last_target[key] = float(value)

    def append_sample(self, sample, timestamp=None):
        if self._size:
            dt = NOMINAL_REPORT_PERIOD
            if timestamp is not None and self._last_raw_ts is not None:
                delta = timestamp - self._last_raw_ts
                if np.isfinite(delta) and delta > 0:
                    dt = delta
            self._time += dt
        if timestamp is not None and np.isfinite(timestamp):
            self._last_raw_ts = timestamp
        target = self._last_target
        self._data[:, self._head] = (
            self._time, target['pos'], sample.get('position_plot', sample.get('position_rad', np.nan)),
            target['vel'], sample.get('velocity', np.nan), target['iq'],
            sample.get('iq_ma', sample.get('current_ma', np.nan)),
            *(sample.get(k + '_ma', np.nan) if sample.get('has_phase_current', True) else np.nan
              for k in ('id', 'ia', 'ib', 'ic')),
        )
        self._head = (self._head + 1) % self.max_points
        self._size = min(self._size + 1, self.max_points)
        self.total_samples += 1
        self.generation += 1

    def clear(self):
        self._head = self._size = self.total_samples = 0
        self._last_raw_ts = None
        self._time = 0.0
        self.generation += 1

    def resize(self, max_points):
        if max_points < 2:
            raise ValueError('Invalid buffer capacity')
        snapshot = self.snapshot()
        keep = min(self._size, int(max_points))
        self.max_points = int(max_points)
        self._data = np.empty((len(CURVE_KEYS)+1, self.max_points), dtype=np.float64)
        if keep:
            for row, key in enumerate(['times'] + CURVE_KEYS):
                self._data[row, :keep] = snapshot[key][-keep:]
        self._size = keep
        self._head = keep % self.max_points
        self.generation += 1

    def _segments(self):
        start = (self._head - self._size) % self.max_points
        if not self._size:
            return []
        if start < self._head:
            return [(start, self._head)]
        return [(start, self.max_points), (0, self._head)]

    def snapshot(self, keys=None):
        return self.snapshot_window(-np.inf, np.inf, keys)

    def snapshot_window(self, x_min, x_max, keys=None):
        keys = CURVE_KEYS if keys is None else list(keys)
        result, slices = {}, []
        for start, end in self._segments():
            times = self._data[0, start:end]
            lo = int(np.searchsorted(times, x_min, side='left'))
            hi = int(np.searchsorted(times, x_max, side='right'))
            slices.append(slice(start + lo, start + hi))
        for name, row in [('times', 0)] + [(k, CURVE_KEYS.index(k) + 1) for k in keys]:
            result[name] = (np.concatenate([self._data[row, s] for s in slices])
                            if slices else np.empty(0, dtype=np.float64))
        return result
