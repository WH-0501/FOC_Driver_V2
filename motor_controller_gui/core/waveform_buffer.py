"""波形数据环形缓冲。"""

import bisect
import time
from collections import deque
from typing import Deque, Dict, Optional, Tuple

from core.waveform_curves import CURVE_KEYS

# 1 kHz 采样下保留 6 s 历史
WAVEFORM_WINDOW_SECONDS = 6.0
WAVEFORM_MAX_POINTS = 6000
# 与固件 PERIOD_REPORT 1 ms 周期对齐
NOMINAL_REPORT_PERIOD = 0.001


class WaveformBuffer:
    """保存单个电机的时间序列数据。"""

    def __init__(
        self,
        max_points: int = WAVEFORM_MAX_POINTS,
        window_seconds: float = WAVEFORM_WINDOW_SECONDS,
    ):
        self.max_points = max_points
        self.window_seconds = window_seconds
        self._last_raw_ts: Optional[float] = None
        self.times: Deque[float] = deque(maxlen=max_points)
        self.channels: Dict[str, Deque[float]] = {key: deque(maxlen=max_points) for key in CURVE_KEYS}
        self._last_target = {"pos": 0.0, "vel": 0.0, "iq": 0.0}

    def set_target(self, pos: Optional[float] = None, vel: Optional[float] = None, iq: Optional[float] = None) -> None:
        if pos is not None:
            self._last_target["pos"] = pos
        if vel is not None:
            self._last_target["vel"] = vel
        if iq is not None:
            self._last_target["iq"] = iq

    def _next_time(self, timestamp: Optional[float]) -> float:
        """生成单调递增的显示时间轴。

        CAN 硬件时间戳在批读时可能重复或精度过低（甚至单位有误），
        此时回退到 1 ms 名义周期，保证 1 kHz 波形连续可辨。
        """
        if not self.times:
            self._last_raw_ts = timestamp
            return 0.0

        last_t = self.times[-1]
        if timestamp is None:
            return last_t + NOMINAL_REPORT_PERIOD

        if self._last_raw_ts is None:
            self._last_raw_ts = timestamp
            return last_t + NOMINAL_REPORT_PERIOD

        delta = timestamp - self._last_raw_ts
        if delta <= 0:
            return last_t + NOMINAL_REPORT_PERIOD

        # delta 过小：时间戳单位/精度异常，按 1 kHz 展开
        if delta < 0.0005:
            return last_t + NOMINAL_REPORT_PERIOD

        # 单次跳变过大（积压批次），避免横轴瞬间拉伸
        if delta > 0.05:
            self._last_raw_ts = timestamp
            return last_t + NOMINAL_REPORT_PERIOD

        self._last_raw_ts = timestamp
        return last_t + delta

    def append_sample(self, sample: Dict[str, float], timestamp: Optional[float] = None) -> None:
        ts = timestamp if timestamp is not None else time.perf_counter()
        t = self._next_time(ts)

        self.times.append(t)
        self.channels["target_pos"].append(self._last_target["pos"])
        self.channels["actual_pos"].append(sample.get("position_deg", 0.0))
        self.channels["target_vel"].append(self._last_target["vel"])
        self.channels["actual_vel"].append(sample.get("velocity", 0.0))
        self.channels["target_iq"].append(self._last_target["iq"])
        self.channels["iq"].append(sample.get("iq_ma", sample.get("current_ma", 0.0)))
        self.channels["id"].append(sample.get("id_ma", 0.0))
        self.channels["ia"].append(sample.get("ia_ma", 0.0))
        self.channels["ib"].append(sample.get("ib_ma", 0.0))
        self.channels["ic"].append(sample.get("ic_ma", 0.0))

        self._trim_window()

    def _trim_window(self) -> None:
        if not self.times:
            return
        latest = self.times[-1]
        cutoff = latest - self.window_seconds
        while self.times and self.times[0] < cutoff:
            self.times.popleft()
            for ch in self.channels.values():
                ch.popleft()

    def clear(self) -> None:
        self._last_raw_ts = None
        self.times.clear()
        for ch in self.channels.values():
            ch.clear()

    def snapshot(self) -> Dict[str, Tuple]:
        result = {"times": tuple(self.times)}
        for key, ch in self.channels.items():
            result[key] = tuple(ch)
        return result

    def snapshot_window(self, x_min: float, x_max: float) -> Dict[str, Tuple]:
        """按时间窗口截取数据，避免绘图时全量扫描缓冲。"""
        n = len(self.times)
        if n == 0:
            empty = {"times": ()}
            empty.update({key: () for key in CURVE_KEYS})
            return empty

        times = self.times
        i0 = bisect.bisect_left(times, x_min)
        i1 = bisect.bisect_right(times, x_max)
        if i1 <= i0:
            i0 = max(0, i1 - 1)
        i1 = min(i1, n)

        result: Dict[str, Tuple] = {"times": tuple(times[i] for i in range(i0, i1))}
        for key, ch in self.channels.items():
            result[key] = tuple(ch[i] for i in range(i0, i1))
        return result
