"""日志时间戳工具。"""

from __future__ import annotations

from datetime import datetime


def format_log_timestamp() -> str:
    """HH:MM:SS.microseconds（6 位微秒）。"""
    now = datetime.now()
    return now.strftime("%H:%M:%S.") + f"{now.microsecond:06d}"


def format_mono_delta_ms(t0: float, t1: float | None = None) -> str:
    """单调时钟间隔，毫秒，保留 3 位小数。"""
    import time

    if t1 is None:
        t1 = time.monotonic()
    return f"{(t1 - t0) * 1000:.3f}"
