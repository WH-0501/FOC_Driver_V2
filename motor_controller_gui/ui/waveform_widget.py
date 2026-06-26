"""实时波形显示。"""

from typing import Dict, List, Optional, Sequence, Tuple

import pyqtgraph as pg
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QWheelEvent
from PyQt5.QtWidgets import (
    QCheckBox,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
)

from core.waveform_buffer import NOMINAL_REPORT_PERIOD, WAVEFORM_WINDOW_SECONDS, WaveformBuffer
from core.waveform_curves import WAVEFORM_CURVES
from ui.curve_selector_widget import CurveSelectorWidget

DEFAULT_X_WINDOW = WAVEFORM_WINDOW_SECONDS
MIN_X_WINDOW = 0.001
MAX_X_WINDOW = 120.0
ZOOM_FACTOR = 1.15
ABS_MAX_PLOT_POINTS = 1500
PLOT_REFRESH_MS = 50
ZOOM_DEBOUNCE_MS = 40

DISPLAY_STYLE = """
QFrame#waveformDisplay {
    border: 1px solid #3a3a55;
    border-radius: 4px;
    background-color: #1a1a2e;
}
"""


class WheelZoomPlotWidget(pg.PlotWidget):
    def __init__(self, on_zoom=None, parent=None):
        super().__init__(parent=parent)
        self._on_zoom = on_zoom

    def wheelEvent(self, event: QWheelEvent) -> None:
        if self._on_zoom is not None:
            delta = event.angleDelta().y()
            if delta == 0:
                return
            vertical = bool(event.modifiers() & Qt.ControlModifier)
            self._on_zoom(delta > 0, vertical)
            event.accept()
            return
        super().wheelEvent(event)


class WaveformWidget(QGroupBox):
    def __init__(self, parent=None):
        super().__init__("波形展示", parent)
        self._buffers: Dict[int, WaveformBuffer] = {}
        self._motor_id: Optional[int] = None
        self._paused = False
        self._x_window = DEFAULT_X_WINDOW
        self._v_zoom = 1.0
        self._build_ui()
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh_plot)
        self._timer.start(PLOT_REFRESH_MS)
        self._zoom_timer = QTimer(self)
        self._zoom_timer.setSingleShot(True)
        self._zoom_timer.timeout.connect(self._refresh_plot)

    def _build_ui(self) -> None:
        layout = QVBoxLayout()

        self._title = QLabel("未选择电机")
        self._title.setStyleSheet("font-weight: bold;")
        layout.addWidget(self._title)

        display = QFrame()
        display.setObjectName("waveformDisplay")
        display.setStyleSheet(DISPLAY_STYLE)
        display_layout = QHBoxLayout(display)
        display_layout.setContentsMargins(0, 0, 0, 0)
        display_layout.setSpacing(0)

        pg.setConfigOptions(antialias=False)
        self._plot = WheelZoomPlotWidget(on_zoom=self._on_wheel_zoom)
        self._plot.setBackground("#1a1a2e")
        self._plot.showGrid(x=True, y=True, alpha=0.25)
        self._plot.setLabel("bottom", "时间", units="s")
        self._plot.setLabel("left", "数值")
        self._plot.setMouseEnabled(x=False, y=False)
        self._plot.enableAutoRange(enable=False)
        self._plot.getViewBox().setAutoVisible(x=False, y=False)
        self._plot.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self._curves = {}
        for key, name, color, line_style, _default, _group in WAVEFORM_CURVES:
            pen = pg.mkPen(color=color, width=1.5, style=line_style)
            curve = self._plot.plot([], [], pen=pen, name=name, clipToView=True)
            self._curves[key] = curve

        self._curve_selector = CurveSelectorWidget()
        self._curve_selector.selection_changed.connect(self._schedule_refresh)

        display_layout.addWidget(self._plot, 1)
        display_layout.addWidget(self._curve_selector)
        layout.addWidget(display, 1)

        ctrl = QHBoxLayout()
        self.pause_btn = QPushButton("暂停")
        self.resume_btn = QPushButton("恢复")
        self.clear_btn = QPushButton("清空")
        self.reset_zoom_btn = QPushButton("重置缩放")
        self.collect_cb = QCheckBox("数据采集")
        self.collect_cb.setChecked(True)
        ctrl.addWidget(self.pause_btn)
        ctrl.addWidget(self.resume_btn)
        ctrl.addWidget(self.clear_btn)
        ctrl.addWidget(self.reset_zoom_btn)
        ctrl.addWidget(self.collect_cb)
        ctrl.addStretch()
        layout.addLayout(ctrl)

        hint = QLabel("滚轮：水平缩放  |  Ctrl+滚轮：垂直缩放")
        hint.setStyleSheet("color: #666;")
        layout.addWidget(hint)

        self.setLayout(layout)

        self.pause_btn.clicked.connect(self._on_pause)
        self.resume_btn.clicked.connect(self._on_resume)
        self.clear_btn.clicked.connect(self.clear_current)
        self.reset_zoom_btn.clicked.connect(self._reset_zoom)

    def _schedule_refresh(self) -> None:
        self._zoom_timer.start(ZOOM_DEBOUNCE_MS)

    def _reset_zoom(self) -> None:
        self._x_window = DEFAULT_X_WINDOW
        self._v_zoom = 1.0
        self._refresh_plot()

    def _on_wheel_zoom(self, zoom_in: bool, vertical: bool) -> None:
        factor = 1.0 / ZOOM_FACTOR if zoom_in else ZOOM_FACTOR
        if vertical:
            self._v_zoom = max(0.1, min(20.0, self._v_zoom * factor))
        else:
            self._x_window = max(MIN_X_WINDOW, min(MAX_X_WINDOW, self._x_window * factor))
        self._update_x_range_only()
        self._zoom_timer.start(ZOOM_DEBOUNCE_MS)

    def _update_x_range_only(self) -> None:
        """滚轮缩放时先更新视窗，完整重绘由防抖定时器触发。"""
        if self._motor_id is None or self._motor_id not in self._buffers:
            return
        times = self._buffers[self._motor_id].times
        if not times:
            return
        x_min, x_max = self._x_limits(times[-1])
        self._plot.setXRange(x_min, x_max, padding=0)

    def _x_limits(self, t_max: float) -> Tuple[float, float]:
        if t_max <= self._x_window:
            return 0.0, self._x_window
        return t_max - self._x_window, t_max

    def _max_plot_points(self) -> int:
        pixel_w = max(320, self._plot.width())
        by_pixel = int(pixel_w * 1.2)
        by_window = int(self._x_window / NOMINAL_REPORT_PERIOD) + 10
        return min(ABS_MAX_PLOT_POINTS, max(80, min(by_pixel, by_window)))

    @staticmethod
    def _downsample_peak(xs: Sequence[float], ys: Sequence[float], max_points: int) -> Tuple[List[float], List[float]]:
        n = len(xs)
        if n <= max_points:
            return list(xs), list(ys)

        bucket = max(1, n // max_points)
        out_x: List[float] = []
        out_y: List[float] = []
        for i in range(0, n, bucket):
            bx = xs[i : i + bucket]
            by = ys[i : i + bucket]
            if not by:
                continue
            lo = min(range(len(by)), key=by.__getitem__)
            hi = max(range(len(by)), key=by.__getitem__)
            if lo == hi:
                out_x.append(bx[lo])
                out_y.append(by[lo])
            elif lo < hi:
                out_x.extend([bx[lo], bx[hi]])
                out_y.extend([by[lo], by[hi]])
            else:
                out_x.extend([bx[hi], bx[lo]])
                out_y.extend([by[hi], by[lo]])
        return out_x, out_y

    def set_motor(self, motor_id: Optional[int]) -> None:
        self._motor_id = motor_id
        if motor_id is None:
            self._title.setText("未选择电机")
        else:
            self._title.setText(f"电机 ID: {motor_id}")
        self._reset_zoom()

    def _get_buffer(self, motor_id: int) -> WaveformBuffer:
        if motor_id not in self._buffers:
            self._buffers[motor_id] = WaveformBuffer()
        return self._buffers[motor_id]

    def set_target(self, motor_id: int, pos=None, vel=None, curr=None, iq=None) -> None:
        iq_val = iq if iq is not None else curr
        self._get_buffer(motor_id).set_target(pos=pos, vel=vel, iq=iq_val)

    def append_sample(self, motor_id: int, sample: Dict, timestamp: Optional[float] = None) -> None:
        if not self.collect_cb.isChecked() or self._paused:
            return
        self._get_buffer(motor_id).append_sample(sample, timestamp=timestamp)

    def clear_current(self) -> None:
        if self._motor_id is not None and self._motor_id in self._buffers:
            self._buffers[self._motor_id].clear()
        self._reset_zoom()

    def _on_pause(self) -> None:
        self._paused = True

    def _on_resume(self) -> None:
        self._paused = False

    def _refresh_plot(self) -> None:
        visible = self._curve_selector.visible_keys()
        if self._motor_id is None or self._motor_id not in self._buffers:
            for key, curve in self._curves.items():
                curve.setData([], [])
                curve.setVisible(key in visible)
            return

        buf = self._buffers[self._motor_id]
        if not buf.times:
            return

        t_max = buf.times[-1]
        x_min, x_max = self._x_limits(t_max)
        window = buf.snapshot_window(x_min, x_max)
        times = window["times"]
        if not times:
            return

        max_pts = self._max_plot_points()
        visible_vals: List[float] = []

        for key, curve in self._curves.items():
            show = key in visible
            curve.setVisible(show)
            if show and key in window:
                xs, ys = self._downsample_peak(times, window[key], max_pts)
                curve.setData(xs, ys, connect="finite")
                visible_vals.extend(ys)
            else:
                curve.setData([], [])

        self._plot.setXRange(x_min, x_max, padding=0)

        if visible_vals:
            v_min, v_max = min(visible_vals), max(visible_vals)
            if v_min == v_max:
                v_min -= 1.0
                v_max += 1.0
            margin = max(1.0, (v_max - v_min) * 0.08) / self._v_zoom
            mid = (v_max + v_min) / 2
            half = (v_max - v_min) / 2 / self._v_zoom + margin
            self._plot.setYRange(mid - half, mid + half, padding=0)
