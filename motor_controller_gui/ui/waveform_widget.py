"""Motor oscilloscope: bounded history, independent display freeze and raw analysis."""
import csv
import time
import threading
from collections import OrderedDict
import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog,
    QGroupBox, QHBoxLayout, QLabel, QMessageBox, QPushButton, QSplitter,
    QVBoxLayout, QWidget)
from core.waveform_buffer import WaveformBuffer, WAVEFORM_WINDOW_SECONDS
from core.waveform_analysis import peak_envelope, spectrum
from core.waveform_curves import WAVEFORM_CURVES, waveform_curves_for
from protocol import ProtocolVersion, pos_uses_rad
from ui.curve_selector_widget import CurveSelectorWidget

PLOT_REFRESH_MS = 33
MAX_MOTOR_BUFFERS = 32
MAX_RING_BYTES = 384 * 1024 * 1024


class TimeScaleAxis(pg.AxisItem):
    """Display-only scaling; curve coordinates and cursor values remain seconds."""
    def __init__(self):
        self.time_mode = True
        self.time_factor = 1.0
        self.time_origin = 0.0
        super().__init__(orientation='bottom')
        self.enableAutoSIPrefix(False)

    def setRange(self, minimum, maximum):
        super().setRange(minimum, maximum)
        self.update_time_scale()

    def update_time_scale(self):
        if not self.time_mode:
            return
        lo, hi = self.range
        self.time_factor = 1000.0 if hi-lo < 1.0 else 1.0
        self.time_origin = float(np.floor(lo)) if self.time_factor == 1000 and abs(lo) >= 1 else 0.0
        label = f'时间（相对 {self.time_origin:.0f} s）' if self.time_origin else '时间'
        self.setLabel(label, units='ms' if self.time_factor == 1000 else 's')
        self.picture = None
        self.update()

    def tickStrings(self, values, scale, spacing):
        if not self.time_mode:
            return super().tickStrings(values, scale, spacing)
        step = abs(spacing*self.time_factor)
        decimals = max(0, int(np.ceil(-np.log10(step)))) if step > 0 else 0
        return [f'{(v-self.time_origin)*self.time_factor:.{min(decimals, 9)}f}' for v in values]


class WaveformViewBox(pg.ViewBox):
    """Only a pan gesture requests history; wheel zoom keeps live acquisition visible."""
    history_requested = pyqtSignal()

    def mouseDragEvent(self, event, axis=None):
        if event.isStart() and event.button() in (Qt.LeftButton, Qt.MiddleButton):
            self.history_requested.emit()
        super().mouseDragEvent(event, axis=axis)


class WaveformWidget(QGroupBox):
    _export_finished = pyqtSignal(str, bool)

    def __init__(self, parent=None):
        super().__init__("电机示波器", parent)
        self._buffers = OrderedDict()
        self._motor_id = None
        self._paused = False
        self._frozen = None
        self._dirty = True
        self._last_render = None
        self._protocol_version = ProtocolVersion.V2_0_0
        self._build_ui()
        self._export_finished.connect(self._on_export_finished)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh_plot)
        self._timer.start(PLOT_REFRESH_MS)

    def _build_ui(self):
        layout = QVBoxLayout(self)
        self._title = QLabel("未选择电机")
        layout.addWidget(self._title)
        row = QHBoxLayout()
        self.pause_btn = QPushButton("冻结显示")
        self.resume_btn = QPushButton("实时跟随")
        self.clear_btn = QPushButton("清空")
        self.reset_zoom_btn = QPushButton("重置视图")
        self.export_btn = QPushButton("导出 CSV")
        for button in (self.pause_btn, self.resume_btn, self.clear_btn, self.reset_zoom_btn, self.export_btn):
            row.addWidget(button)
        self.capacity = QComboBox()
        for label, points in (('缓存 1万点', 10000), ('缓存 12万点', 120000), ('缓存 100万点', 1000000)):
            self.capacity.addItem(label, points)
        self.capacity.setCurrentIndex(1)
        row.addWidget(self.capacity)
        row.addStretch()
        layout.addLayout(row)
        row = QHBoxLayout()
        self.mode = QComboBox()
        self.mode.addItems(["时域", "FFT", "XY", "直方图", "条形图"])
        self.channel = QComboBox()
        self.x_channel = QComboBox()
        for key, name, *_ in WAVEFORM_CURVES:
            self.channel.addItem(name, key)
            self.x_channel.addItem(name, key)
        self.channel.setCurrentIndex(5)
        self.x_channel.setCurrentIndex(6)
        self.fft_size = QComboBox()
        self.fft_size.addItems(["256", "1024", "4096", "16384"])
        self.fft_size.setCurrentText("4096")
        self.sample_dt = QDoubleSpinBox()
        self.sample_dt.setRange(0.001, 1000)
        self.sample_dt.setDecimals(3)
        self.sample_dt.setValue(1)
        self.sample_dt.setSuffix(" ms")
        for label, control in (("分析", self.mode), ("Y / 测量", self.channel), ("XY-X", self.x_channel)):
            row.addWidget(QLabel(label))
            row.addWidget(control)
        layout.addLayout(row)
        row = QHBoxLayout()
        self.window = QDoubleSpinBox()
        self.window.setRange(0.001, 3600)
        self.window.setDecimals(3)
        self.window.setValue(WAVEFORM_WINDOW_SECONDS)
        self.window.setSuffix(" s")
        self.collect_cb = QCheckBox("采集")
        self.collect_cb.setChecked(True)
        self.auto_y = QCheckBox("Y 自动")
        self.auto_y.setChecked(True)
        self.cursor_cb = QCheckBox("双游标")
        for label, control in (("窗口", self.window), ("FFT点数", self.fft_size), ("采样间隔", self.sample_dt)):
            row.addWidget(QLabel(label))
            row.addWidget(control)
        row.addWidget(self.collect_cb)
        row.addWidget(self.auto_y)
        row.addWidget(self.cursor_cb)
        layout.addLayout(row)
        pg.setConfigOptions(antialias=False)
        self._view_box = WaveformViewBox()
        self._view_box.history_requested.connect(self._on_pause)
        self._time_axis = TimeScaleAxis()
        self._plot = pg.PlotWidget(background="#101923", viewBox=self._view_box,
                                   axisItems={"bottom": self._time_axis})
        self._plot.showGrid(x=True, y=True, alpha=0.18)
        self._plot.setLabel('bottom', '时间', units='s')
        self._plot.getAxis('bottom').enableAutoSIPrefix(False)
        self._plot.setMouseEnabled(x=True, y=True)
        self._plot.enableAutoRange(enable=False)
        self._curves = {}
        for key, name, color, style, *_ in WAVEFORM_CURVES:
            self._curves[key] = self._plot.plot(pen=pg.mkPen(color, width=1, style=style),
                                               connect='finite', name=name)
        self._analysis_curve = self._plot.plot(pen=pg.mkPen('#50dbbd', width=1), connect='finite')
        self._bars = pg.BarGraphItem(x=[], height=[], width=0.8, brush='#50dbbd')
        self._plot.addItem(self._bars)
        self._cursor_region = pg.LinearRegionItem(
            values=(2, 4), movable=True,
            pen=pg.mkPen('#87cefa', width=1.5),
            hoverPen=pg.mkPen('#b5e5ff', width=2),
            brush=pg.mkBrush(135, 206, 250, 35),
            hoverBrush=pg.mkBrush(135, 206, 250, 65))
        self._cursor_region.setZValue(10)
        self._plot.addItem(self._cursor_region, ignoreBounds=True)
        self._cursor_region.hide()
        self._cursors = self._cursor_region.lines
        self._cursor_region.sigRegionChanged.connect(self._schedule_refresh)
        self._curve_selector = CurveSelectorWidget()
        self._curve_selector.selection_changed.connect(self._schedule_refresh)
        split = QSplitter()
        split.addWidget(self._plot)
        split.addWidget(self._curve_selector)
        split.setStretchFactor(0, 1)
        layout.addWidget(split, 1)
        self.measurement = QLabel("双游标可测量 Δt、Δy、1/Δt；FFT 使用原始缓存与设置的采样间隔")
        self.measurement.setWordWrap(True)
        layout.addWidget(self.measurement)
        self.metrics = QLabel()
        layout.addWidget(self.metrics)
        for combo in (self.mode, self.channel, self.x_channel, self.fft_size):
            combo.currentIndexChanged.connect(self._schedule_refresh)
        self.mode.currentIndexChanged.connect(self._mode_changed)
        for spin in (self.window, self.sample_dt):
            spin.valueChanged.connect(self._schedule_refresh)
        self.auto_y.toggled.connect(self._schedule_refresh)
        self.cursor_cb.toggled.connect(self._cursor_visibility_changed)
        self._plot.getViewBox().sigRangeChangedManually.connect(self._manual_range)
        self.pause_btn.clicked.connect(self._on_pause)
        self.resume_btn.clicked.connect(self._on_resume)
        self.clear_btn.clicked.connect(self.clear_current)
        self.reset_zoom_btn.clicked.connect(self._reset_zoom)
        self.export_btn.clicked.connect(self._export_csv)
        self.capacity.currentIndexChanged.connect(self._resize_buffer)

    def _evict_for(self, extra_bytes, protect):
        while (sum(b._data.nbytes for b in self._buffers.values()) + extra_bytes > MAX_RING_BYTES
               or len(self._buffers) >= MAX_MOTOR_BUFFERS):
            victims = [k for k in self._buffers if k != protect and k != self._motor_id]
            if not victims:
                break
            del self._buffers[victims[0]]

    def _resize_buffer(self, *_):
        buf = self._buffers.get(self._motor_id)
        if buf is not None:
            size = int(self.capacity.currentData())
            self._evict_for(max(0, size * 88 - buf._data.nbytes), self._motor_id)
            buf.resize(size)
        self._dirty = True

    def _cursor_visibility_changed(self, enabled):
        if enabled:
            lo, hi = self._view_box.viewRange()[0]
            if np.isfinite(hi-lo) and hi > lo:
                self._cursor_region.setRegion((lo+(hi-lo)/3, lo+2*(hi-lo)/3))
        self._cursor_region.setVisible(enabled and self.mode.currentText() == '时域')
        self._schedule_refresh()

    def _schedule_refresh(self, *_):
        self._dirty = True

    def _mode_changed(self, *_):
        self._last_render = None
        self._plot.enableAutoRange(x=True, y=True)
        if self.mode.currentText() == '时域' and self._paused and self._frozen is not None:
            times = self._frozen['times']
            if len(times):
                self._plot.setXRange(max(0, times[-1]-self.window.value()),
                                     max(self.window.value(), times[-1]), padding=0)
        self._dirty = True

    def _manual_range(self, axes):
        # ViewBox emits this for wheel zoom as well as pan/scale gestures.
        # While live, retain the new time span and continue following latest data.
        if axes[0] and not self._paused and self.mode.currentText() == '时域':
            left, right = self._view_box.viewRange()[0]
            self.window.setValue(right - left)
        if axes[1]:
            self.auto_y.setChecked(False)
        self._dirty = True

    def _reset_zoom(self):
        self.window.setValue(WAVEFORM_WINDOW_SECONDS)
        self.auto_y.setChecked(True)
        self._on_resume()

    def apply_protocol_version(self, version):
        if version == self._protocol_version:
            return
        self._protocol_version = version
        curves = waveform_curves_for(version)
        self._curve_selector.apply_curves(curves)
        for index, (key, name, *_) in enumerate(curves):
            self.channel.setItemText(index, name)
            self.x_channel.setItemText(index, name)
        self._buffers.clear()
        self._on_resume()

    def set_motor(self, motor_id):
        self._motor_id = motor_id
        self._title.setText(f"电机 Node-ID {motor_id}" if motor_id is not None else "未选择电机")
        buf = self._buffers.get(motor_id)
        if buf is not None:
            self.capacity.blockSignals(True)
            self.capacity.setCurrentIndex(self.capacity.findData(buf.max_points))
            self.capacity.blockSignals(False)
        self._on_resume()

    def _get_buffer(self, motor_id):
        if motor_id not in self._buffers:
            points = int(self.capacity.currentData())
            self._evict_for(points * 88, motor_id)
            self._buffers[motor_id] = WaveformBuffer(points)
        self._buffers.move_to_end(motor_id)
        return self._buffers[motor_id]

    def set_target(self, motor_id, pos=None, vel=None, curr=None, iq=None):
        self._get_buffer(motor_id).set_target(pos, vel, iq if iq is not None else curr)

    def append_sample(self, motor_id, sample, timestamp=None, received_at=None):
        if self.collect_cb.isChecked():
            self._get_buffer(motor_id).append_sample(sample, timestamp, received_at)

    def clear_current(self):
        if self._motor_id in self._buffers:
            self._buffers[self._motor_id].clear()
        self._on_resume()

    def _on_pause(self):
        if not self._paused:
            buf = self._buffers.get(self._motor_id)
            self._frozen = buf.snapshot() if buf is not None else None
        self._paused = True
        self._dirty = True
        self.pause_btn.setEnabled(False)

    def _on_resume(self):
        self._paused = False
        self._frozen = None
        self._last_render = None
        self._dirty = True
        self.pause_btn.setEnabled(True)

    def _source(self):
        if self._paused:
            if self._frozen is None:
                return None
            if self.mode.currentText() == '时域':
                lo, hi = self._plot.getViewBox().viewRange()[0]
                times = self._frozen['times']
                left = max(0, int(np.searchsorted(times, lo)) - 1)
                right = min(len(times), int(np.searchsorted(times, hi, side='right')) + 1)
                return {key: values[left:right] for key, values in self._frozen.items()}
            return self._frozen
        buf = self._buffers.get(self._motor_id)
        if buf is None or not len(buf):
            return None
        latest = buf.latest_time
        return buf.snapshot_window(max(0, latest - self.window.value()), latest)

    def _show_step_analysis(self):
        from ui.step_analysis_dialog import StepAnalysisDialog
        buf = self._buffers.get(self._motor_id)
        data = self._frozen if self._paused else (buf.snapshot() if buf is not None else None)
        if data is None or not len(data['times']):
            QMessageBox.information(self, '阶跃响应分析', '当前电机没有缓存数据')
            return
        cursor_start = self._cursors[0].value() if self.cursor_cb.isChecked() else None
        StepAnalysisDialog(data, self._motor_id, self, cursor_start=cursor_start,
                           position_unit='rad' if pos_uses_rad(self._protocol_version) else '°').exec_()

    def _export_csv(self):
        buf = self._buffers.get(self._motor_id)
        data = self._frozen if self._paused else (buf.snapshot() if buf is not None else None)
        if data is None or not len(data['times']):
            QMessageBox.information(self, '导出', '当前电机没有缓存数据')
            return
        path, _ = QFileDialog.getSaveFileName(self, '导出原始缓存', f'motor_{self._motor_id}.csv', 'CSV (*.csv)')
        if not path:
            return
        headers = ['time_s'] + [self.channel.itemText(i) for i in range(self.channel.count())]
        self.export_btn.setEnabled(False)
        self.export_btn.setText('正在导出…')
        def write():
            try:
                with open(path, 'w', newline='', encoding='utf-8-sig') as f:
                    writer = csv.writer(f)
                    writer.writerow(headers)
                    writer.writerows(zip(*(data[k] for k in data)))
                self._export_finished.emit(path, True)
            except OSError as exc:
                self._export_finished.emit(str(exc), False)
        threading.Thread(target=write, daemon=False).start()

    def _on_export_finished(self, detail, success):
        self.export_btn.setEnabled(True)
        self.export_btn.setText('导出 CSV')
        if success:
            QMessageBox.information(self, '导出完成', detail)
        else:
            QMessageBox.warning(self, '导出失败', detail)

    def _refresh_plot(self):
        buf = self._buffers.get(self._motor_id)
        token = (self._motor_id, buf.generation if buf is not None else -1, self._paused)
        if not self._dirty and (self._paused or token == self._last_render):
            return
        start = time.perf_counter()
        self._dirty = False
        self._last_render = token
        mode = self.mode.currentText()
        self._time_axis.time_mode = mode == '时域'
        self._time_axis.update_time_scale()
        visible = self._curve_selector.visible_keys()
        for key, curve in self._curves.items():
            curve.setVisible(mode == '时域' and key in visible)
        self._analysis_curve.setVisible(mode in ('FFT', 'XY', '直方图'))
        self._bars.setVisible(mode == '条形图')
        self._cursor_region.setVisible(self.cursor_cb.isChecked() and mode == '时域')
        data = self._source()
        if data is None or not len(data['times']):
            for curve in list(self._curves.values()) + [self._analysis_curve]:
                curve.setData([], [])
            self._bars.setOpts(x=[], height=[])
            self.metrics.setText('等待采样数据')
            return
        t = data['times']
        budget = max(100, min(4000, self._plot.width() * 2))
        y = data[self.channel.currentData()]
        info = ''
        if mode == '时域':
            self._time_axis.update_time_scale()
            self._plot.setLabel('left', '原始数值（各通道单位见选择器）')
            self._plot.enableAutoRange(x=False, y=self.auto_y.isChecked())
            for key in visible:
                xplot, yplot = peak_envelope(t, data[key], budget)
                self._curves[key].setData(xplot, yplot)
            if not self._paused:
                self._plot.setXRange(max(0, t[-1] - self.window.value()), max(self.window.value(), t[-1]), padding=0)
            finite = y[np.isfinite(y)]
            if finite.size:
                info = f"{self.channel.currentText()}  min {finite.min():.5g}  max {finite.max():.5g}  mean {finite.mean():.5g}  RMS {np.sqrt(np.mean(finite**2)):.5g}"
            if self.cursor_cb.isChecked():
                a, b = [c.value() for c in self._cursors]
                dt = b - a
                ia, ib = np.searchsorted(t, [a, b])
                dy_text = '--'
                if t[0] <= a <= t[-1] and t[0] <= b <= t[-1]:
                    dy_text = f'{y[min(ib, len(t)-1)] - y[min(ia, len(t)-1)]:.5g}'
                freq = f'{1/abs(dt):.5g}' if abs(dt) > 1e-12 else '--'
                dt_text = f'{dt*1000:.6g} ms' if abs(dt) < 1 else f'{dt:.6g} s'
                info += f"  |  Δt {dt_text}  1/|Δt| {freq} Hz  Δy {dy_text}"
        elif mode == 'FFT':
            n = int(self.fft_size.currentText())
            x, amplitude = spectrum(y, n, self.sample_dt.value()/1000)
            self._analysis_curve.setData(x, amplitude)
            self._plot.setLabel('bottom', '频率', units='Hz')
            self._plot.setLabel('left', '单边幅值 / Hann / 去直流')
            info = f'需要 {n} 个连续有效样本，当前 {np.isfinite(y).sum()}；频率按设置的设备采样间隔计算'
        elif mode == 'XY':
            # Uniform index limit preserves time ordering; no min/max on nonmonotonic X.
            step = max(1, int(np.ceil(len(t)/budget)))
            self._analysis_curve.setData(data[self.x_channel.currentData()][::step], y[::step])
            self._plot.setLabel('bottom', self.x_channel.currentText())
            self._plot.setLabel('left', self.channel.currentText())
        elif mode == '直方图':
            finite = y[np.isfinite(y)]
            counts, edges = np.histogram(finite, bins=64)
            self._analysis_curve.setData((edges[:-1]+edges[1:])/2, counts)
            self._plot.setLabel('bottom', self.channel.currentText())
            self._plot.setLabel('left', '样本数 / 64 bins')
        else:
            keys = [key for key, *_ in WAVEFORM_CURVES if key in visible]
            self._bars.setOpts(x=np.arange(len(keys)), height=[data[k][-1] for k in keys])
            self._plot.setLabel('bottom', ' / '.join(keys))
            self._plot.setLabel('left', '最新值（混合单位）')
        self.measurement.setText(info or self.channel.currentText())
        clock_info = ''
        if buf is not None:
            intervals = np.asarray(buf.frame_intervals)
            if intervals.size:
                interval = float(np.median(intervals))
                clock_info = (f' | 接收帧时间戳 Δt中位数 {interval*1000:.4g} ms'
                              f' / 均值频率 {1/float(np.mean(intervals)):.5g} Hz')
            clock_info += f' | 时间戳异常/缺失补点 {buf.timestamp_fallbacks}'
            if buf.timestamp_fallbacks:
                clock_info += '（含1ms估算，阶跃时长需核对）'
        self.metrics.setText(f"{'冻结' if self._paused else '实时'} | 可视 {len(t):,} 点 | 缓存 {len(buf) if buf is not None else 0:,}/{buf.max_points if buf is not None else 0:,} | 更新 {(time.perf_counter()-start)*1000:.1f} ms（不含绘制）{clock_info}")
