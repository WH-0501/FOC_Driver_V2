"""Read-only calibration verification, using real firmware samples only."""
import csv
import math
import secrets
import time
import pyqtgraph as pg
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QPushButton,
                             QLabel, QFileDialog, QMessageBox)
from protocol.alignment_verify import VerificationRun, parse_verify_sample, parse_verify_result
from protocol import api_error_name


class AlignmentVerifyDialog(QDialog):
    def __init__(self, motor_id, client, parent=None):
        super().__init__(parent)
        self.motor_id, self.client = motor_id, client
        self.run = None
        self.busy = False
        self.started = self.last_rx = 0
        self.dirty = False
        self.setWindowTitle(f"电机 {motor_id} · 当前电角对齐验证")
        self.setWindowModality(Qt.WindowModal)
        self.resize(880, 760)
        layout = QVBoxLayout(self)
        hint = QLabel("纯 Ud 正反扫一圈电角，约 8 秒；只验证，不修改方向或零偏。\n"
                      "会带动电机轴运动。运行期间等待固件自动失能；本窗口不提供软件急停。")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        row = QHBoxLayout()
        self.start_btn = QPushButton("开始验证当前对齐")
        self.export_btn = QPushButton("导出 CSV")
        self.export_btn.setEnabled(False)
        row.addWidget(self.start_btn); row.addWidget(self.export_btn); row.addStretch()
        layout.addLayout(row)
        self.status = QLabel("尚未验证 · 需支持 0xB1/0xB2 的固件与 CAN-FD")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.current = QLabel("目标 —  反馈 —  最短角差 —  机械位移 —")
        self.current.setWordWrap(True)
        layout.addWidget(self.current)
        self.angle_plot = self._plot("目标与反馈电角（0–360°）", "电角", layout)
        self.target_curve = self.angle_plot.plot(pen=pg.mkPen('#0072B2', width=2), name="目标")
        self.measured_curve = self.angle_plot.plot(pen=pg.mkPen('#D55E00', width=2, style=Qt.DashLine), name="反馈")
        self.error_plot = self._plot("最短角差 · 圆点为稳态验收点", "误差", layout)
        self.error_curve = self.error_plot.plot(pen=pg.mkPen('#009E73', width=2), name="反馈 − 目标")
        self.settled_curve = self.error_plot.plot(pen=None, symbol='o', symbolSize=7, symbolBrush='#D55E00', name="稳态")
        for sign in (-1, 1):
            self.error_plot.addItem(pg.InfiniteLine(angle=0, pos=sign * math.degrees(.35),
                pen=pg.mkPen('#888888', style=Qt.DotLine)))
        self.mechanical_plot = self._plot("电机轴机械位移（非减速输出轴）", "位移", layout)
        self.expected_curve = self.mechanical_plot.plot(pen=pg.mkPen('#0072B2', width=2), name="按极对数期望")
        self.mechanical_curve = self.mechanical_plot.plot(pen=pg.mkPen('#D55E00', width=2, style=Qt.DashLine), name="实测累计")
        self.error_plot.setXLink(self.angle_plot)
        self.mechanical_plot.setXLink(self.angle_plot)
        self.start_btn.clicked.connect(self.start)
        self.export_btn.clicked.connect(self.export_csv)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(100)

    def _plot(self, title, ylabel, layout):
        plot = pg.PlotWidget(title=title)
        plot.setBackground(self.palette().base().color())
        plot.setLabel('left', ylabel, units='°')
        plot.setLabel('bottom', '固件时间', units='s')
        plot.addLegend()
        plot.showGrid(x=True, y=True, alpha=.15)
        layout.addWidget(plot, 1)
        return plot

    def start(self):
        if self.busy:
            return
        session = secrets.randbelow(65535) + 1
        self.run = VerificationRun(session)
        self.busy = True
        self.start_btn.setEnabled(False)
        self.export_btn.setEnabled(False)
        self.started = self.last_rx = time.monotonic()
        self.status.setText("等待固件验证数据…")
        self.current.setText("目标 —  反馈 —  最短角差 —  机械位移 —")
        for curve in self._curves(): curve.clear()
        if not self.client.verify_electrical(self.motor_id, session):
            self._end("发送失败；检查连接与 CAN-FD 模式，未获得验证结果。")

    def _curves(self):
        return (self.target_curve, self.measured_curve, self.error_curve,
                self.settled_curve, self.expected_curve, self.mechanical_curve)

    def receive_sample(self, payload):
        if not self.busy or self.run is None:
            return
        sample = parse_verify_sample(payload)
        if sample is None:
            self.run.invalid = True
            return
        if self.run.accept(sample):
            self.last_rx = time.monotonic()
            self.dirty = True
            phase = ("初始锁定", "正向扫角", "反向回位")[sample['phase']]
            self.status.setText(f"{phase} · {len(self.run.samples)} 点 · 缺帧 {self.run.gaps}")

    def receive_result(self, payload):
        if not self.busy or self.run is None:
            return
        result = parse_verify_result(payload)
        if result is None or not self.run.finish(result):
            return
        if self.run.passed:
            message = f"通过 · 9 个稳态点 · 最大稳态角差 {math.degrees(result['max_error']):.2f}° · 参数未改写"
        elif result['error']:
            message = f"未通过：{api_error_name(result['error'])}（{result['error']}） · 最大已测稳态角差 {math.degrees(result['max_error']):.2f}°"
        else:
            message = f"数据不完整或采样时长异常，不能判为通过：收到 {len(self.run.samples)}/{result['count']} 点，缺帧 {self.run.gaps}"
        self._end(message)

    def _end(self, message):
        self.busy = False
        self.start_btn.setEnabled(True)
        self.export_btn.setEnabled(bool(self.run and self.run.samples))
        self.status.setText(message)
        self.dirty = True
        self.refresh()

    def refresh(self):
        if self.busy:
            elapsed = time.monotonic() - self.started
            if elapsed > 25:
                count = len(self.run.samples) if self.run else 0
                self._end(f"超时：已收 {count} 点，但未收到结束应答；不能判定通过。确认电机已停止后再试。" if count else
                          "超时：未收到验证数据，检查固件支持与连接。确认电机已停止后再试。")
                return
            if time.monotonic() - self.last_rx > 3:
                self.status.setText("暂未收到数据；等待结束应答，不能判为通过…")
        if not self.dirty or self.run is None or not self.run.samples:
            return
        self.dirty = False
        samples = self.run.samples
        x = [s['time_s'] for s in samples]
        # Suppress misleading vertical strokes at 360° -> 0° wrap boundaries.
        for curve, key in ((self.target_curve,'target'), (self.measured_curve,'measured')):
            y = self.run.degrees(key)
            links = [abs(y[i + 1] - y[i]) < 180 for i in range(len(y)-1)] + [False]
            import numpy as np
            curve.setData(x, y, connect=np.asarray(links, dtype=bool))
        self.error_curve.setData(x, self.run.degrees('error'))
        stable = [s for s in samples if s['settled']]
        self.settled_curve.setData([s['time_s'] for s in stable], [math.degrees(s['error']) for s in stable])
        self.expected_curve.setData(x, self.run.degrees('expected_mechanical'))
        self.mechanical_curve.setData(x, self.run.degrees('mechanical'))
        s=samples[-1]
        self.current.setText(f"目标 {math.degrees(s['target']):.2f}°   反馈 {math.degrees(s['measured']):.2f}°   "
                             f"最短角差 {math.degrees(s['error']):+.2f}°   机械位移 {math.degrees(s['mechanical']):+.2f}°")

    def export_csv(self):
        if self.run is None or not self.run.samples:
            return
        path, _ = QFileDialog.getSaveFileName(self, '导出验证数据', f'alignment-{self.motor_id}.csv', 'CSV (*.csv)')
        if not path: return
        try:
            with open(path, 'w', newline='', encoding='utf-8-sig') as f:
                fields = ['session','sequence','time_s','phase','settled','target_deg','measured_deg',
                          'error_deg','mechanical_deg','expected_mechanical_deg','result_error','complete']
                writer = csv.DictWriter(f, fieldnames=fields); writer.writeheader()
                for s in self.run.samples:
                    row={k:s[k] for k in fields[:5]}
                    row.update({k+'_deg':math.degrees(s[k]) for k in ('target','measured','error','mechanical','expected_mechanical')})
                    row.update(result_error=self.run.result['error'] if self.run.result else '', complete=self.run.complete)
                    writer.writerow(row)
        except OSError as e:
            QMessageBox.warning(self, '导出失败', str(e))

    def reject(self):
        if not self.busy: super().reject()

    def closeEvent(self, event):
        if self.busy:
            event.ignore()
        else:
            super().closeEvent(event)
