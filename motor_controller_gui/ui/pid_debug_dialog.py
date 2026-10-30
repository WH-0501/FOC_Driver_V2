"""Modeless diagnostic viewer; never changes the motor control mode or PID."""
import csv
import time
from collections import deque
import pyqtgraph as pg
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QLabel, QComboBox,
    QCheckBox, QSpinBox, QPushButton, QSplitter, QScrollArea, QGroupBox, QWidget, QTableWidget,
    QTableWidgetItem, QFileDialog, QMessageBox, QAbstractItemView)
from protocol.constants import CMD
from protocol.pid_debug import parse_debug, FIELDS

LABELS = {
    'reference': '环内给定', 'feedback': '环内反馈', 'error': '误差（给定−反馈）',
    'iq': '实时 Iq (A)', 'id': '实时 Id (A)', 'dt': 'PID dt (s)',
    'integral': '积分状态', 'output': 'PID输出（后级限幅前）', 'p_term': 'P项（计算）',
    'vbus': '母线电压（滤波）(V)', 'shaft_speed': '轴速度 (rad/s)',
    'kp': 'Kp', 'ki_config': 'Ki配置值', 'ki_effective': '实际Ki（计算）',
    'limit': 'PID输出限幅', 'i_limit': '积分限幅', 'ramp': '输出斜坡',
    'device_tick_us': 'PID更新时刻 (设备us，可能回绕)', 'active': '环正在执行',
}


class PidDebugDialog(QDialog):
    def __init__(self, client, motor_id, parent=None):
        super().__init__(parent)
        self.client, self.motor_id = client, motor_id
        self.requested = (0, 100)
        self.confirmed = 0
        self.pending_since = None
        self._stop_pending = False
        self.last_ack = None
        self.last_rx = [None]*3
        self.buffers = [deque(maxlen=10000) for _ in range(3)]
        self.origin = None
        self._frozen = None
        self.dirty = [False]*3
        self.setWindowTitle(f'三环 PID 调试 · 电机 {motor_id}')
        self.resize(1050, 760)
        layout = QVBoxLayout(self)
        note = QLabel('调试只增加遥测，不切换电机控制方式，不修改PID。正常运行保留原有上报。\n'
                      '每环一帧64B；默认100Hz/环，单环最高1000Hz，总调试帧率≤1000Hz。主循环抽样，不能代替20kHz控制周期录波。\n'
                      '未执行的环显示其上次PID状态；实时Iq/Id、电压及轴速度是发送快照值。关闭窗口退出调试，断联约3秒自动退出。')
        note.setWordWrap(True)
        layout.addWidget(note)
        row = QHBoxLayout()
        self.mode = QComboBox()
        self.mode.addItems(['正常运行', '调试模式'])
        row.addWidget(self.mode)
        self.checks = [QCheckBox(name) for name in ('电流环', '速度环', '位置环')]
        for c in self.checks:
            row.addWidget(c)
        self.checks[0].setChecked(True)
        self.hz = QSpinBox()
        self.hz.setRange(1, 1000)
        self.hz.setValue(100)
        self.hz.setSuffix(' Hz/环')
        row.addWidget(self.hz)
        apply = QPushButton('应用模式')
        apply.clicked.connect(self.apply_mode)
        row.addWidget(apply)
        stop = QPushButton('退出调试')
        stop.clicked.connect(self.stop_debug)
        row.addWidget(stop)
        export = QPushButton('导出调试 CSV')
        export.clicked.connect(self.export_csv)
        row.addWidget(export)
        clear = QPushButton('清空记录')
        clear.clicked.connect(self.clear)
        row.addWidget(clear)
        layout.addLayout(row)
        self.status = QLabel('尚未请求调试；设备默认正常运行')
        layout.addWidget(self.status)
        toolbar = QHBoxLayout()
        self.lock_view = QCheckBox('冻结显示')
        self.lock_view.setToolTip('冻结曲线和数值快照，后台继续采集；可缩放、平移、勾选曲线。取消后恢复最新数据。')
        self.lock_view.toggled.connect(self.view_lock_changed)
        toolbar.addWidget(self.lock_view)
        follow = QPushButton('实时跟随 / 重置视图')
        follow.clicked.connect(self.reset_view)
        toolbar.addWidget(follow)
        toolbar.addWidget(QLabel('显示窗口'))
        self.window_seconds = QSpinBox()
        self.window_seconds.setRange(1, 120)
        self.window_seconds.setValue(6)
        self.window_seconds.setSuffix(' s')
        toolbar.addWidget(self.window_seconds)
        toolbar.addStretch()
        layout.addLayout(toolbar)
        split = QSplitter(Qt.Horizontal)
        layout.addWidget(split, 1)
        self.plot = pg.PlotWidget(background='#101923')
        self.plot.showGrid(x=True, y=True, alpha=.2)
        self.plot.setLabel('bottom', '共同接收时间', units='s')
        self.plot.setLabel('left', '原始数值（单位见右侧）')
        self.plot.getAxis('left').enableAutoSIPrefix(False)
        self.plot.getAxis('bottom').enableAutoSIPrefix(False)
        self.plot.enableAutoRange(x=False, y=True)
        self.plot.getViewBox().sigRangeChangedManually.connect(self.manual_view)
        split.addWidget(self.plot)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(270)
        panel = QWidget()
        side = QVBoxLayout(panel)
        hint = QLabel('选择显示曲线（不改变设备上报）\n不同单位共用纵轴，请按需组合。')
        hint.setWordWrap(True)
        side.addWidget(hint)
        buttons = QHBoxLayout()
        for title, action in [('全选', 'all'), ('全不选', 'none'), ('默认', 'default')]:
            button = QPushButton(title)
            button.clicked.connect(lambda _, choice=action: self.select_curves(choice))
            buttons.addWidget(button)
        side.addLayout(buttons)
        self.curves, self.curve_checks, self.value_labels = {}, {}, {}
        self.loop_infos = []
        palette = ['#55d6be', '#c084fc', '#ffc857', '#66caff', '#ff7b89', '#a8e063']
        for loop, title in enumerate(('电流环（给定/反馈A，输出V）',
                                       '速度环（给定/反馈rad/s，输出A）',
                                       '位置环（给定/反馈rad，输出rad/s）')):
            group = QGroupBox(title)
            group_layout = QVBoxLayout(group)
            info = QLabel('等待数据')
            info.setWordWrap(True)
            group_layout.addWidget(info)
            self.loop_infos.append(info)
            for index, (key, label) in enumerate(LABELS.items()):
                color = palette[index % len(palette)]
                cb = QCheckBox(label)
                cb.setStyleSheet(f'color: {color};')
                cb.setChecked(loop == 0 and key in ('reference', 'feedback'))
                value = QLabel('—')
                value.setMinimumWidth(65)
                field_row = QHBoxLayout()
                field_row.addWidget(cb, 1)
                field_row.addWidget(value)
                group_layout.addLayout(field_row)
                pen = pg.mkPen(color, width=1.3, style=(Qt.SolidLine, Qt.DashLine, Qt.DotLine)[loop])
                curve = self.plot.plot(pen=pen, connect='finite')
                curve.setVisible(cb.isChecked())
                self.curves[loop, key] = curve
                self.curve_checks[loop, key] = cb
                self.value_labels[loop, key] = value
                cb.toggled.connect(lambda checked, i=loop, k=key: self.curve_toggled(i, k, checked))
            side.addWidget(group)
        side.addStretch()
        scroll.setWidget(panel)
        split.addWidget(scroll)
        split.setStretchFactor(0, 1)
        split.setStretchFactor(1, 0)
        split.setSizes([760, 330])
        self.mode.currentIndexChanged.connect(self.mode_changed)
        self.refresh = QTimer(self)
        self.refresh.timeout.connect(self.render)
        self.refresh.start(100)
        self.heartbeat = QTimer(self)
        self.heartbeat.timeout.connect(self.renew)
        self.heartbeat.start(1000)

    def curve_toggled(self, loop, key, checked):
        self.curves[loop, key].setVisible(checked)
        self.dirty[loop] = True

    def select_curves(self, choice):
        for (loop, key), cb in self.curve_checks.items():
            cb.setChecked(choice == 'all' or (choice == 'default' and loop == 0 and key in ('reference', 'feedback')))

    def view_lock_changed(self, locked):
        self._frozen = [list(buf) for buf in self.buffers] if locked else None
        self.plot.enableAutoRange(x=False, y=not locked)
        self.dirty = [True]*3
        self.render()

    def manual_view(self, *_):
        self.lock_view.setChecked(True)

    def reset_view(self):
        self.lock_view.setChecked(False)
        self.plot.enableAutoRange(x=False, y=True)
        self.dirty = [True]*3
        self.render()

    def mode_changed(self, index):
        if index == 0:
            self.stop_debug()

    def stop_debug(self):
        # Cancel lease renewal before sending anything, even on transport failure.
        self.requested = (0, self.hz.value())
        self._stop_pending = True
        self.pending_since = time.perf_counter()
        self.mode.blockSignals(True)
        self.mode.setCurrentIndex(0)
        self.mode.blockSignals(False)
        sent = self.client.configure_pid_debug(self.motor_id, *self.requested, log=False)
        self.status.setText('退出请求已发送，等待设备确认…' if sent else
                            '退出发送失败，已停止续租；将重试，设备也会在续租超时后退出')

    def apply_mode(self):
        if self.mode.currentIndex() == 0:
            self.stop_debug()
            return
        self._stop_pending = False
        mask = sum(1 << i for i, c in enumerate(self.checks) if c.isChecked()) if self.mode.currentIndex() else 0
        if self.mode.currentIndex() and not mask:
            self.status.setText('请至少选择一个调试环')
            return
        if mask.bit_count()*self.hz.value() > 1000:
            self.status.setText('总调试帧率不能超过1000Hz：单环≤1000Hz，双环≤500Hz，三环≤333Hz')
            return
        self.requested = (mask, self.hz.value())
        if self.client.configure_pid_debug(self.motor_id, *self.requested):
            self.pending_since = time.perf_counter()
            self.status.setText('等待设备确认模式…')
        else:
            self.requested = (0, self.hz.value())
            self.status.setText('发送失败：需要已连接的CAN-FD，且不能正在升级')

    def renew(self):
        now = time.perf_counter()
        if self._stop_pending:
            if self.pending_since is not None and now-self.pending_since > 5:
                self._stop_pending = False
                self.pending_since = None
                self.status.setText('退出未获设备确认，已停止续租；请检查连接，不能视为已确认退出')
            else:
                self.client.configure_pid_debug(self.motor_id, 0, self.hz.value(), log=False)
            return
        if self.pending_since is not None and now-self.pending_since > 3:
            self.requested = (0, self.hz.value())
            self.pending_since = None
            self.confirmed = 0
            self.status.setText('模式请求超时：检查固件支持；已停止续租')
        if self.confirmed and self.last_ack is not None and now-self.last_ack > 3:
            self.confirmed = 0
            self.requested = (0, self.hz.value())
            self.status.setText('设备反馈中断，已停止调试续租')
        if self.requested[0] and self.confirmed:
            self.client.configure_pid_debug(self.motor_id, *self.requested, log=False)

    def receive(self, cmd, payload, timestamp):
        if cmd == CMD.DEBUG_CONFIG:
            if len(payload) < 5 or payload[0] != 1:
                return
            if payload[1]:
                self.confirmed = 0
                self.requested = (0, self.hz.value())
                self.pending_since = None
                self.status.setText(f'设备拒绝调试设置，error_t={payload[1]}')
                return
            if payload[2] != self.requested[0]:
                # An older renewal reply must not undo a newer stop/change request.
                return
            self._stop_pending = False
            self.confirmed = payload[2]
            self.last_ack = time.perf_counter()
            self.pending_since = None
            hz = int.from_bytes(payload[3:5], 'big')
            self.status.setText(f'设备确认：调试模式，环掩码{self.confirmed}，{hz}Hz/环' if self.confirmed else '设备确认：正常运行')
            return
        row = parse_debug(cmd, payload)
        if row is None:
            return
        loop = row['loop']
        if self.origin is None:
            self.origin = timestamp
        row['receive_s'] = timestamp-self.origin
        self.buffers[loop].append(row)
        self.last_rx[loop] = time.perf_counter()
        if self._frozen is None:
            self.dirty[loop] = True

    def render(self):
        source = self._frozen if self._frozen is not None else self.buffers
        for i, info in enumerate(self.loop_infos):
            if not source[i]:
                info.setText("冻结快照无数据" if self._frozen is not None else "等待数据")
                continue
            row = source[i][-1]
            age = time.perf_counter()-self.last_rx[i]
            info.setText(f"{'正在执行' if row['active'] else '未执行：PID字段为上次状态'} | "
                         f"{'冻结显示' if self._frozen is not None else ('反馈过期' if age > 1 else '接收中')} | {len(source[i])}/10000点")
            if not self.dirty[i]:
                continue
            self.dirty[i] = False
            data = list(source[i])
            times = [x['receive_s'] for x in data]
            for key in LABELS:
                self.value_labels[i, key].setText(f'{row[key]:.6g}')
                if self.curve_checks[i, key].isChecked():
                    self.curves[i, key].setData(times, [x[key] for x in data])
        if not self.lock_view.isChecked():
            latest = max((buf[-1]['receive_s'] for buf in self.buffers if buf), default=0)
            width = self.window_seconds.value()
            self.plot.setXRange(max(0, latest-width), max(width, latest), padding=0)

    def clear(self):
        self.origin = None
        for i, b in enumerate(self.buffers):
            b.clear()
            self.last_rx[i] = None
            self.loop_infos[i].setText('等待数据')
        for key, curve in self.curves.items():
            curve.setData([], [])
            self.value_labels[key].setText('—')
        self.reset_view()

    def export_csv(self):
        path, _ = QFileDialog.getSaveFileName(self, '导出调试数据', f'motor_{self.motor_id}_pid_debug.csv', 'CSV (*.csv)')
        if not path:
            return
        keys = ['loop', 'receive_s', 'device_tick_us', 'active', 'flags', *FIELDS, 'error', 'p_term', 'ki_effective']
        try:
            with open(path, 'w', newline='', encoding='utf-8-sig') as stream:
                writer = csv.writer(stream)
                writer.writerow(['motor_id', self.motor_id, '电流A/电压V/角度rad/速度rad_s/dt秒；Ki配置值；loop=0电流1速度2位置'])
                writer.writerow(keys)
                for buf in (self._frozen if self._frozen is not None else self.buffers):
                    writer.writerows([[row[k] for k in keys] for row in buf])
        except OSError as exc:
            QMessageBox.warning(self, '导出失败', str(exc))

    def closeEvent(self, event):
        self.stop_debug()
        event.accept()
