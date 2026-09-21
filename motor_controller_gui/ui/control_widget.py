"""单电机运动控制面板。"""

from typing import Optional

from PyQt5.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QLabel,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
)

from core.motor_client import MotorClient
from protocol import (
    ProtocolVersion,
    encode_position,
    pos_uses_rad,
    default_mode_wire,
    mode_hint,
    mode_needs_current,
    mode_needs_position,
    mode_needs_velocity,
    selectable_mode_options,
)


class MotorControlWidget(QGroupBox):
    def __init__(self, motor_id: int, client: MotorClient, parent=None):
        super().__init__("运动控制", parent)
        self.motor_id = motor_id
        self.client = client
        self._reported_mode: Optional[int] = None
        self._build_ui()
        self.apply_protocol_version(self.client.protocol_version)

    def _build_ui(self) -> None:
        layout = QVBoxLayout()

        self.enable_btn = QPushButton("广播使能")
        self.disable_btn = QPushButton("广播失能")
        self.fault_reset_btn = QPushButton("异常复位")
        self.mode_combo = QComboBox()
        self.set_mode_btn = QPushButton("广播设置模式")
        self.sync_btn = QPushButton("同步信号")

        self.pos_label = QLabel("目标位置:")
        self.target_pos = QDoubleSpinBox()
        self.target_pos.setRange(-256.0, 256.0)
        self.target_pos.setDecimals(4)
        self.target_pos.setSingleStep(0.01)
        self.send_pos_btn = QPushButton("发送位置")

        self.vel_label = QLabel("目标速度(RPM):")
        self.target_vel = QSpinBox()
        self.target_vel.setRange(-32768, 32767)
        self.send_vel_btn = QPushButton("发送速度")

        self.curr_label = QLabel("目标电流(mA):")
        self.target_curr = QSpinBox()
        self.target_curr.setRange(-32768, 32767)
        self.send_curr_btn = QPushButton("发送电流")

        grid = QGridLayout()
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(4)

        grid.addWidget(self.enable_btn, 0, 0)
        grid.addWidget(self.disable_btn, 0, 1)
        grid.addWidget(self.fault_reset_btn, 0, 2)

        grid.addWidget(QLabel("模式:"), 1, 0)
        grid.addWidget(self.mode_combo, 1, 1, 1, 2)
        grid.addWidget(self.set_mode_btn, 2, 0, 1, 3)

        grid.addWidget(self.sync_btn, 3, 0, 1, 3)

        grid.addWidget(self.pos_label, 4, 0)
        grid.addWidget(self.target_pos, 4, 1)
        grid.addWidget(self.send_pos_btn, 4, 2)

        grid.addWidget(self.vel_label, 5, 0)
        grid.addWidget(self.target_vel, 5, 1)
        grid.addWidget(self.send_vel_btn, 5, 2)

        grid.addWidget(self.curr_label, 6, 0)
        grid.addWidget(self.target_curr, 6, 1)
        grid.addWidget(self.send_curr_btn, 6, 2)

        layout.addLayout(grid)

        self.mode_hint = QLabel("")
        self.mode_hint.setWordWrap(True)
        layout.addWidget(self.mode_hint)

        self.setLayout(layout)

        self.enable_btn.clicked.connect(lambda: self.client.broadcast_enable(True))
        self.disable_btn.clicked.connect(lambda: self.client.broadcast_enable(False))
        self.fault_reset_btn.clicked.connect(lambda: self.client.fault_reset(self.motor_id))
        self.set_mode_btn.clicked.connect(self._send_mode)
        self.sync_btn.clicked.connect(self.client.broadcast_sync)
        self.send_pos_btn.clicked.connect(self._send_position)
        self.send_vel_btn.clicked.connect(self._send_velocity)
        self.send_curr_btn.clicked.connect(self._send_current)
        self.mode_combo.currentIndexChanged.connect(self._on_mode_changed)

    def apply_reported_mode(self, wire: int) -> None:
        """设备模式变化时同步下拉框；用户本地改选但未下发时不覆盖。"""
        if self._reported_mode is not None and wire == self._reported_mode:
            return

        idx = self.mode_combo.findData(wire)
        if idx < 0:
            self._reported_mode = wire
            return

        self._reported_mode = wire
        if self.mode_combo.currentIndex() == idx:
            return

        self.mode_combo.blockSignals(True)
        self.mode_combo.setCurrentIndex(idx)
        self.mode_combo.blockSignals(False)
        self._on_mode_changed()

    def apply_protocol_version(self, version: ProtocolVersion, preserve_wire: bool = True) -> None:
        self._reported_mode = None
        prev_wire = self.mode_combo.currentData() if preserve_wire else None
        self.mode_combo.blockSignals(True)
        self.mode_combo.clear()
        for opt in selectable_mode_options(version):
            self.mode_combo.addItem(opt.label, opt.wire)
        if prev_wire is not None:
            idx = self.mode_combo.findData(prev_wire)
            if idx >= 0:
                self.mode_combo.setCurrentIndex(idx)
        if self.mode_combo.currentIndex() < 0:
            default_idx = self.mode_combo.findData(default_mode_wire(version))
            if default_idx >= 0:
                self.mode_combo.setCurrentIndex(default_idx)
        self.mode_combo.blockSignals(False)
        self._apply_position_unit(version)
        self._on_mode_changed()

    def _apply_position_unit(self, version: ProtocolVersion) -> None:
        if pos_uses_rad(version):
            self.pos_label.setText("目标位置(rad):")
            self.target_pos.setDecimals(4)
            self.target_pos.setSingleStep(0.01)
        else:
            self.pos_label.setText("目标位置(°):")
            self.target_pos.setDecimals(2)
            self.target_pos.setSingleStep(0.1)

    def set_target_callback(self, callback) -> None:
        self._target_callback = callback

    def _protocol_version(self) -> ProtocolVersion:
        return self.client.protocol_version

    def _current_mode_wire(self) -> int:
        mode_val = self.mode_combo.currentData()
        if mode_val is None:
            return default_mode_wire(self._protocol_version())
        return int(mode_val)

    def _on_mode_changed(self) -> None:
        version = self._protocol_version()
        wire = self._current_mode_wire()
        pos_on = mode_needs_position(version, wire)
        vel_on = mode_needs_velocity(version, wire)
        curr_on = mode_needs_current(version, wire)

        self.target_pos.setEnabled(pos_on)
        self.send_pos_btn.setEnabled(pos_on)
        self.pos_label.setEnabled(pos_on)

        self.target_vel.setEnabled(vel_on)
        self.send_vel_btn.setEnabled(vel_on)
        self.vel_label.setEnabled(vel_on)

        self.target_curr.setEnabled(curr_on)
        self.send_curr_btn.setEnabled(curr_on)
        self.curr_label.setEnabled(curr_on)

        self.mode_hint.setText(mode_hint(version, wire))

    def _notify_target(self, pos=None, vel=None, curr=None) -> None:
        cb = getattr(self, "_target_callback", None)
        if cb:
            cb(pos=pos, vel=vel, curr=curr)

    def _send_mode(self) -> None:
        wire = self._current_mode_wire()
        self.client.broadcast_mode(wire)
        self._on_mode_changed()

    def _send_position(self) -> None:
        version = self._protocol_version()
        wire = self._current_mode_wire()
        if not mode_needs_position(version, wire):
            return
        raw = encode_position(version, self.target_pos.value())
        self._notify_target(pos=self.target_pos.value())
        self.client.broadcast_target_positions([(self.motor_id, raw)])

    def _send_velocity(self) -> None:
        version = self._protocol_version()
        wire = self._current_mode_wire()
        if not mode_needs_velocity(version, wire):
            return
        self._notify_target(vel=float(self.target_vel.value()))
        self.client.broadcast_target_velocities([(self.motor_id, self.target_vel.value())])

    def _send_current(self) -> None:
        version = self._protocol_version()
        wire = self._current_mode_wire()
        if not mode_needs_current(version, wire):
            return
        self._notify_target(curr=float(self.target_curr.value()))
        self.client.broadcast_target_currents([(self.motor_id, self.target_curr.value())])
