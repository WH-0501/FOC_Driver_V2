"""单电机运动控制面板。"""

from PyQt5.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
)

from core.motor_client import MotorClient
from protocol import (
    ProtocolVersion,
    angle_to_pos_raw,
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
        self._build_ui()
        self.apply_protocol_version(self.client.protocol_version)

    def _build_ui(self) -> None:
        layout = QVBoxLayout()

        row1 = QHBoxLayout()
        self.enable_btn = QPushButton("广播使能")
        self.disable_btn = QPushButton("广播失能")
        self.fault_reset_btn = QPushButton("异常复位")
        self.mode_combo = QComboBox()
        self.set_mode_btn = QPushButton("广播设置模式")
        self.origin_btn = QPushButton("广播设原点")
        self.align_btn = QPushButton("广播电角度对齐")
        self.sync_btn = QPushButton("同步信号")
        row1.addWidget(self.enable_btn)
        row1.addWidget(self.disable_btn)
        row1.addWidget(self.fault_reset_btn)
        row1.addWidget(QLabel("模式:"))
        row1.addWidget(self.mode_combo)
        row1.addWidget(self.set_mode_btn)
        row1.addWidget(self.origin_btn)
        row1.addWidget(self.align_btn)
        row1.addWidget(self.sync_btn)
        layout.addLayout(row1)

        row2 = QHBoxLayout()
        self.pos_label = QLabel("目标位置(°):")
        self.target_pos = QDoubleSpinBox()
        self.target_pos.setRange(-36000.0, 36000.0)
        self.target_pos.setDecimals(2)
        self.send_pos_btn = QPushButton("发送位置")
        row2.addWidget(self.pos_label)
        row2.addWidget(self.target_pos)
        row2.addWidget(self.send_pos_btn)

        self.vel_label = QLabel("目标速度(RPM):")
        self.target_vel = QSpinBox()
        self.target_vel.setRange(-32768, 32767)
        self.send_vel_btn = QPushButton("发送速度")
        row2.addWidget(self.vel_label)
        row2.addWidget(self.target_vel)
        row2.addWidget(self.send_vel_btn)

        self.curr_label = QLabel("目标电流(mA):")
        self.target_curr = QSpinBox()
        self.target_curr.setRange(-32768, 32767)
        self.send_curr_btn = QPushButton("发送电流")
        row2.addWidget(self.curr_label)
        row2.addWidget(self.target_curr)
        row2.addWidget(self.send_curr_btn)
        layout.addLayout(row2)

        self.mode_hint = QLabel("")
        self.mode_hint.setWordWrap(True)
        layout.addWidget(self.mode_hint)

        self.setLayout(layout)

        self.enable_btn.clicked.connect(lambda: self.client.broadcast_enable(True))
        self.disable_btn.clicked.connect(lambda: self.client.broadcast_enable(False))
        self.fault_reset_btn.clicked.connect(lambda: self.client.fault_reset(self.motor_id))
        self.set_mode_btn.clicked.connect(self._send_mode)
        self.origin_btn.clicked.connect(self.client.broadcast_set_origin)
        self.align_btn.clicked.connect(self.client.broadcast_align_angle)
        self.sync_btn.clicked.connect(self.client.broadcast_sync)
        self.send_pos_btn.clicked.connect(self._send_position)
        self.send_vel_btn.clicked.connect(self._send_velocity)
        self.send_curr_btn.clicked.connect(self._send_current)
        self.mode_combo.currentIndexChanged.connect(self._on_mode_changed)

    def apply_protocol_version(self, version: ProtocolVersion, preserve_wire: bool = True) -> None:
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
        self._on_mode_changed()

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
        raw = angle_to_pos_raw(self.target_pos.value())
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
