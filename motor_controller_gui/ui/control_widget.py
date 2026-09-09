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
    ControlMode,
    MODE_NAMES,
    SELECTABLE_CONTROL_MODES,
    angle_to_pos_raw,
    mode_needs_current,
    mode_needs_position,
    mode_needs_velocity,
)


class MotorControlWidget(QGroupBox):
    def __init__(self, motor_id: int, client: MotorClient, parent=None):
        super().__init__("运动控制", parent)
        self.motor_id = motor_id
        self.client = client
        self._build_ui()

    def _build_ui(self) -> None:
        layout = QVBoxLayout()

        row1 = QHBoxLayout()
        self.enable_btn = QPushButton("广播使能")
        self.disable_btn = QPushButton("广播失能")
        self.fault_reset_btn = QPushButton("异常复位")
        self.mode_combo = QComboBox()
        for mode in SELECTABLE_CONTROL_MODES:
            self.mode_combo.addItem(MODE_NAMES[mode], int(mode))
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

        default_index = self.mode_combo.findData(int(ControlMode.SPEED_OPEN_LOOP))
        if default_index >= 0:
            self.mode_combo.setCurrentIndex(default_index)
        self._on_mode_changed()

    def set_target_callback(self, callback) -> None:
        self._target_callback = callback

    def _current_mode(self) -> ControlMode:
        mode_val = self.mode_combo.currentData()
        if mode_val is None:
            return ControlMode.IDLE
        return ControlMode(int(mode_val))

    def _on_mode_changed(self) -> None:
        mode = self._current_mode()
        pos_on = mode_needs_position(mode)
        vel_on = mode_needs_velocity(mode)
        curr_on = mode_needs_current(mode)

        self.target_pos.setEnabled(pos_on)
        self.send_pos_btn.setEnabled(pos_on)
        self.pos_label.setEnabled(pos_on)

        self.target_vel.setEnabled(vel_on)
        self.send_vel_btn.setEnabled(vel_on)
        self.vel_label.setEnabled(vel_on)

        self.target_curr.setEnabled(curr_on)
        self.send_curr_btn.setEnabled(curr_on)
        self.curr_label.setEnabled(curr_on)

        if mode is ControlMode.IDLE:
            hint = "空闲模式：请先设置其它模式并失能后再切换。"
        elif mode is ControlMode.HOMING:
            hint = "回零模式：由固件回零流程驱动，无需手动下发目标。"
        elif mode is ControlMode.VOLTAGE:
            hint = "电压模式：Vd/Vq 给定暂未在此面板提供，请通过其它接口配置。"
        elif mode is ControlMode.MIT:
            hint = "MIT 模式：可同时下发位置/速度/电流；刚度阻尼在参数页配置。"
        else:
            parts = []
            if pos_on:
                parts.append("位置")
            if vel_on:
                parts.append("速度")
            if curr_on:
                parts.append("电流/转矩")
            hint = f"当前模式有效给定：{'、'.join(parts) if parts else '无'}"
        self.mode_hint.setText(hint)

    def _notify_target(self, pos=None, vel=None, curr=None) -> None:
        cb = getattr(self, "_target_callback", None)
        if cb:
            cb(pos=pos, vel=vel, curr=curr)

    def _send_mode(self) -> None:
        mode = self._current_mode()
        self.client.broadcast_mode(int(mode))
        self._on_mode_changed()

    def _send_position(self) -> None:
        if not mode_needs_position(self._current_mode()):
            return
        raw = angle_to_pos_raw(self.target_pos.value())
        self._notify_target(pos=self.target_pos.value())
        self.client.broadcast_target_positions([(self.motor_id, raw)])

    def _send_velocity(self) -> None:
        if not mode_needs_velocity(self._current_mode()):
            return
        self._notify_target(vel=float(self.target_vel.value()))
        self.client.broadcast_target_velocities([(self.motor_id, self.target_vel.value())])

    def _send_current(self) -> None:
        if not mode_needs_current(self._current_mode()):
            return
        self._notify_target(curr=float(self.target_curr.value()))
        self.client.broadcast_target_currents([(self.motor_id, self.target_curr.value())])
