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
from protocol import angle_to_pos_raw


class MotorControlWidget(QGroupBox):
    def __init__(self, motor_id: int, client: MotorClient, parent=None):
        super().__init__("运动控制", parent)
        self.motor_id = motor_id
        self.client = client
        self._build_ui()

    def _build_ui(self) -> None:
        layout = QVBoxLayout()

        # 广播类控制（协议无单电机使能/模式，全部广播）
        row1 = QHBoxLayout()
        self.enable_btn = QPushButton("广播使能")
        self.disable_btn = QPushButton("广播失能")
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["PP", "CSP", "CST", "HM", "CSV", "OPEN_VEL"])
        self.set_mode_btn = QPushButton("广播设置模式")
        self.origin_btn = QPushButton("广播设原点")
        self.align_btn = QPushButton("广播电角度对齐")
        self.sync_btn = QPushButton("同步信号")
        row1.addWidget(self.enable_btn)
        row1.addWidget(self.disable_btn)
        row1.addWidget(QLabel("模式:"))
        row1.addWidget(self.mode_combo)
        row1.addWidget(self.set_mode_btn)
        row1.addWidget(self.origin_btn)
        row1.addWidget(self.align_btn)
        row1.addWidget(self.sync_btn)
        layout.addLayout(row1)

        # 本电机目标指令（通过广播帧带 ID）
        row2 = QHBoxLayout()
        row2.addWidget(QLabel("目标位置(°):"))
        self.target_pos = QDoubleSpinBox()
        self.target_pos.setRange(-360.0, 360.0)
        self.target_pos.setDecimals(2)
        self.send_pos_btn = QPushButton("发送位置")
        row2.addWidget(self.target_pos)
        row2.addWidget(self.send_pos_btn)

        row2.addWidget(QLabel("目标速度(RPM):"))
        self.target_vel = QSpinBox()
        self.target_vel.setRange(-32768, 32767)
        self.send_vel_btn = QPushButton("发送速度")
        row2.addWidget(self.target_vel)
        row2.addWidget(self.send_vel_btn)

        row2.addWidget(QLabel("目标电流(mA):"))
        self.target_curr = QSpinBox()
        self.target_curr.setRange(-32768, 32767)
        self.send_curr_btn = QPushButton("发送电流")
        row2.addWidget(self.target_curr)
        row2.addWidget(self.send_curr_btn)
        layout.addLayout(row2)

        self.setLayout(layout)

        self.enable_btn.clicked.connect(lambda: self.client.broadcast_enable(True))
        self.disable_btn.clicked.connect(lambda: self.client.broadcast_enable(False))
        self.set_mode_btn.clicked.connect(self._send_mode)
        self.origin_btn.clicked.connect(self.client.broadcast_set_origin)
        self.align_btn.clicked.connect(self.client.broadcast_align_angle)
        self.sync_btn.clicked.connect(self.client.broadcast_sync)
        self.send_pos_btn.clicked.connect(self._send_position)
        self.send_vel_btn.clicked.connect(self._send_velocity)
        self.send_curr_btn.clicked.connect(self._send_current)

    def set_target_callback(self, callback) -> None:
        self._target_callback = callback

    def _notify_target(self, pos=None, vel=None, curr=None) -> None:
        cb = getattr(self, "_target_callback", None)
        if cb:
            cb(pos=pos, vel=vel, curr=curr)

    def _send_mode(self) -> None:
        self.client.broadcast_mode(self.mode_combo.currentIndex())

    def _send_position(self) -> None:
        raw = angle_to_pos_raw(self.target_pos.value())
        self._notify_target(pos=self.target_pos.value())
        self.client.broadcast_target_positions([(self.motor_id, raw)])

    def _send_velocity(self) -> None:
        self._notify_target(vel=float(self.target_vel.value()))
        self.client.broadcast_target_velocities([(self.motor_id, self.target_vel.value())])

    def _send_current(self) -> None:
        self._notify_target(curr=float(self.target_curr.value()))
        self.client.broadcast_target_currents([(self.motor_id, self.target_curr.value())])
