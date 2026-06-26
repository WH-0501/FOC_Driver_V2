"""电机参数配置面板。"""

from typing import Callable, Dict, Optional

from PyQt5.QtWidgets import (
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
)

from core.motor_client import MotorClient


class MotorParamWidget(QGroupBox):
    """PID / 保护 / 节点 ID 参数配置。"""

    def __init__(
        self,
        motor_id: int,
        client: MotorClient,
        log_fn: Optional[Callable[[str], None]] = None,
        is_enabled_fn: Optional[Callable[[], bool]] = None,
        parent=None,
    ):
        super().__init__("参数配置", parent)
        self.motor_id = motor_id
        self.client = client
        self.log = log_fn or (lambda _msg: None)
        self._is_enabled_fn = is_enabled_fn or (lambda: False)
        self._build_ui()

    def _make_pid_row(self, layout: QGridLayout, row: int, label: str):
        p = QDoubleSpinBox()
        p.setRange(0, 100)
        p.setSingleStep(0.001)
        p.setDecimals(4)
        p.setValue(0.015)
        i = QDoubleSpinBox()
        i.setRange(0, 100)
        i.setSingleStep(0.001)
        i.setDecimals(4)
        i.setValue(0.001)
        d = QDoubleSpinBox()
        d.setRange(0, 100)
        d.setSingleStep(0.001)
        d.setDecimals(4)
        d.setValue(0.0)
        layout.addWidget(QLabel(label), row, 0)
        layout.addWidget(p, row, 1)
        layout.addWidget(i, row, 2)
        layout.addWidget(d, row, 3)
        return p, i, d

    def _build_ui(self) -> None:
        layout = QVBoxLayout()
        grid = QGridLayout()
        grid.addWidget(QLabel("P"), 0, 1)
        grid.addWidget(QLabel("I"), 0, 2)
        grid.addWidget(QLabel("D"), 0, 3)

        self.pos_p, self.pos_i, self.pos_d = self._make_pid_row(grid, 1, "位置环:")
        self.vel_p, self.vel_i, self.vel_d = self._make_pid_row(grid, 2, "速度环:")
        self.cur_p, self.cur_i, self.cur_d = self._make_pid_row(grid, 3, "电流环:")

        self.set_pos_pid_btn = QPushButton("设置")
        self.get_pos_pid_btn = QPushButton("读取")
        self.set_vel_pid_btn = QPushButton("设置")
        self.get_vel_pid_btn = QPushButton("读取")
        self.set_cur_pid_btn = QPushButton("设置")
        self.get_cur_pid_btn = QPushButton("读取")
        grid.addWidget(self.set_pos_pid_btn, 1, 4)
        grid.addWidget(self.get_pos_pid_btn, 1, 5)
        grid.addWidget(self.set_vel_pid_btn, 2, 4)
        grid.addWidget(self.get_vel_pid_btn, 2, 5)
        grid.addWidget(self.set_cur_pid_btn, 3, 4)
        grid.addWidget(self.get_cur_pid_btn, 3, 5)

        grid.addWidget(QLabel("刚度:"), 4, 0)
        self.stiff = QDoubleSpinBox()
        self.stiff.setRange(0, 10)
        self.stiff.setDecimals(4)
        self.stiff.setValue(0.015)
        grid.addWidget(self.stiff, 4, 1)
        grid.addWidget(QLabel("阻尼:"), 4, 2)
        self.damp = QDoubleSpinBox()
        self.damp.setRange(0, 10)
        self.damp.setDecimals(4)
        self.damp.setValue(0.001)
        grid.addWidget(self.damp, 4, 3)
        self.set_stiff_btn = QPushButton("设置")
        self.get_stiff_btn = QPushButton("读取")
        grid.addWidget(self.set_stiff_btn, 4, 4)
        grid.addWidget(self.get_stiff_btn, 4, 5)

        grid.addWidget(QLabel("过温(℃):"), 5, 0)
        self.otp = QSpinBox()
        self.otp.setRange(0, 200)
        self.otp.setValue(85)
        grid.addWidget(self.otp, 5, 1)
        grid.addWidget(QLabel("过流(mA):"), 5, 2)
        self.ocp = QSpinBox()
        self.ocp.setRange(0, 65535)
        self.ocp.setValue(2000)
        grid.addWidget(self.ocp, 5, 3)
        grid.addWidget(QLabel("欠压(V):"), 6, 0)
        self.uvlo = QSpinBox()
        self.uvlo.setRange(0, 255)
        self.uvlo.setValue(16)
        grid.addWidget(self.uvlo, 6, 1)
        grid.addWidget(QLabel("过压(V):"), 6, 2)
        self.ovp = QSpinBox()
        self.ovp.setRange(0, 255)
        self.ovp.setValue(25)
        grid.addWidget(self.ovp, 6, 3)
        self.set_protect_btn = QPushButton("设置")
        self.get_protect_btn = QPushButton("读取")
        grid.addWidget(self.set_protect_btn, 6, 4)
        grid.addWidget(self.get_protect_btn, 6, 5)

        layout.addLayout(grid)

        row = QHBoxLayout()
        row.addWidget(QLabel("新 Node-ID:"))
        self.new_node_id = QSpinBox()
        self.new_node_id.setRange(1, 0x7FE)
        self.new_node_id.setValue(self.motor_id)
        self.set_node_id_btn = QPushButton("设置 Node-ID")
        row.addWidget(self.new_node_id)
        row.addWidget(self.set_node_id_btn)
        row.addStretch()
        layout.addLayout(row)

        btn_row = QHBoxLayout()
        self.save_btn = QPushButton("保存参数")
        self.reset_btn = QPushButton("恢复出厂")
        btn_row.addWidget(self.save_btn)
        btn_row.addWidget(self.reset_btn)
        btn_row.addStretch()
        layout.addLayout(btn_row)

        self.setLayout(layout)

        self.set_pos_pid_btn.clicked.connect(self._set_pos_pid)
        self.get_pos_pid_btn.clicked.connect(self._get_pos_pid)
        self.set_vel_pid_btn.clicked.connect(self._set_vel_pid)
        self.get_vel_pid_btn.clicked.connect(self._get_vel_pid)
        self.set_cur_pid_btn.clicked.connect(self._set_cur_pid)
        self.get_cur_pid_btn.clicked.connect(self._get_cur_pid)
        self.set_stiff_btn.clicked.connect(self._set_stiff)
        self.get_stiff_btn.clicked.connect(self._get_stiff)
        self.set_protect_btn.clicked.connect(self._set_protect)
        self.get_protect_btn.clicked.connect(self._get_protect)
        self.set_node_id_btn.clicked.connect(self._set_node_id)
        self.save_btn.clicked.connect(self._save_params)
        self.reset_btn.clicked.connect(self._reset_factory)

    def _require_disabled(self) -> bool:
        if self._is_enabled_fn():
            QMessageBox.warning(
                self,
                "无法修改参数",
                f"电机 {self.motor_id} 当前为使能状态，请先禁使能后再修改参数。",
            )
            return False
        return True

    def _get_pos_pid(self) -> None:
        self.client.get_pos_pid(self.motor_id)

    def _get_vel_pid(self) -> None:
        self.client.get_vel_pid(self.motor_id)

    def _get_cur_pid(self) -> None:
        self.client.get_cur_pid(self.motor_id)

    def _get_stiff(self) -> None:
        self.client.get_stiff_damp(self.motor_id)

    def _get_protect(self) -> None:
        self.client.get_protect(self.motor_id)

    def _save_params(self) -> None:
        if not self._require_disabled():
            return
        self.client.save_params(self.motor_id)

    def _reset_factory(self) -> None:
        if not self._require_disabled():
            return
        self.client.reset_factory(self.motor_id)

    def _set_pos_pid(self) -> None:
        if not self._require_disabled():
            return
        self.client.set_pos_pid(self.motor_id, self.pos_p.value(), self.pos_i.value(), self.pos_d.value())

    def _set_vel_pid(self) -> None:
        if not self._require_disabled():
            return
        self.client.set_vel_pid(self.motor_id, self.vel_p.value(), self.vel_i.value(), self.vel_d.value())

    def _set_cur_pid(self) -> None:
        if not self._require_disabled():
            return
        self.client.set_cur_pid(self.motor_id, self.cur_p.value(), self.cur_i.value(), self.cur_d.value())

    def _set_stiff(self) -> None:
        if not self._require_disabled():
            return
        self.client.set_stiff_damp(self.motor_id, self.stiff.value(), self.damp.value())

    def _set_protect(self) -> None:
        if not self._require_disabled():
            return
        self.client.set_protect(
            self.motor_id, self.otp.value(), self.ocp.value(), self.uvlo.value(), self.ovp.value()
        )

    def _set_node_id(self) -> None:
        if not self._require_disabled():
            return
        new_id = self.new_node_id.value()
        self.client.set_node_id(self.motor_id, new_id)
        self.log(f"已向电机 {self.motor_id} 发送设置 Node-ID={new_id}，保存参数后重启生效")

    def apply_pid(self, kind: str, values: Dict[str, float]) -> None:
        if kind == "pos":
            self.pos_p.setValue(values.get("P", 0))
            self.pos_i.setValue(values.get("I", 0))
            self.pos_d.setValue(values.get("D", 0))
        elif kind == "vel":
            self.vel_p.setValue(values.get("P", 0))
            self.vel_i.setValue(values.get("I", 0))
            self.vel_d.setValue(values.get("D", 0))
        elif kind == "cur":
            self.cur_p.setValue(values.get("P", 0))
            self.cur_i.setValue(values.get("I", 0))
            self.cur_d.setValue(values.get("D", 0))

    def apply_stiff_damp(self, values: Dict[str, float]) -> None:
        self.stiff.setValue(values.get("stiffness", 0))
        self.damp.setValue(values.get("damping", 0))

    def apply_protect(self, values: Dict[str, float]) -> None:
        self.otp.setValue(int(values.get("over_temperature", 0)))
        self.ocp.setValue(int(values.get("over_current_ma", 0)))
        self.uvlo.setValue(int(values.get("under_voltage_V", 0)))
        self.ovp.setValue(int(values.get("over_voltage_V", 0)))
