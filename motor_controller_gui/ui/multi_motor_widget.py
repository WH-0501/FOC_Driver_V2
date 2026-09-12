"""多电机批量控制面板。"""

from typing import Callable, Dict, List, Optional

from PyQt5.QtWidgets import (
    QDoubleSpinBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSpinBox,
    QTableWidget,
    QVBoxLayout,
)

from core.motor_client import MotorClient
from protocol import ProtocolVersion, encode_position, pos_uses_rad


class MultiMotorWidget(QGroupBox):
    """向多个电机同时下发目标指令。"""

    def __init__(
        self,
        client: MotorClient,
        get_motor_ids: Callable[[], List[int]],
        log_fn: Optional[Callable[[str], None]] = None,
        parent=None,
    ):
        super().__init__("多电机批量控制", parent)
        self.client = client
        self.get_motor_ids = get_motor_ids
        self.log = log_fn or (lambda _msg: None)
        self._build_ui()

    def _build_ui(self) -> None:
        layout = QVBoxLayout()

        self.table = QTableWidget(0, 5)
        self._pos_header = "位置(rad)"
        self.table.setHorizontalHeaderLabels(["电机ID", self._pos_header, "速度(RPM)", "电流(mA)", "启用"])
        layout.addWidget(self.table)

        btn_row = QHBoxLayout()
        self.refresh_btn = QPushButton("刷新电机列表")
        self.add_row_btn = QPushButton("添加行")
        self.send_pos_btn = QPushButton("广播发送位置")
        self.send_vel_btn = QPushButton("广播发送速度")
        self.send_curr_btn = QPushButton("广播发送电流")
        self.enable_all_btn = QPushButton("广播使能")
        self.disable_all_btn = QPushButton("广播失能")
        self.fault_reset_all_btn = QPushButton("广播异常复位")
        btn_row.addWidget(self.refresh_btn)
        btn_row.addWidget(self.add_row_btn)
        btn_row.addWidget(self.send_pos_btn)
        btn_row.addWidget(self.send_vel_btn)
        btn_row.addWidget(self.send_curr_btn)
        btn_row.addWidget(self.enable_all_btn)
        btn_row.addWidget(self.disable_all_btn)
        btn_row.addWidget(self.fault_reset_all_btn)
        layout.addLayout(btn_row)
        self.setLayout(layout)

        self.refresh_btn.clicked.connect(self.refresh_from_motors)
        self.add_row_btn.clicked.connect(lambda: self._add_row())
        self.send_pos_btn.clicked.connect(self._send_positions)
        self.send_vel_btn.clicked.connect(self._send_velocities)
        self.send_curr_btn.clicked.connect(self._send_currents)
        self.enable_all_btn.clicked.connect(lambda: self.client.broadcast_enable(True))
        self.disable_all_btn.clicked.connect(lambda: self.client.broadcast_enable(False))
        self.fault_reset_all_btn.clicked.connect(self._broadcast_fault_reset)

    def apply_protocol_version(self, version: ProtocolVersion) -> None:
        self._pos_header = "位置(rad)" if pos_uses_rad(version) else "位置(°)"
        self.table.setHorizontalHeaderLabels(
            ["电机ID", self._pos_header, "速度(RPM)", "电流(mA)", "启用"]
        )
        for row in range(self.table.rowCount()):
            spin = self.table.cellWidget(row, 1)
            if spin is not None:
                spin.setDecimals(4 if pos_uses_rad(version) else 2)
                spin.setSingleStep(0.01 if pos_uses_rad(version) else 0.1)

    def _add_row(self, motor_id: int = 1) -> None:
        row = self.table.rowCount()
        self.table.insertRow(row)
        id_spin = QSpinBox()
        id_spin.setRange(1, 0x7FE)
        id_spin.setValue(motor_id)
        pos_spin = QDoubleSpinBox()
        pos_spin.setRange(-256.0, 256.0)
        use_rad = pos_uses_rad(self.client.protocol_version)
        pos_spin.setDecimals(4 if use_rad else 2)
        pos_spin.setSingleStep(0.01 if use_rad else 0.1)
        vel_spin = QSpinBox()
        vel_spin.setRange(-32768, 32767)
        curr_spin = QSpinBox()
        curr_spin.setRange(-32768, 32767)
        from PyQt5.QtWidgets import QCheckBox

        enable_cb = QCheckBox()
        enable_cb.setChecked(True)
        self.table.setCellWidget(row, 0, id_spin)
        self.table.setCellWidget(row, 1, pos_spin)
        self.table.setCellWidget(row, 2, vel_spin)
        self.table.setCellWidget(row, 3, curr_spin)
        self.table.setCellWidget(row, 4, enable_cb)

    def refresh_from_motors(self) -> None:
        self.table.setRowCount(0)
        for mid in self.get_motor_ids():
            self._add_row(mid)
        self.log(f"已刷新 {self.table.rowCount()} 个电机到批量控制表")

    def _collect_enabled_rows(self) -> List[int]:
        from PyQt5.QtWidgets import QCheckBox

        rows = []
        for row in range(self.table.rowCount()):
            cb = self.table.cellWidget(row, 4)
            if isinstance(cb, QCheckBox) and cb.isChecked():
                rows.append(row)
        return rows

    def _send_positions(self) -> None:
        targets = []
        for row in self._collect_enabled_rows():
            mid = self.table.cellWidget(row, 0).value()
            pos_val = self.table.cellWidget(row, 1).value()
            targets.append((mid, encode_position(self.client.protocol_version, pos_val)))
        if targets:
            self.client.broadcast_target_positions(targets)
            self.log(f"广播位置指令 -> {targets}")

    def _send_velocities(self) -> None:
        targets = []
        for row in self._collect_enabled_rows():
            mid = self.table.cellWidget(row, 0).value()
            vel = self.table.cellWidget(row, 2).value()
            targets.append((mid, vel))
        if targets:
            self.client.broadcast_target_velocities(targets)
            self.log(f"广播速度指令 -> {targets}")

    def _broadcast_fault_reset(self) -> None:
        self.client.broadcast_fault_reset()
        self.log("已广播异常复位 (0x39)")

    def _send_currents(self) -> None:
        targets = []
        for row in self._collect_enabled_rows():
            mid = self.table.cellWidget(row, 0).value()
            curr = self.table.cellWidget(row, 3).value()
            targets.append((mid, curr))
        if targets:
            self.client.broadcast_target_currents(targets)
            self.log(f"广播电流指令 -> {targets}")
