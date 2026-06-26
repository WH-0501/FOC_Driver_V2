"""电机实时状态显示。"""

from typing import Any, Dict, Optional

from PyQt5.QtWidgets import QGridLayout, QGroupBox, QLabel


class MotorStatusWidget(QGroupBox):
    def __init__(self, motor_id: int, parent=None):
        super().__init__(f"实时状态 (ID={motor_id})", parent)
        self.motor_id = motor_id
        self._enabled: Optional[bool] = None
        layout = QGridLayout()

        self.enable_label = QLabel("使能: --")
        self.mode_label = QLabel("模式: --")
        self.error_label = QLabel("错误: --")
        self.pos_label = QLabel("位置: --")
        self.vel_label = QLabel("速度: --")
        self.iq_label = QLabel("Iq: --")
        self.id_label = QLabel("Id: --")
        self.ia_label = QLabel("Ia: --")
        self.ib_label = QLabel("Ib: --")
        self.ic_label = QLabel("Ic: --")

        layout.addWidget(self.enable_label, 0, 0)
        layout.addWidget(self.mode_label, 0, 1)
        layout.addWidget(self.error_label, 0, 2)
        layout.addWidget(self.pos_label, 1, 0)
        layout.addWidget(self.vel_label, 1, 1)
        layout.addWidget(self.iq_label, 1, 2)
        layout.addWidget(self.id_label, 2, 0)
        layout.addWidget(self.ia_label, 2, 1)
        layout.addWidget(self.ib_label, 2, 2)
        layout.addWidget(self.ic_label, 3, 0)
        self.setLayout(layout)

    def _format_current(self, data: Dict[str, Any], key: str) -> str:
        if not data.get("has_phase_current", True) and key != "iq_ma":
            return "--"
        val = data.get(key)
        if val is None:
            return "--"
        return f"{val} mA"

    def update_status(self, data: Dict[str, Any]) -> None:
        if not data:
            return
        self._enabled = bool(data.get("enable"))
        enable = "是" if self._enabled else "否"
        self.enable_label.setText(f"使能: {enable}")
        self.mode_label.setText(f"模式: {data.get('mode_name', '--')}")
        self.error_label.setText(f"错误: {data.get('error_name', '--')}")
        self.pos_label.setText(
            f"位置: {data.get('position_deg', 0):.2f}° ({data.get('position_raw', 0)})"
        )
        self.vel_label.setText(f"速度: {data.get('velocity', '--')} RPM")
        iq = data.get("iq_ma", data.get("current_ma", "--"))
        self.iq_label.setText(f"Iq: {iq if iq == '--' else f'{iq} mA'}")
        self.id_label.setText(f"Id: {self._format_current(data, 'id_ma')}")
        self.ia_label.setText(f"Ia: {self._format_current(data, 'ia_ma')}")
        self.ib_label.setText(f"Ib: {self._format_current(data, 'ib_ma')}")
        self.ic_label.setText(f"Ic: {self._format_current(data, 'ic_ma')}")

    def is_enabled(self) -> bool:
        return self._enabled is True

    def clear(self) -> None:
        self._enabled = None
        self.enable_label.setText("使能: --")
        self.mode_label.setText("模式: --")
        self.error_label.setText("错误: --")
        self.pos_label.setText("位置: --")
        self.vel_label.setText("速度: --")
        self.iq_label.setText("Iq: --")
        self.id_label.setText("Id: --")
        self.ia_label.setText("Ia: --")
        self.ib_label.setText("Ib: --")
        self.ic_label.setText("Ic: --")
