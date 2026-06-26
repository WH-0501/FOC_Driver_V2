"""单电机综合面板（状态 + 控制 + 参数）。"""

from typing import Callable, Dict, Optional

from PyQt5.QtWidgets import QTabWidget, QVBoxLayout, QWidget

from core.motor_client import MotorClient
from ui.control_widget import MotorControlWidget
from ui.param_widget import MotorParamWidget
from ui.status_widget import MotorStatusWidget


class MotorPanel(QWidget):
    """单个电机的标签页面板。"""

    def __init__(
        self,
        motor_id: int,
        client: MotorClient,
        log_fn: Optional[Callable[[str], None]] = None,
        target_callback=None,
        parent=None,
    ):
        super().__init__(parent)
        self.motor_id = motor_id
        self.status_widget = MotorStatusWidget(motor_id)
        self.control_widget = MotorControlWidget(motor_id, client)
        if target_callback:
            self.control_widget.set_target_callback(target_callback)
        self.param_widget = MotorParamWidget(
            motor_id,
            client,
            log_fn,
            is_enabled_fn=lambda: self.status_widget.is_enabled(),
        )

        layout = QVBoxLayout()
        layout.addWidget(self.status_widget)

        tabs = QTabWidget()
        tabs.addTab(self.control_widget, "运动控制")
        tabs.addTab(self.param_widget, "参数配置")
        layout.addWidget(tabs)
        self.setLayout(layout)

    def update_status(self, data: Dict) -> None:
        self.status_widget.update_status(data)

    def apply_pid(self, kind: str, values: Dict) -> None:
        self.param_widget.apply_pid(kind, values)

    def apply_stiff_damp(self, values: Dict) -> None:
        self.param_widget.apply_stiff_damp(values)

    def apply_protect(self, values: Dict) -> None:
        self.param_widget.apply_protect(values)
