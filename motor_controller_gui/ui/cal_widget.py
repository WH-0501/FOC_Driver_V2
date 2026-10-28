"""电机校准面板：电角度对齐、机械零点、编码器校准等。"""

from PyQt5.QtWidgets import (
    QComboBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from typing import Any, Dict

from core.motor_client import MotorClient
from protocol import (
    ELEC_ALIGN_METHOD_HINTS,
    ELEC_ALIGN_METHOD_NAMES,
    ElecAlignMethod,
    format_align_result_lines,
)


def _hline() -> QFrame:
    line = QFrame()
    line.setFrameShape(QFrame.HLine)
    line.setFrameShadow(QFrame.Sunken)
    return line


class MotorCalWidget(QWidget):
    """校准相关操作集中于此页。"""

    def __init__(self, motor_id: int, client: MotorClient, parent=None):
        super().__init__(parent)
        self.motor_id = motor_id
        self.client = client
        self._build_ui()

    def _build_ui(self) -> None:
        root = QVBoxLayout()
        root.setContentsMargins(4, 4, 4, 4)
        root.setSpacing(8)

        top_hint = QLabel("校准前请失能电机；电角度对齐与设原点须在失能状态下执行。")
        top_hint.setWordWrap(True)
        root.addWidget(top_hint)

        # --- 电角度对齐 ---
        elec_group = QGroupBox("电角度对齐")
        elec_layout = QVBoxLayout()

        self.align_method_combo = QComboBox()
        for method in ElecAlignMethod:
            self.align_method_combo.addItem(
                ELEC_ALIGN_METHOD_NAMES.get(method, str(method)),
                int(method),
            )
        default_idx = self.align_method_combo.findData(int(ElecAlignMethod.UQ))
        if default_idx >= 0:
            self.align_method_combo.setCurrentIndex(default_idx)

        self.align_method_hint = QLabel("")
        self.align_method_hint.setWordWrap(True)
        self.align_method_hint.setStyleSheet("color: #666;")

        self.align_local_btn = QPushButton("本机对齐")

        align_btn_row = QGridLayout()
        align_btn_row.setHorizontalSpacing(6)
        align_btn_row.addWidget(QLabel("对齐方案:"), 0, 0)
        align_btn_row.addWidget(self.align_method_combo, 0, 1)
        align_btn_row.addWidget(self.align_local_btn, 1, 0, 1, 2)

        self.align_result_label = QLabel("对齐结果：尚未执行")
        self.align_result_label.setWordWrap(True)
        self.align_result_label.setStyleSheet("color: #666;")

        elec_layout.addLayout(align_btn_row)
        elec_layout.addWidget(self.align_method_hint)
        elec_layout.addWidget(self.align_result_label)
        elec_group.setLayout(elec_layout)
        root.addWidget(elec_group)

        root.addWidget(_hline())

        # --- 机械零点 ---
        origin_group = QGroupBox("机械零点")
        origin_layout = QVBoxLayout()
        self.origin_btn = QPushButton("广播设原点")
        origin_hint = QLabel("失能状态下将当前关节位置设为零点（更新 theta_offset_rad）。")
        origin_hint.setWordWrap(True)
        origin_hint.setStyleSheet("color: #666;")
        origin_layout.addWidget(self.origin_btn)
        origin_layout.addWidget(origin_hint)
        origin_group.setLayout(origin_layout)
        root.addWidget(origin_group)

        root.addWidget(_hline())

        # --- KTH71 编码器校准 ---
        enc_group = QGroupBox("KTH71 编码器校准")
        enc_layout = QVBoxLayout()

        enc_grid = QGridLayout()
        enc_grid.setHorizontalSpacing(6)
        enc_grid.setVerticalSpacing(4)
        self.kth71_cal_start_btn = QPushButton("开始校准")
        self.kth71_cal_trigger_btn = QPushButton("触发 ANLC")
        self.kth71_cal_stop_btn = QPushButton("停止校准")
        self.kth71_cal_status_btn = QPushButton("查询状态")
        enc_grid.addWidget(self.kth71_cal_start_btn, 0, 0)
        enc_grid.addWidget(self.kth71_cal_trigger_btn, 0, 1)
        enc_grid.addWidget(self.kth71_cal_stop_btn, 1, 0)
        enc_grid.addWidget(self.kth71_cal_status_btn, 1, 1)

        enc_hint = QLabel(
            "须先使能并设模式（速度开环或电流/速度闭环），发送目标转速；"
            "流程：开始校准 → 触发 ANLC → 查询状态 → 停止。"
        )
        enc_hint.setWordWrap(True)
        enc_hint.setStyleSheet("color: #666;")

        enc_layout.addLayout(enc_grid)
        enc_layout.addWidget(enc_hint)
        enc_group.setLayout(enc_layout)
        root.addWidget(enc_group)

        root.addStretch(1)
        self.setLayout(root)

        self.align_method_combo.currentIndexChanged.connect(self._on_align_method_changed)
        self.align_local_btn.clicked.connect(self._align_local)
        self.origin_btn.clicked.connect(self.client.broadcast_set_origin)
        self.kth71_cal_start_btn.clicked.connect(
            lambda: self.client.kth71_cal_start(self.motor_id)
        )
        self.kth71_cal_trigger_btn.clicked.connect(
            lambda: self.client.kth71_cal_trigger(self.motor_id)
        )
        self.kth71_cal_stop_btn.clicked.connect(
            lambda: self.client.kth71_cal_stop(self.motor_id)
        )
        self.kth71_cal_status_btn.clicked.connect(
            lambda: self.client.kth71_cal_status(self.motor_id)
        )

        self._on_align_method_changed()

    def _current_align_method(self) -> int:
        method = self.align_method_combo.currentData()
        if method is None:
            return int(ElecAlignMethod.UQ)
        return int(method)

    def _on_align_method_changed(self) -> None:
        method_val = self._current_align_method()
        try:
            method = ElecAlignMethod(method_val)
        except ValueError:
            self.align_method_hint.setText("")
            return
        self.align_method_hint.setText(ELEC_ALIGN_METHOD_HINTS.get(method, ""))

    def set_motor_id(self, motor_id: int) -> None:
        self.motor_id = motor_id

    def show_align_result(self, result: Dict[str, Any]) -> None:
        self.align_result_label.setText("\n".join(format_align_result_lines(result)))
        color = "#1b5e20" if result.get("ok") else "#b71c1c"
        self.align_result_label.setStyleSheet(f"color: {color};")

    def _align_local(self) -> None:
        self.align_result_label.setText("对齐结果：已发送，等待应答…")
        self.align_result_label.setStyleSheet("color: #666;")
        self.client.align_electric_angle(self.motor_id, self._current_align_method())
