"""电机校准面板：电角度对齐、机械零点、编码器校准等。"""

from PyQt5.QtWidgets import (
    QComboBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QLabel,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from typing import Any, Callable, Dict, Optional

from core.motor_client import MotorClient
from ui.alignment_verify_dialog import AlignmentVerifyDialog
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

    def __init__(
        self,
        motor_id: int,
        client: MotorClient,
        is_enabled_fn: Optional[Callable[[], bool]] = None,
        parent=None,
    ):
        super().__init__(parent)
        self.motor_id = motor_id
        self.client = client
        self._is_enabled_fn = is_enabled_fn or (lambda: False)
        self.verify_dialog = None
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
        default_idx = self.align_method_combo.findData(int(ElecAlignMethod.UD))
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
        self.verify_btn = QPushButton("验证当前对齐 · 实时曲线")
        self.verify_btn.clicked.connect(self._open_verify)
        elec_layout.addWidget(self.verify_btn)
        elec_group.setLayout(elec_layout)
        root.addWidget(elec_group)

        root.addWidget(_hline())

        # --- 机械零点 ---
        origin_group = QGroupBox("机械零点")
        origin_layout = QVBoxLayout()
        self.origin_btn = QPushButton("当前电机设原点")
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

        root.addWidget(_hline())

        # --- 驱动器 Flash（电角对齐 / 机械零点 / 参数页改动等）---
        flash_group = QGroupBox("驱动器参数持久化")
        flash_layout = QVBoxLayout()
        self.save_flash_btn = QPushButton("保存驱动器参数到 Flash")
        flash_hint = QLabel(
            "发送 0x4D，将当前 RAM 中的整份 motor_config 写入 MCU Flash，"
            "包括：电角零偏与 direction（电角度对齐）、机械零点（设原点）、"
            "以及 PID / 保护等参数页已下发项。须失能后操作。\n"
            "KTH71 编码器 ANLC 校准结果保存在编码器芯片内，完成 ANLC 后由芯片自行持久化，"
            "无需也无法通过本按钮保存。"
        )
        flash_hint.setWordWrap(True)
        flash_hint.setStyleSheet("color: #666;")
        self.save_flash_status = QLabel("Flash：尚未保存")
        self.save_flash_status.setWordWrap(True)
        self.save_flash_status.setStyleSheet("color: #666;")
        flash_layout.addWidget(self.save_flash_btn)
        flash_layout.addWidget(flash_hint)
        flash_layout.addWidget(self.save_flash_status)
        flash_group.setLayout(flash_layout)
        root.addWidget(flash_group)

        root.addStretch(1)
        self.setLayout(root)

        self.align_method_combo.currentIndexChanged.connect(self._on_align_method_changed)
        self.align_local_btn.clicked.connect(self._align_local)
        self.origin_btn.clicked.connect(lambda: self.client.set_origin(self.motor_id))
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
        self.save_flash_btn.clicked.connect(self._save_params_to_flash)

        self._on_align_method_changed()

    def _current_align_method(self) -> int:
        method = self.align_method_combo.currentData()
        if method is None:
            return int(ElecAlignMethod.UD)
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
        if self.verify_dialog is not None and self.motor_id != motor_id:
            self.verify_dialog.close()
            self.verify_dialog.deleteLater()
            self.verify_dialog = None
        self.motor_id = motor_id

    def show_align_result(self, result: Dict[str, Any]) -> None:
        self.align_result_label.setText("\n".join(format_align_result_lines(result)))
        color = "#1b5e20" if result.get("ok") else "#b71c1c"
        self.align_result_label.setStyleSheet(f"color: {color};")

    def _align_local(self) -> None:
        self.align_result_label.setText("对齐结果：已发送，等待应答…")
        self.align_result_label.setStyleSheet("color: #666;")
        self.client.align_electric_angle(self.motor_id, self._current_align_method())

    def _require_disabled(self) -> bool:
        if self._is_enabled_fn():
            QMessageBox.warning(
                self,
                "无法保存",
                f"电机 {self.motor_id} 当前为使能状态，请先禁使能后再保存到 Flash。",
            )
            return False
        return True

    def _save_params_to_flash(self) -> None:
        if not self._require_disabled():
            return
        self.save_flash_status.setText("Flash：已发送 0x4D，等待应答…")
        self.save_flash_status.setStyleSheet("color: #666;")
        self.client.save_params(self.motor_id)

    def show_save_params_result(self, ok: bool) -> None:
        if ok:
            self.save_flash_status.setText("Flash：保存成功（掉电后仍有效）")
            self.save_flash_status.setStyleSheet("color: #1b5e20;")
        else:
            self.save_flash_status.setText("Flash：保存失败（见日志）")
            self.save_flash_status.setStyleSheet("color: #b71c1c;")

    def _open_verify(self):
        if self._is_enabled_fn():
            QMessageBox.warning(self, "无法验证", "请先失能电机。")
            return
        if self.verify_dialog is None:
            self.verify_dialog = AlignmentVerifyDialog(self.motor_id, self.client, self)
        self.verify_dialog.show()
        self.verify_dialog.raise_()

    def show_verify_sample(self, payload):
        if self.verify_dialog is not None:
            self.verify_dialog.receive_sample(payload)

    def show_verify_result(self, payload):
        if self.verify_dialog is not None:
            self.verify_dialog.receive_result(payload)
