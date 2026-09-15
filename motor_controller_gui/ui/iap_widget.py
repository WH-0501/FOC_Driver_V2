"""单电机 IAP 固件升级面板。"""

from __future__ import annotations

from pathlib import Path
from typing import Callable, Optional

from PyQt5.QtCore import QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QFileDialog,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from core.iap_service import IapProgress, IapUpgradeService
from core.motor_client import MotorClient
from protocol import (
    IAP_MAX_FILE_SIZE,
    parse_asix_header,
    prepare_iap_firmware_image,
)


class IapWidget(QWidget):
    """固件升级：选择 .bin → IAP 流程 → 进度与状态。"""

    _progress_signal = pyqtSignal(object)
    _done_signal = pyqtSignal(bool, str)

    def __init__(
        self,
        motor_id: int,
        client: MotorClient,
        log_fn: Optional[Callable[[str], None]] = None,
        parent=None,
    ):
        super().__init__(parent)
        self.motor_id = motor_id
        self.client = client
        self._log = log_fn or (lambda _msg: None)
        self._firmware_path: Optional[Path] = None
        self._firmware_data: bytes = b""
        self._awaiting_version = False
        self._version_timer = QTimer(self)
        self._version_timer.setSingleShot(True)
        self._version_timer.timeout.connect(self._version_timeout)
        self._service = IapUpgradeService(client, motor_id, log_fn=self._log)
        self._progress_signal.connect(self._apply_progress)
        self._done_signal.connect(self._apply_done)
        self._build_ui()

    def _build_ui(self) -> None:
        root = QVBoxLayout()

        file_group = QGroupBox("固件文件")
        file_layout = QVBoxLayout()
        row = QHBoxLayout()
        self.file_label = QLabel("未选择文件")
        self.file_label.setWordWrap(True)
        self.browse_btn = QPushButton("选择 .bin…")
        self.browse_btn.clicked.connect(self._browse_firmware)
        row.addWidget(self.file_label, 1)
        row.addWidget(self.browse_btn)
        file_layout.addLayout(row)
        self.info_label = QLabel("")
        self.info_label.setStyleSheet("color: #666;")
        file_layout.addWidget(self.info_label)
        file_group.setLayout(file_layout)
        root.addWidget(file_group)

        ver_group = QGroupBox("当前固件")
        ver_layout = QHBoxLayout()
        self.version_label = QLabel("—")
        self.read_version_btn = QPushButton("读取版本")
        self.read_version_btn.clicked.connect(self._read_version)
        ver_layout.addWidget(QLabel("版本:"))
        ver_layout.addWidget(self.version_label, 1)
        ver_layout.addWidget(self.read_version_btn)
        ver_group.setLayout(ver_layout)
        root.addWidget(ver_group)

        prog_group = QGroupBox("升级进度")
        prog_layout = QVBoxLayout()
        self.status_label = QLabel("就绪")
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        prog_layout.addWidget(self.status_label)
        prog_layout.addWidget(self.progress)
        prog_group.setLayout(prog_layout)
        root.addWidget(prog_group)

        btn_row = QHBoxLayout()
        self.upgrade_btn = QPushButton("开始升级")
        self.upgrade_btn.clicked.connect(self._start_upgrade)
        self.cancel_btn = QPushButton("取消")
        self.cancel_btn.setEnabled(False)
        self.cancel_btn.clicked.connect(self._cancel_upgrade)
        btn_row.addWidget(self.upgrade_btn)
        btn_row.addWidget(self.cancel_btn)
        btn_row.addStretch()
        root.addLayout(btn_row)

        hint = QLabel(
            "流程：ENTER → START → 等擦除应答 → DATA → END → VERIFY → RESET。"
            "单电机使用 Node-ID；擦除期间请勿发 DATA。升级期间电机失能。"
        )
        hint.setWordWrap(True)
        hint.setStyleSheet("color: #888; font-size: 11px;")
        root.addWidget(hint)
        root.addStretch()
        self.setLayout(root)

    def set_motor_id(self, motor_id: int) -> None:
        self.motor_id = motor_id
        self._service.motor_id = motor_id

    def _browse_firmware(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self,
            "选择固件",
            "",
            "固件二进制 (*.bin);;所有文件 (*.*)",
        )
        if not path:
            return
        self._load_firmware(Path(path))

    def _load_firmware(self, path: Path) -> None:
        try:
            raw = path.read_bytes()
        except OSError as exc:
            QMessageBox.warning(self, "打开失败", str(exc))
            return
        try:
            image, prep_note = prepare_iap_firmware_image(raw)
        except ValueError as exc:
            QMessageBox.warning(self, "文件无效", str(exc))
            return
        if len(image) > IAP_MAX_FILE_SIZE:
            QMessageBox.warning(
                self,
                "文件过大",
                f"整包 {len(image)} B 超过上限 {IAP_MAX_FILE_SIZE} B",
            )
            return
        self._firmware_path = path
        self._firmware_data = image
        self.file_label.setText(str(path))
        info = parse_asix_header(image)
        size_line = f"整包: {len(image)} B"
        if info:
            size_line += f"（ASIX FileLen={info['file_len']}，体 {info['body_size']} B）"
        checksum_line = ""
        if info:
            checksum_line = f"Checksum32: 0x{info['checksum32']:08X}"
        self.info_label.setText(
            f"{size_line}"
            + (f" | {checksum_line}" if checksum_line else "")
            + (f"\n{prep_note}" if prep_note else "")
        )

    def _set_busy(self, busy: bool) -> None:
        self.upgrade_btn.setEnabled(not busy)
        self.browse_btn.setEnabled(not busy)
        self.read_version_btn.setEnabled(not busy)
        self.cancel_btn.setEnabled(busy)

    def _read_version(self) -> None:
        if not self.client.can.manager.is_running:
            QMessageBox.warning(self, "未连接", "请先打开设备并启动 CAN 通道")
            return
        self._awaiting_version = True
        self._set_busy(True)
        self.status_label.setText("正在读取固件版本…")
        self.client.clear_iap_wait()
        if not self.client.iap_get_version(self.motor_id):
            self._awaiting_version = False
            self._set_busy(False)
            self.status_label.setText("发送失败")
            self._log(f"读取固件版本失败: 发送 IAP_FW_VERSION 失败 (Node-ID={self.motor_id})")
            return
        self._version_timer.start(5000)

    def _version_timeout(self) -> None:
        if not self._awaiting_version:
            return
        self._awaiting_version = False
        self._set_busy(False)
        self.status_label.setText("读取超时")
        self._log(
            f"读取固件版本失败: 5s 内未收到应答 (Node-ID={self.motor_id})，"
            "请确认 ID 与电机一致且固件在线"
        )

    def _start_upgrade(self) -> None:
        if self._service.running:
            return
        if not self._firmware_path or not self._firmware_data:
            QMessageBox.information(self, "提示", "请先选择固件 .bin 文件")
            return
        if not self.client.can.manager.is_running:
            QMessageBox.warning(self, "未连接", "请先打开设备并启动 CAN 通道")
            return
        reply = QMessageBox.question(
            self,
            "确认升级",
            f"将对电机 ID {self.motor_id} 升级固件：\n{self._firmware_path.name}\n\n"
            "升级过程中请勿断电或断开 USB。是否继续？",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return
        self.progress.setValue(0)
        self._set_busy(True)
        self._service.start(
            self._firmware_path,
            progress_cb=lambda prog: self._progress_signal.emit(prog),
            done_cb=lambda ok, msg: self._done_signal.emit(ok, msg),
        )

    def _cancel_upgrade(self) -> None:
        self._service.cancel()
        self.status_label.setText("正在取消…")

    def _apply_progress(self, prog: IapProgress) -> None:
        pct = int(prog.sent * 100 / prog.total) if prog.total else 0
        self.progress.setValue(min(100, pct))
        self.status_label.setText(prog.message)

    def _apply_done(self, ok: bool, message: str) -> None:
        self._set_busy(False)
        self.progress.setValue(100 if ok else self.progress.value())
        self.status_label.setText(message)
        if ok:
            QMessageBox.information(self, "升级完成", message)
        else:
            QMessageBox.warning(self, "升级失败", message)

    def on_iap_status(self, payload: bytes) -> None:
        pass

    def on_firmware_version(self, version: str) -> None:
        if self._awaiting_version:
            self._awaiting_version = False
            self._version_timer.stop()
            self.version_label.setText(version or "（空）")
            self._set_busy(False)
            self.status_label.setText("就绪")
        else:
            self.version_label.setText(version or "—")
