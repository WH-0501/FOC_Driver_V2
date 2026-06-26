"""CANFD 设备连接管理面板（参考 ZQWL-UCANFD-100E 上位机）。"""

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import (
    QComboBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from canbus.device_profiles import DEVICE_PROFILES
from canbus.device_manager import DeviceState
from canbus.serial_ports import list_serial_ports


class ConnectionPanel(QGroupBox):
    """设备级 + 通道级连接管理，波特率使用内置预设。"""

    open_device_requested = pyqtSignal(str, int, str)
    scan_devices_requested = pyqtSignal(str)
    start_channel_requested = pyqtSignal(int)
    stop_channel_requested = pyqtSignal()
    close_device_requested = pyqtSignal()

    def __init__(self, device_info_fn=None, parent=None):
        super().__init__("CANFD 连接管理", parent)
        self._device_info_fn = device_info_fn
        self._state = DeviceState.CLOSED
        self._build_ui()
        self._update_ui_state()

    def _build_ui(self) -> None:
        root = QVBoxLayout()

        # 第一行：型号 / 索引 / 打开 / 扫描
        row1 = QHBoxLayout()
        self.model_combo = QComboBox()
        for profile in DEVICE_PROFILES:
            self.model_combo.addItem(profile.name, profile.name)
        self.index_combo = QComboBox()
        self.index_combo.addItem("0", 0)
        self.index_combo.addItem("1", 1)
        self.index_combo.addItem("2", 2)
        self.index_combo.addItem("3", 3)
        self.open_btn = QPushButton("打开设备")
        self.scan_btn = QPushButton("扫描设备")
        row1.addWidget(QLabel("CAN卡型号:"))
        row1.addWidget(self.model_combo, 1)
        row1.addWidget(QLabel("索引:"))
        row1.addWidget(self.index_combo)
        row1.addWidget(self.open_btn)
        row1.addWidget(self.scan_btn)
        root.addLayout(row1)

        # SLCAN 串口选择（仅 SLCAN 型号显示）
        self.serial_row = QWidget()
        serial_layout = QHBoxLayout()
        serial_layout.setContentsMargins(0, 0, 0, 0)
        self.serial_combo = QComboBox()
        self.serial_combo.setMinimumWidth(220)
        self.serial_combo.setEditable(True)
        self.refresh_serial_btn = QPushButton("刷新串口")
        serial_layout.addWidget(QLabel("串口:"))
        serial_layout.addWidget(self.serial_combo, 1)
        serial_layout.addWidget(self.refresh_serial_btn)
        self.serial_row.setLayout(serial_layout)
        root.addWidget(self.serial_row)

        # 第二行：设备控制
        row2 = QHBoxLayout()
        self.device_label = QLabel("未连接")
        self.device_start_btn = QPushButton("启动")
        self.device_stop_btn = QPushButton("停止")
        self.close_btn = QPushButton("关闭设备")
        self.info_btn = QPushButton("设备信息")
        row2.addWidget(self.device_label, 1)
        row2.addWidget(self.device_start_btn)
        row2.addWidget(self.device_stop_btn)
        row2.addWidget(self.close_btn)
        row2.addWidget(self.info_btn)
        root.addLayout(row2)

        # 第三行：通道控制
        row3 = QHBoxLayout()
        self.channel_label = QLabel("CAN0")
        self.ch_start_btn = QPushButton("启动")
        self.ch_stop_btn = QPushButton("停止")
        row3.addWidget(self.channel_label)
        row3.addStretch()
        row3.addWidget(self.ch_start_btn)
        row3.addWidget(self.ch_stop_btn)
        root.addLayout(row3)

        self.status_label = QLabel("状态: 未连接")
        self.status_label.setStyleSheet("color: #0066cc;")
        root.addWidget(self.status_label)

        self.setLayout(root)

        self.model_combo.currentTextChanged.connect(self._on_model_changed)
        self.open_btn.clicked.connect(self._on_open)
        self.scan_btn.clicked.connect(self._on_scan)
        self.refresh_serial_btn.clicked.connect(self._refresh_serial_ports)
        self.device_start_btn.clicked.connect(lambda: self.start_channel_requested.emit(0))
        self.device_stop_btn.clicked.connect(self.stop_channel_requested.emit)
        self.ch_start_btn.clicked.connect(lambda: self.start_channel_requested.emit(0))
        self.ch_stop_btn.clicked.connect(self.stop_channel_requested.emit)
        self.close_btn.clicked.connect(self.close_device_requested.emit)
        self.info_btn.clicked.connect(self._show_device_info)

        self._on_model_changed(self.model_combo.currentText())

    def _on_model_changed(self, model_name: str) -> None:
        is_slcan = "SLCAN" in model_name
        self.serial_row.setVisible(is_slcan)
        if is_slcan:
            self._refresh_serial_ports()

    def _refresh_serial_ports(self) -> None:
        current = self.serial_combo.currentData() or self.serial_combo.currentText()
        self.serial_combo.clear()
        ports = list_serial_ports()
        if not ports:
            self.serial_combo.addItem("（未检测到串口）", "")
            return
        for device, desc in ports:
            self.serial_combo.addItem(f"{device}  ({desc})", device)
        if current:
            idx = self.serial_combo.findData(current)
            if idx >= 0:
                self.serial_combo.setCurrentIndex(idx)
            else:
                self.serial_combo.setEditText(str(current))

    def _selected_serial(self) -> str:
        port = self.serial_combo.currentData()
        if not port:
            port = self.serial_combo.currentText().strip()
        if not port or str(port).startswith("（"):
            raise ValueError("请选择串口")
        return str(port)

    def _on_open(self) -> None:
        model = self.model_combo.currentData()
        index = self.index_combo.currentData()
        serial_port = ""
        if self.serial_row.isVisible():
            try:
                serial_port = self._selected_serial()
            except ValueError as exc:
                from PyQt5.QtWidgets import QMessageBox

                QMessageBox.warning(self, "错误", str(exc))
                return
        self.open_device_requested.emit(model, index, serial_port)

    def _on_scan(self) -> None:
        self.scan_devices_requested.emit(self.model_combo.currentData())

    def _show_device_info(self) -> None:
        from PyQt5.QtWidgets import QMessageBox

        if self._device_info_fn:
            QMessageBox.information(self, "设备信息", self._device_info_fn())
            return

        model = self.model_combo.currentText()
        profile = next((p for p in DEVICE_PROFILES if p.name == model), None)
        if not profile:
            return
        info = (
            f"型号: {profile.name}\n"
            f"后端: {profile.backend}\n"
            f"通道数: {profile.channel_count}\n"
            f"CAN FD: {'是' if profile.fd else '否'}\n"
            f"仲裁段: {profile.bitrate // 1000} kbps\n"
            f"数据段: {profile.data_bitrate // 1000} kbps\n"
            f"终端电阻: {'启用' if profile.resistance else '关闭'}"
        )
        QMessageBox.information(self, "设备信息", info)

    def set_device_state(self, state: DeviceState, device_label: str = "", message: str = "") -> None:
        self._state = state
        if device_label:
            self.device_label.setText(device_label)
        if message:
            self.status_label.setText(f"状态: {message}")
        self._update_ui_state()

    def set_scan_result(self, indices: list, message: str = "") -> None:
        current = self.index_combo.currentData()
        self.index_combo.clear()
        if not indices:
            self.index_combo.addItem("0", 0)
            detail = message or "未扫描到设备，可手动选择索引 0 后打开"
            self.status_label.setText(f"状态: {detail}")
            return
        for idx in indices:
            self.index_combo.addItem(str(idx), idx)
        restore = self.index_combo.findData(current)
        if restore >= 0:
            self.index_combo.setCurrentIndex(restore)
        self.status_label.setText(f"状态: 扫描到 {len(indices)} 个设备")

    def _update_ui_state(self) -> None:
        opened = self._state in (DeviceState.OPENED, DeviceState.RUNNING)
        running = self._state == DeviceState.RUNNING

        self.open_btn.setEnabled(not opened)
        self.scan_btn.setEnabled(not opened)
        self.model_combo.setEnabled(not opened)
        self.index_combo.setEnabled(not opened)
        self.serial_combo.setEnabled(not opened)
        self.refresh_serial_btn.setEnabled(not opened)

        self.device_start_btn.setEnabled(opened and not running)
        self.device_stop_btn.setEnabled(running)
        self.ch_start_btn.setEnabled(opened and not running)
        self.ch_stop_btn.setEnabled(running)
        self.close_btn.setEnabled(opened)

        if self._state == DeviceState.CLOSED:
            self.device_label.setText("未连接")
            if not self.status_label.text().startswith("状态: 扫描"):
                self.status_label.setText("状态: 未连接")
            self.status_label.setStyleSheet("color: #666;")
        elif self._state == DeviceState.OPENED:
            self.status_label.setText("状态: 设备已打开，请启动通道")
            self.status_label.setStyleSheet("color: #0066cc;")
        elif self._state == DeviceState.RUNNING:
            self.status_label.setText("状态: 通道已全部启动")
            self.status_label.setStyleSheet("color: #008800;")
