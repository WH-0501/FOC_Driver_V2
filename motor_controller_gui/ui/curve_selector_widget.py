"""波形曲线选择面板。"""

from typing import Dict, Set

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QCheckBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
)

from core.waveform_curves import WAVEFORM_CURVES

SELECTOR_MIN_WIDTH = 210

PANEL_STYLE = """
QFrame#curveSelector {
    background-color: #1a1a2e;
    border-left: 1px solid #3a3a55;
}
QFrame#curveSelector QLabel {
    color: #999;
    font-size: 13px;
    font-weight: bold;
}
QFrame#curveSelector QPushButton {
    font-size: 12px;
    padding: 3px 6px;
}
"""


def _rgb_css(rgb: tuple) -> str:
    return f"rgb({rgb[0]}, {rgb[1]}, {rgb[2]})"


def _checkbox_style(rgb: tuple, checked: bool) -> str:
    if checked:
        color = _rgb_css(rgb)
    else:
        color = _rgb_css(tuple(int(c * 0.45) for c in rgb))
    return f"color: {color}; font-size: 14px; spacing: 6px;"


class CurveSelectorWidget(QFrame):
    """右侧曲线勾选面板，与波形图并排组成一体。"""

    selection_changed = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("curveSelector")
        self.setStyleSheet(PANEL_STYLE)
        self.setMinimumWidth(SELECTOR_MIN_WIDTH)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)
        self._checks: Dict[str, QCheckBox] = {}
        self._colors: Dict[str, tuple] = {}
        self._defaults: Dict[str, bool] = {}
        self._group_labels: Dict[str, QLabel] = {}
        self._curves = WAVEFORM_CURVES
        self._build_ui(WAVEFORM_CURVES)

    def _build_ui(self, curves) -> None:
        outer = QVBoxLayout(self)
        outer.setContentsMargins(10, 8, 10, 8)
        outer.setSpacing(6)

        title = QLabel("显示曲线")
        title.setStyleSheet("color: #ccc; font-weight: bold; font-size: 15px;")
        outer.addWidget(title)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(4)
        all_btn = QPushButton("全选")
        none_btn = QPushButton("全不选")
        default_btn = QPushButton("默认")
        all_btn.clicked.connect(lambda: self.select_all(True))
        none_btn.clicked.connect(lambda: self.select_all(False))
        default_btn.clicked.connect(self.select_defaults)
        btn_row.addWidget(all_btn)
        btn_row.addWidget(none_btn)
        btn_row.addWidget(default_btn)
        outer.addLayout(btn_row)

        groups: Dict[str, list] = {}
        for key, name, color, _style, default, group in curves:
            cb = QCheckBox(name)
            cb.setChecked(default)
            self._colors[key] = color
            self._defaults[key] = default
            cb.setStyleSheet(_checkbox_style(color, default))
            cb.stateChanged.connect(lambda state, k=key: self._on_check_changed(k, state))
            self._checks[key] = cb
            groups.setdefault(group, []).append(cb)

        for group, checks in groups.items():
            group_lbl = QLabel(group)
            self._group_labels[group] = group_lbl
            outer.addWidget(group_lbl)
            for cb in checks:
                outer.addWidget(cb)

        outer.addStretch()

    def _on_check_changed(self, key: str, state: int) -> None:
        cb = self._checks[key]
        color = self._colors[key]
        cb.setStyleSheet(_checkbox_style(color, state == Qt.Checked))
        self.selection_changed.emit()

    def visible_keys(self) -> Set[str]:
        return {key for key, cb in self._checks.items() if cb.isChecked()}

    def select_all(self, checked: bool) -> None:
        for key, cb in self._checks.items():
            cb.blockSignals(True)
            cb.setChecked(checked)
            cb.setStyleSheet(_checkbox_style(self._colors[key], checked))
            cb.blockSignals(False)
        self.selection_changed.emit()

    def apply_curves(self, curves) -> None:
        """切换协议版本时更新位置曲线标签。"""
        self._curves = curves
        pos_keys = {"target_pos", "actual_pos"}
        name_by_key = {key: name for key, name, *_rest in curves}
        group_by_key = {key: group for key, *_rest, group in curves}
        for key in pos_keys:
            if key in self._checks and key in name_by_key:
                self._checks[key].setText(name_by_key[key])
        old_groups = set(self._group_labels.keys())
        new_groups = {group_by_key[k] for k in pos_keys if k in group_by_key}
        for old_g in old_groups:
            if old_g.startswith("位置(") and old_g not in new_groups:
                lbl = self._group_labels.get(old_g)
                if lbl is not None:
                    for new_g in new_groups:
                        if new_g.startswith("位置("):
                            lbl.setText(new_g)
                            self._group_labels[new_g] = lbl
                            del self._group_labels[old_g]
                            break

    def select_defaults(self) -> None:
        for key, cb in self._checks.items():
            default = self._defaults.get(key, False)
            cb.blockSignals(True)
            cb.setChecked(default)
            cb.setStyleSheet(_checkbox_style(self._colors[key], default))
            cb.blockSignals(False)
        self.selection_changed.emit()
