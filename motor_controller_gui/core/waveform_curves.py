"""波形曲线定义。"""

from PyQt5.QtCore import Qt

# key, 显示名, RGB, 线型, 默认显示, 分组
WAVEFORM_CURVES = [
    ("target_pos", "目标位置(°)", (255, 80, 80), Qt.DashLine, False, "位置"),
    ("actual_pos", "实际位置(°)", (255, 200, 80), Qt.SolidLine, True, "位置"),
    ("target_vel", "目标速度(RPM)", (80, 200, 255), Qt.DashLine, False, "速度"),
    ("actual_vel", "实际速度(RPM)", (255, 140, 60), Qt.SolidLine, True, "速度"),
    ("target_iq", "目标Iq(mA)", (80, 255, 160), Qt.DashLine, False, "电流"),
    ("iq", "Iq(mA)", (180, 120, 255), Qt.SolidLine, True, "电流"),
    ("id", "Id(mA)", (120, 180, 255), Qt.SolidLine, False, "电流"),
    ("ia", "Ia(mA)", (255, 100, 180), Qt.SolidLine, False, "电流"),
    ("ib", "Ib(mA)", (100, 255, 200), Qt.SolidLine, False, "电流"),
    ("ic", "Ic(mA)", (200, 100, 255), Qt.SolidLine, False, "电流"),
]

CURVE_KEYS = [c[0] for c in WAVEFORM_CURVES]
