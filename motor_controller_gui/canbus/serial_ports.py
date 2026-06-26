"""枚举本机可用串口。"""

from typing import List, Tuple


def list_serial_ports() -> List[Tuple[str, str]]:
    """返回 [(device, description), ...]。"""
    try:
        from serial.tools import list_ports
    except ImportError:
        return []

    return [(info.device, info.description or "未知设备") for info in list_ports.comports()]
