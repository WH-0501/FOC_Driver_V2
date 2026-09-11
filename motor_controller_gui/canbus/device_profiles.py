"""CAN 设备预设配置（波特率等内置，用户无需手动选择）。"""

from dataclasses import dataclass
from typing import List, Optional

from canbus.zlg_native import (
    CONTROL_CANFD_DLL,
    ZCAN_USBCANFD_100U,
    ZCAN_USBCANFD_200U,
    ZLG_CAN_DLL,
)


@dataclass(frozen=True)
class DeviceProfile:
    name: str
    backend: str
    channel_count: int = 1
    fd: bool = True
    bitrate: int = 1_000_000
    data_bitrate: int = 5_000_000
    resistance: bool = True
    device_type: Optional[int] = None
    driver_dll: Optional[str] = None
    default_channel: str = "0"
    tty_baudrate: int = 115200


DEVICE_PROFILES: List[DeviceProfile] = [
    DeviceProfile(
        name="ZQWL-UCANFD-100E",
        backend="zlgcan",
        device_type=ZCAN_USBCANFD_100U,
        driver_dll=ZLG_CAN_DLL,
        channel_count=1,
    ),
    DeviceProfile(
        name="创芯科技 USBCAN-FD (100E 单通道)",
        backend="zlgcan",
        device_type=ZCAN_USBCANFD_100U,
        driver_dll=CONTROL_CANFD_DLL,
        channel_count=1,
        bitrate=1_000_000,
        data_bitrate=5_000_000,
        resistance=True,
    ),
    DeviceProfile(
        name="创芯科技 USBCAN-FD (200U)",
        backend="zlgcan",
        device_type=ZCAN_USBCANFD_200U,
        driver_dll=CONTROL_CANFD_DLL,
        channel_count=2,
        bitrate=1_000_000,
        data_bitrate=5_000_000,
        resistance=True,
    ),
    DeviceProfile(
        name="SLCAN 适配器 (CANable 等)",
        backend="slcan",
        fd=True,
    ),
    DeviceProfile(
        name="PCAN-USB",
        backend="pcan",
        default_channel="PCAN_USBBUS1",
        fd=True,
    ),
]

DEFAULT_PROFILE_NAME = "ZQWL-UCANFD-100E"

CANFD_BITRATE = 1_000_000
CANFD_DATA_BITRATE = 5_000_000
