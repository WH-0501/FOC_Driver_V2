# -*- coding: utf-8 -*-
"""周立功 zlgcan.dll ctypes 封装（兼容 ZQWL-UCANFD-100E、创芯 USBCANFD-200U 等）。"""

from __future__ import annotations

import sys
import os
import time
from ctypes import (
    POINTER,
    Structure,
    Union,
    byref,
    c_char_p,
    c_int,
    c_long,
    c_longlong,
    c_ubyte,
    c_uint,
    c_ulong,
    c_ulonglong,
    c_ushort,
    c_void_p,
)
from pathlib import Path
from typing import List, Optional, Tuple

try:
    from ctypes import windll
except ImportError:
    windll = None  # Other platforms can still use python-can backends.

ZCAN_USBCANFD_200U = 41
ZCAN_USBCANFD_100U = 42
ZLG_CAN_DLL = "zlgcan.dll"
CONTROL_CANFD_DLL = "ControlCANFD.dll"
BACKEND_ZLGCAN = "zlgcan"
BACKEND_CONTROL_CANFD = "controlcanfd"

ZLG_DEVICE_TYPE_LABELS = {
    ZCAN_USBCANFD_200U: "USBCANFD-200U (双通道)",
    ZCAN_USBCANFD_100U: "USBCANFD-100U (单通道)",
}
ZCAN_USBCANFD_MINI = 43
ZCAN_USBCANFD_400U = 76
INVALID_DEVICE_HANDLE = 0

# 常见 USBCAN-FD 设备类型（创芯 USBCAN-FD 兼容 200U/41，ZQWL-100E 为 100U/42）
ZLG_USBCANFD_DEVICE_TYPES = (
    ZCAN_USBCANFD_100U,
    ZCAN_USBCANFD_200U,
    ZCAN_USBCANFD_MINI,
    ZCAN_USBCANFD_400U,
)


def zlg_device_types_to_try(preferred: int) -> Tuple[int, ...]:
    """扫描用：去重后的设备类型尝试顺序。"""
    ordered: List[int] = []
    for dtype in (preferred, *ZLG_USBCANFD_DEVICE_TYPES):
        if dtype not in ordered:
            ordered.append(dtype)
    return tuple(ordered)


def score_device_type(
    info: Optional[ZCAN_DEVICE_INFO],
    dtype: int,
    expected_channels: int,
) -> int:
    """按硬件 can_Num 评估是否匹配 Profile（双通道以 can_Num>=2 为准，41 类型优先）。"""
    if info is None:
        return -1
    can_num = int(info.can_Num) if info.can_Num else 0

    if expected_channels >= 2:
        if can_num < 2:
            return -1
        score = 100 + can_num
        if dtype == ZCAN_USBCANFD_200U:
            score += 20
        return score

    if expected_channels == 1:
        if can_num >= 2:
            return -1
        if dtype == ZCAN_USBCANFD_100U:
            return 100
        if can_num <= 1:
            return 50
        return -1

    return -1


ZCAN_STATUS_OK = 1
ZCAN_TYPE_CAN = 0
ZCAN_TYPE_CANFD = 1
ZCAN_TYPE_MERGE = 2
ZCAN_DT_ZCAN_CAN_CANFD_DATA = 1

# 与创芯 CANFDToolPro / ZCANPRO 通道参数页默认项对齐
CANFD_STANDARD_ISO = 0  # 0=CAN FD ISO，1=BOSCH 非 ISO
CANFD_STANDARD_ISO_STR = "0"
CANFD_MODE_NORMAL = 0  # InitCAN.canfd.mode：0=正常节点
CANFD_ABIT_BAUD_HZ = 1_000_000  # 仲裁域 1 Mbps（界面显示 80% 采样点由 DLL 内部 timing 决定）
CANFD_DBIT_BAUD_HZ = 5_000_000  # 数据域 5 Mbps（界面显示 75% 采样点）

# USBCANFD @ 60MHz, 1Mbps / 5Mbps 常用 timing（与 ZLG / 创芯二次开发例程一致）
ABIT_TIMING_1M = 101166
DBIT_TIMING_5M = 4260362

class ZCAN_DEVICE_INFO(Structure):
    _fields_ = [
        ("hw_Version", c_ushort),
        ("fw_Version", c_ushort),
        ("dr_Version", c_ushort),
        ("in_Version", c_ushort),
        ("irq_Num", c_ushort),
        ("can_Num", c_ubyte),
        ("str_Serial_Num", c_ubyte * 20),
        ("str_hw_Type", c_ubyte * 40),
        ("reserved", c_ushort * 4),
    ]


class _ZCAN_CHANNEL_CANFD_INIT_CONFIG(Structure):
    _fields_ = [
        ("acc_code", c_uint),
        ("acc_mask", c_uint),
        ("abit_timing", c_uint),
        ("dbit_timing", c_uint),
        ("brp", c_uint),
        ("filter", c_ubyte),
        ("mode", c_ubyte),
        ("pad", c_ushort),
        ("reserved", c_uint),
    ]


class _ZCAN_CHANNEL_CAN_INIT_CONFIG(Structure):
    _fields_ = [
        ("acc_code", c_uint),
        ("acc_mask", c_uint),
        ("reserved", c_uint),
        ("filter", c_ubyte),
        ("timing0", c_ubyte),
        ("timing1", c_ubyte),
        ("mode", c_ubyte),
    ]


class _ZCAN_CHANNEL_INIT_CONFIG(Union):
    _fields_ = [("can", _ZCAN_CHANNEL_CAN_INIT_CONFIG), ("canfd", _ZCAN_CHANNEL_CANFD_INIT_CONFIG)]


class ZCAN_CHANNEL_INIT_CONFIG(Structure):
    _fields_ = [("can_type", c_uint), ("config", _ZCAN_CHANNEL_INIT_CONFIG)]


def make_canfd_init_cfg(mode: int = CANFD_MODE_NORMAL) -> ZCAN_CHANNEL_INIT_CONFIG:
    """创芯 ControlCANFD InitCAN 参数。"""
    init_cfg = ZCAN_CHANNEL_INIT_CONFIG()
    init_cfg.can_type = ZCAN_TYPE_CANFD
    init_cfg.config.canfd.filter = 0
    init_cfg.config.canfd.acc_code = 0
    init_cfg.config.canfd.acc_mask = 0xFFFFFFFF
    init_cfg.config.canfd.mode = mode
    return init_cfg


def make_zqwl_init_cfg() -> ZCAN_CHANNEL_INIT_CONFIG:
    """ZQWL 旧版可用配置：SetValue 波特率 + InitCAN 内嵌 timing（commit 0290c22）。"""
    init_cfg = ZCAN_CHANNEL_INIT_CONFIG()
    init_cfg.can_type = ZCAN_TYPE_CANFD
    init_cfg.config.canfd.acc_code = 0
    init_cfg.config.canfd.acc_mask = 0xFFFFFFFF
    init_cfg.config.canfd.abit_timing = ABIT_TIMING_1M
    init_cfg.config.canfd.dbit_timing = DBIT_TIMING_5M
    init_cfg.config.canfd.brp = 0
    init_cfg.config.canfd.filter = 0
    init_cfg.config.canfd.mode = 0
    return init_cfg


class ZCAN_CAN_FRAME(Structure):
    _fields_ = [
        ("can_id", c_uint, 29),
        ("err", c_uint, 1),
        ("rtr", c_uint, 1),
        ("eff", c_uint, 1),
        ("can_dlc", c_ubyte),
        ("__pad", c_ubyte),
        ("__res0", c_ubyte),
        ("__res1", c_ubyte),
        ("data", c_ubyte * 8),
    ]


class ZCAN_CANFD_FRAME(Structure):
    _fields_ = [
        ("can_id", c_uint, 29),
        ("err", c_uint, 1),
        ("rtr", c_uint, 1),
        ("eff", c_uint, 1),
        ("len", c_ubyte),
        ("brs", c_ubyte, 1),
        ("esi", c_ubyte, 1),
        ("__res", c_ubyte, 6),
        ("__res0", c_ubyte),
        ("__res1", c_ubyte),
        ("data", c_ubyte * 64),
    ]


class ZCAN_TransmitFD_Data(Structure):
    _fields_ = [("frame", ZCAN_CANFD_FRAME), ("transmit_type", c_uint)]


class ZCAN_Receive_Data(Structure):
    _fields_ = [("frame", ZCAN_CAN_FRAME), ("timestamp", c_ulonglong)]


class ZCAN_ReceiveFD_Data(Structure):
    _fields_ = [("frame", ZCAN_CANFD_FRAME), ("timestamp", c_ulonglong)]


class _ZCANdataFlag(Structure):
    _pack_ = 1
    _fields_ = [
        ("frameType", c_uint, 2),
        ("txDelay", c_uint, 2),
        ("transmitType", c_uint, 4),
        ("txEchoRequest", c_uint, 1),
        ("txEchoed", c_uint, 1),
        ("reserved", c_uint, 22),
    ]


class ZCANCANFDData(Structure):
    _pack_ = 1
    _fields_ = [
        ("timestamp", c_ulonglong),
        ("flag", _ZCANdataFlag),
        ("extraData", c_ubyte * 4),
        ("frame", ZCAN_CANFD_FRAME),
    ]


class ZCANDataObj(Structure):
    _pack_ = 1
    _fields_ = [
        ("dataType", c_ubyte),
        ("chnl", c_ubyte),
        ("flag", c_ushort),
        ("extraData", c_ubyte * 4),
        ("zcanCANFDData", ZCANCANFDData),
        ("reserved", c_ubyte * 4),
    ]


def _library_root() -> Path:
    return Path(__file__).resolve().parent.parent


def _candidate_dll_dirs() -> List[Path]:
    root = _library_root()
    candidates: List[Path] = []
    if getattr(sys, "frozen", False):
        candidates.append(Path(sys.executable).resolve().parent / "library")
        meipass = getattr(sys, "_MEIPASS", None)
        if meipass:
            candidates.append(Path(meipass) / "library")
    candidates.extend([
        root / "library",
        Path(r"C:\Program Files (x86)\USB_CANFD TOOL"),
        Path(r"C:\Program Files (x86)\CANFDToolPro"),
        Path(r"C:\Program Files (x86)\CANFD Tool"),
        Path(r"C:\Program Files\USB_CANFD TOOL"),
        Path(r"C:\Program Files (x86)\USB_CANFD TOOL\firmware"),
        Path(r"C:\Program Files (x86)\USB_CAN TOOL"),
        Path(r"C:\Program Files (x86)\ZCANPRO"),
        Path(r"C:\Program Files\ZCANPRO"),
    ])
    found: List[Path] = []
    seen = set()
    for path in candidates:
        key = str(path)
        if key in seen or not path.is_dir():
            continue
        seen.add(key)
        found.append(path)
    return found


def _dll_is_64bit(dll_path: Path) -> bool:
    data = dll_path.read_bytes()[:512]
    pe_off = int.from_bytes(data[0x3C:0x40], "little")
    machine = int.from_bytes(data[pe_off + 4: pe_off + 6], "little")
    return machine == 0x8664


def _dll_backend(dll_path: Path) -> str:
    name = dll_path.name.lower()
    if name == CONTROL_CANFD_DLL.lower():
        return BACKEND_CONTROL_CANFD
    return BACKEND_ZLGCAN


def _discover_zlgcan_dirs() -> List[Path]:
    """收集含 zlgcan.dll 的目录（仅固定路径，避免全盘搜索阻塞 UI）。"""
    seen: set = set()
    ordered: List[Path] = []

    def add(path: Path) -> None:
        if not path.is_dir():
            return
        key = str(path.resolve())
        if key in seen:
            return
        seen.add(key)
        ordered.append(path)

    for path in _candidate_dll_dirs():
        add(path)
    return ordered


def _pick_best_zlgcan_dir(require_kerneldlls: bool = True) -> Optional[Path]:
    want_64 = sys.maxsize > 2**32
    best_score = -1
    best_dir: Optional[Path] = None
    fallback_dir: Optional[Path] = None
    for directory in _discover_zlgcan_dirs():
        dll_path = directory / ZLG_CAN_DLL
        if not dll_path.is_file():
            continue
        if want_64 and not _dll_is_64bit(dll_path):
            continue
        has_kernel = (directory / "kerneldlls").is_dir()
        if not has_kernel:
            if fallback_dir is None:
                fallback_dir = directory
            if require_kerneldlls:
                continue
        score = (1000 if has_kernel else 0) + dll_path.stat().st_size // 1024
        if directory.resolve() == (_library_root() / "library").resolve():
            score += 50
        if score > best_score:
            best_score = score
            best_dir = directory
    return best_dir or fallback_dir


def find_can_driver_dll(preferred: Optional[str] = None) -> Tuple[Optional[Path], str, str]:
    """查找 CAN 驱动 DLL。创芯 USBCANFD 需 ControlCANFD.dll，ZQWL 等用 zlgcan.dll+kerneldlls。"""
    want_64 = sys.maxsize > 2**32
    lib = _library_root() / "library"

    if preferred == ZLG_CAN_DLL:
        zlg_dir = _pick_best_zlgcan_dir(require_kerneldlls=True)
        if zlg_dir is None:
            zlg_dir = _pick_best_zlgcan_dir(require_kerneldlls=False)
        if zlg_dir is not None:
            return zlg_dir / ZLG_CAN_DLL, "", BACKEND_ZLGCAN
        return None, (
            "未找到可用的 zlgcan.dll（ZQWL 等设备需要 zlgcan.dll 与 kerneldlls 目录）。\n"
            f"请从 ZLG 二次开发包 demo_python_64 复制 zlgcan.dll 与 kerneldlls 到 {lib}/"
        ), ""

    if preferred == CONTROL_CANFD_DLL:
        for directory in _candidate_dll_dirs():
            dll_path = directory / CONTROL_CANFD_DLL
            if dll_path.is_file() and (not want_64 or _dll_is_64bit(dll_path)):
                return dll_path, "", BACKEND_CONTROL_CANFD
        return None, (
            f"未找到 64 位 {CONTROL_CANFD_DLL}。\n"
            f"创芯 USBCANFD 请将二次开发包中的 {CONTROL_CANFD_DLL} 复制到 {lib}/"
        ), ""

    order: List[str] = [CONTROL_CANFD_DLL, ZLG_CAN_DLL] if want_64 else [ZLG_CAN_DLL, CONTROL_CANFD_DLL]
    for dll_name in order:
        sub, err, backend = find_can_driver_dll(dll_name)
        if sub is not None:
            return sub, err, backend

    return None, "未找到 zlgcan.dll / ControlCANFD.dll。", ""


def find_zlgcan_dll() -> Tuple[Optional[Path], str]:
    dll_path, err, _backend = find_can_driver_dll(ZLG_CAN_DLL)
    if dll_path is None:
        return None, err
    return dll_path, ""


def _handle(value: int) -> c_void_p:
    return c_void_p(value)


def _handle_int(value) -> int:
    if value is None:
        return 0
    if hasattr(value, "value"):
        return int(value.value)
    return int(value)


class ZlgCanApi:
    def __init__(self, dll_path: Path):
        self.dll_path = dll_path
        self._dll_dir = dll_path.parent
        self.backend = _dll_backend(dll_path)
        if hasattr(os, "add_dll_directory"):
            os.add_dll_directory(str(self._dll_dir))
            kernel = self._dll_dir / "kerneldlls"
            if kernel.is_dir():
                os.add_dll_directory(str(kernel))
        try:
            self._dll = windll.LoadLibrary(str(dll_path))
        except OSError as exc:
            if getattr(exc, "winerror", None) == 193:
                raise OSError(
                    f"DLL 位数与 Python 不匹配（当前 Python 为 64 位）。"
                    f"请将 64 位 {CONTROL_CANFD_DLL} 或 zlgcan.dll+kerneldlls 复制到 library/，"
                    f"或使用 32 位 Python 运行本程序。原始错误: {exc}"
                ) from exc
            raise
        self._bind_functions()

    @property
    def is_controlcanfd(self) -> bool:
        return self.backend == BACKEND_CONTROL_CANFD

    def _bind_functions(self) -> None:
        if self.is_controlcanfd:
            self._bind_controlcanfd()
        else:
            self._bind_zlgcan_legacy()

    def _bind_zlgcan_legacy(self) -> None:
        """ZQWL：恢复 commit 0290c22 的 c_void_p 绑定（创芯改动前已验证可用）。"""
        dll = self._dll
        self._handle_type = c_void_p
        dll.ZCAN_OpenDevice.argtypes = [c_uint, c_uint, c_uint]
        dll.ZCAN_OpenDevice.restype = c_void_p
        dll.ZCAN_CloseDevice.argtypes = [c_void_p]
        dll.ZCAN_CloseDevice.restype = c_uint
        dll.ZCAN_GetDeviceInf.argtypes = [c_void_p, POINTER(ZCAN_DEVICE_INFO)]
        dll.ZCAN_GetDeviceInf.restype = c_uint
        dll.ZCAN_InitCAN.argtypes = [c_void_p, c_uint, POINTER(ZCAN_CHANNEL_INIT_CONFIG)]
        dll.ZCAN_InitCAN.restype = c_void_p
        dll.ZCAN_StartCAN.argtypes = [c_void_p]
        dll.ZCAN_StartCAN.restype = c_uint
        dll.ZCAN_ResetCAN.argtypes = [c_void_p]
        dll.ZCAN_ResetCAN.restype = c_uint
        dll.ZCAN_GetReceiveNum.argtypes = [c_void_p, c_uint]
        dll.ZCAN_GetReceiveNum.restype = c_uint
        dll.ZCAN_TransmitFD.argtypes = [c_void_p, POINTER(ZCAN_TransmitFD_Data), c_uint]
        dll.ZCAN_TransmitFD.restype = c_uint
        dll.ZCAN_ReceiveFD.argtypes = [c_void_p, POINTER(ZCAN_ReceiveFD_Data), c_uint, c_int]
        dll.ZCAN_ReceiveFD.restype = c_uint
        if hasattr(dll, "ZCAN_SetValue"):
            dll.ZCAN_SetValue.argtypes = [c_void_p, c_char_p, c_char_p]
            dll.ZCAN_SetValue.restype = c_uint
        self._receive_can = None
        self._receive_data = None
        self._get_iproperty = None

    def _bind_controlcanfd(self) -> None:
        dll = self._dll
        handle_type = c_ulonglong
        self._handle_type = handle_type
        dll.ZCAN_OpenDevice.argtypes = [c_uint, c_uint, c_uint]
        dll.ZCAN_OpenDevice.restype = handle_type
        dll.ZCAN_CloseDevice.argtypes = [handle_type]
        dll.ZCAN_CloseDevice.restype = c_uint
        dll.ZCAN_GetDeviceInf.argtypes = [handle_type, POINTER(ZCAN_DEVICE_INFO)]
        dll.ZCAN_GetDeviceInf.restype = c_uint
        dll.ZCAN_InitCAN.argtypes = [handle_type, c_uint, POINTER(ZCAN_CHANNEL_INIT_CONFIG)]
        dll.ZCAN_InitCAN.restype = handle_type
        dll.ZCAN_StartCAN.argtypes = [handle_type]
        dll.ZCAN_StartCAN.restype = c_uint
        dll.ZCAN_ResetCAN.argtypes = [handle_type]
        dll.ZCAN_ResetCAN.restype = c_uint
        dll.ZCAN_GetReceiveNum.argtypes = [handle_type, c_uint]
        dll.ZCAN_GetReceiveNum.restype = c_uint
        dll.ZCAN_TransmitFD.argtypes = [handle_type, POINTER(ZCAN_TransmitFD_Data), c_uint]
        dll.ZCAN_TransmitFD.restype = c_uint
        if hasattr(dll, "ZCAN_Receive"):
            dll.ZCAN_Receive.argtypes = [handle_type, POINTER(ZCAN_Receive_Data), c_uint, c_long]
            dll.ZCAN_Receive.restype = c_uint
            self._receive_can = dll.ZCAN_Receive
        else:
            self._receive_can = None
        dll.ZCAN_ReceiveFD.argtypes = [handle_type, POINTER(ZCAN_ReceiveFD_Data), c_uint, c_long]
        dll.ZCAN_ReceiveFD.restype = c_uint
        if hasattr(dll, "ZCAN_ReceiveData"):
            dll.ZCAN_ReceiveData.argtypes = [handle_type, POINTER(ZCANDataObj), c_uint, c_int]
            dll.ZCAN_ReceiveData.restype = c_uint
            self._receive_data = dll.ZCAN_ReceiveData
        else:
            self._receive_data = None
        dll.ZCAN_SetAbitBaud.argtypes = [handle_type, c_ulong, c_ulong]
        dll.ZCAN_SetAbitBaud.restype = c_uint
        dll.ZCAN_SetDbitBaud.argtypes = [handle_type, c_ulong, c_ulong]
        dll.ZCAN_SetDbitBaud.restype = c_uint
        dll.ZCAN_SetCANFDStandard.argtypes = [handle_type, c_ulong, c_ulong]
        dll.ZCAN_SetCANFDStandard.restype = c_uint
        dll.ZCAN_ClearFilter.argtypes = [handle_type]
        dll.ZCAN_SetFilterMode.argtypes = [handle_type, c_ulong]
        dll.ZCAN_SetFilterStartID.argtypes = [handle_type, c_ulong]
        dll.ZCAN_SetFilterEndID.argtypes = [handle_type, c_ulong]
        dll.ZCAN_AckFilter.argtypes = [handle_type]
        self._get_iproperty = None

    def _as_handle(self, value: int):
        return self._handle_type(value)

    def open_device(self, device_type: int, device_index: int) -> int:
        handle = self._dll.ZCAN_OpenDevice(c_uint(device_type), c_uint(device_index), 0)
        return _handle_int(handle)

    def close_device(self, device_handle: int) -> int:
        return self._dll.ZCAN_CloseDevice(self._as_handle(device_handle))

    def get_device_info(self, device_handle: int) -> Optional[ZCAN_DEVICE_INFO]:
        info = ZCAN_DEVICE_INFO()
        ret = self._dll.ZCAN_GetDeviceInf(self._as_handle(device_handle), byref(info))
        return info if ret == ZCAN_STATUS_OK else None

    def set_device_value(self, device_handle: int, path: str, value: str) -> bool:
        if not hasattr(self._dll, "ZCAN_SetValue"):
            return False
        ret = self._dll.ZCAN_SetValue(
            self._as_handle(device_handle),
            path.encode("ascii"),
            value.encode("ascii"),
        )
        return ret == ZCAN_STATUS_OK

    def require_device_value(self, device_handle: int, path: str, value: str) -> None:
        if not self.set_device_value(device_handle, path, value):
            raise RuntimeError(f"ZCAN_SetValue 失败: {path}={value}")

    def set_abit_baud(self, device_handle: int, channel: int, baud: int) -> None:
        ret = self._dll.ZCAN_SetAbitBaud(
            self._as_handle(device_handle), c_ulong(channel), c_ulong(baud)
        )
        if ret != ZCAN_STATUS_OK:
            raise RuntimeError(f"ZCAN_SetAbitBaud 失败: 通道 {channel}, {baud} bps")

    def set_dbit_baud(self, device_handle: int, channel: int, baud: int) -> None:
        ret = self._dll.ZCAN_SetDbitBaud(
            self._as_handle(device_handle), c_ulong(channel), c_ulong(baud)
        )
        if ret != ZCAN_STATUS_OK:
            raise RuntimeError(f"ZCAN_SetDbitBaud 失败: 通道 {channel}, {baud} bps")

    def set_canfd_standard(self, device_handle: int, channel: int, iso: int = 0) -> None:
        ret = self._dll.ZCAN_SetCANFDStandard(
            self._as_handle(device_handle), c_ulong(channel), c_ulong(iso)
        )
        if ret != ZCAN_STATUS_OK:
            raise RuntimeError(f"ZCAN_SetCANFDStandard 失败: 通道 {channel}")

    def configure_channel_filter_std(self, channel_handle: int) -> None:
        ch = self._as_handle(channel_handle)
        self._dll.ZCAN_ClearFilter(ch)
        self._dll.ZCAN_SetFilterMode(ch, c_ulong(0))
        self._dll.ZCAN_SetFilterStartID(ch, c_ulong(0))
        self._dll.ZCAN_SetFilterEndID(ch, c_ulong(0x7FF))
        self._dll.ZCAN_AckFilter(ch)

    def init_can(self, device_handle: int, channel: int, init_cfg: ZCAN_CHANNEL_INIT_CONFIG) -> int:
        handle = self._dll.ZCAN_InitCAN(
            self._as_handle(device_handle), c_uint(channel), byref(init_cfg)
        )
        return _handle_int(handle)

    def start_can(self, channel_handle: int) -> int:
        return int(self._dll.ZCAN_StartCAN(self._as_handle(channel_handle)))

    def reset_can(self, channel_handle: int) -> int:
        return int(self._dll.ZCAN_ResetCAN(self._as_handle(channel_handle)))

    def get_receive_num(self, channel_handle: int, frame_type: int = ZCAN_TYPE_CANFD) -> int:
        return self._dll.ZCAN_GetReceiveNum(self._as_handle(channel_handle), c_uint(frame_type))

    def transmit_fd(self, channel_handle: int, msgs, count: int) -> int:
        # 单帧 Structure 用 byref；数组直接传（自动转为首元素指针），避免 byref(array) 双重引用
        if isinstance(msgs, ZCAN_TransmitFD_Data):
            buf = byref(msgs)
        else:
            buf = msgs
        return int(self._dll.ZCAN_TransmitFD(self._as_handle(channel_handle), buf, count))

    def receive_can(self, channel_handle: int, count: int, timeout_ms: int = 0):
        if not self._receive_can:
            return (ZCAN_Receive_Data * 0)(), 0
        buffer = (ZCAN_Receive_Data * count)()
        ret = self._receive_can(self._as_handle(channel_handle), buffer, count, timeout_ms)
        return buffer, ret

    def receive_fd(self, channel_handle: int, count: int, timeout_ms: int = 0):
        buffer = (ZCAN_ReceiveFD_Data * count)()
        ret = self._dll.ZCAN_ReceiveFD(self._as_handle(channel_handle), buffer, count, timeout_ms)
        return buffer, ret

    def receive_data(self, device_handle: int, count: int, timeout_ms: int = 0):
        if not self._receive_data:
            return (ZCANDataObj * 0)(), 0
        buffer = (ZCANDataObj * count)()
        ret = self._receive_data(self._as_handle(device_handle), buffer, count, timeout_ms)
        return buffer, ret


class ZlgCanSession:
    def __init__(self) -> None:
        self.api: Optional[ZlgCanApi] = None
        self.device_handle: int = 0
        self.channel_handle: int = 0
        self.device_type: int = ZCAN_USBCANFD_100U
        self.device_index: int = 0
        self.channel: int = 0
        self.channel_count: int = 1
        self.use_merge_recv: bool = False
        self._abit_baud: int = CANFD_ABIT_BAUD_HZ
        self._dbit_baud: int = CANFD_DBIT_BAUD_HZ
        self._resistance: bool = True
        self.device_info: Optional[ZCAN_DEVICE_INFO] = None
        self.profile_device_type: int = ZCAN_USBCANFD_100U
        self.open_type_mismatch: str = ""
        self.last_error: str = ""

    @staticmethod
    def hardware_channel_count(info: Optional[ZCAN_DEVICE_INFO]) -> int:
        if info and info.can_Num:
            return max(1, int(info.can_Num))
        return 1

    @staticmethod
    def hardware_type_label(device_type: int) -> str:
        return ZLG_DEVICE_TYPE_LABELS.get(device_type, f"类型 {device_type}")

    @staticmethod
    def hardware_name(info: Optional[ZCAN_DEVICE_INFO]) -> str:
        if not info:
            return ""
        return bytes(info.str_hw_Type).split(b"\x00", 1)[0].decode(errors="ignore")

    @staticmethod
    def hardware_serial(info: Optional[ZCAN_DEVICE_INFO]) -> str:
        if not info:
            return ""
        return bytes(info.str_Serial_Num).split(b"\x00", 1)[0].decode(errors="ignore")

    @staticmethod
    def scan(
        device_type: int = ZCAN_USBCANFD_100U,
        max_index: int = 4,
        expected_channels: int = 1,
        driver_dll: Optional[str] = None,
    ) -> Tuple[List[int], str, Optional[int]]:
        preferred = driver_dll
        if preferred is None and expected_channels >= 2:
            preferred = CONTROL_CANFD_DLL
        dll_path, err, _backend = find_can_driver_dll(preferred)
        if dll_path is None:
            return [], err, None
        try:
            api = ZlgCanApi(dll_path)
        except OSError as exc:
            return [], f"加载 zlgcan.dll 失败: {exc}", None

        found: List[int] = []
        resolved_type: Optional[int] = None
        resolved_info: Optional[ZCAN_DEVICE_INFO] = None
        for index in range(max_index):
            best_score = -1
            best_dtype: Optional[int] = None
            best_info: Optional[ZCAN_DEVICE_INFO] = None
            for dtype in zlg_device_types_to_try(device_type):
                handle = api.open_device(dtype, index)
                if not handle or handle == INVALID_DEVICE_HANDLE:
                    continue
                info = api.get_device_info(handle)
                api.close_device(handle)
                score = score_device_type(info, dtype, expected_channels)
                if score > best_score:
                    best_score = score
                    best_dtype = dtype
                    best_info = info
            if best_dtype is not None and best_score >= 0:
                found.append(index)
                if resolved_type is None:
                    resolved_type = best_dtype
                    resolved_info = best_info
                elif resolved_type != best_dtype:
                    resolved_type = device_type

        if found and resolved_type is not None:
            hw = ZlgCanSession.hardware_name(resolved_info) if resolved_info else ""
            ch = int(resolved_info.can_Num) if resolved_info and resolved_info.can_Num else expected_channels
            hint = (
                f"（{ZlgCanSession.hardware_type_label(resolved_type)}，"
                f"硬件 {ch} 通道{('，' + hw) if hw else ''}）"
            )
            return found, hint, resolved_type
        return [], "未检测到在线设备，请确认 USB 已连接、驱动正常，且已关闭原厂上位机", None

    def open_device(
        self,
        device_type: int,
        device_index: int,
        resolved_type: Optional[int] = None,
        channel_count: int = 1,
        abit_baud: int = CANFD_ABIT_BAUD_HZ,
        dbit_baud: int = CANFD_DBIT_BAUD_HZ,
        resistance: bool = True,
        driver_dll: Optional[str] = None,
    ) -> None:
        self.close_device()
        preferred = driver_dll
        if preferred is None and channel_count >= 2:
            preferred = CONTROL_CANFD_DLL
        dll_path, err, _backend = find_can_driver_dll(preferred)
        if dll_path is None:
            raise RuntimeError(err)
        self.api = ZlgCanApi(dll_path)
        self.device_index = device_index
        self.profile_device_type = device_type
        self._abit_baud = abit_baud
        self._dbit_baud = dbit_baud
        self._resistance = resistance
        self.open_type_mismatch = ""

        if not self.api.is_controlcanfd:
            self._open_device_zqwl(device_type, device_index)
            return

        probes: List[str] = []
        best_score = -1
        best_dtype = 0
        best_handle = 0
        best_info: Optional[ZCAN_DEVICE_INFO] = None

        for dtype in zlg_device_types_to_try(device_type):
            handle = self.api.open_device(dtype, device_index)
            label = self.hardware_type_label(dtype)
            if not handle or handle == INVALID_DEVICE_HANDLE:
                probes.append(f"  类型 {dtype} ({label}): OpenDevice 失败")
                continue
            info = self.api.get_device_info(handle)
            can_num = int(info.can_Num) if info and info.can_Num else 0
            hw = self.hardware_name(info)
            serial = self.hardware_serial(info)
            score = score_device_type(info, dtype, channel_count)
            if score > best_score:
                if best_handle:
                    self.api.close_device(best_handle)
                best_score = score
                best_dtype = dtype
                best_handle = handle
                best_info = info
                probes.append(
                    f"  类型 {dtype} ({label}): 可用 can_Num={can_num}, "
                    f"硬件={hw or '?'}, SN={serial or '?'}"
                )
            else:
                self.api.close_device(handle)
                reason = (
                    f"can_Num={can_num} 不满足 {channel_count} 通道"
                    if score < 0
                    else "得分较低"
                )
                probes.append(f"  类型 {dtype} ({label}): 已打开但跳过 ({reason})")

        if best_score < 0 or not best_handle or best_info is None:
            self.api = None
            probe_text = "\n".join(probes) if probes else "  （无探测结果）"
            has_kernel = (dll_path.parent / "kerneldlls").is_dir()
            if _dll_backend(dll_path) == BACKEND_CONTROL_CANFD:
                hint = (
                    "创芯 USBCANFD 请使用 64 位 ControlCANFD.dll；"
                    "可从 CANFDToolPro 二次开发包复制到 motor_controller_gui/library/。"
                )
            elif not has_kernel:
                hint = (
                    "ZQWL 等设备需要 zlgcan.dll 与 kerneldlls 目录（当前缺少 kerneldlls）。"
                    "请从 ZLG demo_python_64 复制到 motor_controller_gui/library/。"
                )
            else:
                hint = "请确认 CAN 卡型号与 Profile 匹配，并关闭原厂上位机后重试。"
            raise RuntimeError(
                f"无法以 {channel_count} 通道 Profile 打开设备 (索引 {device_index})。\n"
                f"探测结果:\n{probe_text}\n"
                f"当前 DLL: {dll_path}\n"
                f"{hint}"
            )

        self.device_type = best_dtype
        self.device_handle = best_handle
        self.device_info = best_info
        if best_dtype != device_type:
            self.open_type_mismatch = (
                f"Profile 指定 {self.hardware_type_label(device_type)}，"
                f"实际以 {self.hardware_type_label(best_dtype)} 打开"
                f"（can_Num={best_info.can_Num}，{self.hardware_name(best_info)}）"
            )

        self.channel_count = self.hardware_channel_count(self.device_info)
        self.use_merge_recv = False
        self._prepare_all_channels()

    def _open_device_zqwl(self, device_type: int, device_index: int) -> None:
        """ZQWL 旧版打开流程：按 Profile 类型直接 OpenDevice，不做多类型探测。"""
        assert self.api is not None
        handle = self.api.open_device(device_type, device_index)
        if not handle or handle == INVALID_DEVICE_HANDLE:
            raise RuntimeError(
                f"打开设备失败 (索引 {device_index})，请先关闭 USB_CANFD/ZCAN 原厂工具"
            )
        self.device_type = device_type
        self.device_handle = handle
        self.device_info = self.api.get_device_info(handle)
        self.channel_count = self.hardware_channel_count(self.device_info)

    def _prepare_all_channels(self) -> None:
        """创芯 ControlCANFD 打开后预配波特率；zlgcan 推迟到 start_channel 一次性配置。"""
        if not self.api or not self.api.is_controlcanfd:
            return
        for ch in range(self.channel_count):
            self._configure_channel_baud(ch, self._abit_baud, self._dbit_baud, resistance=False)

    def _configure_channel_baud(
        self,
        channel: int,
        abit_baud: int,
        dbit_baud: int,
        resistance: bool = False,
    ) -> None:
        """InitCAN 之前：波特率 / ISO / 终端电阻（创芯 API 或 zlgcan SetValue）。"""
        assert self.api is not None
        if self.api.is_controlcanfd:
            self.api.set_abit_baud(self.device_handle, channel, abit_baud)
            self.api.set_dbit_baud(self.device_handle, channel, dbit_baud)
            self.api.set_canfd_standard(self.device_handle, channel, CANFD_STANDARD_ISO)

    def _configure_channel_zqwl(
        self,
        channel: int,
        abit_baud: int,
        dbit_baud: int,
        resistance: bool,
    ) -> None:
        """ZQWL 旧版通道配置（commit 0290c22）。"""
        assert self.api is not None
        ch = str(channel)
        setters = [
            (f"{ch}/canfd_standard", CANFD_STANDARD_ISO_STR),
            (f"{ch}/canfd_abit_baud_rate", str(abit_baud)),
            (f"{ch}/canfd_dbit_baud_rate", str(dbit_baud)),
        ]
        if resistance:
            setters.append((f"{ch}/initenal_resistance", "1"))
        for path, value in setters:
            self.api.set_device_value(self.device_handle, path, value)

    def _configure_channel_filter_std(self, channel: int) -> None:
        """创芯 ControlCANFD：InitCAN 后配置标准帧全 ID 白名单。"""
        assert self.api is not None
        if self.api.is_controlcanfd and self.channel_handle:
            self.api.configure_channel_filter_std(self.channel_handle)

    def start_channel(
        self,
        channel: int = 0,
        abit_baud: int = 1_000_000,
        dbit_baud: int = 5_000_000,
        resistance: bool = True,
    ) -> None:
        if not self.api or not self.device_handle:
            raise RuntimeError("请先打开设备")

        if self.channel_handle:
            self.stop_channel()

        if channel < 0 or channel >= self.channel_count:
            raise RuntimeError(
                f"该硬件只有 {self.channel_count} 个 CAN 通道"
                f"（{self.hardware_name(self.device_info)}），"
                f"无法启动通道 {channel + 1} (API CAN{channel})。"
                f"请使用通道 1 (API CAN0)。"
            )

        self.channel = channel
        if self.api.is_controlcanfd:
            self._start_channel_controlcanfd(channel, abit_baud, dbit_baud, resistance)
        else:
            self._start_channel_zlgcan(channel, abit_baud, dbit_baud, resistance)

    def _start_channel_controlcanfd(
        self,
        channel: int,
        abit_baud: int,
        dbit_baud: int,
        resistance: bool,
    ) -> None:
        assert self.api is not None
        for ch in range(self.channel_count):
            ch_res = resistance if ch == channel else False
            self._configure_channel_baud(ch, abit_baud, dbit_baud, resistance=ch_res)
        init_cfg = make_canfd_init_cfg()
        ch_handle = self.api.init_can(self.device_handle, channel, init_cfg)
        if not ch_handle:
            raise RuntimeError(f"初始化 CAN{channel} 失败")
        self.channel_handle = ch_handle
        self._configure_channel_filter_std(channel)
        ret = self.api.start_can(self.channel_handle)
        if ret != ZCAN_STATUS_OK:
            self.stop_channel()
            raise RuntimeError(f"启动 CAN{channel} 失败 (StartCAN 返回 {ret})")

    def _start_channel_zlgcan(
        self,
        channel: int,
        abit_baud: int,
        dbit_baud: int,
        resistance: bool,
    ) -> None:
        """ZQWL：InitCAN + SetValue(timing)；部分固件 InitCAN 后已启动，StartCAN 返回 0 仍可用。"""
        assert self.api is not None
        self._configure_channel_zqwl(channel, abit_baud, dbit_baud, resistance)
        init_cfg = make_zqwl_init_cfg()
        self.channel_handle = self.api.init_can(self.device_handle, channel, init_cfg)
        if not self.channel_handle:
            raise RuntimeError(f"初始化 CAN{channel} 失败")
        start_ret = self.api.start_can(self.channel_handle)
        if start_ret != ZCAN_STATUS_OK:
            # InitCAN 成功后内核常已 Task 1 Timer Started；勿 Reset，否则把可用通道关掉
            self.last_error = (
                f"StartCAN 返回 {start_ret}（InitCAN 已成功，通道句柄 {self.channel_handle:#x}，继续运行）"
            )

    def stop_channel(self) -> None:
        if self.api and self.channel_handle:
            self.api.reset_can(self.channel_handle)
            self.channel_handle = 0

    def close_device(self) -> None:
        self.stop_channel()
        if self.api and self.device_handle:
            self.api.close_device(self.device_handle)
        self.api = None
        self.device_handle = 0
        self.device_info = None

    @staticmethod
    def _fill_fd_tx(msg: ZCAN_TransmitFD_Data, arbitration_id: int, data: bytes, is_fd: bool) -> int:
        frame_len = min(len(data), 64)
        msg.transmit_type = 0
        msg.frame.eff = 0
        msg.frame.rtr = 0
        msg.frame.brs = 1 if is_fd else 0
        msg.frame.can_id = arbitration_id & 0x1FFFFFFF
        msg.frame.len = frame_len
        for idx in range(frame_len):
            msg.frame.data[idx] = data[idx]
        return frame_len

    def _transmit_fd_with_retry(self, buf, count: int) -> bool:
        last_ret = 0
        for attempt in range(3):
            last_ret = self.api.transmit_fd(self.channel_handle, buf, count)
            if last_ret > 0:
                return True
            if attempt < 2:
                time.sleep(0.001)
        if last_ret == 0:
            self.last_error = f"ZCAN_TransmitFD 返回 0（count={count}，帧已提交）"
            return True
        self.last_error = f"ZCAN_TransmitFD 失败（count={count}，ret={last_ret}）"
        return False

    def send(self, arbitration_id: int, data: bytes, is_fd: bool = True) -> bool:
        if not self.api or not self.channel_handle:
            self.last_error = "CAN 通道未打开"
            return False
        msg = ZCAN_TransmitFD_Data()
        frame_len = self._fill_fd_tx(msg, arbitration_id, data, is_fd)
        if frame_len <= 0:
            return False
        return self._transmit_fd_with_retry(msg, 1)

    def send_burst(self, frames: list[tuple[int, bytes]], is_fd: bool = True) -> bool:
        """一次 ZCAN_TransmitFD 提交多帧，缩短 ENTER→START 间隔。"""
        if not self.api or not self.channel_handle:
            self.last_error = "CAN 通道未打开"
            return False
        if not frames:
            return True
        count = len(frames)
        buf = (ZCAN_TransmitFD_Data * count)()
        for idx, (arbitration_id, data) in enumerate(frames):
            frame_len = self._fill_fd_tx(buf[idx], arbitration_id, data, is_fd)
            if frame_len <= 0:
                return False
        return self._transmit_fd_with_retry(buf, count)

    def recv(self, timeout: float = 0.1) -> Optional[Tuple[int, bytes, float]]:
        batch = self.recv_batch(max_messages=1, timeout=timeout)
        return batch[0] if batch else None

    def _recv_one_type(
        self,
        frame_type: int,
        max_messages: int,
        timeout_ms: int,
    ) -> List[Tuple[int, bytes, float]]:
        assert self.api is not None
        count = self.api.get_receive_num(self.channel_handle, frame_type)
        if count <= 0:
            return []
        count = min(count, max_messages)
        results: List[Tuple[int, bytes, float]] = []
        if frame_type == ZCAN_TYPE_CAN:
            msgs, ret = self.api.receive_can(self.channel_handle, count, timeout_ms)
            if ret <= 0:
                return []
            for idx in range(ret):
                frame = msgs[idx].frame
                data = bytes(frame.data[: frame.can_dlc])
                ts = msgs[idx].timestamp / 1_000_000.0 if msgs[idx].timestamp else time.time()
                results.append((frame.can_id, data, ts))
        else:
            msgs, ret = self.api.receive_fd(self.channel_handle, count, timeout_ms)
            if ret <= 0:
                return []
            for idx in range(ret):
                frame = msgs[idx].frame
                data = bytes(frame.data[: frame.len])
                ts = msgs[idx].timestamp / 1_000_000.0 if msgs[idx].timestamp else time.time()
                results.append((frame.can_id, data, ts))
        return results

    def _recv_batch_merge(self, max_messages: int, timeout_ms: int) -> List[Tuple[int, bytes, float]]:
        assert self.api is not None
        pending = self.api.get_receive_num(self.device_handle, ZCAN_TYPE_MERGE)
        if pending <= 0:
            return []
        count = min(pending, max_messages)
        objs, ret = self.api.receive_data(self.device_handle, count, timeout_ms)
        if ret <= 0:
            return []
        results: List[Tuple[int, bytes, float]] = []
        for idx in range(ret):
            obj = objs[idx]
            if obj.dataType != ZCAN_DT_ZCAN_CAN_CANFD_DATA:
                continue
            if obj.chnl != self.channel:
                continue
            frame = obj.zcanCANFDData.frame
            data = bytes(frame.data[: frame.len])
            ts = obj.zcanCANFDData.timestamp / 1_000_000.0 if obj.zcanCANFDData.timestamp else time.time()
            results.append((frame.can_id, data, ts))
        return results

    def recv_batch(self, max_messages: int = 64, timeout: float = 0.05) -> List[Tuple[int, bytes, float]]:
        if not self.api or not self.channel_handle:
            return []
        timeout_ms = int(timeout * 1000) if timeout else 0
        if self.use_merge_recv:
            pending = self.api.get_receive_num(self.device_handle, ZCAN_TYPE_MERGE)
            if pending <= 0:
                if timeout <= 0:
                    return []
                time.sleep(min(timeout, 0.001))
                pending = self.api.get_receive_num(self.device_handle, ZCAN_TYPE_MERGE)
                if pending <= 0:
                    return []
            return self._recv_batch_merge(max_messages, timeout_ms)

        pending = max(self.api.get_receive_num(self.channel_handle, ZCAN_TYPE_CAN), 0)
        pending += max(self.api.get_receive_num(self.channel_handle, ZCAN_TYPE_CANFD), 0)
        if pending <= 0:
            if timeout <= 0:
                return []
            time.sleep(min(timeout, 0.001))
            pending = max(self.api.get_receive_num(self.channel_handle, ZCAN_TYPE_CAN), 0)
            pending += max(self.api.get_receive_num(self.channel_handle, ZCAN_TYPE_CANFD), 0)
            if pending <= 0:
                return []
        results: List[Tuple[int, bytes, float]] = []
        for frame_type in (ZCAN_TYPE_CANFD, ZCAN_TYPE_CAN):
            wait = timeout_ms if not results else 0
            chunk = self._recv_one_type(frame_type, max_messages - len(results), wait)
            results.extend(chunk)
            if len(results) >= max_messages:
                break
        return results

    def device_label(self, profile_name: str) -> str:
        return f"{profile_name}  设备{self.device_index}"

    def channel_config_summary(self, abit_baud: int, dbit_baud: int, resistance: bool) -> str:
        return (
            f"CANFD 标准: ISO\n"
            f"工作模式: 正常\n"
            f"仲裁段: {abit_baud // 1000} kbps\n"
            f"数据段: {dbit_baud // 1000} kbps\n"
            f"终端电阻: {'开启' if resistance else '关闭'}\n"
            f"接收模式: 单通道独立接收 (CANFDToolPro 同)\n"
            + (
                f"活动通道: 通道{self.channel + 1} (API CAN{self.channel})\n"
                if self.channel_handle
                else ""
            )
            + f"滤波: 标准帧 0~0x7FF"
        )
