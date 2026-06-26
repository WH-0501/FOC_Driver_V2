# -*- coding: utf-8 -*-
"""周立功 zlgcan.dll ctypes 封装（兼容 ZQWL-UCANFD-100E）。"""

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
    c_ubyte,
    c_uint,
    c_ulonglong,
    c_ushort,
    c_void_p,
    windll,
)
from pathlib import Path
from typing import List, Optional, Tuple

ZCAN_USBCANFD_100U = 42
ZCAN_USBCANFD_200U = 41
ZCAN_USBCANFD_MINI = 43
INVALID_DEVICE_HANDLE = 0
ZCAN_STATUS_OK = 1
ZCAN_TYPE_CANFD = 1

# USBCANFD @ 60MHz, 1Mbps / 5Mbps 常用 timing（与 ZLG 示例一致）
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


class ZCAN_ReceiveFD_Data(Structure):
    _fields_ = [("frame", ZCAN_CANFD_FRAME), ("timestamp", c_ulonglong)]


def _search_dll_dirs() -> List[Path]:
    root = Path(__file__).resolve().parent.parent
    candidates: List[Path] = []
    if getattr(sys, "frozen", False):
        # 单文件 exe 旁的外部 library/（便于不重新打包就更新 DLL）
        candidates.append(Path(sys.executable).resolve().parent / "library")
        # PyInstaller 解压到 _MEIPASS 的内嵌 library/
        meipass = getattr(sys, "_MEIPASS", None)
        if meipass:
            candidates.append(Path(meipass) / "library")
    candidates.extend([
        root / "library",
        Path(r"C:\Program Files (x86)\USB_CANFD TOOL\firmware"),
        Path(r"C:\Program Files (x86)\USB_CANFD TOOL"),
        Path(r"C:\Program Files (x86)\ZCANPRO"),
        Path(r"C:\Program Files\ZCANPRO"),
    ])
    found: List[Path] = []
    seen = set()
    for path in candidates:
        key = str(path)
        if key in seen:
            continue
        seen.add(key)
        if path.is_dir() and (path / "zlgcan.dll").is_file():
            found.append(path)
    return found


def find_zlgcan_dll() -> Tuple[Optional[Path], str]:
    dirs = _search_dll_dirs()
    if dirs:
        return dirs[0] / "zlgcan.dll", ""
    return None, (
        "未找到 zlgcan.dll。请安装 ZCANPRO/USB_CANFD 上位机，"
        "或将 zlgcan.dll 和 kerneldlls 文件夹复制到 motor_controller/library/"
    )


def _handle(value: int) -> c_void_p:
    return c_void_p(value)


class ZlgCanApi:
    def __init__(self, dll_path: Path):
        self._dll_dir = dll_path.parent
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
                    f"请将 64 位 zlgcan.dll 及 kerneldlls 复制到 library/ 目录，"
                    f"或使用 32 位 Python 运行本程序。原始错误: {exc}"
                ) from exc
            raise
        self._bind_functions()

    def _bind_functions(self) -> None:
        dll = self._dll
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

    def open_device(self, device_type: int, device_index: int) -> int:
        handle = self._dll.ZCAN_OpenDevice(c_uint(device_type), c_uint(device_index), 0)
        return 0 if not handle else handle.value if hasattr(handle, "value") else int(handle)

    def close_device(self, device_handle: int) -> int:
        return self._dll.ZCAN_CloseDevice(_handle(device_handle))

    def get_device_info(self, device_handle: int) -> Optional[ZCAN_DEVICE_INFO]:
        info = ZCAN_DEVICE_INFO()
        ret = self._dll.ZCAN_GetDeviceInf(_handle(device_handle), byref(info))
        return info if ret == ZCAN_STATUS_OK else None

    def set_device_value(self, device_handle: int, path: str, value: str) -> bool:
        if not hasattr(self._dll, "ZCAN_SetValue"):
            return False
        ret = self._dll.ZCAN_SetValue(
            _handle(device_handle),
            path.encode("ascii"),
            value.encode("ascii"),
        )
        return ret == ZCAN_STATUS_OK

    def init_can(self, device_handle: int, channel: int, init_cfg: ZCAN_CHANNEL_INIT_CONFIG) -> int:
        handle = self._dll.ZCAN_InitCAN(_handle(device_handle), c_uint(channel), byref(init_cfg))
        return 0 if not handle else handle.value if hasattr(handle, "value") else int(handle)

    def start_can(self, channel_handle: int) -> int:
        return self._dll.ZCAN_StartCAN(_handle(channel_handle))

    def reset_can(self, channel_handle: int) -> int:
        return self._dll.ZCAN_ResetCAN(_handle(channel_handle))

    def get_receive_num(self, channel_handle: int) -> int:
        return self._dll.ZCAN_GetReceiveNum(_handle(channel_handle), ZCAN_TYPE_CANFD)

    def transmit_fd(self, channel_handle: int, msgs, count: int) -> int:
        return self._dll.ZCAN_TransmitFD(_handle(channel_handle), byref(msgs), count)

    def receive_fd(self, channel_handle: int, count: int, timeout_ms: int = 0):
        buffer = (ZCAN_ReceiveFD_Data * count)()
        ret = self._dll.ZCAN_ReceiveFD(_handle(channel_handle), buffer, count, timeout_ms)
        return buffer, ret


class ZlgCanSession:
    def __init__(self) -> None:
        self.api: Optional[ZlgCanApi] = None
        self.device_handle: int = 0
        self.channel_handle: int = 0
        self.device_type: int = ZCAN_USBCANFD_100U
        self.device_index: int = 0
        self.channel: int = 0
        self.device_info: Optional[ZCAN_DEVICE_INFO] = None
        self.last_error: str = ""

    @staticmethod
    def scan(device_type: int = ZCAN_USBCANFD_100U, max_index: int = 4) -> Tuple[List[int], str]:
        dll_path, err = find_zlgcan_dll()
        if dll_path is None:
            return [], err
        try:
            api = ZlgCanApi(dll_path)
        except OSError as exc:
            return [], f"加载 zlgcan.dll 失败: {exc}"

        # 依次尝试常见 CANFD 设备类型
        types_to_try = [device_type, ZCAN_USBCANFD_100U, ZCAN_USBCANFD_200U, ZCAN_USBCANFD_MINI]
        tried = []
        for dtype in types_to_try:
            if dtype in tried:
                continue
            tried.append(dtype)
            found: List[int] = []
            for index in range(max_index):
                handle = api.open_device(dtype, index)
                if handle and handle != INVALID_DEVICE_HANDLE:
                    found.append(index)
                    api.close_device(handle)
            if found:
                return found, ""
        return [], "未检测到在线设备，请确认 USB 已连接、驱动正常，且已关闭原厂上位机"

    def open_device(self, device_type: int, device_index: int) -> None:
        self.close_device()
        dll_path, err = find_zlgcan_dll()
        if dll_path is None:
            raise RuntimeError(err)
        self.api = ZlgCanApi(dll_path)
        self.device_type = device_type
        self.device_index = device_index
        self.device_handle = self.api.open_device(device_type, device_index)
        if not self.device_handle or self.device_handle == INVALID_DEVICE_HANDLE:
            self.api = None
            raise RuntimeError(f"打开设备失败 (索引 {device_index})，请先关闭 USB_CANFD 原厂工具")
        self.device_info = self.api.get_device_info(self.device_handle)

    def _configure_channel(
        self,
        channel: int,
        abit_baud: int,
        dbit_baud: int,
        resistance: bool,
    ) -> None:
        assert self.api is not None
        ch = str(channel)
        # 使用 ZCAN_SetValue，避免 IProperty 函数指针导致 access violation
        setters = [
            (f"{ch}/canfd_standard", "0"),
            (f"{ch}/canfd_abit_baud_rate", str(abit_baud)),
            (f"{ch}/canfd_dbit_baud_rate", str(dbit_baud)),
        ]
        if resistance:
            setters.append((f"{ch}/initenal_resistance", "1"))
        for path, value in setters:
            self.api.set_device_value(self.device_handle, path, value)

    def start_channel(
        self,
        channel: int = 0,
        abit_baud: int = 1_000_000,
        dbit_baud: int = 5_000_000,
        resistance: bool = True,
    ) -> None:
        if not self.api or not self.device_handle:
            raise RuntimeError("请先打开设备")

        self.channel = channel
        self._configure_channel(channel, abit_baud, dbit_baud, resistance)

        init_cfg = ZCAN_CHANNEL_INIT_CONFIG()
        init_cfg.can_type = ZCAN_TYPE_CANFD
        init_cfg.config.canfd.acc_code = 0
        init_cfg.config.canfd.acc_mask = 0xFFFFFFFF
        init_cfg.config.canfd.abit_timing = ABIT_TIMING_1M
        init_cfg.config.canfd.dbit_timing = DBIT_TIMING_5M
        init_cfg.config.canfd.brp = 0
        init_cfg.config.canfd.filter = 0
        init_cfg.config.canfd.mode = 0

        self.channel_handle = self.api.init_can(self.device_handle, channel, init_cfg)
        if not self.channel_handle:
            raise RuntimeError(f"初始化 CAN{channel} 失败")
        if self.api.start_can(self.channel_handle) != ZCAN_STATUS_OK:
            raise RuntimeError(f"启动 CAN{channel} 失败")

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

    def send(self, arbitration_id: int, data: bytes, is_fd: bool = True) -> bool:
        if not self.api or not self.channel_handle:
            return False
        msg = ZCAN_TransmitFD_Data()
        msg.transmit_type = 0
        msg.frame.eff = 0
        msg.frame.rtr = 0
        msg.frame.brs = 1 if is_fd else 0
        msg.frame.can_id = arbitration_id
        msg.frame.len = min(len(data), 64)
        for idx, val in enumerate(data[:64]):
            msg.frame.data[idx] = val
        return self.api.transmit_fd(self.channel_handle, msg, 1) == 1

    def recv(self, timeout: float = 0.1) -> Optional[Tuple[int, bytes, float]]:
        batch = self.recv_batch(max_messages=1, timeout=timeout)
        return batch[0] if batch else None

    def recv_batch(self, max_messages: int = 64, timeout: float = 0.05) -> List[Tuple[int, bytes, float]]:
        """一次 ZCAN_ReceiveFD 取回全部帧，避免只读首帧导致其余被驱动缓冲丢弃。"""
        if not self.api or not self.channel_handle:
            return []
        count = self.api.get_receive_num(self.channel_handle)
        if count <= 0:
            if timeout <= 0:
                return []
            time.sleep(min(timeout, 0.001))
            count = self.api.get_receive_num(self.channel_handle)
            if count <= 0:
                return []
        count = min(count, max_messages)
        timeout_ms = int(timeout * 1000) if timeout else 0
        msgs, ret = self.api.receive_fd(self.channel_handle, count, timeout_ms)
        if ret <= 0:
            return []
        results: List[Tuple[int, bytes, float]] = []
        for idx in range(ret):
            frame = msgs[idx].frame
            data = bytes(frame.data[: frame.len])
            ts = msgs[idx].timestamp / 1_000_000.0 if msgs[idx].timestamp else time.time()
            results.append((frame.can_id, data, ts))
        return results

    def device_label(self, profile_name: str) -> str:
        return f"{profile_name}  设备{self.device_index}"
