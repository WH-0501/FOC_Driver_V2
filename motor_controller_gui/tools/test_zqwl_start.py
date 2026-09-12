# -*- coding: utf-8 -*-
"""ZQWL-UCANFD-100E 启动诊断（命令行）。"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
os.chdir(ROOT)

from canbus.zlg_native import ZCAN_USBCANFD_100U, ZlgCanSession, ZLG_CAN_DLL, find_can_driver_dll


def main() -> int:
    dll_path, err, backend = find_can_driver_dll(ZLG_CAN_DLL)
    print(f"DLL: {dll_path} backend={backend}")
    if dll_path is None:
        print(err)
        return 1

    session = ZlgCanSession()
    try:
        session.open_device(ZCAN_USBCANFD_100U, 0, channel_count=1, driver_dll=ZLG_CAN_DLL)
        print(f"Open OK: dev_handle={session.device_handle:#x} type={session.device_type}")
        info = session.device_info
        if info:
            print(f"  hw={session.hardware_name(info)!r} can_Num={info.can_Num}")
        session.start_channel(0)
        print(f"Start OK: ch_handle={session.channel_handle:#x}")
        session.stop_channel()
        session.close_device()
        return 0
    except Exception as exc:
        print(f"FAIL: {exc}")
        if session.channel_handle:
            print(f"  channel_handle={session.channel_handle:#x}")
        if session.device_handle:
            print(f"  dev_handle={session.device_handle:#x}")
        try:
            session.close_device()
        except Exception:
            pass
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
