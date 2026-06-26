#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电机调试上位机入口。

模块结构:
  protocol/  - CANFD 协议常量与编解码
  can/       - CAN 通信线程
  core/      - 高层 MotorClient API
  ui/        - PyQt5 界面组件

依赖: PyQt5, python-can
"""

from ui.main_window import run

if __name__ == "__main__":
    run()
