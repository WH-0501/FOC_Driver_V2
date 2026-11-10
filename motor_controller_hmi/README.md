# MotorTune · Qt 6 原生 FOC 调试工作台

Windows x64 / C++20 / Qt 6 Widgets。研发与产线双入口，默认协议 V2.0.0。
旧版 `motor_controller_gui` 不参与运行，也没有被修改。`DESIGN.md` 和原始需求文件保留原文。

## 运行

解压发布包后运行 `MotorTune.exe`。连接设置选 Demo 可在无硬件环境使用。

```powershell
.\MotorTune.exe --demo
.\MotorTune.exe --factory
.\MotorTune.exe --factory-locked
```

也可运行 `Install.cmd` 启动安装向导：选择目录、研发/产线角色、产线锁定、桌面快捷方式。配置和 SQLite 位于用户本地应用数据目录，不随覆盖安装删除。产线锁定是工作流限制，不是操作系统权限边界。

## 编译与验证

本次构建环境：Qt 6.12.0 mingw_64 / MinGW 13.1 / CMake / Ninja。

```powershell
.\scripts\build.ps1 -QtRoot C:\Qt\6.12.0\mingw_64 -CompilerRoot C:\Qt\Tools\mingw1310_64
```

脚本配置 Release、编译、运行 CTest、部署 Qt 运行时，输出 `release/`。
`core_tests` 覆盖协议线格式、实际固件成功语义、V1/V2 遥测、IAP 文件校验、触发/故障窗口、FFT、SQLite 版本历史、Demo 安全门禁与采样吞吐。`demo_smoke` 验证原生窗口启动、采集、重绘、停止及退出。

## 操作要点

- 先选择协议、设备、通道，连接后扫描节点。设置模式并等遥测确认后使能。
- 示波器：滚轮缩放，拖动平移，Shift 拖动框选；T1/T2 拖动读点；右键通道修改颜色/别名；独立实时、触发和故障缓冲；CSV 原始数据导出。
- PID 输入不会自动下发。点击写入后处理固件的专用回读；Flash 另行保存。
- 全局 `Ctrl+Shift+E` 请求广播失能和异常复位。软件急停不代替断电保护。
- 日志有界保存 TX/RX 和结果，0x7E ACK 不作为操作成功依据。
- IAP 接受经头、长度和 checksum32 验证的 ASIX `.efw`，默认当前 Node 单播；使用现有固件的 0x80 成功状态格式。
- 产线登记 SN、按步骤标定、保存后自动新增归档版本；再执行测试及人工掉电回读。Demo 记录明确标识模拟。

## 已知边界

这份交付是可编译、可运行的原生实现，但不等于全部需求已经真机验收。当前固件没有写机型、HoldStall、齿槽、Rs/Ls/磁链辨识的明确 CAN 入口，实际产线流程会在缺失步骤停住。31 项测试全部登记，未实现或不宜自动执行的项目会 SKIP，报告判 INCOMPLETE，不伪造通过。

PCAN / SocketCAN / SLCAN / Virtual 通过随包隔离 Python 兼容进程运行；Windows 包含 CPython 及固定依赖。Linux 需要安装对应依赖并重新编译。原生 ZLG/创芯后端不依赖 Python。

完整实现范围、固件冲突、尚需完成的工程项与硬件验收清单见 [docs/ACCEPTANCE.md](docs/ACCEPTANCE.md)；架构见 [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)；兼容说明见 [docs/COMPATIBILITY.md](docs/COMPATIBILITY.md)。
