# 电机调试上位机 (CANFD)

基于 CAN/CANFD 协议的 FOC 电机调试 GUI，支持 ZQWL-UCANFD-100E、创芯科技 USBCAN-FD 等周立功 `zlgcan.dll` 兼容 CAN 盒，以及 SLCAN / PCAN 适配器。

## 环境要求

- Python 3.8+
- Windows 10/11（64 位 Python 需配 64 位 `zlgcan.dll`）
- USB-CANFD 驱动（ZCANPRO、USB_CANFD Tool 或创芯 CANFDToolPro）

## 安装

```bash
cd motor_controller
pip install -r requirements.txt
```

## 启动

```bash
python motor_gui.py
```

## 使用流程

### 1. 连接 CAN 设备

1. 选择 **CAN 卡型号**（创芯选「创芯科技 USBCAN-FD (200U)」，ZQWL 选「ZQWL-UCANFD-100E」）
2. 点击 **扫描设备**，选择正确 **索引**
3. 点击 **打开设备**
4. 点击 **CAN0 启动**

> 使用前请关闭原厂 CANFDToolPro / USB_CANFD / ZCANPRO 上位机，避免占用设备。

**驱动文件：** 将 64 位 `zlgcan.dll` 及 `kerneldlls` 文件夹放到 `library/` 目录。

| 型号 | 说明 |
|------|------|
| ZQWL-UCANFD-100E | 周立功 USBCANFD-100U（类型 42），单通道 |
| 创芯科技 USBCAN-FD (200U) | CANFDToolPro 型号 `USBCANFD` / `USBCANFD-200U`（类型 41），双通道 |

**与 CANFDToolPro 通道参数对应关系（本上位机内置，无需手动设置）：**

| CANFDToolPro 参数 | 本上位机设置 |
|-------------------|-------------|
| CAN 卡型号 USBCANFD | 设备类型 41（200U） |
| 索引 0 | 扫描/打开时选择索引 0 |
| 工作模式：正常 | `InitCAN.mode = 0` |
| CANFD 标准：ISO | `canfd_standard = 0` |
| 仲裁域 1 Mbps (80%) | `canfd_abit_baud_rate = 1000000` |
| 数据域 5 Mbps (75%) | `canfd_dbit_baud_rate = 5000000` |
| 启用滤波：未勾选 | 不配置滤波；双通道 USBCANFD 使用合并接收 |
| 终端电阻 | 默认开启（`initenal_resistance = 1`） |

**通道如何选择：**

| CANFDToolPro | 本上位机下拉框 | API 索引 |
|--------------|----------------|----------|
| 通道 1 | 通道1 (API CAN0) | 0 |
| 通道 2 | 通道2 (API CAN1) | 1 |

打开设备后，在「**CAN 通道**」下拉框选择电机所接物理口，再点 **启动**。单通道 ZQWL 仅显示通道 1。
| SLCAN 适配器 | CANable 等，需选择串口 |
| PCAN-USB | Peak CAN 接口 |

### 2. 添加电机

- 在 **电机管理** 中输入 ID，点击 **添加电机**
- 或点击 **扫描在线电机 (1-21)** 自动发现

### 3. 波形展示（中间主区域）

实时显示当前选中电机的曲线，可通过 **显示曲线** 区域勾选：

| 分组 | 曲线 |
|------|------|
| 位置 | 目标位置、实际位置 |
| 速度 | 目标速度、实际速度 |
| 电流 | 目标 Iq、Iq、Id、Ia、Ib、Ic（单位 mA） |

默认显示：实际位置、实际速度、Iq。支持 **全选 / 全不选 / 默认** 快捷按钮。

> 三相电流及 Id 需**新版固件**周期上报（CAN FD DLC=20）。旧固件（DLC=12，仅 8 字节有效数据）仅显示 Iq，Id/Ia/Ib/Ic 显示为 `--`。

**操作：**

| 操作 | 功能 |
|------|------|
| 滚轮 | 水平缩放（时间窗口） |
| Ctrl + 滚轮 | 垂直缩放 |
| 暂停 / 恢复 | 停止/继续采集 |
| 清空 | 清除当前电机波形 |
| 重置缩放 | 恢复默认 6 秒窗口 |
| 点击图例 | 显示/隐藏对应曲线 |

数据来自固件 1ms 周期上报；发送目标位置/速度/电流指令后，目标曲线同步更新。

### 4. 运动控制（右侧面板）

- **广播使能/失能**、**设置模式**（PP/CSP/CST/HM/CSV）
- 发送 **目标位置(°)**、**目标速度(RPM)**、**目标电流(mA)**
- **设原点**、**电角度对齐**、**同步信号**

### 5. 参数配置

- 位置/速度/电流环 PID
- 刚度阻尼、保护参数
- 保存参数 / 恢复出厂 / 设置 Node-ID

### 6. 多电机批量

在 **多电机批量** 标签页，表格勾选多个电机后广播下发位置/速度/电流。

## 协议说明

- 帧格式：`[CNT, CMD, DATA...]`，Node-ID 在 CAN 仲裁段
- 周期上报：位置(Q7)、速度、Iq(mA)、状态、错误码；新版另含 Id/Ia/Ib/Ic（见下表）

| 固件版本 | CAN FD DLC | 有效 payload | 字段 |
|----------|------------|--------------|------|
| 旧版 | 12 | 8B (+0 填充) | pos, vel, iq, status, error |
| 新版 | 20 | 16B (+0 填充) | pos, vel, iq, id, ia, ib, ic, status, error |

上位机按 payload 长度与尾部填充自动识别格式，无需手动切换。
- 默认 CAN FD：仲裁段 1Mbps，数据段 5Mbps

## 目录结构

```
motor_controller/
├── motor_gui.py          # 入口
├── library/              # zlgcan.dll 及 kerneldlls
├── protocol/             # 协议编解码
├── canbus/               # CAN 设备管理
├── core/                 # MotorClient、波形缓冲
└── ui/                   # 界面组件
```

## 常见问题

**扫描不到设备**
- 确认 DLL 位数与 Python 一致（64 位对 64 位）
- 关闭原厂上位机
- 检查 USB 连接与驱动

**打开成功但无法启动通道**
- 确保先 **打开设备** 再 **启动 CAN0**
- 检查 `kerneldlls` 是否完整

**波形不更新**
- 确认 CAN 通道已启动
- 勾选 **数据采集**
- 电机需在线并周期上报

## 依赖

- PyQt5
- python-can
- pyserial
- pyqtgraph
- numpy

## 打包

打包前确认 `library/` 内已有 64 位 `zlgcan.dll` 及完整 `kerneldlls/`。

```shell
# 方式一（推荐）：使用 spec，已配置 library 数据文件
pyinstaller motor_gui.spec

# 方式二：命令行需显式带上 library，否则 exe 内找不到 zlgcan.dll
pyinstaller -F --noconsole --add-data "library;library" motor_gui.py
```

可选：下载 [UPX](https://github.com/upx/upx/releases)，将 `upx.exe` 放到与 `pyinstaller.exe` 同级目录以减小体积。

产物在 `dist/motor_gui.exe`。若仍报 DLL 错误，可将 `library/` 文件夹放在 exe 同目录作为备用。


## IAP 固件升级

遵循 `motor_controller/docs/CAN-FD通信协议.md` 与 dex-con `iap.c`：

1. **FD** 读固件版本
2. **F8 ENTER**（Node-ID，无应答，state: IDLE→READY）
3. **F9 START**（ASIX 32B 头 + 首段数据，载荷 >32B；从机**同步擦除** Flash）
4. **等待 FC**（`state=2 接收中, error=0x80`；iap.c 注释「应答后才开始下一帧接收」）
5. **FA DATA**（帧计数从 2 递增，62B/帧，末帧 0xFF 补齐）→ **FB END** → **FE VERIFY** → **FF RESET**

注意：
- 固件 `IAP_TIMEOUT=3000ms` **从 START 进入起算且含擦除时间**；FC 到达后剩余窗口通常仅数百 ms，须**立即连发 DATA**。
- ENTER+START 使用 ZCAN 批量提交；发送/接收分锁，避免周期上报阻塞 IAP。
- IAP 关键帧日志异步输出，避免 Qt 阻塞 CAN。
