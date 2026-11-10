# 兼容矩阵与协议依据

连接界面提供自动探测原生 ZLG/创芯后端，失败时汇总每个厂商的具体错误，不会静默切换 Demo。

## 设备

| 设备/后端 | 实现 | 当前验证 |
|---|---|---|
| ZQWL-UCANFD-100E，type 42 | C++ zlgcan.dll，保留 timing 101166 / 4260362，StartCAN=0 兼容旧行为 | 编译/ABI 结构尺寸检查；待实物 |
| 周立功 USBCANFD-200U，type 41 | 同上，双通道选择 | 待实物 |
| 创芯 USBCAN-FD 100E / 200U | C++ ControlCANFD.dll，独立波特率/ISO API、过滤器配置 | 待实物 |
| SLCAN、PCAN 经典 CAN、PCAN FD | python-can 隔离进程 | 进程/依赖检查；待实物 |
| SocketCAN 经典/FD | 保留 Python 兼容后端，需 Linux | 当前 Windows 环境未运行 |
| Virtual CAN-FD | python-can 虚拟总线 | 无硬件进程测试 |
| Demo | C++ 仿真，1 kHz 周期数据 | 自动化生命周期及状态测试 |

原始 `canbus/device_manager.py`、`device_profiles.py`、`zlg_native.py` 按字节复制到 compat/canbus；不 import 原始目录。桥接层为 python-can FD TX 补 BRS。经典 CAN 的 8B 限制显式拒绝；不会把 20/64B 伪装成经典帧。发布保留原有驱动目录及其依赖，相同厂商目录结构不能只拿走顶层 DLL。

## 协议

- V1：PP/CSP/CST/HM/CSV/MIT 线值 0..5，位置为度 Q7；应用统一转换 rad。使能高半字节必须等于 1。
- V2：线值 0..12，位置为 rad Q7；使能高半字节非零。兼容旧 8B 和扩展 16/18B 周期载荷，缺失通道显示缺失值。
- 两版 PID 均 Q12，电流环 Ki 配置值直接编码，不重复乘 10000。
- 兼容 SET PID 自身应答以及当前固件 GET PID 编号回读。位置/速度/电流分别映射 42→47、43→48、44→49，柔顺 45→4A，保护 46→4B。
- 0x4D/4E/54/AC：布尔结果 1 成功。0x39/40/41/4F/AB：error_t 0 成功。不能合并两类结果。
- 0x50 当前固件没有专用应答，ACK 只表示分发，需回读位置验证，不能凭 ACK 报成功。
- 0xAB done 标志为 bit3；0x53 aligned 为 bit0。
- 保留 B1/B2 电角动态验证、B5/B6 限速、B8/ B9/BA/BB PID 调试遥测入口与字段。
- 升级依据仓库 `iap_upgrade.c` 当前 ASIX 整包实现：FileLen BE@12，checksum LE@20，ImageType LE@4=3，62B 数据块，0x80 成功码。协议版本与 IAP 状态格式独立；不自动猜测旧 Bootloader 的刷写方式。

需求文档中的齿槽 0xAE、HoldStall 0x53/54、写机型 0x51 与实际固件冲突，已禁止这些映射。协议依据优先顺序：当前固件 → 旧 GUI 实现及测试 → 需求清单。
