# 扫频 RAM、实时开销与功能隔离（2026-11-20）

本次在上一版设备扫频基础上优化，未烧录或运行电机。保留 16 个频点、两种测量方法、原稳定时间/平均段数、float32 采集统计及既有保护。算法、FOC 适配器、协议仍分为三个模块，不借用其他控制功能的 RAM，也没有引入堆分配。

## RAM 实际占用

同一工程、ARM Compiler 5.06u7、Cortex-M4F、O2/time 和原 `cst-profile.sct`，全部重新编译链接：

| 项目 | 优化前，开启扫频 | 优化后，诊断固件 | 优化后，默认固件 |
|---|---:|---:|---:|
| `FOC_SWEEP_ENABLED` | 1 | 1 | 0 |
| 主 RW 区大小 / 上限 | `0x37A0 / 0x3830` | `0x35D0 / 0x3830` | `0x3288 / 0x3830` |
| 主 RW 区余量 | **144 B** | **608 B** | **1448 B** |
| 全部 RW+ZI（含原堆/栈） | 14280 B | 13816 B | 12976 B |
| ROM 总量 | 49508 B | 54600 B | 42340 B |
| SRAM 调制代码区 / 上限 | `0x43C / 0x600` | `0x410 / 0x600` | `0x410 / 0x600` |

608 B 为固定布局下可继续增加的主区静态数据余量，原 4 KiB 栈和 512 B 堆已计入占用；不是栈剩余空间。没有缩小堆/栈或修改 SRAM 分区。诊断固件相对原实现释放 **464 B** 主 RW 空间，仍应在后续增加功能时复查链接报告。

| 扫频内部 | 优化前 | 优化后 |
|---|---:|---:|
| 单频点结构 | 76 B | 48 B |
| 算法上下文（16 点） | 1272 B | 796 B |
| 适配器其他静态存储（含编译器对齐） | 28 B | 44 B |
| 适配器对象文件 RW+ZI 合计 | 1300 B | 840 B |

算法上下文减少 476 B（37.4%）。新增主动测试时序记录占 20 B；字段布局及对象/区域对齐使整个主区净减少 464 B，不能简单用上下文差值替代链接结果。默认固件不链接扫频上下文。

具体措施：

- 相同周期参考的复数谱 X 在前台预计算一次，移除每段重复的输入投影、Sxx 和互谱累积字段。X 来自**实际 float32 递推波形**，包括有限精度和跨谱线泄漏，不用理想正弦幅值代替测量分母。
- 只在 ISR 累积反馈 Y、ΣY、Σ|Y|²。相同 X 时，`H1=ΣY·conj(X)/(N·|X|²)`，`γ²=|ΣY|²/(N·Σ|Y|²)`，与原多段 H1/相干性定义等价。保存未归一化的 Y，归一化、除法和开方放到前台结果读取。
- 初相位移到 840 B 只读 Flash 表；频率由整数 bin/周期导出；计数器按经验证的最大范围收窄到 uint16/uint8，信号与统计仍用 float32。
- 结果读取采用 double 中间运算以避免大数平方/相除引入额外精度损失，再返回原 float32 协议字段。M4F 不支持硬件 double，因此仅允许其在前台执行。Flash 比旧诊断固件增加约 5 KiB，是显式的 RAM/精度取舍。

## 两层开启，普通控制路径隔离

`project/APP/FOC/sweep/foc_sweep.h` 默认 `FOC_SWEEP_ENABLED=0`。默认构建没有扫频静态数据、ISR 钩子或后台激励。仅保留轻量协议“不支持”应答：error=3，避免主机等待超时；HMI 将提示需要启用扫频的诊断固件。

需要测量时，在 **Keil 工程全局 C/C++ Define** 添加 `FOC_SWEEP_ENABLED=1` 并全量重编译，或使用构建脚本 `--enable-sweep`。不能只给单个源文件定义宏。开启编译后仍默认为空闲，只有合法 `0xBC` 请求才发布测试 owner：action=1 多正弦，action=2 逐频，action=0 停止。`0xBD` 查询不会启动或续租测试。

FOC 只在感测、保护、FSM 之后的控制分发边界检查一次 owner。未测试时调用原 `foc_control_run`；测试时调用独立适配器，经过原电流限幅/软限速，再调用同一个 `foc_current`。普通 PID、调制及参考生成函数里不再放扫频检查。

电压限制由原控制器/调制器执行，适配器在调用后检查 d/q PID 输出、积分和相同的圆形电压边界。一旦限幅，立即失能并使整次结果无效，包含最后采样点。与旧调制前钩子相比，限制可能已由原调制器在当拍施加，再被适配器判为无效；没有绕过原保护，也没有把限幅数据发布为正常完成。

失能、退出、故障、竞争电流命令、IAP 的撤销通知仍保留，属于生命周期边界。未占有测试控制权时提前返回，不再屏蔽中断。不会自动使能、清故障、恢复旧非零参考或改写 PID。

## FOC 中断耗时：静态证据与实测边界

已对实际 ARM 对象文件的指令字节和重定位目标进行比较：默认/诊断两种构建的 `foc_current`（128 B）、`foc_apply_voltage`（184 B）、`foc_control_run`（984 B）及 `foc_modulate_voltage_sincos`（816 B）完全相同。默认构建的 `foc.o` 无扫频符号引用。

诊断固件闲置时，在分发处增加 owner 检查，当前 ARM 输出为 `LDR / LDR / CMP / BNE`，不会调用扫频算法、读额外 DWT 或做浮点计算。分支布局、Flash 等待和总线会影响实际耗时，**四条指令不等于四个周期**，不据此虚构微秒数。

活动测试期间：

- 每个有效采样、每个频点减少两次输入投影乘加；16 点多正弦每拍少 32 次乘法和对应加减。
- 段结束不再做 X 的归一化及互谱乘加；相位复位通过 Flash 表。逐频切换的周期长度/稳定长度提前计算。
- ARM 反汇编确认 `sweep_engine_step` 与 `reset_phase` 无三角函数、除法、软件 double 或 libm 调用。适配器的固定周期阈值在此目标上为编译常量。
- 增加的 DWT 分段计时仅在活动测试执行。保护检查、函数调用和 Flash 表读取仍有成本，所以不能仅凭算术次数减少推断整个 ISR 最坏耗时降低多少。

### 可读出的计时数据

活动诊断状态保持 v1 前 20 字节，扩展为 40 字节有效负载（CAN-FD 可尾部填充）。`0xBC/0xBD` 共用：

| 偏移 | 字段 | 单位/意义 |
|---|---|---|
| 18 | `max_us:u16` | FOC 回调开始至测试保护检查后的最大时间 |
| 20/22/24 | `interval_min/max/last:u16` | 相邻回调开始间隔，μs |
| 26/27 | `reason:u8, magic:u8=0xA5` | 0 无时序错误；1 周期间隔；2 主体超时 |
| 28 | `failure_sample:u32` | 首次时序中止时已采样数量 |
| 32/34/36 | `sense/control/engine_max:u16` | 感测 / 测试控制分支 / 算法 step 的最大 μs |
| 38/39 | `magic:u8=0xA6, revision:u8=1` | 与现有 HMI 扩展解码兼容 |

control 包含 engine 和电流控制器；它们的峰值可能出现在不同拍，不能把各峰值相加作为总 ISR。sense 复用原 FOC 感测计时。max_us 不包含整个 ISR 入口/退出、后续 publish/finish 等尾部工作；中止阈值也不能避免已经发生的超时。20 kHz 下仍按 40–60 μs 间隔和 45 μs 主体阈值中止，未放宽阈值掩盖问题。

板上应先用同一固件、同一感测/遥测配置比较空闲与活动状态，再测试 16 点多正弦段结束拍、逐频切换拍以及停止拍；读取上述扩展，必要时用 GPIO/示波器覆盖完整 ISR 并检查栈高水位。当前没有硬件实测，不能据软件测试声称满足最坏执行时间预算。

## 回归结果与复现

新旧算法以同一 MSVC/O2 环境、相同输入逐样本比较：3–16 点、10/20/40 kHz、两种方法共 **84 组**，含 0.04 A 偏置和确定性噪声，幅值覆盖 0.005/0.5 A。激励序列最大差 **0 A**，增益最大差 **2.10e-6 dB**，相位最大差 **5.91e-6 度**，相干性最大差 **3.58e-7**。这是主机数值回归，不是 ARM libm 逐位相同或真实传感器精度的承诺。

原 800 Hz 一阶离散模型精度测试通过；实际分发/适配器集成、默认关闭无副作用、最后一拍拒绝、取消/限幅/故障、时序中止、协议、原控制模式和 ACK 回归均通过。Qt `sweep_tests` 与 `operation_ui_tests` 2/2 通过，新增 error=3 和 40 字节扩展/填充解码测试。

在仓库根目录执行（旧版源目录需指向本次修改前的完整 motor_controller 快照）：

```powershell
python motor_controller/tests/run_sweep_tests.py --build-dir build-sweep-native --msvc
python motor_controller/tests/run_control_mode_tests.py --build-dir build-control
python motor_controller/tests/run_command_ack_tests.py --build-dir build-ack
python motor_controller/tests/build_sweep_firmware.py --build-dir build-default
python motor_controller/tests/build_sweep_firmware.py --build-dir build-diagnostic --enable-sweep
python motor_controller/tests/check_sweep_isolation.py --default-build build-default --enabled-build build-diagnostic
python motor_controller/tests/compare_sweep_engines.py --baseline-root <旧版motor_controller目录> --build-dir build-compare
```

本次构建产物和对比 JSON 保存于工作区 `foc-sweep-opt/build-*`，没有覆盖仓库既有 HEX，也未安装新的 MotorTune 可执行程序。此前 2026-11-19 的 [验证记录](SWEEP_VALIDATION.md) 保留作优化前基线；当前实现与协议以 [SWEEP.md](SWEEP.md) 和本文为准。
