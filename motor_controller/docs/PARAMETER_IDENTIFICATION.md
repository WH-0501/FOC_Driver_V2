# 电机参数辨识（Rs / Ld / Lq / 永磁磁链）

当前实现为显式触发的离线辨识，使用已有编码器、电流采样和 PWM 调制链。正常使能不会自动辨识。支持单独 Rs/Ld/Lq、单独磁链、完整流程三种模式；Qt 版 MotorTune 已同步增加辨识界面，参见 [上位机操作说明](../../motor_controller_hmi/docs/PARAMETER_IDENTIFICATION.md)。

## 前提与范围

- 三相 PMSM，已知极对数，电流采样方向/增益正确，电流零偏校准完成。
- 本次上电后必须先完成电角对齐，使 `motor_runtime(m)->electrical_align_done` 为真。仅在存储中存在 offset 不满足此条件。
- 从 IDLE、功率级未使能且无故障状态启动。与 KTH71 自校准、扫频、正常运行互斥。
- 磁链阶段需要空载自由旋转。速度单位始终是**电机轴 rad/s**，不乘减速比；例如 20 rad/s 约为 191 rpm。
- Rs/Ld/Lq 阶段仍会施加电压，q 轴交流注入会产生振荡转矩。转子位移超过 0.12 电角 rad，或轴速超过 1 rad/s 时退出，不会继续使用移动后的数据。
- 本实现不自动识别极对数、编码器方向、惯量 J、摩擦或齿槽转矩。`MC_CAL_SEQUENCE`、编码器与抗齿槽的旧占位入口返回 `ERR_NOT_IMPL`，不会隐式串入旋转。

## 流程和计算

1. **Rs**：d 轴电流依次为 `current_a / 2`、`current_a`。无须已知 Rs/L/PID，使用限幅电压积分器建立电流；每级等待 0.8 s，随后平均 0.2 s，计算 `Rs = (Ud2-Ud1)/(Id2-Id1)`。同方向两电流点可减小固定导通压降造成的偏差；电流跟踪误差和波动超过门限即失败。
2. **Ld、Lq**：分别向对应轴注入正弦电压，等待 32 个整周期，测量 64 个整周期。电压与电流均取基波幅值，计算 `L = sqrt((U1/I1)^2-Rs^2)/(2πf)`。实际频率为 `fs/round(fs/f_requested)`；注入幅值不超过 `min(injection_v, 0.5*Rs*current_a)`。要求电流基波超过 5 mA、电抗可分辨，并排除过大非基波成分。幅值法不依赖 PWM/ADC 固定相移，但不能消除实际采样和功率级的幅频误差。
3. **磁链**：使用辨识得到的 Rs/Ld/Lq 构建独立的限流电流调节器，轴速给定在 2 s 内爬升；速度稳定后积分 1 s。计算 `ψf = [∫(Uq-Rs*Iq-ωe*Ld*Id)dt - Lq*ΔIq] / ∫ωe dt`，其中 `ωe = pole_pairs*ωshaft`。总流程最多 30 s，旋转建立阶段最多 8 s；转速不稳定、限压或堵转均拒绝结果。

输出为等效相电阻 Ω、dq 电感 H、永磁磁链 Wb，采用与现有 Park 变换一致的相峰值 dq 约定。对于表贴式、低凸极电机，可用 `(Ld+Lq)/2` 作为 Ls；有凸极性时应保留独立 Ld/Lq。

电压由**调制后的占空比差值和母线电压**重构，再转换到转子 dq 坐标；没有直接用未经限幅的 PID 请求作为辨识电压。它仍是估算的端电压，尚未补偿死区、MOSFET/驱动器导通电阻、线缆电阻和电流传感器增益误差。低阻电机尤其需要台架对照，不能把仿真精度当成实机精度。

公式背景：[Microchip PMSM dq 电压方程](https://onlinedocs.microchip.com/oxy/GUID-AAAF8F97-D139-4A93-A154-AA3C6EE6BB22-en-US-1/GUID-007091E0-0021-4359-A50B-127AF65C0B1C.html)。

## 固件 API

头文件：`project/APP/FOC/identification/foc_identification.h`。

```c
motor_handle_t *m = motor_get_handler();
ident_config_t cfg;
foc_ident_status_t status;

foc_ident_defaults(m, &cfg);
/* 按电机实际能力设置电流、限压、转速及跳闸阈值。 */
error_t err = foc_ident_start(m, IDENT_ALL, &cfg);

/* 前台轮询，不等待阻塞辨识；控制中断自动推进流程。 */
foc_ident_status(&status);
/* status.stage == IDENT_DONE 且 status.result.valid == 3 时，检查结果后： */
err = foc_ident_apply(m);  /* 仅更新 RAM 中的 config.param */
/* 需要掉电保存时再执行原 motor_config_save(&m->config) / CAN 0x4D。 */

foc_ident_stop(m);        /* 可随时请求停止；无活动辨识时无副作用 */
```

默认设置仅作为起始值：0.3 A 测试电流、3 V 电压上限、0.2 V 注入峰值、200 Hz 注入、20 rad/s 目标轴速、1 A 电流跳闸、30 rad/s 轴速跳闸。**默认值仍必须通过现有配置限值校验**，不满足时返回错误，不会扩大保护阈值。

`mc_start_calibration(MC_CAL_RSLS/MC_CAL_FLUX)` 也会调用该实现。先单独 Rs/Ld/Lq，再单独磁链时，需要先应用前一轮结果；`IDENT_ALL` 在同一轮内部直接使用新辨识的 Rs/Ld/Lq，无须中途应用。

辨识期间原配置与正常运行 PID 不变。只有完成并显式应用后才更新参数，不自动重整定正常运行 PID。若从启动到应用期间电机本体配置被改变，则拒绝应用，以免混用旧配置下的结果。

## CAN-FD v1

沿用 `[序号, 命令, payload...]` 帧头，以下偏移均相对 **payload**。所有新命令只接受节点单播，忽略广播。

| 命令 | 功能 | 请求 payload |
|---|---|---|
| `0xC0` | 启动/停止 | 至少 2 字节：version=1、action；启动至少 16 字节，见下表 |
| `0xC1` | 查询阶段、错误及结果 | version=1 |
| `0xC2` | 应用成功结果到 RAM | version=1；只能 IDLE、未使能、无锁存故障时执行 |

`0xC0` action：0=停止，1=Rs/Ld/Lq，2=磁链，3=完整流程。停止只需 `[1,0]`；启动的 7 个参数都必须提供。允许 CAN-FD DLC 尾部填充。

| 偏移 | 类型 | 含义 |
|---|---|---|
| 0 | u8 | version=1 |
| 1 | u8 | action |
| 2 | u16 BE | 测试电流 mA |
| 4 | u16 BE | 电压上限 mV |
| 6 | u16 BE | 注入峰值 mV |
| 8 | u16 BE | 注入频率 Hz |
| 10 | u16 BE | 目标轴速，0.01 rad/s |
| 12 | u16 BE | 瞬时电流跳闸 mA |
| 14 | u16 BE | 轴速跳闸，0.01 rad/s |

例如完整流程默认参数 payload：

```text
01 03 01 2C 0B B8 00 C8 00 C8 07 D0 03 E8 0B B8
```

三个命令均返回 28 字节 payload（加 2 字节帧头后填充至 CAN-FD 32 字节）：

| 偏移 | 类型 | 含义 |
|---|---|---|
| 0 | u8 | version=1 |
| 1 | u8 | 此次 API 调用结果 `error_t`；0 表示请求成功，不代表辨识已经完成 |
| 2 | u8 | 阶段：0 IDLE、1 RS_LOW、2 RS_HIGH、3 LD、4 LQ、5 SPIN、6 FLUX_SAMPLE、7 DONE、8 FAILED、9 CANCELLED |
| 3 | u8 | 辨识错误：0 OK、1 BAD_CONFIG、2 CURRENT、3 MOTION、4 TIMEOUT、5 SIGNAL、6 NONFINITE、7 VOLTAGE、8 EXTERNAL、9 TIMING |
| 4 | u8 | 有效位：bit0=Rs/Ld/Lq，bit1=磁链；中途、失败、取消均为 0 |
| 5..7 | u8[3] | 保留 0 |
| 8 | u32 BE | 已执行的辨识采样时间 ms |
| 12 | float32 BE | Rs，Ω |
| 16 | float32 BE | Ld，H |
| 20 | float32 BE | Lq，H |
| 24 | float32 BE | ψf，Wb |

原 `0x7E` 仍仅确认命令分发，完成状态应轮询 `0xC1`。建议 10–20 Hz 查询。通信没有新增主机心跳条件；主机离线后会在流程完成或超时后退出。可使用 `0xC0 [1,0]` 或原失能命令停止。

## 保护和资源

- 采样中断检查相电流/dq 幅值、静止位移、轴速、非有限值、母线电压裕量和周期偏差；DWT 先按无符号周期数求差再换算时间，正确跨越 CYCCNT 回绕。
- 保留现有硬件故障、过温、母线及软件保护路径。若配置了不适用于静止电流分布的缺相检测阈值，可能提前触发现有保护，需结合实机电流波形确认原因。
- 完成、取消、失败、外部 disarm/shutdown 都撤销辨识的 PWM 控制权并关闭功率级；故障状态保持锁存。成功结束也不恢复原运行模式的输出。
- 启动时驱动唤醒在前台执行，避免将毫秒唤醒延时放进 ADC ISR；结果快照、停止和应用使用短临界区。
- 参数写入/保存入口改为要求 IDLE 且硬件未使能，避免把标定状态当作“已失能”。进入 IAP 时先停止辨识。
- 没有扩展 `motor_config_t` 或更改 Flash 格式、Bootloader 和分区。共享标定三角函数实现以减少代码重复，正常控制仍保留内联计算。

## 验证与实机步骤

2026-11-27 软件验证：

- `tests/run_identification_tests.py`：真实 C 引擎的连续 dq 电机模型、独立 Ld/Lq、PWM 更新延迟、磁链、过流、NaN、位移、无信号和堵转；真实适配器/FSM/协议的拒绝启动、停止、故障、时序、母线、PWM 电压重构、查询及显式应用均通过。
- `tests/run_control_mode_tests.py --cc C:/Qt/Tools/mingw1310_64/bin/gcc.exe`：CST、模式切换、速度限制与故障互锁通过。
- 电角对齐回归、PWM 输出保护测试通过；采样窗口测试的调制、ADC IRQ、板级 PWM 检查通过，末尾原电流测试程序执行被 Windows 拒绝，不能声明整个采样套件通过。
- 实际 Keil ARM Compiler 5.06 工程 66 个对象重新编译链接通过，默认扫频配置保留。ROM 55,072 / 55,296 B，余量 224 B；普通数据区 14,200 / 14,384 B，余量 184 B，另有原编码器状态区。后续扩展必须继续检查资源。
- 原扫频回归未通过：transport 的丢帧断言失败；后续 integration 测试仍引用已移除的 `SWEEP_MULTISINE` / `foc_sweep_start` / `sweep_result_t` 等旧接口，运行脚本还引用不存在的 `sweep_prepare.c`。这些测试及扫频实现未在本次修改中修复。
- 未连接硬件、未烧录，也未证明真实电机上的辨识精度和新增 ISR 最坏执行时间。

从仓库根目录复现新增测试：

```powershell
python motor_controller/tests/run_identification_tests.py --build-dir D:/temp/foc-ident-tests
python motor_controller/tests/run_control_mode_tests.py --build-dir D:/temp/foc-mode-tests --cc C:/Qt/Tools/mingw1310_64/bin/gcc.exe
python motor_controller/tests/build_sweep_firmware.py --build-dir D:/temp/foc-ident-arm
```

首次实机：确认电流零偏与电角对齐 → 按额定电流配置注入及跳闸阈值 → 单独 Rs/Ld/Lq，检查参数重复性和 LCR/电阻测量对照 → 应用结果 → 空载低速磁链辨识 → 检查不同转速下的磁链一致性 → 应用并按需保存。先检查中断耗时、电流波形及 `TIMING` 状态；辨识失败时保留原配置，依据错误原因调整测试条件。
