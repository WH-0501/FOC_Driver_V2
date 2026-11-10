# CAN-FD 扩展协议（V2 FOC 固件）

本文档描述 **[CAN-FD通信协议.md](CAN-FD通信协议.md)** 主 CMD 表中**未单独展开**的功能码：单播分发确认、三环调试流、电角实时验证、软/硬限速等。

- 帧格式、版本差异、周期遥测 `0x00`、`0x7F`、IAP、PID 读写 `0x42`~`0x49` 等仍以主文档为准。
- 固件枚举：`project/APP/comm/protocol/custom/can_fd_custom.h`（`CAN_FD_CMD_*`）。
- Python 常量：`motor_controller_gui/protocol/constants.py`（`CMD`）。
- 三环调试上位机流程与示波器语义：**[三环调试.md](三环调试.md)**。

## 约定

| 项目 | 说明 |
| --- | --- |
| 线上帧 | `[CNT:1][CMD:1][DATA…]`，Node-ID 在 CAN 仲裁段 |
| 下表「偏移」 | 均相对 **DATA[0]**（不含 CNT、CMD） |
| 字节序 | 多字节整数与 **float32** 均为 **大端** |
| float32 | IEEE754 单精度，4 B |
| int32 角度 | 经 `can_fd_rad_to_urad` 等编码时，常见 **1 LSB = 1e-6 rad**（与 `0xAB` 一致） |
| 应答 CMD | 专用应答 **CMD 与请求相同**（如 `0xB8` 请求 → `0xB8` 应答），另可追加 **`0x7E`**（见下） |
| DLC | DATA 不足 CAN-FD 合法档位时尾部填 `0x00`；调试推送固定 **DATA 62 B → 总长 64 B** |

---

## 扩展功能码索引

| CMD | 宏名（固件） | 方向 | 类型 | DATA 长度（典型） |
| --- | --- | --- | --- | --- |
| `0x7E` | `CAN_FD_CMD_COMMAND_ACK` | 设备→主机 | 推送 | 2 |
| `0xB1` | `CAN_FD_CMD_VERIFY_ELECTRICAL` | 双向 | 请求-应答 | 请求 ≥3 / 应答 12 |
| `0xB2` | `CAN_FD_CMD_VERIFY_SAMPLE` | 设备→主机 | 推送 | 32 |
| `0xB5` | `CAN_FD_CMD_GET_SPEED_LIMITS` | 双向 | 请求-应答 | 请求 0 / 应答 20 |
| `0xB6` | `CAN_FD_CMD_SET_CURRENT_SOFT_SPEED` | 双向 | 请求-应答 | 请求 ≥2 / 应答 20 |
| `0xB8` | `CAN_FD_CMD_DEBUG_CONFIG` | 双向 | 请求-应答 | 请求 4 / 应答 5 |
| `0xB9` | `CAN_FD_CMD_DEBUG_CURRENT` | 设备→主机 | 推送 | 62 |
| `0xBA` | `CAN_FD_CMD_DEBUG_SPEED` | 设备→主机 | 推送 | 62 |
| `0xBB` | `CAN_FD_CMD_DEBUG_POSITION` | 设备→主机 | 推送 | 62 |

---

## 0x7E 单播分发确认（`CAN_FD_CMD_COMMAND_ACK`）

设备→主机，**无请求**。在单播命令分发返回后可能紧接专用应答发出。

| 偏移 | 长 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | 1 | `request_cnt` | u8 | 原请求帧 CNT |
| 1 | 1 | `request_cmd` | u8 | 原请求帧 CMD |

| 项目 | 说明 |
| --- | --- |
| 触发 | 非 IAP、CMD≠`0x7E` 的**单播**命令处理返回后 |
| 语义 | 仅表示已进入 `can_fd_custom_handle_command`；**不**表示参数合法或动作成功 |
| IAP | IAP 会话中忽略的普通命令**不发** `0x7E` |

---

## 0xB8 调试流配置（`CAN_FD_CMD_DEBUG_CONFIG`）

**必须单播**；广播时固件**忽略且不应答**。

### 请求 DATA（4 B）

| 偏移 | 长 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | 1 | `version` | u8 | 固定 **1** |
| 1 | 1 | `mask` | u8 | 订阅位：`bit0` 电流环→`0xB9`；`bit1` 速度环→`0xBA`；`bit2` 位置环→`0xBB`。仅允许 **0~7**；**0=关闭** |
| 2 | 2 | `hz` | u16 BE | **每个已置位环**的目标推送频率（Hz）。`mask=0` 时可忽略 |

**参数校验**（失败则应答 `error=ERR_PARAM`，不修改 mask）：

- `payload_len < 4` 或 `version≠1` 或 `mask & ~7 ≠ 0`
- `mask≠0` 时：`1 ≤ hz ≤ 1000`，且 **`hz × popcount(mask) ≤ 1000`**

**示例**（DATA 十六进制，不含 CNT/CMD）：

| 含义 | DATA |
| --- | --- |
| 仅电流环 100 Hz | `01 01 00 64` |
| 三环各 100 Hz | `01 07 00 64` |
| 关闭调试流 | `01 00 00 64`（后两字节任意） |

### 应答 DATA（5 B，CMD=`0xB8`）

| 偏移 | 长 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | 1 | `version` | u8 | **1** |
| 1 | 1 | `error` | u8 | `error_t`：**0** 成功；参数非法为 **`ERR_PARAM`** |
| 2 | 1 | `mask` | u8 | 应用后的 `s_debug_mask`（失败时保持原值） |
| 3 | 2 | `hz` | u16 BE | 当前每环周期对应的频率：`1_000_000 / s_debug_period`（Hz） |

| 副作用 | 说明 |
| --- | --- |
| 周期遥测 | 调试 mask≠0 时，`0x00` 周期上报降为 **约 5 Hz** |
| 推送 | **不会**在本应答后立即发 `0xB9`~`0xBB`；由主循环 `can_fd_debug_poll` 按 `hz` 轮询发送 |
| IAP | 进入 IAP 后固件 **清零 mask** |

主机应以 **应答 `mask`/`hz` 与请求一致** 作为调试流已开启依据，**不得**仅用 `0x7E`。

---

## 0xB9 / 0xBA / 0xBB 调试推送

设备主动发送；**无**逐帧请求配对。CMD：`0xB9` 电流 / `0xBA` 速度 / `0xBB` 位置。

**DATA 总长 62 B**（线上 CNT+CMD+62 → **DLC=64**）。

### 首部

| 偏移 | 长 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | 1 | `version` | u8 | **4**（**3**=5×u16；见下） |
| 1 | 1 | `flags` | u8 | 见下表 |
| 2 | 4 | `timestamp` | u32 BE | 快照时刻 `pid.timestamp`（设备 tick；当前固件多为 DWT→µs 换算值） |

**`flags` 位定义**

| 位 | 含义 |
| --- | --- |
| 0 | **active**：本环在当前控制模式 + `STATE_RUNNING` 下正在参与控制 |
| 1 | 该环 PID 配置了 **输出 ramp**（`cfg.ramp > 0`） |
| 2 | 电机处于 **故障** 状态（`STATE_FAULT`） |
| 3 | **`reference` 为上层给定**（非级联内环 PID 输出）；上位机不应把 `Kp×(reference−feedback)` 当作控制器 P 项 |
| 4~7 | 保留 0 |

固件在 active 时还会置 **bit3=1**（与 bit0 同时出现）。

### float 区

#### payload version = 1（旧）

偏移 **6** 起 **14×float32**，大端；**偏移 = 6 + 4×N**。

| N | 偏移 | 字段 | 单位 / 说明 |
| --- | --- | --- | --- |
| 0 | 6 | `reference` | 上层给定：电流环 **Iq (A)**；速度环 **rad/s**；位置环 **rad** |
| 1 | 10 | `feedback` | 该环 PID 反馈（同单位） |
| 2 | 14 | `iq` | 状态 **Iq (A)** |
| 3 | 18 | `id` | 状态 **Id (A)** |
| 4 | 22 | `pid_dt` | 本环 PID 步长 **(s)** |
| 5 | 26 | `integral` | 积分状态（与 PID 实现一致） |
| 6 | 30 | `output` | PID 输出 |
| 7 | 34 | `vbus` | 滤波母线 **(V)** |
| 8 | 38 | `shaft_velocity` | 轴 **角速度 rad/s** |
| 9 | 42 | `kp` | 配置 Kp |
| 10 | 46 | `ki` | 配置 Ki（电流环积分在固件内另有 ×10000 物理还原，见主文档 PID 说明） |
| 11 | 50 | `output_limit` | 输出限幅 |
| 12 | 54 | `integral_limit` | 积分限幅 |
| 13 | 58 | `ramp` | 输出斜坡配置 |

#### payload version = 2（当前固件）

偏移 **6** 起 **12×float32**（N=0~11 与上表 N=0~11 相同）；**不再**在帧内携带 `integral_limit` / `ramp`。

| 偏移 | 长 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- | --- |
| 54 | 2 | `foc_cur_loop_us` | u16 BE | 电流环执行耗时 **µs**（`motor_runtime.foc_cur_loop_us`，饱和 65535） |
| 56 | 2 | `foc_vel_loop_us` | u16 BE | 速度环耗时 **µs** |
| 58 | 2 | `foc_pos_loop_us` | u16 BE | 位置环耗时 **µs** |
| 60 | 2 | `foc_ctrl_loop_us` | u16 BE | **`foc_control_run` 耗时 µs**（饱和 65535） |

调试电流环时可直接观察 `foc_cur_loop_us` 等，与 `foc.c` 内 DWT 统计一致。

#### payload version = 3

与 v4 相同 float 区；**5×u16 @ 50–58**（无 `foc_sense_update_us`），**60–61 保留 0**。

#### payload version = 4（当前固件）

11×float 同 v3；**6×u16**：

| 偏移 | 字段 |
| --- | --- |
| 50–58 | 同 v3 五路耗时 |
| 60 | `foc_sense_update_us`（`board_get_phase_current` + 电流链 + `encoder_update` + Clarke/Park 等） |

---

## 0xB5 / 0xB6 限速（单播）

广播 **不处理**。应答结构相同，CMD 与请求一致。

### `0xB5` 请求

**0 B**。

### `0xB6` 请求

| 偏移 | 长 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | 2 | `soft_rpm` | u16 BE | 关节端软限速 **RPM**（内部×减速比得轴转速） |
| 2 | 2 | `flags` | u16 BE | 可选；**bit0=1** 启用轴硬超速保护（`CAN_FD_SPEED_LIMIT_FLAG_HARD_OVERSPEED`） |

### `0xB5` / `0xB6` 应答 DATA（20 B）

| 偏移 | 长 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | 1 | `format` | u8 | **3**（当前固件） |
| 1 | 1 | `error` | u8 | `error_t` |
| 2 | 2 | `hard_rpm` | u16 BE | 关节端硬限速上限 **RPM** |
| 4 | 2 | `soft_rpm` | u16 BE | 关节端当前软限速 **RPM** |
| 6 | 1 | `mode` | u8 | 当前 `control_mode_t` |
| 7 | 1 | `status` | u8 | **bit0** 软限速正在限流；**bit1** 硬超速保护已启用 |
| 8 | 4 | `speed_milli` | i32 BE | 关节端速度，**值/1000 ≈ rad/s 量级**（固件由轴速/减速比编码） |
| 12 | 4 | `iq_ref_milli` | i32 BE | Iq 给定，**值/1000 ≈ A** |
| 16 | 4 | `iq_limited_milli` | i32 BE | 限速后 Iq 上限，**值/1000 ≈ A**（转矩/CST 运行时有意义） |

---

## 0xB1 / 0xB2 电角验证

**`0xB1` 须单播**。

### `0xB1` 请求（≥3 B）

| 偏移 | 长 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | 1 | `version` | u8 | **1** |
| 1 | 2 | `session` | u16 BE | 会话号；主机分配 |

### `0xB1` 应答 DATA（12 B，CMD=`0xB1`）

| 偏移 | 长 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | 1 | `version` | u8 | **1** |
| 1 | 1 | `error` | u8 | `error_t` |
| 2 | 2 | `session` | u16 BE | 会话号 |
| 4 | 2 | `sample_count` | u16 BE | 已采/sample 计数（结束时） |
| 6 | 2 | — | — | 保留 |
| 8 | 4 | `max_error_urad` | i32 BE | 最大角误差，**1 LSB = 1e-6 rad** |

验证过程中采样由 **`0xB2`** 推送（可能多帧）。

### `0xB2` 推送 DATA（32 B，CMD=`0xB2`）

| 偏移 | 长 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | 1 | `version` | u8 | **1** |
| 1 | 1 | `phase` | u8 | 验证阶段编号 |
| 2 | 1 | `settled` | u8 | **bit0=1** 表示该点已稳定 |
| 3 | 1 | — | — | 保留 0 |
| 4 | 2 | `session` | u16 BE | 会话号 |
| 6 | 2 | `sequence` | u16 BE | 采样序号 |
| 8 | 4 | `elapsed_ms` | i32 BE | 自会话起经过 **ms** |
| 12 | 4 | `target_urad` | i32 BE | 目标角，1e-6 rad/LSB |
| 16 | 4 | `measured_urad` | i32 BE | 测量角 |
| 20 | 4 | `error_urad` | i32 BE | 误差角 |
| 24 | 4 | `mechanical_urad` | i32 BE | 机械角 |
| 28 | 4 | `expected_mechanical_urad` | i32 BE | 期望机械角 |

流程细节见 **`docs/电角对齐实时验证.md`**。

---

## 与主文档命令的关系

| 能力 | CMD | 文档 |
| --- | --- | --- |
| PID 参数读写 | `0x42`~`0x44` / `0x47`~`0x49` | 主文档 |
| 周期上报频率 | `0x7F` | 主文档 |
| 使能 / 模式 | `0x40` / `0x41` | 主文档 |

---

## 实现备注

- 专用应答经 **16 槽队列** 排队；PTB 忙时在 `comm_poll` 重试，**优先于**周期遥测。
- 调试开启时，每轮 `comm_poll` **最多 1 帧** `0xB9`~`0xBB`（round-robin）。
