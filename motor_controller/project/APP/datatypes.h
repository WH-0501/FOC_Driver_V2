/**
 * @file datatypes.h
 *
 * FOC 数据分层（配置 / 测量 / 观测 / 给定 / 执行 / 调节器）：
 * - motor_param_t：铭牌或离线识别得到的电机本体参数，电流环内通常当只读使用
 * - motor_limits_t：母线/电流/电压限幅与保护
 * - motor_motion_cfg_t：机械侧轨线约束（最大角速度/角加速度/jerk/snap），供规划/外环使用
 * - motor_config_t：param + limits + motion 聚合，便于整体加载/落盘
 * - motor_state_t：运行时测量 + 电气观测
 * - motor_reference_t / motor_actuation_t：给定 / PWM 等
 * - control_mode_t / motor_handle_t.ctrl_mode：外环工作模式（给定 ref 哪条有效）
 * - motor_handle_t：单轴控制器实例
 * - fault_t / motor_fault_report_t：故障码（位掩码）与锁存，便于上位机解析
 */

#ifndef __DATATYPES_H__
#define __DATATYPES_H__

#include <stdint.h>


#define CURRENT_OFFSET_CALIBRATION_TIME         11     ///< 电流零漂平均：以 2 的幂次采样次数的指数部分
#define CURRENT_OFFSET_CALIBRATION_TIMES_SHIFT  (1u << CURRENT_OFFSET_CALIBRATION_TIME) ///< 零漂累计次数 2^n

/** 运行时故障寄存器：status 可表达当前条件，latched 需显式清除才可 Recovery */
typedef struct
{
  fault_t status; ///< 当前瞬时故障位
  fault_t latched;///< 锁存故障位（典型：触发保护直至复位）
} fault_report_t;

typedef enum
{
  STATE_IDLE = 0,                     ///< 空闲
  STATE_STARTUP = 1,                  ///< 启动
  STATE_CURRENT_CALIBRATION = 2,      ///< 电流零漂：PWM 关断下 ADC 累加，完成后转 STATE_IDLE（由 board_current_offset_cal_fsm_step 推进）
  STATE_ENCODER_CALIBRATION = 3,      ///< 编码器校准
  STATE_RSLS_CALIBRATION = 4,         ///< RS/LS 等参数辨识
  STATE_FLUX_CALIBRATION = 5,         ///< 磁链相关校准
  STATE_ELECTRICAL_ALIGNMENT = 6,     ///< 电角度对齐
  STATE_ANTICOGGING = 7,             ///< 齿槽转矩补偿标定
  STATE_RUNNING = 8,                  ///< 闭环运行
  STATE_FAULT = 9,                    ///< 故障
  STATE_MAX = 10,
} fsm_state_t;

/**
 * 控制模式：决定外环如何产生电流目标（ref.id / ref.iq 或由速度环、位置环算出）。
 * 速度/位置模式下的 ref.speed_rad_s、ref.position_rad 与被跟踪的 state 量一致（机械角 [rad]、[rad/s]）。
 */
typedef enum
{
  CTRL_MODE_IDLE = 0,       ///< 空闲
  CTRL_MODE_TORQUE,      ///< 扭矩模式：给定 ref.torque_nm
  CTRL_MODE_SPEED,       ///< 速度环：给定 ref.speed_rad_s
  CTRL_MODE_POSITION,     ///< 位置环：给定 ref.position_rad
  CTRL_MODE_CSP,
  CTRL_MODE_CSV,
  CTRL_MODE_CST,
  CTRL_MODE_MIT,
  CTRL_MODE_HOMING,
  CTRL_MODE_TORQUE_OPEN_LOOP, ///< 电流环开环：给定 ref.id / ref.iq
  CTRL_MODE_SPEED_OPEN_LOOP, ///< 速度环开环：给定 ref.speed_rad_s
  CTRL_MODE_POSITION_OPEN_LOOP, ///< 位置环开环：给定 ref.position_rad
  CTRL_MODE_VOLTAGE,  ///< 电压直接控制模式：给定 ref.v_d / ref.v_q
  CTRL_MODE_RESERVED, ///< 保留
} control_mode_t;

typedef struct
{
  float kp;             ///< 比例系数
  float ki;             ///< 积分系数
  float kd;             ///< 微分系数
  float ramp;           ///< 设定值斜坡或输出斜坡系数（依实现而定）
  float limit;          ///< 输出或积分限幅
  float last_error;     ///< 上一次误差
  float last_output;    ///< 上一次输出
  float last_integral;  ///< 上一次积分项累加
  uint32_t timestamp;   ///< 时间戳，用于积分/微分时间步
} pid_t;

typedef struct
{
  uint16_t adc_raw; ///< 温度通道 ADC 原始计数值
  float value;      ///< 换算后的物理量（如电压 V，或已标定 ℃）
} temperature_t;

typedef struct
{
  uint16_t adc_raw[3];   ///< 三相电流 ADC 原始计数值
  uint16_t adc_offset[3]; ///< 三相零电流平均 ADC（零漂）
  float ampere[3];       ///< 三相电流实测值 [A]
} motor_phase_current_t;

/**
 * 电机本体参数（Rs、Ld/Lq、磁链、极对、编码器、零位等）。
 * 运行期勿在电流环 ISR 内改写；若需在线辨识，应在慢任务中更新并注意同步。
 */
typedef struct
{
  float phase_resistance;         ///< 相电阻 Rs [Ω]
  float inductance_d;             ///< d 轴电感 Ld [H]
  float inductance_q;             ///< q 轴电感 Lq [H]
  float flux_linkage;             ///< 永磁磁链 ψf [Wb]（或等价常数）
  uint8_t pole_pairs;             ///< 极对数 p
  uint32_t encoder_counts_per_rev; ///< 机械旋转一圈编码器计数（分辨率定义依传感器）
  float theta_elec_offset_rad;    ///< 电角零偏置 [rad]，对齐/校准后写入
} motor_param_t;

/**
 * 保护与控制用限幅（非电机铭牌常数，但同样建议跑环时只读；部分由上位机下发）。
 */
typedef struct
{
  float vbus_nominal;      ///< 标称母线电压 [V]
  float vbus_uv_threshold; ///< 欠压阈值 [V]，母线低于此值可关断或报故障；≤0 关闭
  float vbus_ov_threshold; ///< 过压阈值 [V]，母线高于此值报故障（再生/电源异常等）；≤0 关闭
  float id_limit;          ///< d 轴电流指令/反馈限幅 [A]
  float iq_limit;          ///< q 轴电流限幅 [A]
  float vd_limit;          ///< d 轴电压输出限幅 [V]
  float vq_limit;          ///< q 轴电压输出限幅 [V]
  float temp_limit;        ///< 温度限幅 [℃]
  float sw_overcurrent_trip; ///< 上位机配置：软件过流阈值 [A]，max(|Ia|,|Ib|,|Ic|) 超此值报故障；≤0 关闭
  uint16_t sw_overcurrent_hold_ms; ///< 上位机配置：超阈值持续该时间 [ms] 后确认过流；0 为立即（实现侧定）
  float phase_diag_i_avg_min_a; ///< 缺相/不平衡检测门限：三相平均电流 |( |Ia|+|Ib|+|Ic| )/3 | 需 > 此值 [A]；≤0 关闭
  float phase_imbalance_ratio_max; ///< 缺相判据：(Imax-Imin)/avg 超此比值报 MOTOR_FAULT_PHASE_LOSS；≤0 关闭
  float phase_imbalance_warn_ratio; ///< 不平衡预警：超此比值报 MOTOR_FAULT_CURRENT_IMBALANCE（应 < phase_imbalance_ratio_max）；≤0 关闭
} motor_limits_t;

/**
 * 机械侧运动学上限（相对转子机械角，均指绝对值上限；用于梯形/S 曲线等规划与超速保护）。
 * 单位见字段名后缀；不需要的项填 0，实现里按 >0 再启用。
 */
typedef struct
{
  float max_speed_rad_s;   ///< 最大角速度 [rad/s]（|ω|）
  float max_accel_rad_s2;  ///< 最大角加速度 [rad/s²]（|dω/dt|）
  float max_jerk_rad_s3;   ///< 最大加加速度（jerk）[rad/s³]
  float max_snap_rad_s4;   ///< 最大 snap（角加速度对时间的导数）[rad/s⁴]，不用则 0
} motor_motion_cfg_t;

/** 上电加载的静态配置：本体参数 + 电气限幅 + 运动轨线约束 */
typedef struct
{
  motor_param_t param;         ///< 电机铭牌/辨识参数
  motor_limits_t limits;       ///< 电气与保护限幅
  motor_motion_cfg_t motion;   ///< 机械运动轨线约束
} motor_config_t;

typedef struct
{
  float id;               ///< d 轴电流给定 [A]
  float iq;               ///< q 轴电流给定 [A]
  float torque_nm;        ///< 转矩给定 [N·m]（若外环用转矩模式）
  float speed_rad_s; ///< 机械角速度给定 [rad/s]
  float position_rad;   ///< 机械角位置给定 [rad]
} motor_reference_t;

typedef struct
{
  float duty_a;     ///< A 相（或桥臂 A）PWM 占空比
  float duty_b;     ///< B 相占空比
  float duty_c;     ///< C 相占空比
  uint8_t pwm_enable; ///< 非 0 时允许输出 PWM
} motor_actuation_t;

typedef struct
{
  float vbus; ///< 当前母线电压测量 [V]

  float v_a; ///< a 相电压（重构或测量）[V]
  float v_b; ///< b 相电压 [V]
  float v_c; ///< c 相电压 [V]

  float i_d; ///< d 轴电流 [A]
  float i_q; ///< q 轴电流 [A]
  float v_d; ///< d 轴电压 [V]
  float v_q; ///< q 轴电压 [V]
  float v_alpha; ///< α 轴电压 [V]
  float v_beta;  ///< β 轴电压 [V]
  float i_alpha; ///< α 轴电流 [A]
  float i_beta;  ///< β 轴电流 [A]

  float theta_elec_rad; ///< 电角 [rad]，Park/逆 Park 用
  float sin_elec;       ///< sin(θ_e)，可与 θ 同步更新减少三角运算
  float cos_elec;       ///< cos(θ_e)

  float position_rad;       ///< 机械角位置 [rad]
  float speed_rad_s;        ///< 机械角速度 [rad/s]

  float temperature;        ///< 电机或关键温区温度（物理量与标定一致，如 ℃）

  motor_phase_current_t phase_current;  ///< 三相电流采样与标定结果
  temperature_t board_temp;             ///< 板载/驱动端温度采样
  bool current_calibrating;          ///< 非 0：电流零漂校准中，ampere 按标称偏置换算
} motor_state_t;

typedef struct
{
  motor_config_t config; ///< 静态配置（参数、限幅、运动约束）
  control_mode_t ctrl_mode; ///< 当前控制模式（与 ref 字段含义对应）
  motor_reference_t ref; ///< 各环给定值，即：目标值
  motor_state_t state;   ///< 运行状态与观测量
  motor_actuation_t out; ///< 调制/功率级输出
  fsm_state_t fsm; ///< 运行状态机当前状态

  pid_t id_pid;         ///< d 轴电流环 PID 状态
  pid_t iq_pid;         ///< q 轴电流环 PID 状态
  pid_t velocity_pid;   ///< 速度环 PID 状态
  pid_t position_pid;   ///< 位置环 PID 状态

  fault_report_t fault; ///< 故障码与锁存
} motor_handle_t;

#endif /* __DATATYPES_H__ */
