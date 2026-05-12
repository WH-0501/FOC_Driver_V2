/**
 * @file datatypes.h
 *
 * FOC 数据分层（配置 / 测量 / 观测 / 给定 / 执行 / 调节器）：
 * - motor_param_t：铭牌或离线识别得到的电机本体参数，电流环内通常当只读使用
 * - motor_limits_t：母线/电流/电压限幅与保护
 * - motor_motion_cfg_t：机械侧轨线约束（|ω|、|α|、jerk、snap），供规划/外环使用
 * - motor_config_t：param + limits + motion 聚合，便于整体加载/落盘
 * - motor_state_t：运行时测量 + 电气观测
 * - motor_reference_t / motor_actuation_t：给定 / PWM 等
 * - motor_handle_t：单轴控制器实例
 * - motor_fault_code_t / motor_fault_report_t：故障码（位掩码）与锁存，便于上位机解析
 */

#ifndef __DATATYPES_H__
#define __DATATYPES_H__

#include <stdint.h>

#define CURRENT_OFFSET_CALIBRATION_TIME         11     ///< 电流零漂平均：以 2 的幂次采样次数的指数部分
#define CURRENT_OFFSET_CALIBRATION_TIMES_SHIFT  (1u << CURRENT_OFFSET_CALIBRATION_TIME) ///< 零漂累计次数 2^n

typedef enum
{
  MOTOR_FAULT_NONE               = 0,        ///< 无故障
  MOTOR_FAULT_BUS_UV             ,  ///< 母线欠压
  MOTOR_FAULT_BUS_OV             ,  ///< 母线过压
  MOTOR_FAULT_SW_OVERCURRENT     ,  ///< 软件过流（相电流超阈）
  MOTOR_FAULT_HW_OVERCURRENT     ,  ///< 硬件过流或驱动故障引脚
  MOTOR_FAULT_OVERTEMP_MOTOR     ,  ///< 电机过温
  MOTOR_FAULT_OVERTEMP_DRIVER    ,  ///< 驱动/逆变器过温
  MOTOR_FAULT_ENCODER            ,  ///< 编码器通信/计数异常
  MOTOR_FAULT_POSITION_TRACK     ,  ///< 位置跟随/偏差过大
  MOTOR_FAULT_OVER_SPEED         ,  ///< 超速
  MOTOR_FAULT_ALIGN_FAILED       ,  ///< 初始对齐/辨识失败
  MOTOR_FAULT_CALIBRATION        ,  ///< 校准过程错误
  MOTOR_FAULT_CONTROL_SATURATION ,  ///< 电压/电流长期饱和（可选）
  MOTOR_FAULT_WATCHDOG           ,  ///< 控制看门狗/任务超时
  MOTOR_FAULT_PARAM              ,  ///< 参数非法或未初始化
  MOTOR_FAULT_PHASE_LOSS         ,  ///< 缺相/断线（三相电流严重不平衡或某相近零）
  MOTOR_FAULT_SHORT_CIRCUIT      ,  ///< 输出短路/直通等（多由硬件故障脚或专用检测上报）
  MOTOR_FAULT_STALL              ,  ///< 堵转（速度/电流组合判据，多由应用或观测器置位）
  MOTOR_FAULT_CURRENT_IMBALANCE  ,  ///< 三相电流不平衡（未达到缺相阈值时的预警）
  MOTOR_FAULT_CURRENT_SENSOR     ,  ///< 电流采样、ADC 自检或零漂异常
  MOTOR_FAULT_COMM_TIMEOUT       ,  ///< 通讯超时（如 CAN、EtherCAT）
  MOTOR_FAULT_STORAGE            ,  ///< EEPROM/Flash 读写或 CRC 校验失败
  MOTOR_FAULT_BRAKE              ,  ///< 制动电阻/泄放回路异常或过温
  MOTOR_FAULT_DESATURATION       ,  ///< 驱动 Desat/退饱和保护
  MOTOR_FAULT_GATE_DRIVER        ,  ///< 预驱故障（如 nFAULT）、驱动电源异常
  MOTOR_FAULT_UNDER_VOLTAGE_LOGIC ,  ///< 控制电源/逻辑欠压（非母线）
  MOTOR_FAULT_PHASE_U_OPEN       ,  ///< U 相开路（细分上报，可选）
  MOTOR_FAULT_PHASE_V_OPEN       ,  ///< V 相开路
  MOTOR_FAULT_PHASE_W_OPEN       ,  ///< W 相开路
  /* bit28、29 预留 */
  MOTOR_FAULT_USER_CMD           ,  ///< 上位机指令急停/故障注入
  MOTOR_FAULT_RESERVED           ,  ///< 保留
} motor_fault_code_t;

/** 与 API 返回值配合：非故障类错误 */
typedef enum
{
  MOTOR_OK                = 0,   ///< 成功
  MOTOR_ERR_NULL          = 1, ///< 空指针
  MOTOR_ERR_PARAM         = 2, ///< 参数非法
  MOTOR_ERR_STATE         = 3, ///< 状态不允许（如已故障锁定）
  MOTOR_ERR_TIMEOUT       = 4, ///< 超时
  MOTOR_ERR_NOT_IMPL      = 5, ///< 未实现
  MOTOR_ERR_COMM          = 6, ///< 通讯错误
  MOTOR_ERR_STORAGE       = 7, ///< 存储错误
  MOTOR_ERR_SENSOR        = 8, ///< 传感器错误
} motor_result_t;

/** 运行时故障寄存器：status 可表达当前条件，latched 需显式清除才可 Recovery */
typedef struct
{
  motor_fault_code_t status; ///< 当前瞬时故障位
  motor_fault_code_t latched;///< 锁存故障位（典型：触发保护直至复位）
} motor_fault_report_t;

typedef enum
{
  STATE_IDLE = 0,                     ///< 空闲
  STATE_STARTUP = 1,                  ///< 启动
  STATE_CURRENT_CALIBRATION = 2,      ///< 电流校准
  STATE_ENCODER_CALIBRATION = 3,      ///< 编码器校准
  STATE_RSLS_CALIBRATION = 4,         ///< RS/LS 等参数辨识
  STATE_FLUX_CALIBRATION = 5,         ///< 磁链相关校准
  STATE_ELECTRICAL_ALIGNMENT = 6,     ///< 电角度对齐
  STATE_ANTICOGGING = 7,             ///< 齿槽转矩补偿标定
  STATE_RUNNING = 8,                  ///< 闭环运行
} motor_fsm_state_t;

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
 * 机械侧运动学约束：用于速度/位置环前级规划（梯形/S 曲线、软限速等）。
 * 单位一律为机械角：角速度 [rad/s]、角加速度 [rad/s²]、jerk（加加速度）[rad/s³]、snap [rad/s⁴]。
 * 暂不用高阶约束时可置 0，规划器内应判断 >0 再启用。
 */
typedef struct
{
  float omega_max_abs_mech_rad_s;  ///< 机械角速度绝对值上限 |ω| [rad/s]
  float alpha_max_abs_mech_rad_s2; ///< 机械角加速度绝对值上限 |α| [rad/s²]
  float jerk_max_abs_mech_rad_s3;  ///< 机械角 jerk（加加速度）绝对值上限 [rad/s³]
  float snap_max_abs_mech_rad_s4; ///< 机械角 snap 绝对值上限 [rad/s⁴]，不用可置 0
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
  float omega_mech_rad_s; ///< 机械角速度给定 [rad/s]
  float theta_mech_rad;   ///< 机械角位置给定 [rad]
  float torque_nm;        ///< 转矩给定 [N·m]（若外环用转矩模式）
} motor_reference_t;

typedef struct
{
  float duty_a;     ///< A 相（或桥臂 A）PWM 占空比，归一化 0~1（或依驱动约定）
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

  float theta_mech_rad;     ///< 机械角位置 [rad]
  float omega_mech_rad_s;   ///< 机械角速度 [rad/s]

  float temperature;        ///< 电机或关键温区温度（物理量与标定一致，如 ℃）

  motor_phase_current_t m_phase_current;  ///< 三相电流采样与标定结果
  temperature_t board_temp;             ///< 板载/驱动端温度采样
  uint8_t current_calibrating;          ///< 非 0：电流零漂校准中，ampere 按标称偏置换算
} motor_state_t;

typedef struct
{
  motor_config_t config; ///< 静态配置（参数、限幅、运动约束）
  motor_reference_t ref; ///< 各环给定值，即：目标值
  motor_state_t state;   ///< 运行状态与观测量
  motor_actuation_t out; ///< 调制/功率级输出
  motor_fsm_state_t fsm; ///< 运行状态机当前状态

  pid_t m_id_pid;         ///< d 轴电流环 PID 状态
  pid_t m_iq_pid;         ///< q 轴电流环 PID 状态
  pid_t m_velocity_pid;   ///< 速度环 PID 状态
  pid_t m_position_pid;   ///< 位置环 PID 状态

  motor_fault_report_t fault; ///< 故障码与锁存
} motor_handle_t;

#endif /* __DATATYPES_H__ */
