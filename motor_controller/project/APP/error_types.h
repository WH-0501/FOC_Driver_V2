/**
 * @file error_types.h
 * @brief 故障码与错误码类型定义（无 motor_handle_t 依赖）。
 */
#ifndef __ERROR_TYPES_H__
#define __ERROR_TYPES_H__

typedef enum
{
  FAULT_NONE               = 0,        ///< 无故障
  FAULT_BUS_UV             = 1u << 0,  ///< 母线欠压
  FAULT_BUS_OV             = 1u << 1,  ///< 母线过压
  FAULT_SW_OVERCURRENT     = 1u << 2,  ///< 软件过流（相电流超阈）
  FAULT_HW_OVERCURRENT     = 1u << 3,  ///< 硬件过流或驱动故障引脚
  FAULT_OVERTEMP_MOTOR     = 1u << 4,  ///< 电机过温
  FAULT_OVERTEMP_DRIVER    = 1u << 5,  ///< 驱动/逆变器过温
  FAULT_ENCODER            = 1u << 6,  ///< 编码器通信/计数异常
  FAULT_POSITION_TRACK     = 1u << 7,  ///< 位置跟随/偏差过大
  FAULT_OVER_SPEED         = 1u << 8,  ///< 超速
  FAULT_ALIGN_FAILED       = 1u << 9,  ///< 初始对齐/辨识失败
  FAULT_CALIBRATION        = 1u << 10, ///< 校准过程错误
  FAULT_CONTROL_SATURATION = 1u << 11, ///< 电压/电流长期饱和（可选）
  FAULT_WATCHDOG           = 1u << 12, ///< 控制看门狗/任务超时
  FAULT_PARAM              = 1u << 13, ///< 参数非法或未初始化
  FAULT_PHASE_LOSS         = 1u << 14, ///< 缺相/断线（三相电流严重不平衡或某相近零）
  FAULT_SHORT_CIRCUIT      = 1u << 15, ///< 输出短路/直通等（多由硬件故障脚或专用检测上报）
  FAULT_STALL              = 1u << 16, ///< 堵转（速度/电流组合判据，多由应用或观测器置位）
  FAULT_CURRENT_IMBALANCE  = 1u << 17, ///< 三相电流不平衡（未达到缺相阈值时的预警）
  FAULT_CURRENT_SENSOR     = 1u << 18, ///< 电流采样、ADC 自检或零漂异常
  FAULT_COMM_TIMEOUT       = 1u << 19, ///< 通讯超时（如 CAN、EtherCAT）
  FAULT_STORAGE            = 1u << 20, ///< EEPROM/Flash 读写或 CRC 校验失败
  FAULT_BRAKE              = 1u << 21, ///< 制动电阻/泄放回路异常或过温
  FAULT_DESATURATION       = 1u << 22, ///< 驱动 Desat/退饱和保护
  FAULT_GATE_DRIVER        = 1u << 23, ///< 预驱故障（如 nFAULT）、驱动电源异常
  FAULT_UNDER_VOLTAGE_LOGIC = 1u << 24, ///< 控制电源/逻辑欠压（非母线）
  FAULT_PHASE_W_OPEN       = 1u << 27, ///< W 相开路
  FAULT_USER_CMD           = 1u << 28, ///< 上位机指令急停/故障注入
    /* bit29、30 预留 */
  FAULT_RESERVED           = 1u << 30, ///< 保留
} fault_t;

/** 与 API 返回值配合：非故障类错误 */
typedef enum
{
  ERR_NONE          = 0, ///< 成功
  ERR_NULL          = 1, ///< 空指针
  ERR_PARAM         = 2, ///< 参数非法
  ERR_STATE         = 3, ///< 状态不允许（如已故障锁定）
  ERR_TIMEOUT       = 4, ///< 超时
  ERR_NOT_IMPL      = 5, ///< 未实现
  ERR_COMM          = 6, ///< 通讯错误
  ERR_STORAGE       = 7, ///< 存储错误
  ERR_SENSOR        = 8, ///< 传感器错误
  ERR_FAIL          = 9, ///< 失败，未定义错误
} error_t;

/** 由测量与 limits 周期性自动刷新的瞬时故障位（不含仅由应用/驱动引脚置位的位） */
#define FAULT_POLL_MASK                                                                                 \
  ((fault_t)(FAULT_BUS_UV | FAULT_BUS_OV | FAULT_SW_OVERCURRENT |                                      \
             FAULT_OVERTEMP_MOTOR | FAULT_PHASE_LOSS | FAULT_CURRENT_IMBALANCE |                      \
             FAULT_OVER_SPEED))

#endif /* __ERROR_TYPES_H__ */
