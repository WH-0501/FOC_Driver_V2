/**
 * @file datatypes.h
 *
 * 电流数据分层（避免与 i_d/i_q 混淆）：
 * - motor_phase_current_t：传感器域 — adc_raw、零漂 adc_offset、换算后的实测相电流 ampere[3]。
 * - motor_state_t：控制/观测域 — 不含与 ampere[]    同义重复的三相 float；Clarke/Park 用 i_alpha/i_beta、i_d/i_q。
 *   若算法需要 i_a/i_b/i_c 名字，请用 m_phase_current.ampere[0..2] 或自行宏映射。
 */

#ifndef __DATATYPES_H__
#define __DATATYPES_H__

#include <stdint.h>

#define CURRENT_OFFSET_CALIBRATION_TIME         11
#define CURRENT_OFFSET_CALIBRATION_TIMES_SHIFT  (1u << CURRENT_OFFSET_CALIBRATION_TIME)

typedef enum
{
  STATE_IDLE = 0,
  STATE_STARTUP = 1,
  STATE_CURRENT_CALIBRATION = 2,
  STATE_ENCODER_CALIBRATION = 3,
  STATE_RSLS_CALIBRATION = 4,
  STATE_FLUX_CALIBRATION = 5,
  STATE_ELECTRICAL_ALIGNMENT = 6,
  STATE_ANTICOGGING = 7,
  STATE_RUNNING = 8,
} motor_fsm_state_t;

typedef struct
{
  float phase_resistance;
  float phase_inductance;
  float flux_linkage;
} motor_param_t;

typedef struct
{
  float kp;
  float ki;
  float kd;
  float ramp;
  float limit;
  float last_error;
  float last_output;
  float last_integral;
  uint32_t timestamp;
} pid_t;

typedef struct
{
  uint16_t adc_raw;
  float value;
} temperature_t;

/** ADC 与实测三相电流（A）；与 i_d/i_q/αβ 不同坐标系，不重复。 */
typedef struct
{
  uint16_t adc_raw[3];
  uint16_t adc_offset[3];
  float ampere[3];
} motor_phase_current_t;

typedef struct
{
  float vbus;

  float v_a;
  float v_b;
  float v_c;

  /* d-q、α-β 为变换后量；相电流实测见 m_phase_current.ampere[] */
  float i_d;
  float i_q;
  float v_alpha;
  float v_beta;
  float i_alpha;
  float i_beta;

  float temperature;

  motor_phase_current_t m_phase_current;
  temperature_t board_temp;
  uint8_t current_calibrating;
} motor_state_t;

typedef struct
{
  motor_fsm_state_t m_fsm_state;
  motor_state_t m_state;

  pid_t m_id_pid;
  pid_t m_iq_pid;
  pid_t m_velocity_pid;
  pid_t m_position_pid;
} motor_handle_t;

#endif /* __DATATYPES_H__ */
