#include "can_fd_custom_codec.h"
#include "can_fd_custom.h"
#include "common.h"
#include "foc_math.h"
#include <stddef.h>

/* V1 保护子码（周期 error：BIT7=0，BIT6~0=子码） */
#define CAN_FD_PROT_HW_OCP           0x01u
#define CAN_FD_PROT_SW_OCP           0x02u
#define CAN_FD_PROT_SW_OVP           0x04u
#define CAN_FD_PROT_SW_UVP           0x06u
#define CAN_FD_PROT_PHASE_LOSS       0x07u
#define CAN_FD_PROT_ENCODER          0x08u
#define CAN_FD_PROT_OVERTEMP         0x09u
#define CAN_FD_PROT_COMM             0x0Au
#define CAN_FD_PROT_POS_CMD          0x0Bu
#define CAN_FD_PROT_SHAFT_OVERSPEED  0x10u
#define CAN_FD_PROT_ALIGN_FAILED     0x11u
#define CAN_FD_PROT_CALIBRATION      0x12u
#define CAN_FD_PROT_CTRL_SATURATION  0x13u
#define CAN_FD_PROT_WATCHDOG         0x14u
#define CAN_FD_PROT_PARAM            0x15u
#define CAN_FD_PROT_SHORT_CIRCUIT    0x16u
#define CAN_FD_PROT_STALL            0x17u
#define CAN_FD_PROT_CURR_IMBALANCE   0x18u
#define CAN_FD_PROT_CURR_SENSOR      0x19u
#define CAN_FD_PROT_STORAGE          0x1Au
#define CAN_FD_PROT_BRAKE            0x1Bu
#define CAN_FD_PROT_DESAT            0x1Cu
#define CAN_FD_PROT_GATE_NFAULT      0x1Du
#define CAN_FD_PROT_LOGIC_UV         0x1Eu
#define CAN_FD_PROT_PHASE_W_OPEN     0x1Fu
#define CAN_FD_PROT_USER_CMD         0x20u
#define CAN_FD_PROT_UNCLASSIFIED     0x7Fu

uint16_t can_fd_codec_read_u16_be(const uint8_t *p)
{
  return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

int16_t can_fd_codec_read_i16_be(const uint8_t *p)
{
  return (int16_t)can_fd_codec_read_u16_be(p);
}

void can_fd_codec_write_u16_be(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)(v >> 8);
  p[1] = (uint8_t)v;
}

void can_fd_codec_write_i16_be(uint8_t *p, int16_t v)
{
  can_fd_codec_write_u16_be(p, (uint16_t)v);
}

float can_fd_codec_q12_to_float(uint16_t raw)
{
  return (float)raw / CAN_FD_PID_FORMAT;
}

uint16_t can_fd_codec_float_to_q12(float v)
{
  float scaled = v * CAN_FD_PID_FORMAT;

  if (scaled < 0.0f)
  {
    scaled = 0.0f;
  }
  if (scaled > 65535.0f)
  {
    scaled = 65535.0f;
  }

  return (uint16_t)(scaled + 0.5f);
}

/** 位置 Q7：raw / 128 -> [rad] */
float can_fd_codec_pos_q7_to_rad(int16_t raw)
{
  return (float)raw / CAN_FD_POS_SCALE;
}

/** 位置 Q7：[rad] -> raw（Q7 定点，1 LSB = 1/128 rad） */
int16_t can_fd_codec_rad_to_pos_q7(float rad)
{
  float scaled = rad * CAN_FD_POS_SCALE;

  if (scaled > 32767.0f)
  {
    scaled = 32767.0f;
  }
  if (scaled < -32768.0f)
  {
    scaled = -32768.0f;
  }

  if (scaled >= 0.0f)
  {
    return (int16_t)(scaled + 0.5f);
  }

  return (int16_t)(scaled - 0.5f);
}

float can_fd_codec_vel_q7_rpm_to_rad_s(int16_t raw)
{
  return RPM_TO_RAD_PER_SEC((float)raw /*/ CAN_FD_POS_SCALE*/);
}

int16_t can_fd_codec_rad_s_to_vel_q7_rpm(float rad_s)
{
  float rpm = RAD_PER_SEC_TO_RPM(rad_s);

  if (rpm > 32767.0f)
  {
    rpm = 32767.0f;
  }
  if (rpm < -32768.0f)
  {
    rpm = -32768.0f;
  }

  /* 与 0x02 目标速度一致：关节端（减速端）RPM，勿乘 gear_ratio */
  if (rpm >= 0.0f)
  {
    return (int16_t)(rpm + 0.5f);
  }
  return (int16_t)(rpm - 0.5f);
}

/*
 * V2.0.0 PID 线值（Q12）：wire = round(K * 4096)，K = wire / 4096。
 * 最小非零 K 步进 = 1/4096；与 MIT 刚度/阻尼同一 Q12 语义。
 * 电流环（0x44/0x49）：Kp/Kd 为物理值；Ki 为 config 缩放值（Ki_phys / FOC_PID_CUR_KI_SCALE）。
 */
void can_fd_codec_pack_pid(float kp, float ki, float kd, uint8_t out[6])
{
  can_fd_codec_write_u16_be(&out[0], can_fd_codec_float_to_q12(kp));
  can_fd_codec_write_u16_be(&out[2], can_fd_codec_float_to_q12(ki));
  can_fd_codec_write_u16_be(&out[4], can_fd_codec_float_to_q12(kd));
}

void can_fd_codec_unpack_pid(const uint8_t in[6], float *kp, float *ki, float *kd)
{
  if (kp != NULL)
  {
    *kp = can_fd_codec_q12_to_float(can_fd_codec_read_u16_be(&in[0]));
  }
  if (ki != NULL)
  {
    *ki = can_fd_codec_q12_to_float(can_fd_codec_read_u16_be(&in[2]));
  }
  if (kd != NULL)
  {
    *kd = can_fd_codec_q12_to_float(can_fd_codec_read_u16_be(&in[4]));
  }
}

void can_fd_codec_pack_protect(const motor_limits_t *lim, uint8_t out[6])
{
  uint16_t otp_c;
  uint16_t ocp_ma;
  uint8_t uv_v;
  uint8_t ov_v;

  if (lim == NULL)
  {
    return;
  }

  otp_c = (uint16_t)(lim->temp_limit + 0.5f);
  if (otp_c > 65535u)
  {
    otp_c = 65535u;
  }

  ocp_ma = (uint16_t)(lim->sw_ocp * 1000.0f + 0.5f);
  uv_v = (uint8_t)(lim->vbus_uv_threshold + 0.5f);
  ov_v = (uint8_t)(lim->vbus_ov_threshold + 0.5f);

  can_fd_codec_write_u16_be(&out[0], otp_c);
  can_fd_codec_write_u16_be(&out[2], ocp_ma);
  out[4] = uv_v;
  out[5] = ov_v;
}

void can_fd_codec_unpack_protect(const uint8_t in[6], motor_limits_t *lim)
{
  if (lim == NULL)
  {
    return;
  }

  lim->temp_limit = (float)can_fd_codec_read_u16_be(&in[0]);
  lim->sw_ocp = (float)can_fd_codec_read_u16_be(&in[2]) / 1000.0f;
  lim->vbus_uv_threshold = (float)in[4];
  lim->vbus_ov_threshold = (float)in[5];
}

uint8_t can_fd_codec_fault_to_error(fault_t fault)
{
  if (fault == FAULT_NONE)
  {
    return CAN_FD_ERR_PROTECT(0u);
  }

  if (fault & FAULT_SHORT_CIRCUIT)
  {
    return CAN_FD_ERR_PROTECT(CAN_FD_PROT_SHORT_CIRCUIT);
  }
  if (fault & FAULT_HW_OVERCURRENT)
  {
    return CAN_FD_ERR_PROTECT(CAN_FD_PROT_HW_OCP);
  }
  if (fault & FAULT_GATE_DRIVER)
  {
    return CAN_FD_ERR_PROTECT(CAN_FD_PROT_GATE_NFAULT);
  }
  if (fault & FAULT_DESATURATION)
  {
    return CAN_FD_ERR_PROTECT(CAN_FD_PROT_DESAT);
  }
  if (fault & FAULT_SW_OVERCURRENT)
  {
    return CAN_FD_ERR_PROTECT(CAN_FD_PROT_SW_OCP);
  }
  if (fault & FAULT_PHASE_LOSS)
  {
    return CAN_FD_ERR_PROTECT(CAN_FD_PROT_PHASE_LOSS);
  }
  if (fault & FAULT_PHASE_W_OPEN)
  {
    return CAN_FD_ERR_PROTECT(CAN_FD_PROT_PHASE_W_OPEN);
  }
  if (fault & FAULT_BUS_OV)
  {
    return CAN_FD_ERR_PROTECT(CAN_FD_PROT_SW_OVP);
  }
  if (fault & FAULT_BUS_UV)
  {
    return CAN_FD_ERR_PROTECT(CAN_FD_PROT_SW_UVP);
  }
  if (fault & FAULT_UNDER_VOLTAGE_LOGIC)
  {
    return CAN_FD_ERR_PROTECT(CAN_FD_PROT_LOGIC_UV);
  }
  if (fault & (FAULT_OVERTEMP_MOTOR | FAULT_OVERTEMP_DRIVER))
  {
    return CAN_FD_ERR_PROTECT(CAN_FD_PROT_OVERTEMP);
  }
  if (fault & FAULT_ENCODER)
  {
    return CAN_FD_ERR_PROTECT(CAN_FD_PROT_ENCODER);
  }
  if (fault & FAULT_CURRENT_SENSOR)
  {
    return CAN_FD_ERR_PROTECT(CAN_FD_PROT_CURR_SENSOR);
  }
  if (fault & FAULT_OVER_SPEED)
  {
    return CAN_FD_ERR_PROTECT(CAN_FD_PROT_SHAFT_OVERSPEED);
  }
  if (fault & FAULT_STALL)
  {
    return CAN_FD_ERR_PROTECT(CAN_FD_PROT_STALL);
  }
  if (fault & FAULT_CURRENT_IMBALANCE)
  {
    return CAN_FD_ERR_PROTECT(CAN_FD_PROT_CURR_IMBALANCE);
  }
  if (fault & FAULT_POSITION_TRACK)
  {
    return CAN_FD_ERR_PROTECT(CAN_FD_PROT_POS_CMD);
  }
  if (fault & FAULT_ALIGN_FAILED)
  {
    return CAN_FD_ERR_PROTECT(CAN_FD_PROT_ALIGN_FAILED);
  }
  if (fault & FAULT_CALIBRATION)
  {
    return CAN_FD_ERR_PROTECT(CAN_FD_PROT_CALIBRATION);
  }
  if (fault & FAULT_CONTROL_SATURATION)
  {
    return CAN_FD_ERR_PROTECT(CAN_FD_PROT_CTRL_SATURATION);
  }
  if (fault & FAULT_WATCHDOG)
  {
    return CAN_FD_ERR_PROTECT(CAN_FD_PROT_WATCHDOG);
  }
  if (fault & FAULT_PARAM)
  {
    return CAN_FD_ERR_PROTECT(CAN_FD_PROT_PARAM);
  }
  if (fault & FAULT_COMM_TIMEOUT)
  {
    return CAN_FD_ERR_PROTECT(CAN_FD_PROT_COMM);
  }
  if (fault & FAULT_STORAGE)
  {
    return CAN_FD_ERR_PROTECT(CAN_FD_PROT_STORAGE);
  }
  if (fault & FAULT_BRAKE)
  {
    return CAN_FD_ERR_PROTECT(CAN_FD_PROT_BRAKE);
  }
  if (fault & FAULT_USER_CMD)
  {
    return CAN_FD_ERR_PROTECT(CAN_FD_PROT_USER_CMD);
  }

  return CAN_FD_ERR_PROTECT(CAN_FD_PROT_UNCLASSIFIED);
}

uint8_t can_fd_codec_pad_canfd_len(uint8_t len)
{
  static const uint8_t dlc_lengths[16] = {
      0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
  };
  uint8_t i;

  for (i = 0u; i < 16u; ++i)
  {
    if (len <= dlc_lengths[i])
    {
      return dlc_lengths[i];
    }
  }

  return 64u;
}
