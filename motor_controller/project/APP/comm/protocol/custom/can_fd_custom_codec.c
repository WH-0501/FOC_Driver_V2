#include "can_fd_custom_codec.h"
#include "can_fd_custom.h"
#include "common.h"
#include "foc_math.h"
#include <stddef.h>

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

  // if (rpm > 32767.0f / CAN_FD_POS_SCALE)
  // {
  //   rpm = 32767.0f / CAN_FD_POS_SCALE;
  // }
  // if (rpm < -32768.0f / CAN_FD_POS_SCALE)
  // {
  //   rpm = -32768.0f / CAN_FD_POS_SCALE;
  // }

  // rad/s 已转换为输出轴,因此需要再转换为输入轴.
  rpm = rpm * GEAR_RATIO;

  return (int16_t)(rpm /* * CAN_FD_POS_SCALE */);
}

void can_fd_codec_pack_pid(float kp, float ki, float kd, uint8_t out[6])
{
  can_fd_codec_write_u16_be(&out[0], can_fd_codec_float_to_q12(kp / PID_PARAM_FACTOR));
  can_fd_codec_write_u16_be(&out[2], can_fd_codec_float_to_q12(ki / PID_PARAM_FACTOR));
  can_fd_codec_write_u16_be(&out[4], can_fd_codec_float_to_q12(kd / PID_PARAM_FACTOR));
}

void can_fd_codec_unpack_pid(const uint8_t in[6], float *kp, float *ki, float *kd)
{
  if (kp != NULL)
  {
    *kp = can_fd_codec_q12_to_float(can_fd_codec_read_u16_be(&in[0])) * PID_PARAM_FACTOR;
  }
  if (ki != NULL)
  {
    *ki = can_fd_codec_q12_to_float(can_fd_codec_read_u16_be(&in[2])) * PID_PARAM_FACTOR;
  }
  if (kd != NULL)
  {
    *kd = can_fd_codec_q12_to_float(can_fd_codec_read_u16_be(&in[4])) * PID_PARAM_FACTOR;
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
    return 0u;
  }

  if (fault & (FAULT_OVERTEMP_MOTOR | FAULT_OVERTEMP_DRIVER))
  {
    return 1u;
  }
  if (fault & FAULT_SW_OVERCURRENT)
  {
    return 2u;
  }
  if (fault & FAULT_BUS_UV)
  {
    return 3u;
  }
  if (fault & FAULT_BUS_OV)
  {
    return 4u;
  }
  if (fault & (FAULT_HW_OVERCURRENT | FAULT_GATE_DRIVER | FAULT_SHORT_CIRCUIT))
  {
    return 5u;
  }
  if (fault & FAULT_ENCODER)
  {
    return 6u;
  }
  if (fault & FAULT_PHASE_LOSS)
  {
    return 7u;
  }

  return 0u;
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
