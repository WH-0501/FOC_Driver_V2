#ifndef __CAN_FD_CUSTOM_CODEC_H__
#define __CAN_FD_CUSTOM_CODEC_H__

#include <stdint.h>
#include <stdbool.h>
#include "error_types.h"
#include "datatypes.h"

uint16_t can_fd_codec_read_u16_be(const uint8_t *p);
int16_t can_fd_codec_read_i16_be(const uint8_t *p);

void can_fd_codec_write_u16_be(uint8_t *p, uint16_t v);
void can_fd_codec_write_i16_be(uint8_t *p, int16_t v);

float can_fd_codec_q12_to_float(uint16_t raw);
uint16_t can_fd_codec_float_to_q12(float v);

float can_fd_codec_pos_q7_to_rad(int16_t raw);
int16_t can_fd_codec_rad_to_pos_q7(float rad);

float can_fd_codec_vel_q7_rpm_to_rad_s(int16_t raw);
int16_t can_fd_codec_rad_s_to_vel_q7_rpm(float rad_s);

void can_fd_codec_pack_pid(float kp, float ki, float kd, uint8_t out[6]);
void can_fd_codec_unpack_pid(const uint8_t in[6], float *kp, float *ki, float *kd);

void can_fd_codec_pack_protect(const motor_limits_t *lim, uint8_t out[6]);
void can_fd_codec_unpack_protect(const uint8_t in[6], motor_limits_t *lim);

uint8_t can_fd_codec_fault_to_error(fault_t fault);

uint8_t can_fd_codec_pad_canfd_len(uint8_t len);

#endif /* __CAN_FD_CUSTOM_CODEC_H__ */
