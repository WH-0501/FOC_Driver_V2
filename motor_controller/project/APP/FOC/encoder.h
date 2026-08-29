#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "encoder_types.h"

/**
 * 编码器观测算法：SPI 读角后的 unwrap、速度估计、滤波。
 * 不持有 motor_axis_t；多路调用与 state 发布由 motor_axis.c 负责。
 */

void encoder_dev_init_once(encoder_dev_t dev);
void encoder_observer_init(encoder_observer_t *obs);

bool encoder_observer_update(encoder_observer_t *obs,
                             const encoder_bind_t *bind,
                             encoder_role_t role,
                             float default_zero_rad,
                             int8_t default_dir,
                             uint32_t time_us);

#endif /* __ENCODER_H__ */
