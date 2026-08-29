#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "datatypes.h"
#include "encoder_types.h"

/**
 * 编码器模块：单/双路绑定、观测算法、发布 motor_state 机械观测量。
 */

void encoder_init(void);
void encoder_update(motor_handle_t *m);

void encoder_dev_init_once(encoder_dev_t dev);
void encoder_observer_init(encoder_observer_t *obs);

bool encoder_observer_update(encoder_observer_t *obs,
                             const encoder_bind_t *bind,
                             encoder_role_t role,
                             float default_zero_rad,
                             int8_t default_dir,
                             uint32_t time_us);

#endif /* __ENCODER_H__ */
