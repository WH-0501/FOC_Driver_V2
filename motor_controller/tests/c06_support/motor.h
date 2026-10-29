#ifndef C06_MOTOR_STUB_H
#define C06_MOTOR_STUB_H
#include "datatypes.h"
typedef struct {
  float current_soft_speed_rad_s,current_iq_limited;
  bool current_speed_limited;
  uint8_t fsm_enter_pending;
  bool electrical_align_done;
  float open_loop_elec_angle_rad,open_loop_omega,open_loop_uq,open_loop_iq;
  uint32_t foc_pos_loop_us,foc_vel_loop_us,foc_cur_loop_us,foc_ctrl_loop_us;
} motor_runtime_t;
motor_handle_t *motor_get_handler(void);
motor_runtime_t *motor_runtime(motor_handle_t *m);
bool motor_is_armed(const motor_handle_t *m);
void motor_disarm(motor_handle_t *m);
void motor_shutdown(motor_handle_t *m);
void motor_sync_arm_with_mode(motor_handle_t *m);
#endif
