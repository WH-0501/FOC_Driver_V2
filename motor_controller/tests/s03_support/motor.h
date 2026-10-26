#include "datatypes.h"
typedef struct { uint32_t offset_sum[3]; uint32_t offset_cnt; uint8_t offset_active; } motor_runtime_t;
motor_runtime_t *motor_runtime(motor_handle_t *m);
void motor_enter_dc_cal_hw(motor_handle_t *m);
void motor_disarm(motor_handle_t *m);
