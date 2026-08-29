#include "datatypes.h"
#include "error_types.h"
#include "gate/gate_driver.h"
#include "gate/mp6540.h"

typedef struct
{
  bool armed;
  uint8_t axis_id;
  MP6540_Handle_t mp6540;
  const gate_driver_t *gate;
} motor_driver_t;

error_t motor_driver_init(motor_driver_t *drv, uint8_t axis_id);
const gate_driver_t *motor_driver_get_gate(const motor_driver_t *drv);

void motor_driver_arm(motor_driver_t *drv);
void motor_driver_disarm(motor_driver_t *drv);
void motor_driver_shutdown(motor_driver_t *drv);
bool motor_driver_is_armed(const motor_driver_t *drv);

void motor_driver_prepare_dc_offset_cal(motor_driver_t *drv);
error_t motor_driver_set_pwm(motor_driver_t *drv, const motor_actuation_t *actuation);
