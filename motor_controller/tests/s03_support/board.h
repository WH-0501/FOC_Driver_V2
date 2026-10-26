#include "datatypes.h"
#include "error_types.h"
error_t current_hw_calibration_begin(uint32_t *state);
error_t current_hw_calibration_read(motor_handle_t *m);
void current_hw_calibration_end(uint32_t state);
