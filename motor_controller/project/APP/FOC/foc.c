#include "foc.h"
#include <string.h>

motor_state_t g_motor_state;

void foc_init(void)
{
  memset(&g_motor_state, 0, sizeof(g_motor_state));
}

void foc_update(void)
{
}

#if defined(__ARMCC_VERSION)
__weak void foc_current_loop_control(void)
#else
__attribute__((weak)) void foc_current_loop_control(void)
#endif
{
    get_phase_current();
}

void foc_state_machint_loop(void)
{

}
