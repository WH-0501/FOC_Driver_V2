#include "board.h"

void board_init(motor_handle_t *m)
{
  (void)m;
  current_hw_init();
}

void board_deinit(void)
{
  current_hw_deinit();
}
