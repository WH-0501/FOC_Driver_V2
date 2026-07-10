#include "board.h"

static board_current_loop_cb_t s_current_loop_cb;

int board_get_gate_hw(gate_hw_binding_t *hw)
{
  (void)hw;
  return -1;
}

void board_register_current_loop_callback(board_current_loop_cb_t cb)
{
  s_current_loop_cb = cb;
}

void board_init(void)
{
}

void board_deinit(void)
{
  (void)current_hw_deinit();
}

void board_current_loop_start(void)
{
  (void)current_hw_init();
}

void board_current_loop_stop(void)
{
  (void)current_hw_deinit();
}

void board_invoke_current_loop(void)
{
  if (s_current_loop_cb != NULL)
  {
    s_current_loop_cb();
  }
}
