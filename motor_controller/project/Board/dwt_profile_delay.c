#include "dwt_profile_delay.h"
#include "system_at32m412_416.h"

static DWTCycleTime_t dwt_cycle_tim = {
  .now_records = {0},
  .duration_records = {0},
  .dwt_freq_MHz = 0,
  .dwt_freq_MHz_inv = 0,
};

void dwt_init(void)
{
  /* Trace enable is not initialization of this application's conversion state.
   * A bootloader/debugger may leave Trace on with CYCCNT stopped. Preserve
   * the counter, always refresh conversion and enable counting. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  dwt_cycle_tim.dwt_freq_MHz = SystemCoreClock / 1000000; // 转换为 MHz
  dwt_cycle_tim.dwt_freq_MHz_inv = 1.0f / (float)dwt_cycle_tim.dwt_freq_MHz;
}

void dwt_profile_mark_start(uint32_t slot)
{
  if (slot >= DWT_PROFILE_SLOT_MAX) {
    return;
  }
  dwt_cycle_tim.now_records[slot] = DWT->CYCCNT;
}

void dwt_profile_mark_stop(uint32_t slot)
{
  if (slot >= DWT_PROFILE_SLOT_MAX) {
    return;
  }
  dwt_cycle_tim.duration_records[slot] = DWT->CYCCNT - dwt_cycle_tim.now_records[slot];
}

float dwt_profile_get_elapsed_us(uint32_t slot)
{
  if (slot >= DWT_PROFILE_SLOT_MAX) {
    return 0.0f;
  }
  return (float)dwt_cycle_tim.duration_records[slot] * dwt_cycle_tim.dwt_freq_MHz_inv;
}

uint32_t dwt_get_ticks(void)
{
  return DWT->CYCCNT;
}

uint32_t dwt_get_ticks_us(void)
{
  return (uint32_t)(dwt_get_ticks() * dwt_cycle_tim.dwt_freq_MHz_inv);
}

void dwt_blocking_delay_us(uint32_t us)
{
  if (us == 0) {
    return;
  }

  if (!dwt_cycle_tim.dwt_freq_MHz || !(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) ||
      !(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) dwt_init();

  uint32_t start_ticks = dwt_get_ticks();
  uint32_t delay_ticks = (uint32_t)(us * dwt_cycle_tim.dwt_freq_MHz);
  uint32_t end_ticks = start_ticks + delay_ticks;

  // 处理计数器溢出
  if (end_ticks >= start_ticks) {
    while (dwt_get_ticks() < end_ticks) {
      __NOP();
    }
  } else {
    while (dwt_get_ticks() >= start_ticks) {
      __NOP();
    }
    while (dwt_get_ticks() < end_ticks) {
      __NOP();
    }
  }
}

void dwt_blocking_delay_ms(uint32_t ms)
{
  dwt_blocking_delay_us(ms * 1000);
}
