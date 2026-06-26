/**
 * @file dwt_profile_delay.h
 * @author your name (you@domain.com)
 * @brief 基于 DWT 实现 性能统计、延时等功能
 * @version 0.1
 * @date 2026-05-08
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef __DWT_PROFILE_DELAY_H__
#define __DWT_PROFILE_DELAY_H__

#include "at32m412_416.h"

#define DWT_PROFILE_SLOT_MAX 10

typedef struct {
  uint32_t now_records[DWT_PROFILE_SLOT_MAX];
  uint32_t duration_records[DWT_PROFILE_SLOT_MAX];
  uint32_t dwt_freq_MHz; // DWT 频率，单位 MHz
  float dwt_freq_MHz_inv; // DWT 频率的倒数. 每 CPU 周期多少 us
} DWTCycleTime_t;

#define delay_ms(ms) dwt_blocking_delay_ms(ms)
#define delay_us(us) dwt_blocking_delay_us(us)

/**
 * @brief 初始化 DWT
 * 
 */
void dwt_init(void);
/**
 * @brief 标记开始
 * 
 * @param slot 槽位
 */
void dwt_profile_mark_start(uint32_t slot);
/**
 * @brief 标记停止
 * 
 * @param slot 槽位
 */
void dwt_profile_mark_stop(uint32_t slot);
/**
 * @brief 获取经过的时间
 * 
 * @param slot 槽位
 * @return float 经过的时间，单位 us
 */
float dwt_profile_get_elapsed_us(uint32_t slot);

/**
 * @brief 获取 DWT 计数器值
 * 
 * @return uint32_t DWT 计数器值
 */
uint32_t dwt_get_ticks(void);

/**
 * @brief 获取 DWT 计数器值，单位 us
 * 
 * @return uint32_t DWT 计数器值，单位 us
 */
uint32_t dwt_get_ticks_us(void);

/**
 * @brief 阻塞延时 us
 * 
 * @param us 延时时间，单位 us
 */
void dwt_blocking_delay_us(uint32_t us);
/**
 * @brief 阻塞延时 ms
 * 
 * @param ms 延时时间，单位 ms
 */
void dwt_blocking_delay_ms(uint32_t ms);

#endif /* __DWT_PROFILE_DELAY_H__ */
