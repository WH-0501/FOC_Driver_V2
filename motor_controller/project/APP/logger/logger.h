/**
 * @file logger.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-06-26
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef __LOGGER_H__
#define __LOGGER_H__

#include <stdio.h>
#include <stdint.h>

#ifndef LOGGER_UART_TX_BUF_SIZE
#define LOGGER_UART_TX_BUF_SIZE 256u
#endif

#ifndef LOGGER_UART_DMA_TIMEOUT
#define LOGGER_UART_DMA_TIMEOUT 1000000u
#endif

#ifndef LOGGER_UART_DMA_RING_SIZE
#define LOGGER_UART_DMA_RING_SIZE 1024u
#endif

// 日志输出方式
typedef enum {
    LOG_OUTPUT_UART = 0,  // 使用 UART 输出
    LOG_OUTPUT_RTT,       // 使用 RTT 输出
    LOG_OUTPUT_BOTH       // 同时使用 UART 和 RTT
  } LogOutputType;

// 日志级别
typedef enum {
    LOGGER_LEVEL_DEBUG = 0,
    LOGGER_LEVEL_INFO = 1,
    LOGGER_LEVEL_WARNING = 2,
    LOGGER_LEVEL_ERROR = 3,
} LogLevel;

typedef struct
{
  uint16_t ring_size;
  uint16_t used;
  uint16_t free;
  uint16_t tx_inflight;
  uint8_t dma_busy;
  uint32_t dropped_messages;
  uint32_t dropped_bytes;
} logger_stats_t;

extern LogLevel global_log_level;
extern const char* log_level_strings[];

#define LOGGER_RTT_CHANNEL  0  // 默认使用通道 0
#define LOGGER_CORE_NAME    "MC"
#define LOGGER_NAME         "MC_Logger"

// 日志输出宏
#define LOG(level, format, ...) \
  do { \
    if (level >= global_log_level) { \
      logger_output(level, "[%s] [%s] %s:%d: " format "\r\n", \
                    log_level_strings[level], LOGGER_CORE_NAME, __func__, __LINE__, ##__VA_ARGS__); \
  } \
  } while (0)

#define LOG_DEBUG(format, ...) LOG(LOGGER_LEVEL_DEBUG, format, ##__VA_ARGS__)
#define LOG_INFO(format, ...)  LOG(LOGGER_LEVEL_INFO,  format, ##__VA_ARGS__)
#define LOG_WARN(format, ...)  LOG(LOGGER_LEVEL_WARNING,  format, ##__VA_ARGS__)
#define LOG_ERROR(format, ...) LOG(LOGGER_LEVEL_ERROR, format, ##__VA_ARGS__)

// 初始化 logger 模块
void Logger_Init(LogOutputType output_type);

// 设置日志等级
void SetLogLevel(LogLevel level);

// 设置输出方式
void Logger_SetOutputType(LogOutputType output_type);

// 获取当前输出方式
LogOutputType Logger_GetOutputType(void);

// 内部输出函数(供宏使用)
void logger_output(LogLevel level, const char* format, ...);

/* AT32: DMA1 Channel3 中断入口调用 */
void logger_dma_irq_handler(void);
void logger_get_stats(logger_stats_t *stats);

#endif /* __LOGGER_H__ */
