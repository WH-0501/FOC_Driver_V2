#include "logger.h"
#include <stdarg.h>
#include <string.h>

// 如果启用 RTT 支持，包含 RTT 头文件
#ifdef LOGGER_ENABLE_RTT
#include "SEGGER_RTT.h"
#endif

LogLevel global_log_level = LOGGER_LEVEL_INFO;
static LogOutputType logger_output_type = LOG_OUTPUT_UART; // 默认使用 UART

const char* log_level_strings[] =
{
  "DEBUG",
  "INFO",
  "WARN",
  "ERROR"
};

#if defined(PLATFORM_STM32)
#include "stm32h7xx_hal.h"
extern UART_HandleTypeDef huart1;
#define LOGGER_UART_HANDLE huart1
#elif defined(PLATFORM_AT32)
#include "wk_usart.h"
#include "wk_dma.h"
#endif

#ifdef LOGGER_ENABLE_RTT
static char rtt_buffer[1024];
#endif

#if defined(PLATFORM_AT32)
#ifndef LOGGER_UART_USE_DMA
#define LOGGER_UART_USE_DMA 1
#endif
static uint8_t s_logger_dma_tx_buf[LOGGER_UART_TX_BUF_SIZE];
#if LOGGER_UART_USE_DMA
static uint8_t s_logger_dma_ring[LOGGER_UART_DMA_RING_SIZE];
static volatile uint16_t s_logger_dma_head;
static volatile uint16_t s_logger_dma_tail;
static volatile uint16_t s_logger_dma_tx_len;
static volatile uint8_t s_logger_dma_busy;
static volatile uint32_t s_logger_dma_dropped_messages;
static volatile uint32_t s_logger_dma_dropped_bytes;
#endif
#endif

#if defined(PLATFORM_AT32)
static void logger_output_uart_polling_at32(const char* str, size_t len)
{
  size_t i;
  for (i = 0u; i < len; ++i)
  {
    while (usart_flag_get(USART1, USART_TDBE_FLAG) == RESET)
    {
    }
    usart_data_transmit(USART1, (uint16_t)(uint8_t)str[i]);
    while (usart_flag_get(USART1, USART_TDC_FLAG) == RESET)
    {
    }
  }
}

#if LOGGER_UART_USE_DMA
static uint16_t logger_dma_ring_available(void)
{
  uint16_t head;
  uint16_t tail;

  head = s_logger_dma_head;
  tail = s_logger_dma_tail;
  if (head >= tail)
  {
    return (uint16_t)(LOGGER_UART_DMA_RING_SIZE - (head - tail) - 1u);
  }
  return (uint16_t)(tail - head - 1u);
}

static uint16_t logger_dma_ring_used(void)
{
  uint16_t head;
  uint16_t tail;

  head = s_logger_dma_head;
  tail = s_logger_dma_tail;
  if (head >= tail)
  {
    return (uint16_t)(head - tail);
  }
  return (uint16_t)(LOGGER_UART_DMA_RING_SIZE - (tail - head));
}

static void logger_dma_start_transfer_locked(void)
{
  uint16_t tail;
  uint16_t head;
  uint16_t tx_len;

  if (s_logger_dma_busy != 0u)
  {
    return;
  }

  tail = s_logger_dma_tail;
  head = s_logger_dma_head;
  if (tail == head)
  {
    return;
  }

  if (head > tail)
  {
    tx_len = (uint16_t)(head - tail);
  }
  else
  {
    tx_len = (uint16_t)(LOGGER_UART_DMA_RING_SIZE - tail);
  }

  if (tx_len == 0u)
  {
    return;
  }

  s_logger_dma_tx_len = tx_len;
  s_logger_dma_busy = 1u;
  dma_channel_enable(DMA1_CHANNEL3, FALSE);
  dma_flag_clear(DMA1_GL3_FLAG);
  wk_dma_channel_config(DMA1_CHANNEL3, (uint32_t)&USART1->dt, (uint32_t)&s_logger_dma_ring[tail], tx_len);
  dma_channel_enable(DMA1_CHANNEL3, TRUE);
}

static void logger_output_uart_dma_at32(const char* str, size_t len)
{
  size_t i;
  uint32_t primask;
  uint16_t free_space;

  if ((str == NULL) || (len == 0u))
  {
    return;
  }

  if (len > (size_t)LOGGER_UART_TX_BUF_SIZE)
  {
    len = (size_t)LOGGER_UART_TX_BUF_SIZE;
  }
  memcpy(s_logger_dma_tx_buf, str, len);

  primask = __get_PRIMASK();
  __disable_irq();
  free_space = logger_dma_ring_available();
  if ((size_t)free_space < len)
  {
    s_logger_dma_dropped_bytes += (uint32_t)(len - (size_t)free_space);
    s_logger_dma_dropped_messages += 1u;
    len = (size_t)free_space;
  }
  for (i = 0u; i < len; ++i)
  {
    s_logger_dma_ring[s_logger_dma_head] = s_logger_dma_tx_buf[i];
    s_logger_dma_head = (uint16_t)((s_logger_dma_head + 1u) % LOGGER_UART_DMA_RING_SIZE);
  }
  logger_dma_start_transfer_locked();
  if (primask == 0u)
  {
    __enable_irq();
  }
}

void logger_dma_irq_handler(void)
{
  uint32_t primask;

#if !defined(PLATFORM_AT32) || !LOGGER_UART_USE_DMA
  return;
#else
  if (dma_flag_get(DMA1_DTERR3_FLAG) != RESET)
  {
    dma_channel_enable(DMA1_CHANNEL3, FALSE);
    dma_flag_clear(DMA1_GL3_FLAG);
    s_logger_dma_busy = 0u;
    s_logger_dma_tx_len = 0u;
    return;
  }

  if (dma_flag_get(DMA1_FDT3_FLAG) == RESET)
  {
    return;
  }

  dma_channel_enable(DMA1_CHANNEL3, FALSE);
  dma_flag_clear(DMA1_GL3_FLAG);

  primask = __get_PRIMASK();
  __disable_irq();
  s_logger_dma_tail = (uint16_t)((s_logger_dma_tail + s_logger_dma_tx_len) % LOGGER_UART_DMA_RING_SIZE);
  s_logger_dma_tx_len = 0u;
  s_logger_dma_busy = 0u;
  logger_dma_start_transfer_locked();
  if (primask == 0u)
  {
    __enable_irq();
  }
#endif
}
#else
void logger_dma_irq_handler(void)
{
}
#endif
#endif

void logger_get_stats(logger_stats_t *stats)
{
  if (stats == NULL)
  {
    return;
  }

#if defined(PLATFORM_AT32) && LOGGER_UART_USE_DMA
  {
    uint32_t primask;
    uint16_t used;
    uint16_t free_space;

    primask = __get_PRIMASK();
    __disable_irq();
    used = logger_dma_ring_used();
    free_space = logger_dma_ring_available();
    stats->ring_size = LOGGER_UART_DMA_RING_SIZE;
    stats->used = used;
    stats->free = free_space;
    stats->tx_inflight = s_logger_dma_tx_len;
    stats->dma_busy = s_logger_dma_busy;
    stats->dropped_messages = s_logger_dma_dropped_messages;
    stats->dropped_bytes = s_logger_dma_dropped_bytes;
    if (primask == 0u)
    {
      __enable_irq();
    }
  }
#else
  stats->ring_size = 0u;
  stats->used = 0u;
  stats->free = 0u;
  stats->tx_inflight = 0u;
  stats->dma_busy = 0u;
  stats->dropped_messages = 0u;
  stats->dropped_bytes = 0u;
#endif
}

static void logger_output_uart(const char* str, size_t len)
{
  if (len == 0u)
  {
    return;
  }

#if defined(PLATFORM_STM32)
  HAL_UART_Transmit(&LOGGER_UART_HANDLE, (uint8_t *)str, (uint16_t)len, HAL_MAX_DELAY);
#elif defined(PLATFORM_AT32)
#if LOGGER_UART_USE_DMA
  logger_output_uart_dma_at32(str, len);
#else
  logger_output_uart_polling_at32(str, len);
#endif
#else
  (void)str;
  (void)len;
#endif
}

static int logger_uart_getchar(void)
{
#if defined(PLATFORM_STM32)
  uint8_t ch = 0u;
  HAL_UART_Receive(&LOGGER_UART_HANDLE, &ch, 1u, 0xffffu);
  return (int)ch;
#elif defined(PLATFORM_AT32)
  while (usart_flag_get(USART1, USART_RDBF_FLAG) == RESET)
  {
  }
  return (int)(uint8_t)usart_data_receive(USART1);
#else
  return -1;
#endif
}

void logger_output(LogLevel level, const char* format, ...)
{
  va_list args;

  // 根据输出方式选择输出通道
  switch (logger_output_type)
  {
    case LOG_OUTPUT_UART:
    {
      // UART 输出：先格式化再发送
      char buffer[256];
      int len;

      va_start(args, format);
      len = vsnprintf(buffer, sizeof(buffer), format, args);
      va_end(args);

      if (len > 0 && len < (int)sizeof(buffer))
      {
        logger_output_uart(buffer, len);
      }
      break;
    }

#ifdef LOGGER_ENABLE_RTT
    case LOG_OUTPUT_RTT:
    {
      char buffer[256];
      int len;

      va_start(args, format);
      len = vsnprintf(buffer, sizeof(buffer), format, args);
      va_end(args);

      if (len > 0 && len < (int)sizeof(buffer))
      {
        // SEGGER_RTT_printf(LOGGER_RTT_CHANNEL, "%s", buffer);
        // 使用 SEGGER_RTT_Write 替代 SEGGER_RTT_printf，避免格式化开销
        unsigned int written = SEGGER_RTT_Write(LOGGER_RTT_CHANNEL, buffer, len);
        (void)written; // 避免未使用变量警告
      }
      break;
    }

    case LOG_OUTPUT_BOTH:
    {
      // 同时输出到 UART 和 RTT
      char buffer[256];
      int len;

      va_start(args, format);
      len = vsnprintf(buffer, sizeof(buffer), format, args);
      va_end(args);

      if (len > 0 && len < (int)sizeof(buffer))
      {
        // 输出到 UART
        logger_output_uart(buffer, len);

        // SEGGER_RTT_printf(LOGGER_RTT_CHANNEL, "%s", buffer);
        unsigned int written = SEGGER_RTT_Write(LOGGER_RTT_CHANNEL, buffer, len);
        (void)written; // 避免未使用变量警告
      }
      break;
    }
#endif

    default:
    {
      // 默认使用 UART
      char buffer[256];
      int len;

      va_start(args, format);
      len = vsnprintf(buffer, sizeof(buffer), format, args);
      va_end(args);

      if (len > 0 && len < (int)sizeof(buffer))
      {
        logger_output_uart(buffer, len);
      }
      break;
    }
  }
}

// 初始化 logger 模块    决定调用哪种方式输出log，M4调用这个函数时，参数为 LOG_OUTPUT_UART，M7调用这个函数时，参数为 LOG_OUTPUT_RTT
void Logger_Init(LogOutputType output_type)
{
  logger_output_type = output_type;
#if defined(PLATFORM_AT32) && LOGGER_UART_USE_DMA
  s_logger_dma_head = 0u;
  s_logger_dma_tail = 0u;
  s_logger_dma_tx_len = 0u;
  s_logger_dma_busy = 0u;
  s_logger_dma_dropped_messages = 0u;
  s_logger_dma_dropped_bytes = 0u;
  dma_interrupt_enable(DMA1_CHANNEL3, DMA_FDT_INT, TRUE);
  dma_interrupt_enable(DMA1_CHANNEL3, DMA_DTERR_INT, TRUE);
#endif

#ifdef LOGGER_ENABLE_RTT
  // 初始化 RTT (如果使用 RTT)
  if (output_type == LOG_OUTPUT_RTT || output_type == LOG_OUTPUT_BOTH) {
    SEGGER_RTT_Init();

    // 设置通道名称（可选，用于 RTT Control Panel）
    SEGGER_RTT_SetTerminal(LOGGER_RTT_TERMINAL_ID);
    SEGGER_RTT_SetNameUpBuffer(LOGGER_RTT_CHANNEL, LOGGER_NAME);
    if (LOGGER_RTT_CHANNEL == 1) {
      SEGGER_RTT_ConfigUpBuffer(1,
                                LOGGER_NAME,
                                logger_rtt_buffer,
                                sizeof(logger_rtt_buffer),
                                SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    } else {
      SEGGER_RTT_SetNameUpBuffer(LOGGER_RTT_CHANNEL, LOGGER_NAME);
    }
  }
#endif
}

// 设置日志等级
void SetLogLevel(LogLevel level)
{
  global_log_level = level;
}

// 设置输出方式
void Logger_SetOutputType(LogOutputType output_type)
{
  logger_output_type = output_type;
}

// 获取当前输出方式
LogOutputType Logger_GetOutputType(void)
{
  return logger_output_type;
}

// 为了向后兼容，保留 fputc 和 fgetc 函数
// 这些函数用于 printf 重定向（如果代码中直接使用 printf）
int fputc(int ch, FILE *f)
{
  char c = (char)ch;

  switch (logger_output_type)
  {
    case LOG_OUTPUT_UART:
      logger_output_uart(&c, 1u);
      break;

#ifdef LOGGER_ENABLE_RTT
    case LOG_OUTPUT_RTT:
      SEGGER_RTT_PutChar(LOGGER_RTT_CHANNEL, c);
      break;

    case LOG_OUTPUT_BOTH:
      logger_output_uart(&c, 1u);
      SEGGER_RTT_PutChar(LOGGER_RTT_CHANNEL, c);
      break;
#endif

    default:
      logger_output_uart(&c, 1u);
      break;
  }

  return ch;
}

int fgetc(FILE *f)
{
  (void)f;
  return logger_uart_getchar();
}
