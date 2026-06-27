#include "logger.h"
#include <stdarg.h>
#include <string.h>
#include "board.h"

#ifdef LOGGER_ENABLE_RTT
#include "SEGGER_RTT.h"
#endif

#if defined(PLATFORM_AT32)
#include "wk_usart.h"
#endif

LogLevel global_log_level = LOGGER_LEVEL_INFO;
static LogOutputType logger_output_type = LOG_OUTPUT_UART;

const char* log_level_strings[] =
{
  "DEBUG",
  "INFO",
  "WARN",
  "ERROR"
};

static void logger_output_uart(const char* str, size_t len)
{
  if ((str == NULL) || (len == 0u))
  {
    return;
  }

  (void)board_uart_log_try((const uint8_t *)str, (uint16_t)len);
}

static int logger_uart_getchar(void)
{
#if defined(PLATFORM_AT32)
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
  char buffer[256];
  int len;

  (void)level;

  switch (logger_output_type)
  {
    case LOG_OUTPUT_UART:
    default:
      va_start(args, format);
      len = vsnprintf(buffer, sizeof(buffer), format, args);
      va_end(args);
      if ((len > 0) && (len < (int)sizeof(buffer)))
      {
        logger_output_uart(buffer, (size_t)len);
      }
      break;

#ifdef LOGGER_ENABLE_RTT
    case LOG_OUTPUT_RTT:
      va_start(args, format);
      len = vsnprintf(buffer, sizeof(buffer), format, args);
      va_end(args);
      if ((len > 0) && (len < (int)sizeof(buffer)))
      {
        unsigned int written = SEGGER_RTT_Write(LOGGER_RTT_CHANNEL, buffer, len);
        (void)written;
      }
      break;

    case LOG_OUTPUT_BOTH:
      va_start(args, format);
      len = vsnprintf(buffer, sizeof(buffer), format, args);
      va_end(args);
      if ((len > 0) && (len < (int)sizeof(buffer)))
      {
        unsigned int written = SEGGER_RTT_Write(LOGGER_RTT_CHANNEL, buffer, len);
        logger_output_uart(buffer, (size_t)len);
        (void)written;
      }
      break;
#endif
  }
}

void Logger_Init(LogOutputType output_type)
{
  logger_output_type = output_type;
  board_uart_stream_mode_set(0u);

#ifdef LOGGER_ENABLE_RTT
  if ((output_type == LOG_OUTPUT_RTT) || (output_type == LOG_OUTPUT_BOTH))
  {
    SEGGER_RTT_Init();
    SEGGER_RTT_SetNameUpBuffer(LOGGER_RTT_CHANNEL, LOGGER_NAME);
  }
#endif
}

void SetLogLevel(LogLevel level)
{
  global_log_level = level;
}

void Logger_SetOutputType(LogOutputType output_type)
{
  logger_output_type = output_type;
}

LogOutputType Logger_GetOutputType(void)
{
  return logger_output_type;
}

void logger_dma_irq_handler(void)
{
  board_uart_dma_irq_handler();
}

void logger_get_stats(logger_stats_t *stats)
{
  board_uart_diag_t diag;

  if (stats == NULL)
  {
    return;
  }

  board_uart_get_diag(&diag);
  stats->ring_size = diag.ring_size;
  stats->used = diag.used;
  stats->free = diag.free;
  stats->tx_inflight = diag.tx_inflight;
  stats->dma_busy = diag.dma_busy;
  stats->dropped_messages = diag.dropped_messages;
  stats->dropped_bytes = diag.dropped_bytes;
}

int fputc(int ch, FILE *f)
{
  char c;

  (void)f;
  c = (char)ch;

  switch (logger_output_type)
  {
    case LOG_OUTPUT_UART:
    default:
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
  }

  return ch;
}

int fgetc(FILE *f)
{
  (void)f;
  return logger_uart_getchar();
}
