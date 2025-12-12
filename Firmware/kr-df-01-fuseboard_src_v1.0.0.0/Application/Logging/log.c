#include <stdio.h>
#include <tx_api.h>

#include "log.h"
#include "SEGGER_RTT.h"

#define COLOR_INFO     "\033[0;32m"     // Green
#define COLOR_WARNING  "\033[0;33m"     // Yellow
#define COLOR_ERROR    "\033[0;31m"     // Red
#define COLOR_CRITICAL "\033[1;31m"     // Bold Red
#define COLOR_RESET    "\033[0m"        // Reset color

static const char* Get_ANSI_Color(ulog_level_t severity)
{
  switch (severity)
  {
  case ULOG_INFO_LEVEL:
    return COLOR_INFO;
  case ULOG_WARNING_LEVEL:
    return COLOR_WARNING;
  case ULOG_ERROR_LEVEL:
    return COLOR_ERROR;
  case ULOG_CRITICAL_LEVEL:
    return COLOR_CRITICAL;
  default:
    return COLOR_RESET;
  }
}

static void Log_RTT_Write(ulog_level_t severity, const char *msg)
{
  const double time = (double)tx_time_get() / TX_TIMER_TICKS_PER_SECOND;
  printf("%s%05.3f [%s]: %s%s\r\n", Get_ANSI_Color(severity), time, ulog_level_name(severity), msg, COLOR_RESET);
}

void log_init()
{
  SEGGER_RTT_Init();

  ULOG_INIT();
  ULOG_SUBSCRIBE(Log_RTT_Write, ULOG_DEBUG_LEVEL);
  ULOG_WARNING("Boot.");
}
