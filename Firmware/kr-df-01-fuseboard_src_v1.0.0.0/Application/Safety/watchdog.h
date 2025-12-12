/**
  ******************************************************************************
  * @file    fuseboard_watchdog.h
  * @brief   Software + hardware based watchdog.
  ******************************************************************************
  */
#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Public types --------------------------------------------------------------*/

/* Watchdog threads */
typedef enum
{
  WATCHDOG_THREAD_RELAYS,
  WATCHDOG_THREAD_LEDS,
  WATCHDOG_THREAD_MAIN,
  WATCHDOG_THREAD_COMMS,
  WATCHDOG_THREAD_UI,
  WATCHDOG_THREAD_ADC,
  WATCHDOG_THREAD_LOG,
  WATCHDOG_THREAD_CONFIG,
  WATCHDOG_THREAD_COUNT
} WATCHDOG_THREAD;

/* Public defines ------------------------------------------------------------*/
#define WATCHDOG_INTERVAL_VARIADIC 0

/* Public function declarations ----------------------------------------------*/

void watchdog_take_wwdg_init_time();
void watchdog_configure_thread(WATCHDOG_THREAD thread_id, uint32_t interval_ms, uint32_t timeout_ms);
void watchdog_kick_thread(WATCHDOG_THREAD thread_id);

#ifdef __cplusplus
}
#endif
