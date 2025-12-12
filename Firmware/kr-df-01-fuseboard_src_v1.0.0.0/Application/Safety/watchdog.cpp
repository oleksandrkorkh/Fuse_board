/**
  ******************************************************************************
  * @file    fuseboard_watchdog.h
  * @brief   Software + hardware based watchdog.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "common.h"
#include "main.h"
#include "watchdog.h"
#include "tx_api.h"

extern WWDG_HandleTypeDef hwwdg;

typedef struct
{
  uint32_t update_interval_ms;  // TODO Use it someday.
  uint32_t watchdog_timeout_ms;
  uint32_t last_kick_time;      // when was last watchdog kick triggered
} watchdog_thread_t;

/* Constants -----------------------------------------------------------------*/

/* Window watchdog refresh time window is between 34 and 66 ms based on WWDG1 initialization code.
 * Pick 50 milliseconds as it is right between them. */
#define WATCHDOG_KICK_PERIOD_MS 50

/* Variables -----------------------------------------------------------------*/
static uint32_t wwdg_init_timestamp = 0;
static watchdog_thread_t watchdog_threads[WATCHDOG_THREAD_COUNT] = { 0 };
static WATCHDOG_THREAD thread_that_caused_watchdog_reset = WATCHDOG_THREAD_COUNT;

/* Public functions ----------------------------------------------------------*/

/**
 * Record WWDG initialization time.
 * It is for MX_WWDG_Init function only.
 */
void watchdog_take_wwdg_init_time()
{
  wwdg_init_timestamp = HAL_GetTick();
}

/**
 * Thread specific configuration.
 * @param thread_id: Thread to configure
 * @param interval_ms: Thread periodic operation time in milliseconds. 0 if operating on free schedule.
 * @param timeout_ms: Thread kick timeout in milliseconds.
 */
void watchdog_configure_thread(WATCHDOG_THREAD thread_id, uint32_t interval_ms, uint32_t timeout_ms)
{
  ASSERT_FATAL(thread_id < WATCHDOG_THREAD_COUNT, "Invalid thread");

  watchdog_threads[thread_id].update_interval_ms = interval_ms;
  watchdog_threads[thread_id].watchdog_timeout_ms = timeout_ms;

  /* Start counting timeout from this moment */
  watchdog_threads[thread_id].last_kick_time = tx_time_get();
}

/**
 * Thread specific WD "kick".
 * @param thread_id: Thread to kick.
 */
void watchdog_kick_thread(WATCHDOG_THREAD thread_id)
{
  watchdog_threads[thread_id].last_kick_time = tx_time_get();
}

void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg)
{
  UNUSED(hwwdg);

  /* Get program counter when the exception was triggered */
  uint32_t pc = __get_PSP();

  /* TODO Save PC down as helpful information when WD reset occurs */
  UNUSED(pc);
}

/**
 * Watchdog own thread.
 *
 * ATTENTION!
 * It is very important that calling this functions shall be done within 65 ms after
 * calling MX_WWDG_Init in main.c otherwise the real watchdog in MCU times out.
 */
static void watchdog_thread_entry()
{
  bool watchdog_active = true;

  /* Check how much time did it take to get here from MX_WWDG_Init and make
   * a minimum delay that is required to get the kick within the time window.
   * If too much time have elapsed, then WD has already reset the MCU... */
  uint32_t elapsed_time = HAL_GetTick() - wwdg_init_timestamp;
  if (elapsed_time < WATCHDOG_KICK_PERIOD_MS)
  {
    sleep_ms(WATCHDOG_KICK_PERIOD_MS - elapsed_time);
  }
  else
  {
    ULOG_WARNING("It takes too much time until first WD kick: %u ms", elapsed_time);
  }

  /* Make the first kick */
  HAL_WWDG_Refresh(&hwwdg);

  /* Now continue kicking with regular schedule */
  uint32_t wake_time = tx_time_get();
  while (true)
  {
    sleep_until_ms(&wake_time, WATCHDOG_KICK_PERIOD_MS);
    if (watchdog_active)
    {
      HAL_WWDG_Refresh(&hwwdg);

      for (int i = 0; i < WATCHDOG_THREAD_COUNT; ++i)
      {
        /* Check for timeout only on configured threads */
        if (watchdog_threads[i].watchdog_timeout_ms > 0)
        {
          if ((wake_time - watchdog_threads[i].last_kick_time) > watchdog_threads[i].watchdog_timeout_ms)
          {
            thread_that_caused_watchdog_reset = (WATCHDOG_THREAD)i;
            watchdog_active = false;
          }
        }
      }
    }
  }
}

extern "C"
{
VOID Watchdog_Thread_Entry(ULONG)
{
  watchdog_thread_entry();
}
}
