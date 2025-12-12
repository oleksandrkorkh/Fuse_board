/**
  ******************************************************************************
  * @file    util.h
  * @brief   Utility functions
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "util.h"
#include "main.h"

/* Public functions ----------------------------------------------------------*/

/**
 * Countdown timer value without underflowing.
 * @param timer: Timer value.
 * @param step: Countdown value.
 * @return true if timer has reached zero, false if not.
 */
bool util_countdown(uint64_t * timer, uint64_t step)
{
  ASSERT_FATAL(timer != NULL, "Timer pointer is NULL");

  if (*timer > step)
  {
    *timer -= step;
  }
  else
  {
    *timer = 0;
  }

  return *timer == 0;
}
