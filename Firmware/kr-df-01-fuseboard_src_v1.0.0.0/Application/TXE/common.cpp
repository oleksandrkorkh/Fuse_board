#include "common.h"

void sleep_ms(uint32_t ms)
{
  tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND * ms / 1000);
}

void sleep_until_ms(uint32_t *previous_wake_time, uint32_t time_increment_ms)
{
  const uint32_t current_time = tx_time_get();
  uint32_t time_to_wake = *previous_wake_time + (time_increment_ms * TX_TIMER_TICKS_PER_SECOND / 1000);

  if ((time_to_wake > current_time))
  {
    tx_thread_sleep(time_to_wake - current_time);
  }
  *previous_wake_time = time_to_wake;
}
