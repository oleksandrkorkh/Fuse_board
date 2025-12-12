#pragma once
#ifdef __cplusplus

#include <tx_api.h>
#include <tx_thread.h>
#include <tx_timer.h>
#include <stm32h5xx_hal.h>
#include "tx_extension.h"
#include "main.h"
#include "log.h"

extern "C"
{
#endif

#include <stdint.h>

void sleep_ms(uint32_t ms);

// Delay a thread a fixed interval;
// previous_wake_time: used to store the previous time of sleep, and must be initialized to current time before starting the loop;
// time_increment_ms: fixed update interval of the thread;
void sleep_until_ms(uint32_t *previous_wake_time, uint32_t time_increment_ms);

#ifdef __cplusplus
}
#endif
