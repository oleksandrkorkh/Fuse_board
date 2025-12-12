#pragma once
/**
 * tx_extension.h
 * Helper functions for ThreadX RTOS
 */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <tx_api.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Macros --------------------------------------------------------------------*/
#define TX_MUTEX_SECTION(pmtx) for (bool is_taken = __tx_extension_take_mutex(pmtx, __FILE__, sizeof(__FILE__), __LINE__); is_taken == true; __tx_extension_release_mutex(pmtx, &is_taken, __FILE__, sizeof(__FILE__), __LINE__))

/* Get number of OS ticks from milliseconds. It rounds time upwards. E.g. 1 ms becomes 10 ms. */
#define TX_TICKS_MS(ms) ((((ms) * TX_TIMER_TICKS_PER_SECOND) + 999) / 1000)

/* "Private" function prototypes ---------------------------------------------*/
bool __tx_extension_take_mutex(TX_MUTEX *mutex_ptr, const char* file_path, uint32_t file_path_length, uint32_t line);
void __tx_extension_release_mutex(TX_MUTEX *mutex_ptr, bool *is_taken, const char* file_path, uint32_t file_path_length, uint32_t line);

/* Public functions ----------------------------------------------------------*/
ULONG tx_time_get_ms();

#ifdef __cplusplus
}
#endif
