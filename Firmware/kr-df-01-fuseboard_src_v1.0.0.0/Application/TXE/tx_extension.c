/**
 * tx_extension.c
 * Helper functions for ThreadX RTOS
 */

#include "tx_extension.h"
#include "main.h"
#include "ulog.h"

// This function is defined in main.c
extern void Error_Handler(void);

/**
 * Try to take mutex with critical error checking
 * @param mutex_ptr: Pointer to mutex.
 * @param file_path: File path
 * @param file_path_length: File path length
 * @param line: Line number
 */
bool __tx_extension_take_mutex(TX_MUTEX *mutex_ptr, const char* file_path, uint32_t file_path_length, uint32_t line)
{
  // Try to get mutex
  uint32_t result = tx_mutex_get(mutex_ptr, TX_WAIT_FOREVER);
  if (result != TX_SUCCESS)
  {
    ulog_message(ULOG_CRITICAL_LEVEL, file_path, file_path_length, line, "Mutex get error %u", result);
    Error_Handler();

    // If it happens to return, then return with failed action
    return false;
  }

  return true;
}

/**
 * Try to release mutex with critical error checking
 * @param mutex_ptr: Pointer to mutex.
 * @param is_taken: Pointer to taken flag variable.
 * @param file_path: File path
 * @param file_path_length: File path length
 * @param line: Line number
 */
void __tx_extension_release_mutex(TX_MUTEX *mutex_ptr, bool *is_taken, const char* file_path, uint32_t file_path_length, uint32_t line)
{
  ASSERT_FATAL(is_taken != NULL, "Invalid taken flag");

  // Is mutex taken ?
  if (*is_taken == true)
  {
    // Try to release it
    uint32_t result = tx_mutex_put(mutex_ptr);
    if (result != TX_SUCCESS)
    {
      ulog_message(ULOG_CRITICAL_LEVEL, file_path, file_path_length, line, "Mutex put error %u", result);
      Error_Handler();
    }

    *is_taken = false;
  }
}

/**
 * Get time (from start-up) as milliseconds
 * @return Time in milliseconds
 */
ULONG tx_time_get_ms()
{
  return tx_time_get() * (1000 / TX_TIMER_TICKS_PER_SECOND);
}
