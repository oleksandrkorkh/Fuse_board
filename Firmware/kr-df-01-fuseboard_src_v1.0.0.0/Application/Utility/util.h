#pragma once
/**
  ******************************************************************************
  * @file    util.h
  * @brief   Utility functions
  ******************************************************************************
  */

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

/* Macros --------------------------------------------------------------------*/

/**
 * Minimum value of two value
 * @param a: Value A
 * @param B: Value B
 * @return Smaller of A and B
 */
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

/**
 * Maximum value of two value
 * @param a: Value A
 * @param B: Value B
 * @return Greater of A and B
 */
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

/**
 * @brief Count elements in array
 * @param array: Array (of any type)
 */
#ifndef COUNT_OF
#define COUNT_OF(array) (sizeof(array) / sizeof((array)[0]))
#endif

/* Public function declarations ----------------------------------------------*/
bool util_countdown(uint64_t * timer, uint64_t step);

#ifdef __cplusplus
}
#endif
