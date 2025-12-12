/**
 * @brief   Binary utility functions.
 * @author  Krakul OÜ
 * @ingroup components
 * @details This component provides utility functions to convert binary data.
 *
 *          DO NOT CHANGE IN PROJECT!
 */

/* Double inclusion protection -----------------------------------------------*/
#ifndef INC_BINUTILS_H
#define INC_BINUTILS_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Language boundary ---------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

/* Public types --------------------------------------------------------------*/

/**< Endianness */
typedef enum {
  binutils_LITTLE,
  binutils_BIG,
} binutils_endianness;

/* Public functions ----------------------------------------------------------*/

extern uint16_t binutils_bytes_to_uint16(const uint8_t bytes[], uint32_t index, binutils_endianness end);
extern uint32_t binutils_bytes_to_uint24(const uint8_t bytes[], uint32_t index, binutils_endianness end);
extern uint32_t binutils_bytes_to_uint32(const uint8_t bytes[], uint32_t index, binutils_endianness end);
extern uint64_t binutils_bytes_to_uint64(const uint8_t bytes[], uint32_t index, binutils_endianness end);
extern void     binutils_uint16_to_bytes(uint16_t value, uint8_t bytes[], uint32_t index, binutils_endianness end);
extern void     binutils_uint24_to_bytes(uint32_t value, uint8_t bytes[], uint32_t index, binutils_endianness end);
extern void     binutils_uint32_to_bytes(uint32_t value, uint8_t bytes[], uint32_t index, binutils_endianness end);
extern void     binutils_uint63_to_bytes(uint64_t value, uint8_t bytes[], uint32_t index, binutils_endianness end);

/* Language boundary ---------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif
