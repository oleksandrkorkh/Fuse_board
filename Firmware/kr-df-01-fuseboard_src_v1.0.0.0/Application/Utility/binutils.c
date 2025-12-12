/**
 * @brief   Binary utility functions.
 * @author  Krakul OÜ
 * @ingroup components
 * @details This component provides utility functions to convert binary data.
 *
 *          DO NOT CHANGE IN PROJECT!
 */

/* Includes ------------------------------------------------------------------*/
#include "binutils.h"

/**
  * @brief  Converting byte array to 16-bit unsigned integer
  * @param  bytes: Byte array
  * @param  index: Array start index
  * @param  end: Endianness
  * @retval 16-bit unsigned integer
  */
uint16_t binutils_bytes_to_uint16(const uint8_t bytes[], uint32_t index, binutils_endianness end)
{
  uint16_t value;

  if (end == binutils_LITTLE) {
    value =
      ((uint16_t)bytes[index + 1] << 8u) |
      ((uint16_t)bytes[index]);
  } else {
    value =
      ((uint16_t)bytes[index] << 8u) |
      ((uint16_t)bytes[index + 1]);
  }

  return value;
}

/**
  * @brief  Converting byte array to 24-bit unsigned integer
  * @param  bytes: Byte array
  * @param  index: Array start index
  * @param  end: Endianness
  * @retval 32-bit unsigned integer, but only 24 lowest bits are set.
  */
uint32_t binutils_bytes_to_uint24(const uint8_t bytes[], uint32_t index, binutils_endianness end)
{
  uint32_t value;

  if (end == binutils_LITTLE) {
    value =
      ((uint32_t)bytes[index + 2] << 16u) |
      ((uint32_t)bytes[index + 1] <<  8u) |
      ((uint32_t)bytes[index    ]);
  } else {
    value =
      ((uint32_t)bytes[index    ] << 16u) |
      ((uint32_t)bytes[index + 1] <<  8u) |
      ((uint32_t)bytes[index + 2]);
  }

  return value;
}

/**
  * @brief  Converting byte array to 32-bit unsigned integer
  * @param  bytes: Byte array
  * @param  index: Array start index
  * @param  end: Endianness
  * @retval 32-bit unsigned integer
  */
uint32_t binutils_bytes_to_uint32(const uint8_t bytes[], uint32_t index, binutils_endianness end)
{
  uint32_t value;

  if (end == binutils_LITTLE) {
    value =
      ((uint32_t)bytes[index + 3] << 24u) |
      ((uint32_t)bytes[index + 2] << 16u) |
      ((uint32_t)bytes[index + 1] <<  8u) |
      ((uint32_t)bytes[index    ]);
  } else {
    value =
      ((uint32_t)bytes[index    ] << 24u) |
      ((uint32_t)bytes[index + 1] << 16u) |
      ((uint32_t)bytes[index + 2] <<  8u) |
      ((uint32_t)bytes[index + 3]);
  }

  return value;
}

/**
  * @brief  Converting byte array to 64-bit unsigned integer
  * @param  bytes: Byte array
  * @param  index: Array start index
  * @param  end: Endianness
  * @retval 64-bit unsigned integer
  */
uint64_t binutils_bytes_to_uint64(const uint8_t bytes[], uint32_t index, binutils_endianness end)
{
  uint64_t value;

  if (end == binutils_LITTLE) {
    value =
      ((uint64_t)bytes[index + 7] << 56u) |
      ((uint64_t)bytes[index + 6] << 48u) |
      ((uint64_t)bytes[index + 5] << 40u) |
      ((uint64_t)bytes[index + 4] << 32u) |
      ((uint64_t)bytes[index + 3] << 24u) |
      ((uint64_t)bytes[index + 2] << 16u) |
      ((uint64_t)bytes[index + 1] <<  8u) |
      ((uint64_t)bytes[index    ]);
  } else {
    value =
      ((uint64_t)bytes[index    ] << 56u) |
      ((uint64_t)bytes[index + 1] << 48u) |
      ((uint64_t)bytes[index + 2] << 40u) |
      ((uint64_t)bytes[index + 3] << 32u) |
      ((uint64_t)bytes[index + 4] << 24u) |
      ((uint64_t)bytes[index + 5] << 16u) |
      ((uint64_t)bytes[index + 6] <<  8u) |
      ((uint64_t)bytes[index + 7]);
  }

  return value;
}

/**
  * @brief  Converting 16-bit unsigned integer to byte array
  * @param  value: 16-bit unsigned integer
  * @param  bytes: Byte array
  * @param  index: Array start index
  * @param  end: Endianness
  */
void binutils_uint16_to_bytes(uint16_t value, uint8_t bytes[], uint32_t index, binutils_endianness end)
{
  if (end == binutils_LITTLE) {
    bytes[index + 1] = (value >> 8u) & 0xFFu;
    bytes[index    ] = value & 0xFFu;
  } else {
    bytes[index    ] = (value >> 8u) & 0xFFu;
    bytes[index + 1] = value & 0xFFu;
  }
}

/**
  * @brief  Converting 24-bit unsigned integer to byte array
  * @param  value: 32-bit unsigned integer, but only 24 lowest bits are used.
  * @param  bytes: Byte array
  * @param  index: Array start index
  * @param  end: Endianness
  */
void binutils_uint24_to_bytes(uint32_t value, uint8_t bytes[], uint32_t index, binutils_endianness end)
{
  if (end == binutils_LITTLE) {
    bytes[index + 2] = (value >> 16u) & 0xFFu;
    bytes[index + 1] = (value >>  8u) & 0xFFu;
    bytes[index    ] = value & 0xFFu;
  } else {
    bytes[index    ] = (value >> 16u) & 0xFFu;
    bytes[index + 1] = (value >>  8u) & 0xFFu;
    bytes[index + 2] = value & 0xFFu;
  }
}

/**
  * @brief  Converting 32-bit unsigned integer to byte array
  * @param  value: 32-bit unsigned integer
  * @param  bytes: Byte array
  * @param  index: Array start index
  * @param  end: Endianness
  */
void binutils_uint32_to_bytes(uint32_t value, uint8_t bytes[], uint32_t index, binutils_endianness end)
{
  if (end == binutils_LITTLE) {
    bytes[index + 3] = (value >> 24u) & 0xFFu;
    bytes[index + 2] = (value >> 16u) & 0xFFu;
    bytes[index + 1] = (value >>  8u) & 0xFFu;
    bytes[index    ] = value & 0xFFu;
  } else {
    bytes[index    ] = (value >> 24u) & 0xFFu;
    bytes[index + 1] = (value >> 16u) & 0xFFu;
    bytes[index + 2] = (value >>  8u) & 0xFFu;
    bytes[index + 3] = value & 0xFFu;
  }
}

/**
  * @brief  Converting 64-bit unsigned integer to byte array
  * @param  value: 64-bit unsigned integer
  * @param  bytes: Byte array
  * @param  index: Array start index
  * @param  end: Endianness
  */
void binutils_uint64_to_bytes(uint64_t value, uint8_t bytes[], uint32_t index, binutils_endianness end)
{
  if (end == binutils_LITTLE) {
    bytes[index + 7] = (value >> 56u) & 0xFFu;
    bytes[index + 6] = (value >> 48u) & 0xFFu;
    bytes[index + 5] = (value >> 40u) & 0xFFu;
    bytes[index + 4] = (value >> 32u) & 0xFFu;
    bytes[index + 3] = (value >> 24u) & 0xFFu;
    bytes[index + 2] = (value >> 16u) & 0xFFu;
    bytes[index + 1] = (value >>  8u) & 0xFFu;
    bytes[index    ] = value & 0xFFu;
  } else {
    bytes[index    ] = (value >> 56u) & 0xFFu;
    bytes[index + 1] = (value >> 48u) & 0xFFu;
    bytes[index + 2] = (value >> 40u) & 0xFFu;
    bytes[index + 3] = (value >> 32u) & 0xFFu;
    bytes[index + 4] = (value >> 24u) & 0xFFu;
    bytes[index + 5] = (value >> 16u) & 0xFFu;
    bytes[index + 6] = (value >>  8u) & 0xFFu;
    bytes[index + 7] = value & 0xFFu;
  }
}
