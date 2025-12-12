#include "dp_info.h"
#include <inttypes.h>
#include <stm32h573xx.h>
#include "main.h"
#include "build_info.h"

/* MAC vendor ID for "Private".
 * Based on this list:
 * https://gist.github.com/aallan/b4bb86db86079509e6159810ae9bd3e4 */
#define MAC_VENDOR_ID 0x604BAA

// Offset of modbus ID from 0. In case of ID range overlapping other unknown devices
#define DEVICE_ID_OFFSET 0

static uint32_t device_serial_number = 0;
static uint8_t  device_hw_revision   = 0;

static uint8_t  slave_count = 0;
static uint8_t  device_id   = 0;

static DP_DEVICE_MODE device_mode = DP_MASTER;
static DP_IP_MODE ip_mode         = DP_IP_MODE_NOT_AVAILABLE;

static const __attribute__((section(".fw_info"))) DP_FW_INFO fw_info =
{
#ifdef DEBUG
    .is_debug   = true,
    .version    = FW_VERSION "D",
#else
    .is_debug   = false,
    .version    = FW_VERSION,
#endif
    .build_date = BUILD_DATE,
    .build_time = BUILD_TIME,
    .commit_id  = GIT_COMMIT_ID
};

_Static_assert(sizeof(FW_VERSION) <= 16, "FW_VERSION char size too large");

/**
 * Read device info from OTP memory.
 * If OTP memory first 2 words (4 bytes) are not written, NMI exception is thrown due to ECC error.
 *
 * Read this instruction about OTP access dangers:
 * https://community.st.com/t5/stm32-mcus/how-to-avoid-a-hardfault-when-icache-is-enabled-on-the-stm32h5/ta-p/630085
 */
void dp_init_device_info()
{
  uint16_t otp0, otp1;

  /* Perform two 16-bit reads (because 8-bit throws hard fault).
   * If these are not written (fresh device) then NMI fault is thrown */
  otp0 = ((uint16_t *)FLASH_OTP_BASE)[0];
  otp1 = ((uint16_t *)FLASH_OTP_BASE)[1];

  /* Parse serial number that is stored in MSB byte order */
  device_serial_number =
      ((otp0 & 0x00FF) << 16) |
       (otp0 & 0xFF00) |
       (otp1 & 0x00FF);

  /* Parse HW revision that is stored in 4th byte (second word higher byte) */
  device_hw_revision = (otp1 >> 8);
}

/**
 * Get device serial number.
 * @return Serial number as 24-bit unsigned integer.
 */
uint32_t dp_get_serial_number()
{
  return device_serial_number;
}

/**
 * Get device hardware revision number.
 * @return Hardware revision number as 8-bit unsigned integer.
 */
uint8_t dp_get_hw_revision()
{
  return device_hw_revision;
}

uint32_t dp_get_mac_vendor_id()
{
  return MAC_VENDOR_ID;
}

void dp_set_ip_mode(DP_IP_MODE mode)
{
  ip_mode = mode;
}

DP_IP_MODE dp_get_ip_mode()
{
  return ip_mode;
}

void dp_set_device_mode(DP_DEVICE_MODE mode)
{
  device_mode = mode;
}

DP_DEVICE_MODE dp_get_device_mode()
{
  return device_mode;
}

uint8_t dp_get_slave_count()
{
  return slave_count;
}

void dp_set_slave_count(uint8_t count)
{
  slave_count = count;
}

uint8_t dp_get_device_id()
{
  return device_id;
}

void dp_set_device_id(uint8_t id)
{
  device_id = id;
}

uint8_t dp_get_min_slave_id()
{
  return DEVICE_ID_OFFSET + 1;
}

uint8_t dp_get_max_slave_id()
{
  return DEVICE_ID_OFFSET + slave_count;
}

const DP_FW_INFO* dp_get_fw_info()
{
    return &fw_info;
}
