#include "dev_reg_functions.h"
#include "dp_info.h"
#include "main.h"
#include "binutils.h"

/**
 * Swap byte pairs to make compatible with Modbus 16-bit big-endian registers
 */
static void swap_pairs(uint8_t *data, size_t len)
{
  ASSERT_FATAL(data != NULL, "Swap data NULL pointer");
  ASSERT_FATAL((len % 2) == 0, "Swap data is not even sized");

  for (size_t i = 0; i + 1 < len; i += 2)
  {
    uint8_t tmp = data[i];
    data[i] = data[i + 1];
    data[i + 1] = tmp;
  }
}

nmbs_error fb_dev_reg_get_slave_count(uint16_t *registers_out, uint16_t quantity)
{
  ASSERT_FATAL(registers_out != NULL, "Registers are NULL");

  uint16_t count = dp_get_slave_count();

  uint16_t quantity_in_bytes = quantity * sizeof(uint16_t);
  if (quantity_in_bytes != sizeof(count))
  {
    return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
  }

  registers_out[0] = count;
  return NMBS_ERROR_NONE;
}

nmbs_error fb_dev_reg_get_output_count(uint16_t *registers_out, uint16_t quantity)
{
  ASSERT_FATAL(registers_out != NULL, "Registers are NULL");

  uint16_t count = DP_RELAY_COUNT;

  uint16_t quantity_in_bytes = quantity * sizeof(uint16_t);
  if (quantity_in_bytes != sizeof(count))
  {
    return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
  }

  registers_out[0] = count;
  return NMBS_ERROR_NONE;
}

nmbs_error fb_dev_reg_get_stm32_uid(uint16_t *registers_out, uint16_t quantity)
{
  ASSERT_FATAL(registers_out != NULL, "Registers are NULL");

  uint8_t uid[3 * sizeof(uint32_t)] = { 0 };

  uint16_t quantity_in_bytes = quantity * sizeof(uint16_t);
  if (quantity_in_bytes != sizeof(uid))
  {
    return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
  }

  binutils_uint32_to_bytes(HAL_GetUIDw0(), uid, 0, binutils_BIG);
  binutils_uint32_to_bytes(HAL_GetUIDw1(), uid, 4, binutils_BIG);
  binutils_uint32_to_bytes(HAL_GetUIDw2(), uid, 8, binutils_BIG);
  swap_pairs(uid, sizeof(uid));
  (void)memcpy(registers_out, uid, sizeof(uid));

  return NMBS_ERROR_NONE;
}

nmbs_error fb_dev_reg_get_mac(uint16_t *registers_out, uint16_t quantity)
{
  ASSERT_FATAL(registers_out != NULL, "Registers are NULL");

  uint8_t mac[6] = { 0 };

  uint16_t quantity_in_bytes = quantity * sizeof(uint16_t);
  if (quantity_in_bytes != sizeof(mac))
  {
    return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
  }

  binutils_uint24_to_bytes(dp_get_mac_vendor_id(), mac, 0, binutils_BIG);
  binutils_uint24_to_bytes(dp_get_serial_number(), mac, 3, binutils_BIG);
  swap_pairs(mac, sizeof(mac));
  (void)memcpy(registers_out, mac, sizeof(mac));

  return NMBS_ERROR_NONE;
}

nmbs_error fb_dev_reg_get_serial_number(uint16_t *registers_out, uint16_t quantity)
{
  ASSERT_FATAL(registers_out != NULL, "Registers are NULL");

  uint8_t serial[sizeof(uint32_t)] = { 0 };

  uint16_t quantity_in_bytes = quantity * sizeof(uint16_t);
  if (quantity_in_bytes != sizeof(serial))
  {
    return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
  }

  binutils_uint32_to_bytes(dp_get_serial_number(), serial, 0, binutils_BIG);
  swap_pairs(serial, sizeof(serial));
  (void)memcpy(registers_out, serial, sizeof(serial));

  return NMBS_ERROR_NONE;
}

nmbs_error fb_dev_reg_get_hardware_revision(uint16_t *registers_out, uint16_t quantity)
{
  ASSERT_FATAL(registers_out != NULL, "Registers are NULL");

  uint16_t hw_rev = dp_get_hw_revision();

  uint16_t quantity_in_bytes = quantity * sizeof(uint16_t);
  if (quantity_in_bytes != sizeof(hw_rev))
  {
    return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
  }

  registers_out[0] = hw_rev;
  return NMBS_ERROR_NONE;
}

nmbs_error fb_dev_reg_get_bootloader_ver_ascii(uint16_t *registers_out, uint16_t quantity)
{
  ASSERT_FATAL(registers_out != NULL, "Registers are NULL");
  UNUSED(quantity);

  return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; // Bootloader does not exist yet
}

nmbs_error fb_dev_reg_get_firmware_ver_ascii(uint16_t *registers_out, uint16_t quantity)
{
  ASSERT_FATAL(registers_out != NULL, "Registers are NULL");

  uint16_t quantity_in_bytes = quantity * sizeof(uint16_t);
  if (quantity_in_bytes != sizeof(DP_FW_INFO::version))
  {
    return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
  }

  const DP_FW_INFO* fw_info = dp_get_fw_info();
  char version[sizeof(DP_FW_INFO::version)] = { 0 };

  (void)snprintf(version, sizeof(version), fw_info->version);
  swap_pairs((uint8_t *)version, sizeof(version));
  (void)memcpy(registers_out, version, sizeof(version));

  return NMBS_ERROR_NONE;
}
