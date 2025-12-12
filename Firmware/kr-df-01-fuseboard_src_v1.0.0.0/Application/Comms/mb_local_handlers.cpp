//!  RTU handlers
/*!
 All handlers that are used by nanomodbus RTU for its read/write functions
 */

#include "mb_local_handlers.h"
#include <nanomodbus.h>
#include "rs485.h"
#include "comms.h"
#include "dp_info.h"
#include "mb_data_translator.h"

namespace local_handlers
{
// ModBus data handlers
nmbs_error mb_local_read_coils(uint16_t address, uint16_t quantity, nmbs_bitfield coils_out, uint8_t unit_id, void *arg)
{
  UNUSED(unit_id);
  UNUSED(arg);

  nmbs_error result = NMBS_ERROR_NONE;
  uint16_t quantity_in_bytes = quantity * sizeof(uint16_t);

  if ((address >= MB_REGISTER_LIMITS::mb_coils_register_address_start) &&
     ((address + quantity_in_bytes) <= MB_REGISTER_LIMITS::mb_coils_register_address_end))
  {
    result = mb_get_coil_register_value(address, quantity, coils_out);
  }
  else
  {
    result = NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
  }

  return result;
}

nmbs_error mb_local_write_multiple_coils(uint16_t address, uint16_t quantity, const nmbs_bitfield coils, uint8_t unit_id, void *arg)
{
  UNUSED(unit_id);
  UNUSED(arg);

  nmbs_error result = NMBS_ERROR_NONE;
  uint16_t quantity_in_bytes = quantity * sizeof(uint16_t);

  if ((address >= MB_REGISTER_LIMITS::mb_coils_register_address_start) &&
     ((address + quantity_in_bytes) <= MB_REGISTER_LIMITS::mb_coils_register_address_end))
  {
    result = mb_set_coil_register_value(address, quantity, coils);
  }
  else
  {
    result = NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
  }

  return result;
}

nmbs_error mb_local_write_single_coil(uint16_t address, bool value, uint8_t unit_id, void *arg)
{
  uint8_t coil = 0;
  if (value)
  {
    coil |= 0x01;
  }

  return mb_local_write_multiple_coils(address, 1, &coil, unit_id, arg);
}

nmbs_error mb_local_read_holding_registers(uint16_t address, uint16_t quantity, uint16_t *registers_out, uint8_t unit_id, void *arg)
{
  UNUSED(unit_id);
  UNUSED(arg);

  nmbs_error result = NMBS_ERROR_NONE;
  uint16_t quantity_in_bytes = quantity * sizeof(uint16_t);

  if ((address >= MB_REGISTER_LIMITS::mb_holding_register_address_start) &&
     ((address + quantity_in_bytes) <= MB_REGISTER_LIMITS::mb_holding_register_address_end))
  {
    result = mb_get_holding_register_value(address, registers_out, quantity);
  }
  else
  {
    result = NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
  }
  // Write registers values to our holding_registers
  return result;
}

nmbs_error mb_local_write_multiple_registers(uint16_t address, uint16_t quantity, const uint16_t *registers, uint8_t unit_id, void *arg)
{
  UNUSED(unit_id);
  UNUSED(arg);

  nmbs_error result = NMBS_ERROR_NONE;
  uint16_t quantity_in_bytes = quantity * sizeof(uint16_t);

  if ((address >= MB_REGISTER_LIMITS::mb_holding_register_address_start) &&
     ((address + quantity_in_bytes) <= MB_REGISTER_LIMITS::mb_holding_register_address_end))
  {
    result = mb_set_holding_register_value(address, registers, quantity);
  }
  else
  {
    result = NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
  }
  // Write registers values to our holding_registers
  return result;
}

nmbs_error mb_local_write_single_register(uint16_t address, uint16_t value, uint8_t unit_id, void *arg)
{
  uint16_t reg = value;
  return mb_local_write_multiple_registers(address, 1, &reg, unit_id, arg);
}

nmbs_error mb_local_read_input_registers(uint16_t address, uint16_t quantity, uint16_t *registers_out, uint8_t unit_id, void *arg)
{
  UNUSED(unit_id);
  UNUSED(arg);

  nmbs_error result = NMBS_ERROR_NONE;
  uint16_t quantity_in_bytes = quantity * sizeof(uint16_t);

  if ((address >= MB_REGISTER_LIMITS::mb_input_register_address_start) &&
     ((address + quantity_in_bytes) <= MB_REGISTER_LIMITS::mb_input_register_address_end))
  {
    result = mb_get_input_register_value(address, registers_out, quantity);
  }
  else
  {
    result = NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
  }

  return result;
}

nmbs_error mb_local_read_discrete_inputs(uint16_t address, uint16_t quantity, nmbs_bitfield inputs_out, uint8_t unit_id, void* arg)
{
  UNUSED(unit_id);
  UNUSED(arg);

  nmbs_error result = NMBS_ERROR_NONE;
  uint16_t quantity_in_bytes = quantity * sizeof(uint16_t);

  if ((address >= MB_REGISTER_LIMITS::mb_discrete_input_register_address_start) &&
     ((address + quantity_in_bytes) <= MB_REGISTER_LIMITS::mb_discrete_input_register_address_end))
  {
    result = mb_get_discrete_input_register_value(address, inputs_out, quantity); // TODO: finish this
  }
  else
  {
    result = NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
  }

  return result;
}

}
