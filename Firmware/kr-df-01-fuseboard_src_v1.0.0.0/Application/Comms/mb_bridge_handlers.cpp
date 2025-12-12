//!  TCP handlers
/*!
 All handlers that are used by nanomodbus TCP for its read/write functions
 */

#include "dp_info.h"
#include "comms.h"
#include "mb_bridge_handlers.h"
#include "mb_local_handlers.h"
#include "main.h"
#include <nanomodbus.h>
#include "rs485.h"

namespace bridge_handlers
{

/* Pointer to RTU client */
static nmbs_t* rtu_client;

/**
 * Initialize Modbus bridge
 * @param remote_rtu_client: Reference to RTU client
 */
void mb_bridge_init(nmbs_t * remote_rtu_client)
{
  ASSERT_FATAL(remote_rtu_client != NULL, "RTU client is NULL");
  rtu_client = remote_rtu_client;
}

// ModBus data handlers
nmbs_error mb_bridge_read_coils(uint16_t address, uint16_t quantity, nmbs_bitfield coils_out, uint8_t unit_id, void *arg)
{
  ASSERT_FATAL(dp_get_device_mode() == DP_DEVICE_MODE::DP_MASTER, "Only master can be bridge");
  UNUSED(arg);

  if (unit_id == dp_get_device_id())
  {
    // Process request locally
    return local_handlers::mb_local_read_coils(address, quantity, coils_out, unit_id, arg);
  }

  // Slave device is addressed
  if ((dp_get_slave_count() > 0) && (unit_id >= dp_get_min_slave_id()) && (unit_id <= dp_get_max_slave_id()))
  {
    nmbs_set_destination_rtu_address(rtu_client, unit_id);
    return nmbs_read_coils(rtu_client, address, quantity, coils_out);
  }

  return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
}

nmbs_error mb_bridge_write_multiple_coils(uint16_t address, uint16_t quantity, const nmbs_bitfield coils, uint8_t unit_id, void *arg)
{
  ASSERT_FATAL(dp_get_device_mode() == DP_DEVICE_MODE::DP_MASTER, "Only master can be bridge");
  UNUSED(arg);

  if (unit_id == dp_get_device_id())
  {
    // Process request locally
    return local_handlers::mb_local_write_multiple_coils(address, quantity, coils, unit_id, arg);
  }

  if ((dp_get_slave_count() > 0) && (unit_id >= dp_get_min_slave_id()) && (unit_id <= dp_get_max_slave_id()))
  {
    nmbs_set_destination_rtu_address(rtu_client, unit_id);
    return nmbs_write_multiple_coils(rtu_client, address, quantity, coils);
  }

  return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
}

nmbs_error mb_bridge_write_single_coil(uint16_t address, bool value, uint8_t unit_id, void *arg)
{
  ASSERT_FATAL(dp_get_device_mode() == DP_DEVICE_MODE::DP_MASTER, "Only master can be bridge");
  UNUSED(arg);

  if (unit_id == dp_get_device_id())
  {
    // Process request locally
    return local_handlers::mb_local_write_single_coil(address, value, unit_id, arg);
  }

  if ((dp_get_slave_count() > 0) && (unit_id >= dp_get_min_slave_id()) && (unit_id <= dp_get_max_slave_id()))
  {
    nmbs_set_destination_rtu_address(rtu_client, unit_id);
    return nmbs_write_single_coil(rtu_client, address, value);
  }

  return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
}

nmbs_error mb_bridge_read_holding_registers(uint16_t address, uint16_t quantity, uint16_t *registers_out, uint8_t unit_id, void *arg)
{
  ASSERT_FATAL(dp_get_device_mode() == DP_DEVICE_MODE::DP_MASTER, "Only master can be bridge");
  UNUSED(arg);

  if (unit_id == dp_get_device_id())
  {
    // Process request locally
    return local_handlers::mb_local_read_holding_registers(address, quantity, registers_out, unit_id, arg);
  }

  if ((dp_get_slave_count() > 0) && (unit_id >= dp_get_min_slave_id()) && (unit_id <= dp_get_max_slave_id()))
  {
    nmbs_set_destination_rtu_address(rtu_client, unit_id);
    return nmbs_read_holding_registers(rtu_client, address, quantity, registers_out);
  }

  return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
}

nmbs_error mb_bridge_write_multiple_registers(uint16_t address, uint16_t quantity, const uint16_t *registers, uint8_t unit_id, void *arg)
{
  ASSERT_FATAL(dp_get_device_mode() == DP_DEVICE_MODE::DP_MASTER, "Only master can be bridge");
  UNUSED(arg);

  if (unit_id == dp_get_device_id())
  {
    // Process request locally
    return local_handlers::mb_local_write_multiple_registers(address, quantity, registers, unit_id, arg);
  }

  if ((dp_get_slave_count() > 0) && (unit_id >= dp_get_min_slave_id()) && (unit_id <= dp_get_max_slave_id()))
  {
    nmbs_set_destination_rtu_address(rtu_client, unit_id);
    return nmbs_write_multiple_registers(rtu_client, address, quantity, registers);
  }

  return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
}

nmbs_error mb_bridge_write_single_register(uint16_t address, uint16_t value, uint8_t unit_id, void *arg)
{
  ASSERT_FATAL(dp_get_device_mode() == DP_DEVICE_MODE::DP_MASTER, "Only master can be bridge");
  UNUSED(arg);

  if (unit_id == dp_get_device_id())
  {
    // Process request locally
    return local_handlers::mb_local_write_single_register(address, value, unit_id, arg);
  }

  if ((dp_get_slave_count() > 0) && (unit_id >= dp_get_min_slave_id()) && (unit_id <= dp_get_max_slave_id()))
  {
    nmbs_set_destination_rtu_address(rtu_client, unit_id);
    return nmbs_write_single_register(rtu_client, address, value);
  }

  return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
}

nmbs_error mb_bridge_read_input_registers(uint16_t address, uint16_t quantity, uint16_t *registers_out, uint8_t unit_id, void *arg)
{
  ASSERT_FATAL(dp_get_device_mode() == DP_DEVICE_MODE::DP_MASTER, "Only master can be bridge");
  UNUSED(arg);

  if (unit_id == dp_get_device_id())
  {
    // Process request locally
    return local_handlers::mb_local_read_input_registers(address, quantity, registers_out, unit_id, arg);
  }

  if ((dp_get_slave_count() > 0) && (unit_id >= dp_get_min_slave_id()) && (unit_id <= dp_get_max_slave_id()))
  {
    nmbs_set_destination_rtu_address(rtu_client, unit_id);
    return nmbs_read_input_registers(rtu_client, address, quantity, registers_out);
  }

  return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
}

nmbs_error mb_bridge_read_discrete_inputs(uint16_t address, uint16_t quantity, nmbs_bitfield inputs_out, uint8_t unit_id, void* arg)
{
  ASSERT_FATAL(dp_get_device_mode() == DP_DEVICE_MODE::DP_MASTER, "Only master can be bridge");
  UNUSED(arg);

  if (unit_id == dp_get_device_id())
  {
    // Process request locally
    return local_handlers::mb_local_read_discrete_inputs(address, quantity, inputs_out, unit_id, arg);
  }

  if ((dp_get_slave_count() > 0) && (unit_id >= dp_get_min_slave_id()) && (unit_id <= dp_get_max_slave_id()))
  {
    nmbs_set_destination_rtu_address(rtu_client, unit_id);
    return nmbs_read_discrete_inputs(rtu_client, address, quantity, inputs_out);
  }

  return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
}

/**
 * Broadcast write multiple coils command to all the slaves
 * @param address: Register address
 * @param quantity: Coils quantity
 * @param coils: Coil values
 * @returns Last failed slave error code or NONE if all succeeded
 */
nmbs_error mb_bridge_broadcast_write_multiple_coils(uint16_t address, uint16_t quantity, const nmbs_bitfield coils)
{
  nmbs_error last_error = NMBS_ERROR_NONE;
  uint8_t slave_count = dp_get_slave_count();
  uint8_t first_slave_id = dp_get_min_slave_id();

  for (uint8_t i = 0; i < slave_count; i++)
  {
    nmbs_set_destination_rtu_address(rtu_client, first_slave_id + i);
    nmbs_error error = nmbs_write_multiple_coils(rtu_client, address, quantity, coils);
    if (error != NMBS_ERROR_NONE)
    {
      last_error = error;
    }
  }

  return last_error;
}

}
