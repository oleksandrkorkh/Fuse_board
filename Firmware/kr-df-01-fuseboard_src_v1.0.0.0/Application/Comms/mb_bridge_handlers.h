/**
 * @brief Handler functions for TCP client/RTU master for selection between
 *        handling received messages on the local device or to transmit the messages
 *        over RTU to another slave device indicated in the message
 */

#pragma once

#include <stdint.h>
#include <nanomodbus.h>

namespace bridge_handlers
{
void mb_bridge_init(nmbs_t* remote_rtu_client);
nmbs_error mb_bridge_read_coils(uint16_t address, uint16_t quantity, nmbs_bitfield coils_out, uint8_t unit_id, void *arg);
nmbs_error mb_bridge_write_multiple_coils(uint16_t address, uint16_t quantity, const nmbs_bitfield coils, uint8_t unit_id, void *arg);
nmbs_error mb_bridge_write_single_coil(uint16_t address, bool value, uint8_t unit_id, void *arg);
nmbs_error mb_bridge_read_holding_registers(uint16_t address, uint16_t quantity, uint16_t *registers_out, uint8_t unit_id, void *arg);
nmbs_error mb_bridge_write_multiple_registers(uint16_t address, uint16_t quantity, const uint16_t *registers, uint8_t unit_id, void *arg);
nmbs_error mb_bridge_write_single_register(uint16_t address, uint16_t value, uint8_t unit_id, void *arg);
nmbs_error mb_bridge_read_input_registers(uint16_t address, uint16_t quantity, uint16_t *registers_out, uint8_t unit_id, void *arg);
nmbs_error mb_bridge_read_discrete_inputs(uint16_t address, uint16_t quantity, nmbs_bitfield inputs_out, uint8_t unit_id, void* arg);
nmbs_error mb_bridge_broadcast_write_multiple_coils(uint16_t address, uint16_t quantity, const nmbs_bitfield coils);
}

