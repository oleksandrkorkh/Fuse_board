/**
 * @brief Handler functions for locally handling RTU/TCP message funcitions
 */

#pragma once

#include <stdint.h>
#include <nanomodbus.h>

namespace local_handlers
{
nmbs_error mb_local_read_coils(uint16_t address, uint16_t quantity, nmbs_bitfield coils_out, uint8_t unit_id, void *arg);
nmbs_error mb_local_write_multiple_coils(uint16_t address, uint16_t quantity, const nmbs_bitfield coils, uint8_t unit_id, void *arg);
nmbs_error mb_local_write_single_coil(uint16_t address, bool value, uint8_t unit_id, void *arg);
nmbs_error mb_local_read_holding_registers(uint16_t address, uint16_t quantity, uint16_t *registers_out, uint8_t unit_id, void *arg);
nmbs_error mb_local_write_multiple_registers(uint16_t address, uint16_t quantity, const uint16_t *registers, uint8_t unit_id, void *arg);
nmbs_error mb_local_write_single_register(uint16_t address, uint16_t value, uint8_t unit_id, void *arg);
nmbs_error mb_local_read_input_registers(uint16_t address, uint16_t quantity, uint16_t *registers_out, uint8_t unit_id, void *arg);
nmbs_error mb_local_read_discrete_inputs(uint16_t address, uint16_t quantity, nmbs_bitfield inputs_out, uint8_t unit_id, void* arg);
}
