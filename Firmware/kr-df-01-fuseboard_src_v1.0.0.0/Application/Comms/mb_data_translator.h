/**
 * Used to take modbus data and make use of it according to Fuse board SW interface specification
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "nanomodbus.h"

#ifdef __cplusplus
extern "C"
{
#endif

struct MB_REGISTER_LIMITS
{
  inline static const uint16_t mb_coils_register_address_start          = 0x0010;
  inline static const uint16_t mb_coils_register_address_end            = 0x270F;
  inline static const uint16_t mb_discrete_input_register_address_start = 0x3000;
  inline static const uint16_t mb_discrete_input_register_address_end   = 0x4E1F;
  inline static const uint16_t mb_input_register_address_start          = 0x8000;
  inline static const uint16_t mb_input_register_address_end            = 0x9C3F;
  inline static const uint16_t mb_holding_register_address_start        = 0xA000;
  inline static const uint16_t mb_holding_register_address_end          = 0xC350;
} ;

struct MB_COIL_REGISTER_ITEMS
{
  inline static const uint16_t coils_state_address   = MB_REGISTER_LIMITS::mb_coils_register_address_start;
  inline static const uint16_t port_action_address   = 0x1000;
  inline static const uint16_t device_action_address = 0x2000;
};

struct MB_INPUT_REGISTER_ITEMS
{
  inline static const uint16_t mb_port_state_address   = MB_REGISTER_LIMITS::mb_input_register_address_start;
  inline static const uint16_t mb_device_state_address = 0x9000;
};

struct MB_HOLDING_REGISTER_ITEMS
{
  inline static const uint16_t mb_device_id  = MB_REGISTER_LIMITS::mb_holding_register_address_start;
  inline static const uint16_t mb_device_cfg = 0xA800;
  inline static const uint16_t mb_port_cfg   = 0xB000;
};


// Functions to access and modify holding registers
nmbs_error mb_get_holding_register_value(uint16_t address, uint16_t *registers, uint16_t quantity);
nmbs_error mb_set_holding_register_value(uint16_t address, const uint16_t *registers, uint16_t quantity);

// Function to access input registers
nmbs_error mb_get_input_register_value(uint16_t address, uint16_t *registers_out, uint16_t quantity);

// Function to access discrete input registers
nmbs_error mb_get_discrete_input_register_value(uint16_t address, nmbs_bitfield inputs_out, uint16_t quantity);

// Functions to write and read coil registers
nmbs_error mb_get_coil_register_value(uint16_t address, uint16_t quantity, nmbs_bitfield coils);
nmbs_error mb_set_coil_register_value(uint16_t address, uint16_t quantity, const nmbs_bitfield coils);

#ifdef __cplusplus
}
#endif
