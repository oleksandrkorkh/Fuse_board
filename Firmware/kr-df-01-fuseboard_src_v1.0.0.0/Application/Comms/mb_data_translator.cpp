#include "dev_reg_functions.h"
#include "mb_data_translator.h"
#include "op_functions.h"
#include "main.h"
#include "nanomodbus.h"
#include "config.h"
#include <stm32h5xx_hal.h>

// Returns function from message address value
// Gets the function from address value by getting the last 2 bytes from it
static uint8_t mb_find_msg_func(uint16_t address)
{
  return address & 0xFF;
}

// Checks if addressed port is within valid range and returns it. -1 if out of range
// Gets the relay port from address value by getting the 0b111 masked value from the first 2nd byte of address
static int8_t mb_find_msg_port(uint16_t address)
{
  const uint8_t port_number = (address >> 8) & 0xF; // Extract the 2nd 4 bits
  return port_number <= DP_RELAY_COUNT ? port_number : -1;
}


nmbs_error mb_get_coil_register_value(uint16_t address, uint16_t quantity, nmbs_bitfield coils)
{
  /* Reading output control status ? */
  if ((address >= MB_COIL_REGISTER_ITEMS::coils_state_address) &&
      (address + quantity <= MB_COIL_REGISTER_ITEMS::coils_state_address + DP_RELAY_COUNT))
  {
    /* Extract port from lowest nibble (4 bits) */
    uint8_t start_port = (address & 0xF);
    return fb_output_control_get(start_port, quantity, coils);
  }

  /* Everything else is not supported */
  return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
}

nmbs_error mb_set_coil_register_value(uint16_t address, uint16_t quantity, const nmbs_bitfield coils)
{
  uint8_t function_type = mb_find_msg_func(address);

  if (address >= MB_COIL_REGISTER_ITEMS::device_action_address)
  {
    switch (function_type)
    {
      case 0x00:
        return fb_device_reset(address, coils, quantity);

      case 0x01:
        return fb_config_change((const uint8_t *)coils, quantity);

      case 0x02:
        return fb_reformat_flash((const uint8_t *)coils, quantity);

      default:
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    }
  }
  else if (address >= MB_COIL_REGISTER_ITEMS::port_action_address)
  {
    int8_t port_number = mb_find_msg_port(address);
    if (port_number < 0)
    {
      return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    }

    switch (function_type)
    {
      case 0x00:
        return fb_output_led_control((const uint8_t *)coils, quantity, port_number);

      case 0x03:
        return fb_output_self_reset_req((const uint8_t *)coils, quantity, port_number);

      case 0x04:
        return fb_output_fault_flags_reset((const uint8_t *)coils, quantity, port_number);

      default:
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    }
  }
  else if (address >= MB_COIL_REGISTER_ITEMS::coils_state_address)
  {
    /* Extract port from lowest nibble (4 bits) */
    uint8_t start_port = (address & 0xF);
    return fb_output_control(start_port, quantity, coils);
  }

  /* Everything else is not supported */
  return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
}

nmbs_error mb_get_holding_register_value(uint16_t address, uint16_t *registers_out, uint16_t quantity)
{
  uint16_t quantity_in_bytes  = quantity * sizeof(uint16_t);
  size_t   field_offset       = address & 0xFF;
  uint16_t field_offset_bytes = field_offset * sizeof(uint16_t);
  int8_t   port_number        = mb_find_msg_port(address);

  if (address >= MB_HOLDING_REGISTER_ITEMS::mb_port_cfg)
  {
    if ((port_number < 0) ||
       ((field_offset_bytes + quantity_in_bytes) > sizeof(CONFIG_PORT)))
    {
      return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    }
    else
    {
      CONFIG_PORT port;
      uint16_t *port_registers = (uint16_t *)&port;

      config_get_port_data(&port, port_number);
      (void)memcpy(registers_out, &port_registers[field_offset], quantity_in_bytes);
      return NMBS_ERROR_NONE;
    }
  }
  else if (address >= MB_HOLDING_REGISTER_ITEMS::mb_device_cfg)
  {
    if ((field_offset_bytes + quantity_in_bytes) > sizeof(CONFIG_GENERAL))
    {
      return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    }
    else
    {
      CONFIG_GENERAL gen;
      uint16_t *gen_registers = (uint16_t *)&gen;

      config_get_general_data(&gen);
      (void)memcpy(registers_out, &gen_registers[field_offset], quantity_in_bytes);
      return NMBS_ERROR_NONE;
    }
  }
  else if (address >= MB_HOLDING_REGISTER_ITEMS::mb_device_id)
  {
    uint8_t msg_func = mb_find_msg_func(address);
    switch(msg_func)
    {
      case 0x00:
        return fb_dev_reg_get_slave_count(registers_out, quantity);

      case 0x01:
        return fb_dev_reg_get_output_count(registers_out, quantity);

      case 0x02:
        return fb_dev_reg_get_stm32_uid(registers_out, quantity);

      case 0x08:
        return fb_dev_reg_get_mac(registers_out, quantity);

      case 0x12:
        return fb_dev_reg_get_serial_number(registers_out, quantity);

      case 0x14:
        return fb_dev_reg_get_hardware_revision(registers_out, quantity);

      case 0x20:
        return fb_dev_reg_get_bootloader_ver_ascii(registers_out, quantity);

      case 0x28:
        return fb_dev_reg_get_firmware_ver_ascii(registers_out, quantity);

      default:
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    }
  }

  return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
}

nmbs_error mb_set_holding_register_value(uint16_t address, const uint16_t *registers, uint16_t quantity)
{
  nmbs_error result = NMBS_ERROR_NONE;
  uint16_t quantity_in_bytes = quantity * sizeof(uint16_t);

  size_t  field_offset        = address & 0xFF;
  uint16_t field_offset_bytes = field_offset * sizeof(uint16_t);
  int8_t  port_number         = mb_find_msg_port(address);

  // starting if from checking biggest address item
  if (address >= MB_HOLDING_REGISTER_ITEMS::mb_port_cfg) // port cfg items
  {
    if ((port_number < 0) || // Not all commands here use ports, so checking here
       ((field_offset_bytes + quantity_in_bytes) > sizeof(CONFIG_PORT)))
    {
      result = NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    }
    else
    {
      CONFIG_PORT port;
      uint16_t *port_registers = (uint16_t *)&port;

      config_get_port_data(&port, port_number);
      (void)memcpy(&port_registers[field_offset], registers, quantity_in_bytes);
      if (!config_set_port_data(&port, port_number))
      {
        return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
      }
    }
  }
  else if (address >= MB_HOLDING_REGISTER_ITEMS::mb_device_cfg) // device cfg items
  {
    if ((field_offset_bytes + quantity_in_bytes) > sizeof(CONFIG_GENERAL))
    {
      result = NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    }
    else
    {
      CONFIG_GENERAL gen;
      uint16_t *gen_registers = (uint16_t *)&gen;

      config_get_general_data(&gen);
      (void)memcpy(&gen_registers[field_offset], registers, quantity_in_bytes);
      if (!config_set_general_data(&gen))
      {
        return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
      }
    }
  }
  else if (address >= MB_HOLDING_REGISTER_ITEMS::mb_device_id) // device ID items
  {
    result = NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS; // placeholder for future task
  }
  else
  {
    result = NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
  }

  return result;
}

nmbs_error mb_get_input_register_value(uint16_t address, uint16_t *registers_out, uint16_t quantity)
{
  nmbs_error status     = NMBS_ERROR_NONE;
  size_t   field_offset = address & 0xFF;
  int8_t   port_number  = mb_find_msg_port(address);

  if (port_number < 0) // All commands here are using ports, so check first
  {
    status = NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
  }
  else
  {
    if (address >= MB_INPUT_REGISTER_ITEMS::mb_device_state_address)
    {
      uint32_t value = 0;
      switch (field_offset)
      {
        case 0x00:
          status = fb_uptime(quantity, &value);
          break;
        default:
          status = NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
      }

      if (status == NMBS_ERROR_NONE)
      {
        registers_out[0] = (uint16_t)(value >> 16);   // High 16 bits
        registers_out[1] = (uint16_t)(value & 0xFFFF);
      }
    }
    else if (address >= MB_INPUT_REGISTER_ITEMS::mb_port_state_address) // port state items
    {
      for (uint16_t i = 0; i < quantity; i++)
      {
        uint16_t value = 0;

        switch (field_offset + i)
        {
          case 0x00:
            status = fb_output_state(port_number, quantity, &value);
            break;
          case 0x01:
            status = fb_output_state_reason(port_number, quantity, &value);
            break;
          case 0x02:
            status = fb_output_fault_flags(port_number, quantity, &value);
            break;
          case 0x03:
            status = fb_read_output_voltage(port_number, quantity, &value);
            break;
          case 0x04:
            status = fb_read_output_current(port_number, quantity, (int16_t *)&value);
            break;
          case 0x05:
            status = fb_read_output_power(port_number, quantity, (int16_t *)&value);
            break;
          case 0x06:
            status = fb_read_output_energy_high(port_number, quantity, &value);
            break;
          case 0x07:
            status = fb_read_output_energy_mid_high(port_number, quantity, &value);
            break;
          case 0x08:
            status = fb_read_output_energy_mid_low(port_number, quantity, &value);
            break;
          case 0x09:
            status = fb_read_output_energy_low(port_number, quantity, &value);
            break;
          case 0x0A:
            status = fb_read_button_states(port_number, quantity, &value);
            break;
          default:
            status = NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
        }

        if (status == NMBS_ERROR_NONE)
        {
          registers_out[i] = value;
        }
      }
    }
    else
    {
      status = NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    }
  }

  return status;
}

nmbs_error mb_get_discrete_input_register_value(uint16_t address, nmbs_bitfield inputs_out, uint16_t quantity)
{
  UNUSED(address);
  UNUSED(inputs_out);
  UNUSED(quantity);

  return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
}
