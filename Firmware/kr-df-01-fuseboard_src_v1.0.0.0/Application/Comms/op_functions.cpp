#include "op_functions.h"
#include "dp_info.h"
#include <stm32h5xx_hal.h>
#include "comms.h"
#include "nanomodbus.h"
#include "config.h"
#include "leds.h"
#include "relays.h"
#include "buttons.h"
#include "mb_bridge_handlers.h"
#include "binutils.h"
#include "tx_extension.h"
#include "nvm.h"
#include <math.h>

struct FB_OP_DATA_SIZES_BYTES
{
  static const uint8_t device_reset = 1;
  static const uint8_t config_change = 1;
  static const uint8_t reformat_flash = 1;

  static const uint8_t uptime = 4;

  static const uint8_t output_control = 1;

  static const uint8_t output_state = 2;
  static const uint8_t output_state_reason = 2;
  static const uint8_t output_fault_flags = 2;

  static const uint8_t read_output_voltage = 2;
  static const uint8_t read_output_current = 2;
  static const uint8_t read_output_power = 2;
  static const uint8_t read_output_energy = 4;
  static const uint8_t read_button_switch_states = 2;

  static const uint8_t output_led_control = 3;
  static const uint8_t output_self_reset_req = 1;
  static const uint8_t output_fault_flags_reset = 1;
};

// Makes sure the amount of bytes expected by a message is not over the limit of its type
static bool fb_check_byte_quantity_validity(uint16_t quantity, uint8_t compare_quantity_bytes)
{
  bool result = false;

  if ((quantity * 2) == compare_quantity_bytes)
  {
    result = true;
  }

  return result;
}

static bool fb_check_bits_quantity_validity(uint16_t quantity, uint8_t compare_quantity_bytes)
{
  bool result = false;

  if (quantity == compare_quantity_bytes * 8)
  {
    result = true;
  }

  return result;
}

nmbs_error fb_device_reset(uint16_t address, const uint8_t *coils, uint16_t quantity)
{
  // Coil registers quantity is bit based, not byte
  if (!(fb_check_bits_quantity_validity(quantity, FB_OP_DATA_SIZES_BYTES::device_reset)))
  {
    return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
  }

  /* Reset of this unit ? */
  if (coils[0] == 0x55)
  {
    HAL_NVIC_SystemReset();
  }
  /* Reset of all the units (only applicable for master) ? */
  else if ((coils[0] == 0x66) && (dp_get_device_mode() == DP_DEVICE_MODE::DP_MASTER))
  {
    const uint8_t slave_coils[1] = { 0x55 };
    (void)bridge_handlers::mb_bridge_broadcast_write_multiple_coils(address, quantity, slave_coils);
    HAL_NVIC_SystemReset();
  }
  else
  {
    return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
  }

  return NMBS_EXCEPTION_SERVER_DEVICE_FAILURE; // Reaches here if no reset happened
}

nmbs_error fb_config_change(const uint8_t *coils, uint16_t quantity)
{
  if (!(fb_check_bits_quantity_validity(quantity, FB_OP_DATA_SIZES_BYTES::config_change)))
  {
    return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
  }

  CONFIG_GENERAL clear_gen_cfg = {0};
  CONFIG_PORT clear_port_cfg = {0};
  CONFIG_OPERATION_EVENT_RES event = CONFIG_FLASH_OP_FAILED;

  switch (coils[0])
  {
    /* Reset current configuration to all zeroes */
    case 0xAA:
      ULOG_INFO("Clear configuration to zeroes");
      (void)config_set_general_data(&clear_gen_cfg);
      for (uint8_t port_num = 0; port_num < DP_RELAY_COUNT; port_num++)
      {
        (void)config_set_port_data(&clear_port_cfg, port_num);
      }
      return NMBS_ERROR_NONE;

    /* Make current configuration as factory default */
    case 0xBB:
      config_add_op_queue(CONFIG_SAVE_FACTORY_DEF_CFG);
      if (config_check_res_queue(&event))
      {
        if (event == CONFIG_FLASH_OP_SUCCESS)
        {
          return NMBS_ERROR_NONE;
        }
      }
      return NMBS_EXCEPTION_SERVER_DEVICE_FAILURE;

    /* Restore factory default configuration */
    case 0xCC:
      config_add_op_queue(CONFIG_LOAD_FACTORY_DEF_CFG);
      if (config_check_res_queue(&event))
      {
        if (event == CONFIG_FLASH_OP_SUCCESS)
        {
          return NMBS_ERROR_NONE;
        }
      }
      return NMBS_EXCEPTION_SERVER_DEVICE_FAILURE;

    /* Save current configuration into Flash */
    case 0xDD:
      config_add_op_queue(CONFIG_SAVE_ACTIVE_CFG);
      if (config_check_res_queue(&event))
      {
        if (event == CONFIG_FLASH_OP_SUCCESS)
        {
          return NMBS_ERROR_NONE;
        }
      }
      return NMBS_EXCEPTION_SERVER_DEVICE_FAILURE;

    default:
      break;
    }

  return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
}

nmbs_error fb_reformat_flash(const uint8_t *coils, uint16_t quantity)
{
  ASSERT_FATAL(coils != NULL, "coils pointer NULL");

  if (!(fb_check_bits_quantity_validity(quantity, FB_OP_DATA_SIZES_BYTES::reformat_flash)))
  {
    return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
  }

  switch (coils[0])
  {
    case 0x11:
      nvm_reset_and_format();
      /* This function does not return! If it does, then there's a problem */
      return NMBS_EXCEPTION_SERVER_DEVICE_FAILURE;

    default:
      break;
  }

  return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
}

nmbs_error fb_uptime(uint16_t quantity, uint32_t *value)
{
  if (!(fb_check_byte_quantity_validity(quantity, FB_OP_DATA_SIZES_BYTES::uptime)))
  {
    return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
  }

  uint32_t s = tx_time_get_ms() / 1000;

  *value = s;

  return NMBS_ERROR_NONE;
}

nmbs_error fb_output_control(uint8_t start_port, uint16_t quantity, const nmbs_bitfield coils)
{
  if (start_port + quantity > DP_RELAY_COUNT)
  {
    ULOG_ERROR("Output control out of range (port %u, quantity %u)", start_port, quantity);
    return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
  }

  for (uint16_t i = 0; i < quantity; i++)
  {
    relays::Relay relay = static_cast<relays::Relay>(start_port + i);
    bool state = (nmbs_bitfield_read(coils, i) != 0);

    ULOG_DEBUG("Control relay #%u to %s", relay, state ? "ON" : "OFF");
    if (state)
    {
      relays::control_relay(relay, relays::REQ_ON, relays::REQSRC_MODBUS);
    }
    else
    {
      relays::control_relay(relay, relays::REQ_OFF, relays::REQSRC_MODBUS);
    }
  }

  return NMBS_ERROR_NONE;
}

nmbs_error fb_output_control_get(uint8_t start_port, uint16_t quantity, nmbs_bitfield coils_out)
{
  if (start_port + quantity > DP_RELAY_COUNT)
  {
    ULOG_ERROR("Output control read out of range (port %u, quantity %u)", start_port, quantity);
    return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
  }

  for (uint16_t i = 0; i < quantity; i++)
  {
    relays::Relay relay = static_cast<relays::Relay>(start_port + i);
    relays::Request request = relays::get_control_state(relay);

    switch (request)
    {
      case relays::REQ_OFF:
        nmbs_bitfield_unset(coils_out, i);
        break;
      case relays::REQ_ON:
        nmbs_bitfield_set(coils_out, i);
        break;
      case relays::REQ_RESET:
        /* Reset will lead to (re)activation after a timer so "ON" is quite okay to return */
        nmbs_bitfield_set(coils_out, i);
        break;
      default:
        ULOG_CRITICAL("Invalid relay control request: %u", request);
        break;
    }
  }

  return NMBS_ERROR_NONE;
}

nmbs_error fb_output_state(uint8_t port, uint16_t quantity, uint16_t *value)
{
  UNUSED(quantity);

  relays::RelayStatus status = relays::get_relay_status(static_cast<relays::Relay>(port));

  switch (status.state)
  {
    case relays::RELAY_INIT:
    case relays::RELAY_STARTUP_DELAY:
    case relays::RELAY_OFF:
      *value = 0;
      break;

    case relays::RELAY_ON:
    case relays::RELAY_TURNING_ON:
    case relays::RELAY_TURNING_OFF:
    case relays::RELAY_UNDERCURRENT:
    case relays::RELAY_OVERCURRENT:
      *value = 1;
      break;

    case relays::RELAY_EXCESS_VOLTAGE:
      *value = 2;
      break;

    case relays::RELAY_ABSENT_VOLTAGE:
      *value = 3;
      break;

    case relays::RELAY_RESET:
      *value = 4;
      break;

    case relays::RELAY_PROTECTING:
      *value = 5;
      break;

    case relays::RELAY_RECOVERY_END:
      *value = 6;
      break;

    default:
      // It could be programming error that there's new state... but here it is not catastrophic
      ULOG_ERROR("Unhandled relay %u state: %u", port, status.state);
      *value = 0xFF;
      break;
  }

  return NMBS_ERROR_NONE;
}

nmbs_error fb_output_state_reason(uint8_t port, uint16_t quantity, uint16_t *value)
{
  UNUSED(quantity);

  relays::RelayStatus relay_status = relays::get_relay_status(static_cast<relays::Relay>(port));
  *value = (uint16_t)relay_status.state_reason;

  return NMBS_ERROR_NONE;
}

nmbs_error fb_output_fault_flags(uint8_t port, uint16_t quantity, uint16_t *value)
{
  UNUSED(quantity);

  relays::RelayStatus relay_status = relays::get_relay_status(static_cast<relays::Relay>(port));
  *value = (uint16_t)relay_status.fault_flags;

  return NMBS_ERROR_NONE;
}

nmbs_error fb_read_output_voltage(uint8_t port_index, uint16_t quantity, uint16_t *value)
{
  UNUSED(quantity);

  relays::RelayStatus status;
  status = relays::get_relay_status(static_cast<relays::Relay>(port_index));
  *value = (uint16_t)(status.voltage_v * (float)1000.0); //mV

  return NMBS_ERROR_NONE;
}

nmbs_error fb_read_output_current(uint8_t port_index, uint16_t quantity, int16_t *value)
{
  UNUSED(quantity);

  relays::RelayStatus status;
  status = relays::get_relay_status(static_cast<relays::Relay>(port_index));
  *value = (uint16_t)((status.current_a * (float)1000.0) * 0.1f); //mA, documentation factor is x10

  return NMBS_ERROR_NONE;
}

nmbs_error fb_read_output_power(uint8_t port_index, uint16_t quantity, int16_t *value)
{
  UNUSED(quantity);

  relays::RelayStatus status;
  status = relays::get_relay_status(static_cast<relays::Relay>(port_index));
  *value = (uint16_t)((status.power_w * (float)1000.0) * 0.01f); //mW, documentation factor is x100

  return NMBS_ERROR_NONE;
}

nmbs_error fb_read_output_energy_high(uint8_t port_index, uint16_t quantity, uint16_t *value)
{
  UNUSED(quantity);
  relays::RelayStatus status;
  status = relays::get_relay_status(static_cast<relays::Relay>(port_index));
  uint64_t joules64 = (uint64_t)round(status.energy_j);
  *value = (uint16_t)((joules64) >> 48); //J

  return NMBS_ERROR_NONE;
}

nmbs_error fb_read_output_energy_mid_high(uint8_t port_index, uint16_t quantity, uint16_t *value)
{
  UNUSED(quantity);

  relays::RelayStatus status;
  status = relays::get_relay_status(static_cast<relays::Relay>(port_index));
  uint64_t joules64 = (uint64_t)round(status.energy_j);
  *value = (uint16_t)((joules64 >> 32) & 0xFFFF); //J

  return NMBS_ERROR_NONE;
}

nmbs_error fb_read_output_energy_mid_low(uint8_t port_index, uint16_t quantity, uint16_t *value)
{
  UNUSED(quantity);

  relays::RelayStatus status;
  status = relays::get_relay_status(static_cast<relays::Relay>(port_index));
  uint64_t joules64 = (uint64_t)round(status.energy_j);
  *value = (uint16_t)((joules64 >> 16) & 0xFFFF); //J

  return NMBS_ERROR_NONE;
}

nmbs_error fb_read_output_energy_low(uint8_t port_index, uint16_t quantity, uint16_t *value)
{
  UNUSED(quantity);

  relays::RelayStatus status;
  status = relays::get_relay_status(static_cast<relays::Relay>(port_index));
  uint64_t joules64 = (uint64_t)round(status.energy_j);
  *value = (uint16_t)((joules64) & 0xFFFF); //J

  return NMBS_ERROR_NONE;
}

nmbs_error fb_read_button_states(uint8_t port_index, uint16_t quantity, uint16_t *value)
{
  UNUSED(quantity);

  *value = get_button_state(static_cast<ui::Button>(port_index));

  return NMBS_ERROR_NONE;
}

nmbs_error fb_output_led_control(const uint8_t *coils, uint16_t quantity, uint8_t port_index)
{
  if (!(fb_check_bits_quantity_validity(quantity, FB_OP_DATA_SIZES_BYTES::output_led_control)) ||
      (GET_LED_FOR_RELAY(port_index) >= leds::LED::LED_COUNT))
  {
    return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
  }

  /* Get LED index */
  leds::LED led = static_cast<leds::LED>(GET_LED_FOR_RELAY(port_index));

  /* Parse LED color */
  uint16_t rgb565 = binutils_bytes_to_uint16(coils, 0, binutils_BIG);
  leds::Color color = leds::rgb565_to_color(rgb565);

  /* Convert control time from 100ms/bit to milliseconds */
  uint16_t time_ms = coils[2] * 100;

  ULOG_DEBUG("LED %u to [%u %u %u] for %u ms", led, color.r, color.g, color.b, time_ms);
  if (!(leds::temporary_led_color_set(led, color, time_ms)))
  {
    return NMBS_EXCEPTION_SERVER_DEVICE_FAILURE;
  }

  return NMBS_ERROR_NONE;
}

nmbs_error fb_output_self_reset_req(const uint8_t *coils, uint16_t quantity, uint8_t port_index)
{
  if (!(fb_check_bits_quantity_validity(quantity, FB_OP_DATA_SIZES_BYTES::output_self_reset_req)) ||
       (port_index >= DP_RELAY_COUNT))
  {
    return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
  }

  for (uint16_t i = 0; i < quantity; i++)
  {
    const relays::Relay rel_n = static_cast<relays::Relay>(port_index + i);
    if (nmbs_bitfield_read(coils, i) != 0)
    {
      relays::control_relay(rel_n, relays::REQ_RESET, relays::REQSRC_MODBUS);
    }
  }

  return NMBS_ERROR_NONE;
}

nmbs_error fb_output_fault_flags_reset(const uint8_t *coils, uint16_t quantity, uint8_t port_index)
{
  if (!(fb_check_bits_quantity_validity(quantity, FB_OP_DATA_SIZES_BYTES::output_fault_flags_reset)) ||
       (port_index >= DP_RELAY_COUNT))
  {
    return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
  }

  uint8_t flags_to_reset = coils[0] & 0x0F;
  relays::reset_fault_flags(static_cast<relays::Relay>(port_index), flags_to_reset);

  return NMBS_ERROR_NONE;
}
