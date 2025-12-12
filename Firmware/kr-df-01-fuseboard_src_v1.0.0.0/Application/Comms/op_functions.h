#pragma once

#include "nanomodbus.h"

nmbs_error fb_device_reset(uint16_t address, const nmbs_bitfield coils, uint16_t quantity);
nmbs_error fb_config_change(const uint8_t *coils, uint16_t quantity);
nmbs_error fb_reformat_flash(const uint8_t *coils, uint16_t quantity);

nmbs_error fb_uptime(uint16_t quantity, uint32_t *value);

nmbs_error fb_output_control(uint8_t start_port, uint16_t quantity, const nmbs_bitfield coils);
nmbs_error fb_output_control_get(uint8_t start_port, uint16_t quantity, nmbs_bitfield coils_out);

nmbs_error fb_output_state(uint8_t port, uint16_t quantity, uint16_t *value);
nmbs_error fb_output_state_reason(uint8_t port, uint16_t quantity, uint16_t *value);
nmbs_error fb_output_fault_flags(uint8_t port, uint16_t quantity, uint16_t *value);

nmbs_error fb_read_output_voltage(uint8_t port_index, uint16_t quantity, uint16_t *value);
nmbs_error fb_read_output_current(uint8_t port_index, uint16_t quantity, int16_t *value);
nmbs_error fb_read_output_power  (uint8_t port_index, uint16_t quantity, int16_t *value);
nmbs_error fb_read_output_energy_high(uint8_t port_index, uint16_t quantity, uint16_t *value); // higher order half word
nmbs_error fb_read_output_energy_mid_high(uint8_t port_index, uint16_t quantity, uint16_t *value);
nmbs_error fb_read_output_energy_mid_low(uint8_t port_index, uint16_t quantity, uint16_t *value);
nmbs_error fb_read_output_energy_low (uint8_t port_index, uint16_t quantity, uint16_t *value); // lower order half word
nmbs_error fb_read_button_states(uint8_t port_index, uint16_t quantity, uint16_t *value);

nmbs_error fb_output_led_control(const uint8_t *coils, uint16_t quantity, uint8_t port_index);
nmbs_error fb_output_self_reset_req(const uint8_t *coils, uint16_t quantity, uint8_t port_index);
nmbs_error fb_output_fault_flags_reset(const uint8_t *coils, uint16_t quantity, uint8_t port_index);
