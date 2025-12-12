#pragma once

#include <nanomodbus.h>
#include <stdint.h>
#include <inttypes.h>

nmbs_error fb_dev_reg_get_slave_count(uint16_t *registers_out, uint16_t quantity);
nmbs_error fb_dev_reg_get_output_count(uint16_t *registers_out, uint16_t quantity);
nmbs_error fb_dev_reg_get_stm32_uid(uint16_t *registers_out, uint16_t quantity);
nmbs_error fb_dev_reg_get_mac(uint16_t *registers_out, uint16_t quantity);
nmbs_error fb_dev_reg_get_serial_number(uint16_t *registers_out, uint16_t quantity);
nmbs_error fb_dev_reg_get_hardware_revision(uint16_t *registers_out, uint16_t quantity);
nmbs_error fb_dev_reg_get_bootloader_ver_ascii(uint16_t *registers_out, uint16_t quantity);
nmbs_error fb_dev_reg_get_firmware_ver_ascii(uint16_t *registers_out, uint16_t quantity);
