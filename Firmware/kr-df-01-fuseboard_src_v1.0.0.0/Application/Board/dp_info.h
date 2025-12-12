#pragma once

#include "app_netxduo.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define DP_RELAY_COUNT 8U

/* Serial number is a 24-bit unsigned number, which means maximum value has 8 digits */
#define DP_SERIAL_NUMBER_MAX_DIGITS 8U

typedef enum
{
  DP_MASTER = 0,
  DP_SLAVE
} DP_DEVICE_MODE;

typedef enum
{
  DP_IP_MODE_NOT_AVAILABLE = 0,
  DP_IP_MODE_STATIC,
  DP_IP_MODE_DYNAMIC
} DP_IP_MODE;

#pragma pack(push, 1)

typedef struct
{
  bool    is_debug;
  uint8_t reserved1[15];
  char    version[16];
  char    build_date[16];
  char    build_time[16];
  char    commit_id[16];

  /* 80 bytes to here */
  uint8_t reserved2[176];
} DP_FW_INFO;

#pragma pack(pop)

_Static_assert(sizeof(DP_FW_INFO) == 256, "Device info block size not 256 bytes");

void     dp_init_device_info();
uint32_t dp_get_serial_number();
uint8_t  dp_get_hw_revision();
uint32_t dp_get_mac_vendor_id();

DP_IP_MODE dp_get_ip_mode();
void dp_set_ip_mode(DP_IP_MODE mode);

// Getter/setter for MASTER or SLAVE mode
DP_DEVICE_MODE dp_get_device_mode();
void dp_set_device_mode(DP_DEVICE_MODE mode);

// Functions to get max and min allowed slave IDs
uint8_t dp_get_min_slave_id();
uint8_t dp_get_max_slave_id();

// Function to get firmware version
const DP_FW_INFO* dp_get_fw_info();

// Getters and setters for slave count and device id variables
uint8_t dp_get_slave_count();
void dp_set_slave_count(uint8_t count);
uint8_t dp_get_device_id();
void dp_set_device_id(uint8_t id);

#ifdef __cplusplus
}
#endif
