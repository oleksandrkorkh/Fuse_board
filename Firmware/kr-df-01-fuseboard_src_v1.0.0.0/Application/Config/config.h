#pragma once

#include <tx_api.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include "dp_info.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define CONFIG_DATA_MAX_SIZE 256

/**
 * normal_state:  Relay normal state: NO - 0 or NC - 1
 * startup_state: Output off / On. Applied with a delay after
 *                firmware start-up. 0 - OFF 1 - ON
 * startup_delay: Time in milliseconds after start-up when to
 *                change output state to its start-up state. unit - ms
 *
 * forward_current_limit:  0 to +15A limit for each output. Unit - mA
 * reverse_current_limit:  0 to -15A limit for each output.
 *                         Limit is expressed in positive numbers. Unit - mA
 * activation_filter_time: Current limit ignoring time (milliseconds) when output is turned on.
 *                         To filter out spikes caused by in-rush current. Unit - ms
 * operation_filter_time:  Current limit ignoring time (milliseconds) when output is already on.
 *                         To filter out spikes that occur during operation. Unit - ms
 *
 * recovery_mode:      Behavior after the current limit is exceeded and output
 *                     is turned off for device protection.
 *                     • No recovery, stay off.
 *                     Off-on sequence or reset command required to turn on again.
 *                     • Try to automatically turn it on up to 10 times.
 *                     Apply delay (recovery off time) between retries.
 *                     After retries, stay off. Controlled off-on or reset required.
 *                     Note: must have a limit on retries to increase relay life.
 *                     0 – No recovery
 *                     1 – Retry once
 *                     2 – Retry twice
 *                     …
 *                     10 – Retry 10 times
 *
 * recovery_off_time: Time to wait in milliseconds before trying to turn the output on again.
 *                    Unit - ms
 *
 * reset_off_time: Time to wait in milliseconds in off state when doing output automatic reset.
 *                 Unit - ms
 */

#pragma pack(push, 2)  // 2-byte alignment

typedef struct {
  uint16_t normal_state;
  uint16_t startup_state;
  uint16_t startup_delay;
  uint16_t forward_current_limit;
  uint16_t reverse_current_limit;
  uint16_t activation_filter_time;
  uint16_t operation_filter_time;
  uint16_t recovery_mode;
  uint16_t recovery_off_time;
  uint16_t reset_off_time; // 20 bytes used until here
  uint16_t reserved[6];    // 32 bytes total, for possibility of future additions
} CONFIG_PORT;

_Static_assert(sizeof(CONFIG_PORT) == 32, "CONFIG PORT structure is not expected size");

typedef struct {
  uint32_t ip_address;
  uint16_t reserved[14];
} CONFIG_GENERAL;

_Static_assert(sizeof(CONFIG_GENERAL) == 32, "CONFIG GENERAL structure is not expected size");

typedef struct {
  CONFIG_GENERAL general;
  CONFIG_PORT    port[DP_RELAY_COUNT]; // example = 32 bytes each * 8 relays = 256 bytes
  uint32_t       crc32;                // Must be last field
} CONFIG;

#pragma pack(pop)

typedef enum {
  CONFIG_SAVE_ACTIVE_CFG = 0,
  CONFIG_SAVE_FACTORY_DEF_CFG,
  CONFIG_LOAD_FACTORY_DEF_CFG,
  CONFIG_OPERATION_TYPE_COUNT
} CONFIG_OPERATION_EVENT;

typedef enum {
  CONFIG_FLASH_OP_FAILED = 0,
  CONFIG_FLASH_OP_SUCCESS,
  CONFIG_FLASH_OP_RES_COUNT
} CONFIG_OPERATION_EVENT_RES;

void config_init();

UINT config_add_op_queue   (CONFIG_OPERATION_EVENT event);
bool config_check_res_queue(CONFIG_OPERATION_EVENT_RES *event);

void config_get_general_data(CONFIG_GENERAL *general);
void config_get_port_data(CONFIG_PORT *port, uint8_t port_index);
bool config_set_general_data(const CONFIG_GENERAL *general);
bool config_set_port_data(const CONFIG_PORT *port, uint8_t port_index);

#ifdef __cplusplus
}
#endif

