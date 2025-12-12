#include "config.h"
#include "config_default.h"

#define DEFAULT_IP_ADDRESS IP_ADDRESS(192, 168, 1, 100) // 192.168.1.100 in hexadecimal

#define DEFAULT_FORWARD_CURRENT_LIMIT  15000 // mA
#define DEFAULT_REVERSE_CURRENT_LIMIT  1500  // mA
#define DEFAULT_ACTIVATION_FILTER_TIME 200   // ms
#define DEFAULT_OPERATION_FILTER_TIME  200   // ms

#define DEFAULT_NORMAL_STATE  0
#define DEFAULT_STARTUP_STATE 0
#define DEFAULT_STARTUP_DELAY 500

#define DEFAULT_RECOVERY_MODE     0
#define DEFAULT_RECOVERY_OFF_TIME 1000 // ms

#define DEFAULT_RESET_OFF_TIME 2000 // ms

void config_set_firmware_default(CONFIG *cfg)
{
  cfg->general.ip_address = DEFAULT_IP_ADDRESS;

  for (uint8_t i = 0; i < DP_RELAY_COUNT; i++)
  {
    cfg->port[i].forward_current_limit  = DEFAULT_FORWARD_CURRENT_LIMIT;
    cfg->port[i].reverse_current_limit  = DEFAULT_REVERSE_CURRENT_LIMIT;
    cfg->port[i].activation_filter_time = DEFAULT_ACTIVATION_FILTER_TIME;
    cfg->port[i].operation_filter_time  = DEFAULT_OPERATION_FILTER_TIME;

    cfg->port[i].normal_state  = DEFAULT_NORMAL_STATE;
    cfg->port[i].startup_state = DEFAULT_STARTUP_STATE;
    cfg->port[i].startup_delay = DEFAULT_STARTUP_DELAY;

    cfg->port[i].recovery_mode     = DEFAULT_RECOVERY_MODE;
    cfg->port[i].recovery_off_time = DEFAULT_RECOVERY_OFF_TIME;

    cfg->port[i].reset_off_time = DEFAULT_RESET_OFF_TIME;
  }

  cfg->crc32 = 0;
}
