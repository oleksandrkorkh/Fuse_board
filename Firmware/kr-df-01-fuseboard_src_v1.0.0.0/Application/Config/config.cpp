#include <stdbool.h>
#include <stdint.h>
#include "config.h"
#include "config_default.h"
#include <tx_api.h>
#include "common.h"
#include "watchdog.h"
#include <stm32h5xx_hal.h>
#include "log.h"
#include "main.h"
#include "ulog.h"
#include <tx_extension.h>
#include "nvm.h"
#include "crc.h"
#include "config_test.h"

//#define CONFIG_DEBUG_TEST

#define CONFIG_EVENT_QUEUE_MEM_SIZE        128
#define OUTER_QUEUE_ACCESS_WAIT_TIMEOUT_MS 500
#define LOCAL_QUEUE_ACCESS_WAIT_TIMEOUT_MS 100

static void config_find_valid();
static CONFIG_OPERATION_EVENT_RES config_op_to_flash(CONFIG_OPERATION_EVENT event);

static UINT config_add_res_queue(CONFIG_OPERATION_EVENT_RES event);
static void config_check_op_queue();

const char *factory_def_cfg_filename = "factory_def_cfg";
const char *live_cfg_filename        = "live_cfg";

static CONFIG live_cfg;

static UCHAR    config_op_event_queue_mem[CONFIG_EVENT_QUEUE_MEM_SIZE];
static TX_QUEUE config_op_event_queue;

static UCHAR    config_res_event_queue_mem[CONFIG_EVENT_QUEUE_MEM_SIZE];
static TX_QUEUE config_res_event_queue;

static TX_MUTEX config_general_rwlock_mutex;
static TX_MUTEX config_port_rwlock_mutex;

void config_init()
{
  tx_queue_create(&config_op_event_queue,  (char* )"Config operation events", sizeof(CONFIG_OPERATION_EVENT), config_op_event_queue_mem, CONFIG_EVENT_QUEUE_MEM_SIZE);
  tx_queue_create(&config_res_event_queue, (char* )"Config response events",  sizeof(CONFIG_OPERATION_EVENT_RES), config_res_event_queue_mem, CONFIG_EVENT_QUEUE_MEM_SIZE);

  tx_mutex_create(&config_general_rwlock_mutex, (char* )"config_general_rwlock_mutex", TX_NO_INHERIT);
  tx_mutex_create(&config_port_rwlock_mutex,    (char* )"config_port_rwlock_mutex", TX_NO_INHERIT);

  config_find_valid();
}

UINT config_add_op_queue(CONFIG_OPERATION_EVENT event)
{
  ASSERT_FATAL(event < CONFIG_OPERATION_TYPE_COUNT, "Config save type non-existent");

  UINT response = tx_queue_send(&config_op_event_queue, &event, TX_TICKS_MS(OUTER_QUEUE_ACCESS_WAIT_TIMEOUT_MS));

  if (response != TX_SUCCESS)
  {
    ULOG_CRITICAL("config op queue not enqueable");
  }

  return response;
}

static void config_check_op_queue(uint32_t update_interval_ms)
{
  UINT status;
  CONFIG_OPERATION_EVENT event;

  status = tx_queue_receive(&config_op_event_queue, &event, TX_TICKS_MS(update_interval_ms));
  if (status == TX_SUCCESS)
  {
    CONFIG_OPERATION_EVENT_RES response = config_op_to_flash(event);
    config_add_res_queue(response);
  }
  else if (status != TX_QUEUE_EMPTY)
  {
    ULOG_CRITICAL("config_op_event_queue error: %u", status);
  }
}

static UINT config_add_res_queue(CONFIG_OPERATION_EVENT_RES event)
{
  ASSERT_FATAL(event < CONFIG_FLASH_OP_RES_COUNT, "Config response type non-existent");

  UINT response = tx_queue_send(&config_res_event_queue, &event, TX_TICKS_MS(LOCAL_QUEUE_ACCESS_WAIT_TIMEOUT_MS));

  if (response != TX_SUCCESS)
  {
    ULOG_ERROR("config op queue not enqueable");
  }

  return response;
}

bool config_check_res_queue(CONFIG_OPERATION_EVENT_RES *event)
{
  UINT status;

  status = tx_queue_receive(&config_res_event_queue, event, TX_TICKS_MS(OUTER_QUEUE_ACCESS_WAIT_TIMEOUT_MS));
  if (status == TX_SUCCESS)
  {
    ULOG_DEBUG("response queue item received");
  }
  else if (status == TX_QUEUE_EMPTY)
  {
    ULOG_DEBUG("response queue empty");
  }
  else
  {
    ULOG_ERROR("response queue error: %u", status);
  }

  return (status == TX_SUCCESS);
}

// returns true if crc correct
static bool config_util_compare_data_crc_to_data(CONFIG *config, uint32_t length)
{
  const uint32_t read_crc = config->crc32;
  uint32_t crc = crc_gen_crc32((uint32_t*)config, length - sizeof(uint32_t));

  return read_crc == crc ? true : false;
}

// returns true if success
static bool config_save_to_flash(const char *cfg_filename, CONFIG *cfg, const uint32_t cfg_buf_size)
{
  cfg->crc32 = crc_gen_crc32((uint32_t *)cfg, cfg_buf_size - sizeof(uint32_t));
  bool save_result = nvm_write(cfg_filename, (uint8_t *)cfg, cfg_buf_size, NVM_OP_OVERWRITE);
  return save_result;
}

// returns true if valid config found
static bool config_load_from_flash(const char *cfg_filename, CONFIG *cfg_buf, const uint32_t cfg_buf_size)
{
  bool valid_cfg_found = false;

  if (nvm_read(cfg_filename, (uint8_t *)cfg_buf, cfg_buf_size, NVM_READ_ONLY))
  {
    bool crc_comp_result = config_util_compare_data_crc_to_data(cfg_buf, cfg_buf_size);
    if (crc_comp_result)
    {
      ULOG_DEBUG("Found config %s file in flash", cfg_filename);
      valid_cfg_found = true;
    }
    else
    {
      ULOG_DEBUG("Found config %s file in flash, but data crc incorrect", cfg_filename);
    }
  }
  else
  {
    ULOG_WARNING("No config %s file found", cfg_filename);
  }

  return valid_cfg_found;
}

static void config_find_valid()
{
  if (!(config_load_from_flash(live_cfg_filename, &live_cfg, sizeof(live_cfg))))
  {
    if (!(config_load_from_flash(factory_def_cfg_filename, &live_cfg, sizeof(live_cfg))))
    {
      ULOG_WARNING("No valid configs found in filesystem. Using firmware default config");
      config_set_firmware_default(&live_cfg);
      bool status = config_save_to_flash(live_cfg_filename, &live_cfg, sizeof(live_cfg));
      if (!(status))
      {
        ULOG_ERROR("Writing firm default to live cfg flash failed");
      }
    }
  }

  return;
}

static CONFIG_OPERATION_EVENT_RES config_op_to_flash(CONFIG_OPERATION_EVENT event)
{
  ASSERT_FATAL(event < CONFIG_OPERATION_TYPE_COUNT, "Config save type non-existent");
  CONFIG_OPERATION_EVENT_RES response = CONFIG_FLASH_OP_FAILED;
  bool status = 0;

  if (event == CONFIG_SAVE_ACTIVE_CFG)
  {
    ULOG_INFO("Save current configuration");
    status = config_save_to_flash(live_cfg_filename, &live_cfg, sizeof(live_cfg));
    if (status)
    {
      response = CONFIG_FLASH_OP_SUCCESS;
    }
  }
  else if (event == CONFIG_SAVE_FACTORY_DEF_CFG)
  {
    ULOG_INFO("Save current configuration as factory default");
    status = config_save_to_flash(factory_def_cfg_filename, &live_cfg, sizeof(live_cfg));
    if (status)
    {
      response = CONFIG_FLASH_OP_SUCCESS;
    }
  }
  else if (event == CONFIG_LOAD_FACTORY_DEF_CFG)
  {
    ULOG_INFO("Load factory default configuration");
    status = config_load_from_flash(factory_def_cfg_filename, &live_cfg, sizeof(live_cfg)); // returns true or false, 1 or 0
    if (status)
    {
      response = CONFIG_FLASH_OP_SUCCESS;
    }
  }
  else
  {
    ULOG_ERROR("CONFIG_SAVE_TYPE value error: %u", &event);
  }

  return response;
}

static bool config_confirm_general_values(const CONFIG_GENERAL *general)
{
  // TODO: add items in case of additional general config values
  return true;
}

static bool config_confirm_port_values(const CONFIG_PORT *port)
{
  if ((port->normal_state  > 1) ||
      (port->startup_state > 1) ||
      (port->recovery_mode > 10))
  {
    return false;
  }

  return true;
}

void config_get_general_data(CONFIG_GENERAL *general)
{
  ASSERT_FATAL(general != NULL, "Pointer is NULL");

  TX_MUTEX_SECTION(&config_general_rwlock_mutex)
  {
    *general = live_cfg.general;
  }
}

void config_get_port_data(CONFIG_PORT *port, uint8_t port_index)
{
  ASSERT_FATAL(port != NULL, "Pointer is NULL");
  ASSERT_FATAL(port_index < DP_RELAY_COUNT, "Port number too high");

  TX_MUTEX_SECTION(&config_port_rwlock_mutex)
  {
    *port = live_cfg.port[port_index];
  }
}

bool config_set_general_data(const CONFIG_GENERAL *general)
{
  ASSERT_FATAL(general != NULL, "Pointer is NULL");

  if (config_confirm_general_values(general))
  {
    TX_MUTEX_SECTION(&config_general_rwlock_mutex)
    {
      live_cfg.general = *general;
    }

    return true;
  }
  else
  {
    return false;
  }
}

bool config_set_port_data(const CONFIG_PORT *port, uint8_t port_index)
{
  ASSERT_FATAL(port != NULL, "Pointer is NULL");
  ASSERT_FATAL(port_index < DP_RELAY_COUNT, "Port number too high");

  if (config_confirm_port_values(port))
  {
    TX_MUTEX_SECTION(&config_port_rwlock_mutex)
    {
      live_cfg.port[port_index] = *port;
    }

    return true;
  }
  else
  {
    return false;
  }

}

static void config_thread_entry()
{
#ifdef CONFIG_DEBUG_TEST
  config_save_load_test_1();

  config_add_op_queue(SAVE_FACTORY_DEF_CFG);
  config_check_op_queue();
  config_save_load_test_2();

  config_add_op_queue(LOAD_FACTORY_DEF_CFG);
  config_check_op_queue();
  config_save_load_test_3();

  config_add_op_queue(SAVE_FACTORY_DEF_CFG);
  config_check_op_queue();
  config_save_load_test_4();

  config_add_op_queue(LOAD_FACTORY_DEF_CFG);
  config_check_op_queue();
  config_save_load_test_5();

  config_set_firmware_default(&live_cfg);
  config_add_op_queue(SAVE_FACTORY_DEF_CFG);
  config_check_op_queue();
#endif

  const uint32_t update_interval_ms = 100;

  /* Loop gets shorter than interval time if OP queue is filled faster than loop works,
   * so use variadic interval. */
  watchdog_configure_thread(WATCHDOG_THREAD_CONFIG, WATCHDOG_INTERVAL_VARIADIC, 2000);

  while (true)
  {
    config_check_op_queue(update_interval_ms);
    watchdog_kick_thread(WATCHDOG_THREAD_CONFIG);
  }
}

extern "C"
{
VOID Config_Thread_Entry(ULONG)
{
  config_thread_entry();
}
}
