#include <nanomodbus.h>
#include "app_netxduo.h"
#include "comms.h"
#include "common.h"
#include "fb_main.h"
#include "watchdog.h"
#include "dp_info.h"
#include "rs485.h"
#include "mb_bridge_handlers.h"
#include "mb_local_handlers.h"

namespace comms
{
/* Private define ------------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
static int32_t modbus_tcp_read(uint8_t *buf, uint16_t count, int32_t byte_timeout_ms, void *arg);
static int32_t modbus_tcp_write(const uint8_t *buf, uint16_t count, int32_t byte_timeout_ms, void *arg);
static int32_t modbus_rs485_read(uint8_t *buf, uint16_t count, int32_t byte_timeout_ms, void *arg);
static int32_t modbus_rs485_write(const uint8_t *buf, uint16_t count, int32_t byte_timeout_ms, void *arg);

/* Private constants ---------------------------------------------------------*/
static const nmbs_platform_conf mb_tcp_server_conf =
{
  .transport = nmbs_transport::NMBS_TRANSPORT_TCP,
  .read      = modbus_tcp_read,
  .write     = modbus_tcp_write,
  .arg       = NULL,
};

static const nmbs_callbacks mb_tcp_server_callbacks =
{
  .read_coils                     = bridge_handlers::mb_bridge_read_coils,
  .read_discrete_inputs           = bridge_handlers::mb_bridge_read_discrete_inputs,
  .read_holding_registers         = bridge_handlers::mb_bridge_read_holding_registers,
  .read_input_registers           = bridge_handlers::mb_bridge_read_input_registers,
  .write_single_coil              = bridge_handlers::mb_bridge_write_single_coil,
  .write_single_register          = bridge_handlers::mb_bridge_write_single_register,
  .write_multiple_coils           = bridge_handlers::mb_bridge_write_multiple_coils,
  .write_multiple_registers       = bridge_handlers::mb_bridge_write_multiple_registers,
  .read_file_record               = NULL, // TODO Could use for reading out error log.
  .write_file_record              = NULL, // TODO Could use for bootloading.
  .read_device_identification     = NULL, // TODO Take into use?
  .read_device_identification_map = NULL,
};

static const nmbs_platform_conf mb_rtu_server_conf =
{
  .transport = nmbs_transport::NMBS_TRANSPORT_RTU,
  .read      = modbus_rs485_read,
  .write     = modbus_rs485_write,
  .arg       = NULL,
};

static const nmbs_callbacks mb_rtu_server_callbacks =
{
  .read_coils                     = local_handlers::mb_local_read_coils,
  .read_discrete_inputs           = local_handlers::mb_local_read_discrete_inputs,
  .read_holding_registers         = local_handlers::mb_local_read_holding_registers,
  .read_input_registers           = local_handlers::mb_local_read_input_registers,
  .write_single_coil              = local_handlers::mb_local_write_single_coil,
  .write_single_register          = local_handlers::mb_local_write_single_register,
  .write_multiple_coils           = local_handlers::mb_local_write_multiple_coils,
  .write_multiple_registers       = local_handlers::mb_local_write_multiple_registers,
  .read_file_record               = NULL, // TODO Could use for reading out error log.
  .write_file_record              = NULL, // TODO Could use for bootloading.
  .read_device_identification     = NULL, // TODO Take into use?
  .read_device_identification_map = NULL,
};

const nmbs_platform_conf mb_rtu_client_conf =
{
  .transport = nmbs_transport::NMBS_TRANSPORT_RTU,
  .read      = modbus_rs485_read,
  .write     = modbus_rs485_write,
  .arg       = NULL,
};

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static nmbs_t modbus_tcp_server;
static nmbs_t modbus_rtu_server;
static nmbs_t modbus_rtu_client;

/* Public variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* TODO
 * Nanomodbus need improvement - it should give an argument that tells if the
 * RX function call is to check if there is header data in the buffer or it is
 * a follow up RX call that expects there to be actual data in the buffer.
 * In first case we don't want RX function to wait for timeout, in second case it
 * must have a timeout.
 */

static int32_t modbus_tcp_read(uint8_t *buf, uint16_t count, int32_t byte_timeout_ms, void *arg)
{
  tcp_peer_t *peer = (tcp_peer_t*) arg;
  uint32_t rx_count;
  uint32_t ret = netxduo_tcp_rx(peer, buf, count, &rx_count, byte_timeout_ms);
  if (ret == TX_QUEUE_EMPTY)
  {
    /* Return as many bytes as we got so far (could be zero or more) */
    return rx_count;
  }
  else if (ret != TX_SUCCESS)
  {
    return NMBS_ERROR_TRANSPORT;
  }

  return rx_count;
}

static int32_t modbus_tcp_write(const uint8_t *buf, uint16_t count, int32_t byte_timeout_ms, void *arg)
{
  tcp_peer_t *peer = (tcp_peer_t*) arg;
  uint32_t ret = netxduo_tcp_tx(peer, buf, count, byte_timeout_ms);
  if (ret != TX_SUCCESS)
  {
    return NMBS_ERROR_TRANSPORT;
  }

  return count;
}

static int32_t modbus_rs485_read(uint8_t *buf, uint16_t count, int32_t byte_timeout_ms, void *arg)
{
  UNUSED(arg);

  return rs485_receive(buf, count, byte_timeout_ms);
}

static int32_t modbus_rs485_write(const uint8_t *buf, uint16_t count, int32_t byte_timeout_ms, void *arg)
{
  UNUSED(arg);

  /* nanomodbus uses same inter-byte delay for reception (read) and transmission (write).
   * But transmission does not require some flexible timing as reception, because it is deterministic
   * by the driver design and if there is a transmission error then it can be detected much faster.
   * As RS-485 baudrate is 115200 bps, inter-byte timing is 86 us and hence minimum of 1 ms can be used. */
  byte_timeout_ms = 1;

  return rs485_transmit(buf, count, byte_timeout_ms);
}

static void modbus_tcp_server_init()
{
  nmbs_error err = nmbs_server_create(&modbus_tcp_server, dp_get_device_id(), &mb_tcp_server_conf, &mb_tcp_server_callbacks);
  if (err != NMBS_ERROR_NONE)
  {
    ULOG_CRITICAL("Modbus TCP server could not be created: %s", nmbs_strerror(err));
    Error_Handler();
  }

  nmbs_set_byte_timeout(&modbus_tcp_server, 100);
  nmbs_set_read_timeout(&modbus_tcp_server, 500);
}

static void modbus_tcp_server_update()
{
  tcp_peer_t current_peer = {};
  tcp_peer_t next_peer = netxduo_tcp_get_next_peer(current_peer);

  while ((next_peer.ip != 0) && (next_peer.port != 0))
  {
    nmbs_set_platform_arg(&modbus_tcp_server, &next_peer);
    nmbs_error err = nmbs_server_poll(&modbus_tcp_server);
    if (err != NMBS_ERROR_NONE)
    {
      ULOG_ERROR("Modbus TCP server polling error: %s", nmbs_strerror(err));
    }

    current_peer = next_peer;
    next_peer = netxduo_tcp_get_next_peer(current_peer);
  }
}

static void modbus_rtu_server_init()
{
  nmbs_error err = nmbs_server_create(&modbus_rtu_server, dp_get_device_id(), &mb_rtu_server_conf, &mb_rtu_server_callbacks);
  if (err != NMBS_ERROR_NONE)
  {
    ULOG_CRITICAL("Modbus RTU server could not be created: %s", nmbs_strerror(err));
    Error_Handler();
  }

  /* Read timeout is used by nanomodbus when waiting for the first header byte.
   * From server side, it is unknown when does some request come and header byte shall
   * not be waited, but checked, because waiting blocks comms thread execution.
   * Without having timeout, RS-485 read function returns data only if it is already in the
   * buffer, otherwise it exits quickly. */
  nmbs_set_read_timeout(&modbus_rtu_server, 0);

  /* Inter-byte timeout. In contrary to first byte timeout, this must be non-zero
   * because it is used to check if all the remaining bytes of the request come in time.
   * It is also used when writing bytes, so it must be long enough to transmit bytes. */
  nmbs_set_byte_timeout(&modbus_rtu_server, 100);
}

static void modbus_rtu_server_update()
{
  nmbs_error err = nmbs_server_poll(&modbus_rtu_server);
  if (err != NMBS_ERROR_NONE)
  {
    ULOG_ERROR("Modbus RTU server polling error: %s", nmbs_strerror(err));
  }
}

static void modbus_rtu_client_init()
{
  nmbs_error err = nmbs_client_create(&modbus_rtu_client, &mb_rtu_client_conf);
  if (err != NMBS_ERROR_NONE)
  {
    ULOG_CRITICAL("Modbus RTU client could not be created: %s", nmbs_strerror(err));
    Error_Handler();
  }

  /* Read timeout is used by nanomodbus when waiting for the first header byte.
   * In master mode device uses it as a timeout when slave should start responding to master,
   * so it should be bigger than inter-byte timeout. */
  nmbs_set_read_timeout(&modbus_rtu_client, 500);

  /* Inter-byte timeout. This must be non-zero because it is used by nanomodbus when
   * reading bytes that follow header byte and when writing bytes. */
  nmbs_set_byte_timeout(&modbus_rtu_client, 100);
}

/**
 * Communication thread entry
 */
static void comms_thread_entry()
{
  /* Wait for DIP switches configuration */
  watchdog_configure_thread(WATCHDOG_THREAD_COMMS, 50, 100);
  while (!get_initial_switches_set())
  {
    sleep_ms(50);
    watchdog_kick_thread(WATCHDOG_THREAD_COMMS);
  }

  /* Initialize device mode specific communication */
  DP_DEVICE_MODE device_mode = dp_get_device_mode();
  if (device_mode == DP_MASTER)
  {
    modbus_tcp_server_init();
    modbus_rtu_client_init();
    bridge_handlers::mb_bridge_init(&modbus_rtu_client);
  }
  else if (device_mode == DP_SLAVE)
  {
    modbus_rtu_server_init();
  }

  /* Commnication working loop */
  uint32_t wake_time = tx_time_get();
  const uint32_t update_interval_ms = 5;

  /* Use 10s timeout because in worst case scenario all 10 TCP clients
   * send request at the same time and that should be handled in a single loop. */
  watchdog_configure_thread(WATCHDOG_THREAD_COMMS, update_interval_ms, 10000);

  while (true)
  {
    if (device_mode == DP_MASTER)
    {
      modbus_tcp_server_update();
    }
    else if (device_mode == DP_SLAVE)
    {
      modbus_rtu_server_update();
    }

    // Make sure loop runs at regular interval
    sleep_until_ms(&wake_time, update_interval_ms);
    watchdog_kick_thread(WATCHDOG_THREAD_COMMS);
  }
}

/* Public functions ----------------------------------------------------------*/

/**
 * Communication initialization
 */
void initialize_comms()
{
  rs485_init(UART8);
}

}

extern "C"
{
VOID Fuseboard_Comms_Thread_Entry(ULONG)
{
  comms::comms_thread_entry();
}
}

