#include "dp_info.h"
#include "comms.h"
#include "fb_main.h"
#include "leds.h"
#include "buttons.h"
#include "relays.h"
#include "common.h"
#include "board.h"
#include "adc.h"
#include "watchdog.h"
#include "SEGGER_RTT.h"
#include "nvm.h"
#include "config.h"
#include "log_flash.h"

#include <string.h>
#include <stdio.h>

static char rtt_cmd[32] = { 0 };
bool initial_switches_set = false;

/**
 * Handle DIP switches change event
 * @param value: Switches value?
 */
static void handle_switches_change(uint32_t value)
{
  UNUSED(value);

  // Set device conf from DIP switches only once
  if (!initial_switches_set)
  {
    ULOG_INFO("Updating configuration based on switches...");

    // TODO: Update configuration and mode based on switches
    uint8_t deviceModeSwitch = ((ui::current_switch_statuses >> 0) & 1);
    if (deviceModeSwitch == 1)
    {
      dp_set_device_mode(DP_MASTER);
    }
    else
    {
      dp_set_device_mode(DP_SLAVE);
    }

    uint8_t ipSwitch = ((ui::current_switch_statuses >> 1) & 1);
    if (ipSwitch == 1)
    {
      dp_set_ip_mode(DP_IP_MODE_STATIC);
    }
    else
    {
      dp_set_ip_mode(DP_IP_MODE_DYNAMIC);
    }

    uint8_t slave_count_or_address = ((ui::current_switch_statuses >> 2) & 0b111);
    if (dp_get_device_mode() == DP_DEVICE_MODE::DP_MASTER)
    {
      dp_set_device_id(0);
      dp_set_slave_count(slave_count_or_address);
    }
    else
    {
      if (slave_count_or_address == 0)
      {
        slave_count_or_address = 1; // Slave ID must not never be 0, so setting it to 1
      }

      dp_set_device_id(slave_count_or_address);
      dp_set_slave_count(0);
    }

    initial_switches_set = true;
  }
}

/**
 * Handle button press event
 * @param value: Button value
 */
static void handle_button_press(uint32_t value)
{
  relays::Relay rel_n = (relays::Relay)value;
  ULOG_INFO("Manually toggling relay #%u", rel_n);

  relays::RelayStatus status = relays::get_relay_status(rel_n);
  switch (status.state)
  {
    case relays::RELAY_OFF:
    case relays::RELAY_TURNING_OFF:
    case relays::RELAY_EXCESS_VOLTAGE:
      relays::control_relay(rel_n, relays::REQ_ON, relays::REQSRC_BUTTON);
      break;

    default:
      relays::control_relay(rel_n, relays::REQ_OFF, relays::REQSRC_BUTTON);
      break;
  }
}

static void handle_ui()
{
  std::optional<ui::Event> ui_event = ui::poll_ui_event();
  while (ui_event.has_value())
  {
    switch (ui_event->type)
    {
      case ui::SWITCHES_CHANGED:
        handle_switches_change(ui_event->value);
        break;

      case ui::BUTTON_PRESSED:
        handle_button_press(ui_event->value);
        break;

      case ui::BUTTON_RELEASED:
        // No function for this yet
        break;

      default:
        ULOG_ERROR("Unknown UI event: %d", ui_event->type);
        break;
    }

    ui_event = ui::poll_ui_event();
  }
}

/**
 * Update network indicator based on network link status
 */
static void handle_link_status()
{
  switch (netxduo_get_link_status())
  {
    case LINK_STATUS_NOT_AVAILABLE:
    case LINK_STATUS_DISCONNECTED:
      leds::control_led(leds::LED_STATUS_2, leds::RED);
      break;
    case LINK_STATUS_AQUIRING_IP:
      leds::control_led(leds::LED_STATUS_2, leds::YELLOW, leds::BLINK, 100, 900);
      break;
    case LINK_STATUS_CONNECTED:
      leds::control_led(leds::LED_STATUS_2, leds::GREEN);
      break;
  };
}

static void handle_board()
{
  static uint32_t last_status_dump_time_s = 0;
  const uint32_t time_s = tx_time_get() / TX_TIMER_TICKS_PER_SECOND;
  const uint32_t status_dump_interval_s = 3;

  if (time_s - last_status_dump_time_s > status_dump_interval_s)
  {
    board::BoardStatus brdStatus = board::get_board_status();
    ULOG_DEBUG("Temperature and v_in: %.3fC, %.3fV", brdStatus.temperature, brdStatus.v_in);

    last_status_dump_time_s = time_s;
    ULOG_DEBUG("Next relay status dump in %us", status_dump_interval_s);
  }
}

static void handle_relays()
{
  static uint32_t last_status_dump_time_s = 0;
  const uint32_t time_s = tx_time_get() / TX_TIMER_TICKS_PER_SECOND;
  const uint32_t status_dump_interval_s = 3;

  for (relays::Relay rel_n : relays::RELAYS)
  {
    relays::RelayStatus status = relays::get_relay_status(rel_n);

    if (time_s - last_status_dump_time_s > status_dump_interval_s)
    {
      ULOG_DEBUG("Relay #%u: state %d, %.3fV, %.3fA", rel_n, status.state, status.voltage_v, status.current_a);
    }

    switch (status.state)
    {
      case relays::RELAY_INIT:
      case relays::RELAY_STARTUP_DELAY:
      case relays::RELAY_OFF:
        leds::control_led(GET_LED_FOR_RELAY(rel_n), leds::OFF);
        break;

      case relays::RELAY_ON:
      case relays::RELAY_TURNING_ON:
      case relays::RELAY_TURNING_OFF:
        leds::control_led(GET_LED_FOR_RELAY(rel_n), leds::GREEN);
        break;

      case relays::RELAY_UNDERCURRENT:
      case relays::RELAY_OVERCURRENT:
        leds::control_led(GET_LED_FOR_RELAY(rel_n), leds::ORANGE);
        break;

      case relays::RELAY_EXCESS_VOLTAGE:
        leds::control_led(GET_LED_FOR_RELAY(rel_n), leds::YELLOW, leds::BLINK, 250, 250);
        break;

      case relays::RELAY_ABSENT_VOLTAGE:
        leds::control_led(GET_LED_FOR_RELAY(rel_n), leds::RED);
        break;

      case relays::RELAY_RESET:
        leds::control_led(GET_LED_FOR_RELAY(rel_n), leds::BLUE);
        break;

      case relays::RELAY_PROTECTING:
      case relays::RELAY_RECOVERY_END:
        leds::control_led(GET_LED_FOR_RELAY(rel_n), leds::RED, leds::BLINK, 250, 250);
        break;

      default:
        ULOG_CRITICAL("Unhandled relay %u state: %u", rel_n, status.state);
        break;
    }

    // TODO: Update relays with modbus
  }

  if (time_s - last_status_dump_time_s > status_dump_interval_s)
  {
    last_status_dump_time_s = time_s;
    ULOG_DEBUG("Next relay status dump in %us", status_dump_interval_s);
  }
}

/**
 * Handle incoming RTT terminal commands.
 */
static void handle_rtt_commands(void)
{
  int32_t key, len;

  /* Make strlen safe by ensuring the command buffer has zero-delimiter */
  rtt_cmd[sizeof(rtt_cmd) - 1] = 0;
  len = strlen(rtt_cmd);

  /* Read RTT input buffer character by character */
  while ((key = SEGGER_RTT_GetKey()) >= 0)
  {
    /* New-line ? */
    if (key == '\n')
    {
      /* Append zero-delimiter */
      rtt_cmd[len] = '\0';

      /* If previous character was carrier-return, then remove that */
      if (rtt_cmd[len - 1] == '\r')
      {
        rtt_cmd[len - 1] = '\0';
      }

      /* Process command */
      if (strcmp(rtt_cmd, "ping") == 0)
      {
        SEGGER_RTT_TerminalOut(0, "pong\n");
      } else
      {
        SEGGER_RTT_TerminalOut(0, "error: unknown command\n");
      }

      /* Clear command */
      (void) memset(rtt_cmd, 0, sizeof(rtt_cmd));
      len = 0;
    }
    /* Room for command characters ? */
    else if (len < (int32_t) sizeof(rtt_cmd) - 1)
    {
      rtt_cmd[len++] = (char) key;
    }
  }
}

bool get_initial_switches_set()
{
  return initial_switches_set;
}

static void main_thread_entry()
{
  // TODO: Add error checking from previous execution.
  // TODO: Add watchdog
  // TODO: Add better error-handling
  nvm_init();
  log_flash_init();
  config_init();
  ui::initialize_ui(500);
  leds::initialize_leds();
  comms::initialize_comms();
  board::initialize_board_data();
  relays::initialize_relays(85.0f);
  adc::initialize_adc();

  // relays::configure_relay(relays::RELAY_1, relays::TURN_OFF, 1.8f);

  // TODO: Control status LEDs based on system state:
  leds::control_led(leds::LED_STATUS_1, leds::RED, leds::BLINK, 100, 900);
  leds::control_led(leds::LED_RELAY_1, leds::OFF);

  uint32_t wake_time = tx_time_get();
  const uint32_t update_interval_ms = 100;

  watchdog_configure_thread(WATCHDOG_THREAD_MAIN, update_interval_ms, update_interval_ms * 4);

  while (true)
  {
    handle_ui();
    handle_link_status();
    handle_board();
    handle_relays();
    handle_rtt_commands();
    sleep_until_ms(&wake_time, update_interval_ms);
    watchdog_kick_thread(WATCHDOG_THREAD_MAIN);
  }
}

extern "C"
{
VOID Fuseboard_Main_Thread_Entry(ULONG)
{
  main_thread_entry();
}
}

