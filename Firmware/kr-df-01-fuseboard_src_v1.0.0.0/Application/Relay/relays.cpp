#include "relays.h"
#include "common.h"
#include "adc.h"
#include "tx_extension.h"
#include "ll_relays.h"
#include "dp_info.h"
#include "config.h"
#include "util.h"
#include "watchdog.h"

namespace relays
{
_Static_assert(DP_RELAY_COUNT == RELAY_COUNT, "Relay counts do not match");

/* Constants -----------------------------------------------------------------*/
const Relay RELAYS[RELAY_COUNT] =
{
  RELAY_1,
  RELAY_2,
  RELAY_3,
  RELAY_4,
  RELAY_5,
  RELAY_6,
  RELAY_7,
  RELAY_8
};

const char* RELAY_STATE_NAMES[RELAY_STATES_COUNT] =
{
  "INIT",         /* RELAY_INIT           */
  "STARTUP",      /* RELAY_STARTUP_DELAY  */
  "OFF",          /* RELAY_OFF            */
  "TURN_ON",      /* RELAY_TURNING_ON     */
  "ON",           /* RELAY_ON             */
  "TURN_OFF",     /* RELAY_TURNING_OFF    */
  "RESET",        /* RELAY_RESET          */
  "ON_UNDER_CUR", /* RELAY_UNDERCURRENT   */
  "ON_OVER_CUR",  /* RELAY_OVERCURRENT    */
  "OFF_EXC_VOLT", /* RELAY_EXCESS_VOLTAGE */
  "ON_ABS_VOLT",  /* RELAY_ABSENT_VOLTAGE */
  "PROTECT",      /* RELAY_PROTECTING     */
  "RECOVER_END",  /* RELAY_RECOVERY_END   */
};

#define ABSENT_VOLTAGE_LIMIT 5.0f // In volts

/* Private variables ---------------------------------------------------------*/
static TX_MUTEX mutex;
static RelayConfig configs[RELAY_COUNT];
static RelayStatus statuses[RELAY_COUNT];
static float total_abs_current_limit_a = 0.0f; // Total absolute current limit through fuse board

/* Private functions ---------------------------------------------------------*/

/**
 * Initialize relays
 * @param total_current_limit: Maximum allowed output current limit
 */
void initialize_relays(float total_current_limit)
{
  ASSERT_FATAL(!isnan(total_current_limit), "Invalid total current limit");

  // Create mutex for thread-safe data exchange
  uint32_t result = tx_mutex_create(&mutex, (char *)"Relay data mutex", TX_INHERIT);
  ASSERT_FATAL(result == TX_SUCCESS, "Mutex create error %u", result);

  // Set all ports total current limit
  total_abs_current_limit_a = total_current_limit;

  // Reset relay statuses
  (void)memset(statuses, 0, sizeof(statuses));

  // Get relay configurations
  for (Relay rel_n : RELAYS)
  {
    update_relay_config(rel_n);
  }
}

/**
 * Control relay coil
 * @param rel_n: Relay index
 * @param config: Relay configuration
 * @param active: true to make contact, false to break contact
 * @return true if there was actual change
 */
static bool ctrl_relay(Relay rel_n, const RelayConfig* config, bool active)
{
  bool prev_state = ll_relays_get_relay(rel_n);

  if (config->normal_state == NORM_OPEN)
  {
    ll_relays_set_relay(rel_n, active);
    return prev_state != active;
  }
  else if (config->normal_state == NORM_CLOSED)
  {
    ll_relays_set_relay(rel_n, !active);
    return prev_state != !active;
  }
  else
  {
    ULOG_CRITICAL("Invalid relay %u normal state: %u", rel_n, config->normal_state);
    return false;
  }
}

/**
 * Get relay state reason from request source.
 * @param status: Relay status
 * @return State reason
 */
static StateReason state_reason_from_request_source(RelayStatus* status)
{
  if (status->request_source == REQSRC_BUTTON)
  {
    return REASON_BUTTON_PRESS;
  }
  else if (status->request_source == REQSRC_MODBUS)
  {
    return REASON_CONTROL_REQUEST;
  }
  else if (status->request_source == REQSRC_STARTUP_STATE)
  {
    return REASON_START_UP_DEFAULT;
  }
  else
  {
    ULOG_ERROR("Unknown request source: %u", status->request_source);
    return REASON_CONTROL_REQUEST; // Need to use something....
  }
}

/**
 * Helper function to set new state.
 * @param rel_n: Relay index
 * @param status: Relay status
 * @param state: New state
 * @param timer_ms: New timer value
 * @param reason: New state reason
 */
static void set_state(Relay rel_n, RelayStatus* status, State state, uint64_t timer_ms, StateReason reason)
{
  status->state = state;
  status->timer_ms = timer_ms;

  if (reason != KEEP_REASON)
  {
    status->state_reason = reason;
  }

  ULOG_DEBUG("Relay #%u state = %s, timer = %u ms", rel_n, RELAY_STATE_NAMES[state], (uint32_t)timer_ms);
}

/**
 * Sub-state machine to handle faults check during OFF state.
 * @param rel_n: Relay index
 * @param config: Relay configuration
 * @param status: Relay status
 * @param time_interval_ms: Interval time in milliseconds
 */
static void relay_substate_off(Relay rel_n, const RelayConfig* config, RelayStatus* status, uint32_t time_interval_ms)
{
  UNUSED(rel_n);

  // Excessive voltage ?
  if (status->voltage_v > ABSENT_VOLTAGE_LIMIT)
  {
    if (status->state != RELAY_EXCESS_VOLTAGE)
    {
      set_state(rel_n, status, RELAY_EXCESS_VOLTAGE, config->operation_filter_time_ms, KEEP_REASON);
    }
    else if (util_countdown(&status->timer_ms, time_interval_ms))
    {
      // Keep relay OFF but register fault
      status->fault_flags |= EXCESSIVE_VOLTAGE_DETECTED;
    }
  }
  // Normal off conditions
  else
  {
    if (status->state != RELAY_OFF)
    {
      set_state(rel_n, status, RELAY_OFF, 0, KEEP_REASON);
    }
  }
}

/**
 * Sub-state machine to handle faults check during ON state.
 * @param rel_n: Relay index
 * @param config: Relay configuration
 * @param status: Relay status
 * @param time_interval_ms: Interval time in milliseconds
 */
static void relay_substate_on(Relay rel_n, const RelayConfig* config, RelayStatus* status, uint32_t time_interval_ms)
{
  // Negative current exceeded ?
  if (status->current_a < config->neg_current_limit_a)
  {
    if (status->state != RELAY_UNDERCURRENT)
    {
      set_state(rel_n, status, RELAY_UNDERCURRENT, config->operation_filter_time_ms, KEEP_REASON);
    }
    else if (util_countdown(&status->timer_ms, time_interval_ms))
    {
      // Turn off
      (void)ctrl_relay(rel_n, config, false);
      set_state(rel_n, status, RELAY_PROTECTING, config->recovery_off_time_ms, REASON_PROTECTION);

      // Remember tripping current and fault
      status->tripping_current_a = status->current_a;
      status->fault_flags |= REVERSE_CURRENT_EXCEEDED;
    }
  }
  // Positive current exceeded ?
  else if (status->current_a > config->pos_current_limit_a)
  {
    if (status->state != RELAY_OVERCURRENT)
    {
      set_state(rel_n, status, RELAY_OVERCURRENT, config->operation_filter_time_ms, KEEP_REASON);
    }
    else if (util_countdown(&status->timer_ms, time_interval_ms))
    {
      // Turn off
      (void)ctrl_relay(rel_n, config, false);
      set_state(rel_n, status, RELAY_PROTECTING, config->recovery_off_time_ms, REASON_PROTECTION);

      // Remember tripping current and fault
      status->tripping_current_a = status->current_a;
      status->fault_flags |= FORWARD_CURRENT_EXCEEDED;
    }
  }
  // Absence of voltage ?
  else if (status->voltage_v < ABSENT_VOLTAGE_LIMIT)
  {
    if (status->state != RELAY_ABSENT_VOLTAGE)
    {
      set_state(rel_n, status, RELAY_ABSENT_VOLTAGE, config->operation_filter_time_ms, KEEP_REASON);
    }
    else if (util_countdown(&status->timer_ms, time_interval_ms))
    {
      // Keep relay ON but register fault
      status->fault_flags |= ABSENCE_OF_VOLTAGE_DETECTED;
    }
  }
  // Normal current and voltage
  else
  {
    if (status->state != RELAY_ON)
    {
      set_state(rel_n, status, RELAY_ON, 0, KEEP_REASON);
    }
  }
}

/**
 * Relay state machine.
 * @param rel_n: Relay index
 * @param config: Relay configuration
 * @param status: Relay status
 * @param time_interval_ms: Interval time in milliseconds
 */
static void relay_statemachine(Relay rel_n, const RelayConfig* config, RelayStatus* status, uint32_t time_interval_ms)
{
  switch (status->state)
  {
    case RELAY_INIT:
      set_state(rel_n, status, RELAY_STARTUP_DELAY, config->startup_delay_ms, REASON_STARTING_UP);
      break;

    case RELAY_STARTUP_DELAY:
      if (util_countdown(&status->timer_ms, time_interval_ms))
      {
        if (config->startup_state)
        {
          control_relay(rel_n, REQ_ON, REQSRC_STARTUP_STATE); // have to set request as otherwise it will be turned off again by default request
          (void)ctrl_relay(rel_n, config, true);
          set_state(rel_n, status, RELAY_TURNING_ON, config->activation_filter_time_ms, REASON_START_UP_DEFAULT);
        }
        else
        {
          control_relay(rel_n, REQ_OFF, REQSRC_STARTUP_STATE);
          (void)ctrl_relay(rel_n, config, false);
          set_state(rel_n, status, RELAY_TURNING_OFF, config->activation_filter_time_ms, REASON_START_UP_DEFAULT);
        }
      }
      break;

    case RELAY_OFF:
    case RELAY_EXCESS_VOLTAGE:
      // Check request
      if (status->request == REQ_ON)
      {
        // Turn on
        if (ctrl_relay(rel_n, config, true))
        {
          set_state(rel_n, status, RELAY_TURNING_ON, config->activation_filter_time_ms, state_reason_from_request_source(status));
          break;
        }
      }
      else if (status->request == REQ_OFF)
      {
        // Turn off again even because normal state config may have been changed.
        if (ctrl_relay(rel_n, config, false))
        {
          set_state(rel_n, status, RELAY_TURNING_OFF, config->activation_filter_time_ms, REASON_CONFIG_CHANGE);
          break;
        }
      }
      else if (status->request == REQ_RESET)
      {
        // Should be already off, but do it again and make a reset delay before turning on
        (void)ctrl_relay(rel_n, config, false);
        set_state(rel_n, status, RELAY_RESET, config->reset_off_time_ms, REASON_RESET_REQUEST);
        break;
      }

      // Check for faults
      relay_substate_off(rel_n, config, status, time_interval_ms);
      break;

    case RELAY_TURNING_ON:
      // During transition time don't check for errors
      if (util_countdown(&status->timer_ms, time_interval_ms))
      {
        set_state(rel_n, status, RELAY_ON, 0, KEEP_REASON);
      }
      break;

    case RELAY_ON:
    case RELAY_UNDERCURRENT:
    case RELAY_OVERCURRENT:
    case RELAY_ABSENT_VOLTAGE:
      // Request checks
      if (status->request == REQ_ON)
      {
        // Turn on again even if off because normal state may change.
        if (ctrl_relay(rel_n, config, true))
        {
          set_state(rel_n, status, RELAY_TURNING_ON, config->activation_filter_time_ms, REASON_CONFIG_CHANGE);
          break;
        }
      }
      else if (status->request == REQ_OFF)
      {
        // Turn off
        if (ctrl_relay(rel_n, config, false))
        {
          set_state(rel_n, status, RELAY_TURNING_OFF, config->activation_filter_time_ms, state_reason_from_request_source(status));
          break;
        }
      }
      else if (status->request == REQ_RESET)
      {
        // Set output off and on again after a reset time
        (void)ctrl_relay(rel_n, config, false);
        set_state(rel_n, status, RELAY_RESET, config->reset_off_time_ms, REASON_RESET_REQUEST);
        break;
      }

      // Check for faults
      relay_substate_on(rel_n, config, status, time_interval_ms);
      break;

    case RELAY_TURNING_OFF:
      // During transition time don't check for errors
      if (util_countdown(&status->timer_ms, time_interval_ms))
      {
        set_state(rel_n, status, RELAY_OFF, 0, KEEP_REASON);
      }
      break;

    case RELAY_RESET:
      // Wait until reset off time has elapsed
      if (util_countdown(&status->timer_ms, time_interval_ms))
      {
        // After off-time, turn output on again
        (void)ctrl_relay(rel_n, config, true);
        set_state(rel_n, status, RELAY_TURNING_ON, config->activation_filter_time_ms, REASON_RESET_REQUEST);

        // Set relay request ON because reset ends with ON state anyway
        // and we can't keep resetting (if it stays at REQ_RESET).
        // It needs to consider potential simultaneous external control request,
        // so do it with mutex.
        control_relay(rel_n, REQ_ON, REQSRC_MODBUS);
      }
      break;

    case RELAY_PROTECTING:
      // Wait until recovery cool-down time has elapsed
      if (util_countdown(&status->timer_ms, time_interval_ms))
      {
        // Timer expired; attempt a recovery
        // In the case of running out of recovery attempts, relay is kept off
        if (status->recovery_attempts < config->recovery_mode)
        {
          status->recovery_attempts++;
          set_state(rel_n, status, RELAY_TURNING_ON, config->activation_filter_time_ms, REASON_RECOVERY);
        }
        else
        {
          // Recovery sequence has ended
          set_state(rel_n, status, RELAY_RECOVERY_END, 0, REASON_RECOVERY_END);
        }
      }
      break;

    case RELAY_RECOVERY_END:
      // Only way out of recovery end mode is to turn off and on again or make a reset
      if ((status->request == REQ_OFF) || (status->request == REQ_RESET))
      {
        status->recovery_attempts = 0;
        set_state(rel_n, status, RELAY_OFF, 0, state_reason_from_request_source(status));
      }
      break;

    default:
      ULOG_CRITICAL("Unhandled relay %u state: %u", rel_n, status->state);
      break;
  }
}

/**
 * Handle single relay.
 * @param rel_n: Relay index
 * @param time_interval_ms: Time since last call of this function on this relay
 */
static void handle_relay(Relay rel_n, uint32_t time_interval_ms)
{
  ASSERT_FATAL(rel_n < RELAY_COUNT, "Invalid relay num");

  RelayStatus status;
  RelayConfig config;

  update_relay_config(rel_n);

  /* Get a copy of relay status and configuration to work with.
   * This ensures that any change to input variables during execution of
   * this function does not create undefined behavior.
   *
   * Undefined behavior is when at one line configuration parameter or input
   * variable has one value and at the next line where same value is used,
   * it's different. */
  TX_MUTEX_SECTION(&mutex)
  {
    status = statuses[rel_n];
    config = configs[rel_n];
  }

  // Measure output port physical parameters
  float adc_v;
  float adc_i;

  adc::get_adc_relay_result_values(rel_n, &adc_v, &adc_i);
  status.voltage_v = adc_v;
  status.current_a = adc_i;
  status.power_w   = adc_v * adc_i;

  // Accumulate total transmitted energy
  status.energy_j += fabs(status.power_w) * ((float)time_interval_ms / 1000.0f);

  // Handle faults clearing request
  if (status.fault_flags_clear_request != 0)
  {
    status.fault_flags &= ~status.fault_flags_clear_request;
    status.fault_flags_clear_request = 0;
  }

  // Run relay state machine
  relay_statemachine(rel_n, &config, &status, time_interval_ms);

  // Update relay status at once
  TX_MUTEX_SECTION(&mutex)
  {
    // DO NOT OVERWRITE INPUT DATA VARIABLES HERE
    statuses[rel_n].state              = status.state;
    statuses[rel_n].timer_ms           = status.timer_ms;
    statuses[rel_n].state_reason       = status.state_reason;
    statuses[rel_n].fault_flags        = status.fault_flags;
    statuses[rel_n].recovery_attempts  = status.recovery_attempts;
    statuses[rel_n].voltage_v          = status.voltage_v;          // Actual voltage (V)
    statuses[rel_n].current_a          = status.current_a;          // Actual current (A)
    statuses[rel_n].power_w            = status.power_w;            // Actual power (W)
    statuses[rel_n].energy_j           = status.energy_j;           // Energy consumed since start (J)
    statuses[rel_n].tripping_current_a = status.tripping_current_a; // Tripping current (A);
  }
}

/**
 * Relays thread entry point
 * Never returns!
 */
static void relays_thread_entry()
{
  float total_current_a = 0.0f;
  uint32_t wake_time = tx_time_get();
  const uint32_t update_interval_ms = 100;

  watchdog_configure_thread(WATCHDOG_THREAD_RELAYS, update_interval_ms, update_interval_ms * 4);

  while (true)
  {
    total_current_a = 0.0f;
    for (Relay rel_n : RELAYS)
    {
      handle_relay(rel_n, update_interval_ms);
      total_current_a += fabs(statuses[rel_n].current_a);
    }

    // Check for total current limit
    if (total_current_a > total_abs_current_limit_a)
    {
      ULOG_ERROR("Total current exceeds system limit: %.3f > %.3f!", total_current_a, total_abs_current_limit_a);
      // TODO: Report error over modbus; what else can we do?
    }

    sleep_until_ms(&wake_time, update_interval_ms);
    watchdog_kick_thread(WATCHDOG_THREAD_RELAYS);
  }
}

/**
 * Update relay configuration.
 * @param num: Relay number
 */
void update_relay_config(Relay num)
{
  ASSERT_FATAL(num < RELAY_COUNT, "Invalid relay num");

  // Get current port configuration
  CONFIG_PORT cfg;
  config_get_port_data(&cfg, num);

  // Set the relay buffered configuration in thread-safe way
  TX_MUTEX_SECTION(&mutex)
  {
    configs[num].normal_state              = static_cast<NormalState>(cfg.normal_state);
    configs[num].startup_state             = static_cast<bool>(cfg.startup_state);
    configs[num].startup_delay_ms          = cfg.startup_delay;
    configs[num].neg_current_limit_a       = ((float)cfg.reverse_current_limit / -1000.0f); // -1000 making it Amps and negative
    configs[num].pos_current_limit_a       = ((float)cfg.forward_current_limit /  1000.0f);
    configs[num].activation_filter_time_ms = cfg.activation_filter_time;
    configs[num].operation_filter_time_ms  = cfg.operation_filter_time;
    configs[num].recovery_mode             = cfg.recovery_mode;
    configs[num].recovery_off_time_ms      = cfg.recovery_off_time;
    configs[num].reset_off_time_ms         = cfg.reset_off_time;
  }
}

/**
 * Control relay state. Takes effect in relay thread.
 * @param num: Relay number
 * @param request: Request type
 * @param request_source: Request source
 */
void control_relay(Relay num, Request request, RequestSource request_source)
{
  ASSERT_FATAL(num < RELAY_COUNT, "Invalid relay num");
  ASSERT_FATAL((request == REQ_OFF) || (request == REQ_ON) || (request == REQ_RESET), "Invalid request");
  ASSERT_FATAL((request_source == REQSRC_BUTTON) || (request_source == REQSRC_MODBUS) || (request_source == REQSRC_STARTUP_STATE), "Invalid request");

  TX_MUTEX_SECTION(&mutex)
  {
    statuses[num].request = request;
    statuses[num].request_source = request_source;
  }
}

/**
 * Get relay control state (does not mean relay has this state).
 * @param num: Relay number
 * @return Control state
 */
Request get_control_state(Relay num)
{
  ASSERT_FATAL(num < RELAY_COUNT, "Invalid relay num");

  // Atomic access
  return statuses[num].request;
}

/**
 * Get relay status.
 * @param num: Relay number
 * @return Copy of relay status structure.
 */
RelayStatus get_relay_status(Relay num)
{
  ASSERT_FATAL(num < RELAY_COUNT, "Invalid relay num");

  // Take a copy within critical section
  RelayStatus status;
  TX_MUTEX_SECTION(&mutex)
  {
    status = statuses[num];
  }

  return status;
}

/**
 * Reset fault flags with a mask
 * @param num: Relay number
 * @param mask: Fault flags mask (bits that are set will be cleared)
 */
void reset_fault_flags(Relay num, uint8_t mask)
{
  ASSERT_FATAL(num < RELAY_COUNT, "Invalid relay num");

  TX_MUTEX_SECTION(&mutex)
  {
    /* Can't clear immediately because relay logic works using buffers
     * and it could be that clearing won't have affect. So use request. */
    statuses[num].fault_flags_clear_request = mask;
  }
}


// End of the namespace
}

extern "C"
{
VOID Fuseboard_Relays_Thread_Entry(ULONG)
{
  relays::relays_thread_entry();
}
}

