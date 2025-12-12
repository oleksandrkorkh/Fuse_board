#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <tx_api.h>


#ifdef __cplusplus
extern "C"
{
#endif

namespace relays
{

/* Public types --------------------------------------------------------------*/
enum Relay
{
  RELAY_1     = 0,
  RELAY_2     = 1,
  RELAY_3     = 2,
  RELAY_4     = 3,
  RELAY_5     = 4,
  RELAY_6     = 5,
  RELAY_7     = 6,
  RELAY_8     = 7,
  RELAY_COUNT = 8,
};

enum NormalState
{
  NORM_OPEN         = 0,
  NORM_CLOSED       = 1,
  NORM_STATE_COUNT  = 2
};

enum Request
{
  REQ_OFF   = 0,
  REQ_ON    = 1,
  REQ_RESET = 2
};

enum RequestSource
{
  REQSRC_BUTTON        = 0,
  REQSRC_MODBUS        = 1,
  REQSRC_STARTUP_STATE = 2
};

enum State
{
  RELAY_INIT            = 0,
  RELAY_STARTUP_DELAY   = 1,
  RELAY_OFF             = 2,
  RELAY_TURNING_ON      = 3,
  RELAY_ON              = 4,
  RELAY_TURNING_OFF     = 5,
  RELAY_RESET           = 6,
  RELAY_UNDERCURRENT    = 7,  // ON  & Too much current flows into the board
  RELAY_OVERCURRENT     = 8,  // ON  & Too much current flows out of the board
  RELAY_EXCESS_VOLTAGE  = 9,  // OFF & Relay is off, but there's voltage on output
  RELAY_ABSENT_VOLTAGE  = 10, // ON  & Relay is on, but there's no output voltage
  RELAY_PROTECTING      = 11,
  RELAY_RECOVERY_END    = 12,
  RELAY_STATES_COUNT    = 13
};

enum StateReason
{
  REASON_STARTING_UP      = 0,
  REASON_START_UP_DEFAULT = 1,
  REASON_CONTROL_REQUEST  = 2, // Control from Modbus
  REASON_BUTTON_PRESS     = 3,
  REASON_RESET_REQUEST    = 4, // Control from Modbus
  REASON_CONFIG_CHANGE    = 5,
  REASON_PROTECTION       = 6,
  REASON_RECOVERY         = 7,
  REASON_RECOVERY_END     = 8,
  REASON_COUNT            = 9,
  KEEP_REASON = REASON_COUNT
};

enum FaultFlags
{
  FORWARD_CURRENT_EXCEEDED    = 0b0001,
  REVERSE_CURRENT_EXCEEDED    = 0b0010,
  EXCESSIVE_VOLTAGE_DETECTED  = 0b0100,
  ABSENCE_OF_VOLTAGE_DETECTED = 0b1000,
};

struct RelayConfig
{
  NormalState normal_state;
  bool        startup_state;
  uint16_t    startup_delay_ms;

  float neg_current_limit_a; // Negative current (flowing into the board) limit in amperes
  float pos_current_limit_a; // Positive current (flowing out of the board) limit in amperes

  uint16_t activation_filter_time_ms;
  uint16_t operation_filter_time_ms;
  uint8_t  recovery_mode;
  uint16_t recovery_off_time_ms;
  uint16_t reset_off_time_ms;
};

struct RelayStatus
{
  // Input data
  Request request;
  RequestSource request_source;
  uint8_t fault_flags_clear_request;

  // State variables
  State state;
  uint64_t timer_ms;
  StateReason state_reason;

  // Fault variables
  uint8_t fault_flags;
  uint32_t recovery_attempts;

  // Measurement data
  float voltage_v;           // Actual voltage (V)
  float current_a;           // Actual current (A)
  float power_w;             // Actual power (W)
  float energy_j;            // Energy consumed since start (J)
  float tripping_current_a;  // Current (A) value at which the limit was exceeded.
};

/* Constants -----------------------------------------------------------------*/
extern const Relay RELAYS[RELAY_COUNT];

/* Function declarations -----------------------------------------------------*/
void initialize_relays(float total_current_limit);
void update_relay_config(Relay num);
void control_relay(Relay num, Request request, RequestSource request_source);
Request get_control_state(Relay num);
RelayStatus get_relay_status(Relay num);
void reset_fault_flags(Relay num, uint8_t mask);
}

#ifdef __cplusplus
}
#endif
