#pragma once

#include <optional>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

namespace ui
{

enum Switch
{
  SW1 = 0, SW2, SW3, SW4, SW5, SW6, SW7, SW8, SW_COUNT
};
static const Switch SWITCHES[] = { SW1, SW2, SW3, SW4, SW5, SW6, SW7, SW8 };

enum Button
{
  BTN1 = 0, BTN2, BTN3, BTN4, BTN5, BTN6, BTN7, BTN8, BTN_COUNT
};
static const Button BUTTONS[] = { BTN1, BTN2, BTN3, BTN4, BTN5, BTN6, BTN7, BTN8 };

enum EventType
{
  SWITCHES_CHANGED, /* Event value is a bit-field representing all switch statuses. */
  BUTTON_PRESSED, /* Event value is the pressed button enum. */
  BUTTON_RELEASED, /* Event value is the released button enum. */
};

struct Event
{
  EventType type;
  uint32_t value;
};

void initialize_ui(uint32_t button_min_press_duration_ms);
uint8_t get_button_state(Button btn);

extern uint8_t current_switch_statuses;

std::optional<Event> poll_ui_event();

}

#ifdef __cplusplus
}
#endif
