#include "buttons.h"
#include "common.h"
#include "watchdog.h"
#include "log.h"

#include <ll_inputs.h>
#include <stdio.h>

namespace ui
{

static uint32_t user_button_min_press_duration_ms = 0;

#define UI_EVENT_QUEUE_MEM_SIZE 128
static UCHAR ui_event_queue_mem[UI_EVENT_QUEUE_MEM_SIZE];
static TX_QUEUE ui_event_queue;
bool initial_switch_statuses_set = false;
uint8_t  current_switch_statuses = 0;

struct ButtonState
{
  uint8_t state;
  int32_t state_update_delay_ms;
};
static ButtonState buttons[BTN_COUNT] = { };

static void update_switches()
{
  uint8_t new_switch_statuses = 0;
  for (Switch sw : SWITCHES)
  {
    const uint8_t status = ll_inputs_read_switch(sw);
    new_switch_statuses |= status << sw;
  }
  // TODO: might need debounce filter if planning to do live updates using switches
  if ((current_switch_statuses != new_switch_statuses) || !initial_switch_statuses_set)
  {
    initial_switch_statuses_set = true;
    current_switch_statuses = new_switch_statuses;
    Event event = Event { SWITCHES_CHANGED, new_switch_statuses };
    if (tx_queue_send(&ui_event_queue, &event, TX_NO_WAIT) != TX_SUCCESS)
    {
      ULOG_ERROR("Could not post UI event");
    }
    ULOG_DEBUG("UI: switches changed: 0x%x", new_switch_statuses);
  }
}

static void update_buttons(uint32_t elapsed_ms)
{
  for (Button btn : BUTTONS)
  {
    const uint8_t new_state = ll_inputs_read_button(btn);
    ButtonState& current = buttons[btn];
    if (current.state == 1 && new_state == 0)
    {
      Event event = Event { BUTTON_RELEASED, btn };
      if (tx_queue_send(&ui_event_queue, &event, TX_NO_WAIT) != TX_SUCCESS)
      {
        ULOG_ERROR("Could not post UI event");
      }
      current.state = 0;
      ULOG_DEBUG("UI: button #%u released", btn + 1);
    }
    else if (current.state == 0 && new_state == 1)
    {
      if (current.state_update_delay_ms > 0)
      {
        current.state_update_delay_ms -= elapsed_ms;
        if (current.state_update_delay_ms <= 0)
        {
          Event event = Event { BUTTON_PRESSED, btn };
          if (tx_queue_send(&ui_event_queue, &event, TX_NO_WAIT) != TX_SUCCESS)
          {
            ULOG_ERROR("Could not post UI event");
          }
          current.state = 1;
          ULOG_DEBUG("UI: button #%u pressed", btn + 1);
        }
      }
    }
    else
    {
      current.state_update_delay_ms = user_button_min_press_duration_ms;
    }
  }
}

uint8_t get_button_state(Button btn)
{
  // checks if the exact value exists
  size_t buttons_size = (sizeof(BUTTONS) / sizeof(BUTTONS[0]));
  for (size_t i = 0; i < (buttons_size); ++i)
  {
    if (BUTTONS[i] == btn)
    {
      break;
    }
    else if ((i + 1) == buttons_size) // last item in array didnt equal button number
    {
      return 2;
    }
  }

  ButtonState current = buttons[btn];
  return current.state;
}

static void ui_thread_entry()
{
  tx_queue_create(&ui_event_queue, (char* )"UI events", sizeof(Event) / 4, ui_event_queue_mem, UI_EVENT_QUEUE_MEM_SIZE);
  uint32_t wake_time = tx_time_get();
  const uint32_t update_interval_ms = 100;

  watchdog_configure_thread(WATCHDOG_THREAD_UI, update_interval_ms, update_interval_ms * 4);

  while (true)
  {
    update_switches();
    update_buttons(update_interval_ms);
    sleep_until_ms(&wake_time, update_interval_ms);
    watchdog_kick_thread(WATCHDOG_THREAD_UI);
  }
}

void initialize_ui(uint32_t button_min_press_duration_ms)
{
  user_button_min_press_duration_ms = button_min_press_duration_ms;
}

std::optional<Event> poll_ui_event()
{
  Event evt;
  if (tx_queue_receive(&ui_event_queue, &evt, TX_NO_WAIT) == TX_SUCCESS)
  {
    return evt;
  }
  return std::nullopt;
}

}

/* Ugly C API ---------------------------------------------------------------*/

extern "C"
{
VOID Fuseboard_UI_Thread_Entry(ULONG)
{
  ui::ui_thread_entry();
}
}

