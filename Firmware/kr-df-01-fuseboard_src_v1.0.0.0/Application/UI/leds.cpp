#include <stdbool.h>
#include "leds.h"
#include "common.h"
#include "watchdog.h"
#include "log.h"

namespace leds
{

struct LEDState
{
  uint8_t  num;

  /* Normal state */
  Color    color            = Color { 0, 0, 0 };
  Mode     mode             = Mode::CONSTANT;
  uint32_t blink_on_ms      = 0;
  uint32_t blink_off_ms     = 0;
  int32_t  next_toggle_ms   = 0;
  bool     on               = false;

  /* Temporary/test state */
  Color    temp_color       = Color { 0, 0, 0 };
  int32_t  temp_color_time  = 0;
  bool     temp_color_state = false;
};

/* Private variables ---------------------------------------------------------*/
static TX_MUTEX mutex;
static struct LEDState leds[LED_COUNT] =
{
    LEDState { LED_STATUS_1 },
    LEDState { LED_STATUS_2 },
    LEDState { LED_RELAY_1  },
    LEDState { LED_RELAY_2  },
    LEDState { LED_RELAY_3  },
    LEDState { LED_RELAY_4  },
    LEDState { LED_RELAY_5  },
    LEDState { LED_RELAY_6  },
    LEDState { LED_RELAY_7  },
    LEDState { LED_RELAY_8  }
};

void update_led(LEDState &led, uint32_t elapsed_ms)
{
  /* Temporary control active ? */
  if (led.temp_color_state)
  {
    ll_leds_set_led(led.num, led.temp_color.r, led.temp_color.g, led.temp_color.b);

    /* Run the countdown timer */
    led.temp_color_time -= elapsed_ms;
    if (led.temp_color_time <= 0)
    {
      led.temp_color_state = false;
    }
  }
  /* Otherwise it's normal control */
  else
  {
    if (led.mode == Mode::CONSTANT)
    {
      ll_leds_set_led(led.num, led.color.r, led.color.g, led.color.b);
    }
    else if (led.mode == Mode::BLINK)
    {
      led.next_toggle_ms -= elapsed_ms;
      if (led.next_toggle_ms <= 0)
      {
        led.on = !led.on;
        led.next_toggle_ms = led.on ? led.blink_on_ms : led.blink_off_ms;
      }
      if (led.on)
      {
        ll_leds_set_led(led.num, led.color.r, led.color.g, led.color.b);
      }
      else
      {
        ll_leds_set_led(led.num, 0, 0, 0);
      }
    }
    else
    {
      ULOG_CRITICAL("Invalid LED %u mode: %u", led.num, led.mode);
    }
  }
}

static void leds_thread_entry()
{
  ll_leds_init();

  uint32_t wake_time = tx_time_get();
  const uint32_t update_interval_ms = 100;

  watchdog_configure_thread(WATCHDOG_THREAD_LEDS, update_interval_ms, update_interval_ms * 4);

  while (true)
  {
    for (LEDState& led : leds)
    {
      TX_MUTEX_SECTION(&mutex)
      {
        update_led(led, update_interval_ms);
      }
    }

    ll_leds_flush();
    sleep_until_ms(&wake_time, update_interval_ms);
    watchdog_kick_thread(WATCHDOG_THREAD_LEDS);
  }
}

void initialize_leds()
{
  /* Create mutex for thread-safe data exchange */
  uint32_t result = tx_mutex_create(&mutex, (char *)"LED control mutex", TX_INHERIT);
  ASSERT_FATAL(result == TX_SUCCESS, "Mutex create error %u", result);
}


/**
 * @brief Convert LED color from RGB565 encoding into RGB888.
 * @note It's not the most precise method, but it's simple
 * @param rgb565: 16-bit color.
 * @return Color struct
 */
Color rgb565_to_color(uint16_t rgb565)
{
  leds::Color color =
  {
    /*                          RRRRRGGGGGGBBBBB            */
    .r = (uint8_t)(((rgb565 & 0b1111100000000000) >> 11) * 8),
    .g = (uint8_t)(((rgb565 & 0b0000011111100000) >>  5) * 4),
    .b = (uint8_t)(((rgb565 & 0b0000000000011111))       * 8),
  };

  return color;
}

bool temporary_led_color_set(LED num, Color color, uint16_t time_ms)
{
  ASSERT_FATAL(num < LED_COUNT, "Invalid LED num");

  /* Limit temporary time to 30 seconds */
  if (time_ms >= 30000)
  {
    ULOG_ERROR("LED temporary control time too long: %u", time_ms);
    return false;
  }

  TX_MUTEX_SECTION(&mutex)
  {
    LEDState& led = leds[num];

    led.temp_color_state = true;
    led.temp_color       = color;
    led.temp_color_time  = time_ms;
  }

  return true;
}

void control_led(LED num, Color color, Mode mode, uint32_t blink_on_ms, uint32_t blink_off_ms)
{
  ASSERT_FATAL(num < LED_COUNT, "Invalid LED num");

  TX_MUTEX_SECTION(&mutex)
  {
    LEDState& led = leds[(uint8_t)num];

    led.color        = color;
    led.mode         = mode;
    led.blink_on_ms  = blink_on_ms;
    led.blink_off_ms = blink_off_ms;
  }
}

}

/* Ugly C API ---------------------------------------------------------------*/

extern "C"
{
VOID Fuseboard_LEDS_Thread_Entry(ULONG)
{
  leds::leds_thread_entry();
}
}

