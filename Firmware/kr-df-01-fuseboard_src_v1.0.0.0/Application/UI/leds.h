#pragma once

#include <ll_leds.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define GET_LED_FOR_RELAY(relay_num) (leds::LED)((uint8_t)relay_num + (uint8_t)leds::LED::LED_RELAY_1)

namespace leds
{

enum LED
{
  LED_STATUS_1 = 0,
  LED_STATUS_2 = 1,
  LED_RELAY_1  = 2,
  LED_RELAY_2  = 3,
  LED_RELAY_3  = 4,
  LED_RELAY_4  = 5,
  LED_RELAY_5  = 6,
  LED_RELAY_6  = 7,
  LED_RELAY_7  = 8,
  LED_RELAY_8  = 9,
  LED_COUNT    = 10
};

enum Mode
{
  CONSTANT, BLINK,
};

struct Color
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

const Color OFF    = Color {   0,   0,   0 };
const Color RED    = Color { 255,   0,   0 };
const Color GREEN  = Color {   0, 255,   0 };
const Color BLUE   = Color {   0,   0, 255 };
const Color YELLOW = Color { 255, 255,   0 };
const Color ORANGE = Color { 255, 128,   0 };

void initialize_leds();
void control_led(LED num, Color color, Mode mode = CONSTANT, uint32_t blink_on_ms = 1000, uint32_t blink_off_ms = 1000);
Color rgb565_to_color(uint16_t rgb565);
bool temporary_led_color_set(LED num, Color color, uint16_t time_ms);

}

#ifdef __cplusplus
}
#endif
