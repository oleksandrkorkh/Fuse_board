#ifndef __LEDS_H
#define __LEDS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define LEDS_TIM          htim2
#define LEDS_TIM_CHANNEL  TIM_CHANNEL_3
#define LEDS_TIM_PERIOD   (155) // 1.25us period
#define LEDS_T0H          (25)  // 0.2us
#define LEDS_T1H          (123) // 1.0us
#define LEDS_RST_PERIOD_N (80)  // 80 periods for reset (need 80us minimum)
#define MAX_LED 10

// Sets all LEDs off.
void ll_leds_init(void);

// Update local RGB value for given LED; must be followed by LEDS_Flush to write values to HW.
void ll_leds_set_led(uint8_t n, uint8_t r, uint8_t g, uint8_t b);

// Syncs local RGB values with HW.
void ll_leds_flush();

#ifdef __cplusplus
}
#endif

#endif
