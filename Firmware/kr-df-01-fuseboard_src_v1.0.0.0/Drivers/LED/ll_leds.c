#include <ll_leds.h>
#include "main.h"

#include <memory.h>
#include "log.h"

/* Variables -----------------------------------------------*/

extern TIM_HandleTypeDef LEDS_TIM;
static uint8_t led_data[MAX_LED][4];
#define PWM_COUNT ((24 * MAX_LED) + LEDS_RST_PERIOD_N)
static uint8_t pwm_data[PWM_COUNT];

/* Functions -----------------------------------------------*/

void ll_leds_init(void)
{
  memset(led_data, 0, sizeof(led_data));
  memset(pwm_data, 0, sizeof(pwm_data));
  ll_leds_flush();
  HAL_TIM_PWM_Start_DMA(&LEDS_TIM, LEDS_TIM_CHANNEL, (uint32_t*)pwm_data, PWM_COUNT);
}

void ll_leds_set_led(uint8_t n, uint8_t r, uint8_t g, uint8_t b)
{
  ASSERT_FATAL(n < MAX_LED, "Invalid LED num");

  led_data[n][0] = n;
  led_data[n][1] = g;
  led_data[n][2] = r;
  led_data[n][3] = b;
}

void ll_leds_flush(void)
{
  uint32_t indx = LEDS_RST_PERIOD_N;
  uint32_t color;

  for (int i = 0; i < MAX_LED; i++)
  {
    color = ((led_data[i][1] << 16) | (led_data[i][2] << 8) | (led_data[i][3]));

    for (int i = 23; i >= 0; i--)
    {
      if (color & (1 << i))
      {
        pwm_data[indx] = LEDS_T1H;
      }
      else
      {
        pwm_data[indx] = LEDS_T0H;
      }
      indx++;
    }
  }
}

