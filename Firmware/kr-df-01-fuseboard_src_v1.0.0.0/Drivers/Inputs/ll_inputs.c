#include "ll_inputs.h"
#include "main.h"
#include "log.h"

#include <stm32h5xx_hal.h>

uint8_t ll_inputs_read_switch(uint8_t num)
{
  ASSERT_FATAL(num < 8, "Invalid switch num");

  switch (num)
  {
  case 0:
    return HAL_GPIO_ReadPin(SW_1_GPIO_Port, SW_1_Pin);
  case 1:
    return HAL_GPIO_ReadPin(SW_2_GPIO_Port, SW_2_Pin);
  case 2:
    return HAL_GPIO_ReadPin(SW_3_GPIO_Port, SW_3_Pin);
  case 3:
    return HAL_GPIO_ReadPin(SW_4_GPIO_Port, SW_4_Pin);
  case 4:
    return HAL_GPIO_ReadPin(SW_5_GPIO_Port, SW_5_Pin);
  case 5:
    return HAL_GPIO_ReadPin(SW_6_GPIO_Port, SW_6_Pin);
  case 6:
    return HAL_GPIO_ReadPin(SW_7_GPIO_Port, SW_7_Pin);
  case 7:
    return HAL_GPIO_ReadPin(SW_8_GPIO_Port, SW_8_Pin);
  default:
    ULOG_ERROR("Invalid switch: %u", num);
    return 0;
  }
}

uint8_t ll_inputs_read_button(uint8_t num)
{
  ASSERT_FATAL(num < 8, "Invalid input num");

  switch (num)
  {
  case 0:
    return HAL_GPIO_ReadPin(BTN_1_GPIO_Port, BTN_1_Pin);
  case 1:
    return HAL_GPIO_ReadPin(BTN_2_GPIO_Port, BTN_2_Pin);
  case 2:
    return HAL_GPIO_ReadPin(BTN_3_GPIO_Port, BTN_3_Pin);
  case 3:
    return HAL_GPIO_ReadPin(BTN_4_GPIO_Port, BTN_4_Pin);
  case 4:
    return HAL_GPIO_ReadPin(BTN_5_GPIO_Port, BTN_5_Pin);
  case 5:
    return HAL_GPIO_ReadPin(BTN_6_GPIO_Port, BTN_6_Pin);
  case 6:
    return HAL_GPIO_ReadPin(BTN_7_GPIO_Port, BTN_7_Pin);
  case 7:
    return HAL_GPIO_ReadPin(BTN_8_GPIO_Port, BTN_8_Pin);
  default:
    ULOG_ERROR("Invalid button: %u", num);
    return 0;
  }
}

