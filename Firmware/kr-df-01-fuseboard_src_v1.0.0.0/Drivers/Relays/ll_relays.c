#include "main.h"
#include "log.h"
#include "ll_relays.h"

void ll_relays_set_relay(uint8_t num, bool state)
{
  ASSERT_FATAL(num < 8, "Invalid relay num");
  ASSERT_FATAL((state == false) || (state == true), "Invalid relay state");

  switch (num)
  {
    case 0:
      HAL_GPIO_WritePin(OUT_1_EN_GPIO_Port, OUT_1_EN_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
      break;
    case 1:
      HAL_GPIO_WritePin(OUT_2_EN_GPIO_Port, OUT_2_EN_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
      break;
    case 2:
      HAL_GPIO_WritePin(OUT_3_EN_GPIO_Port, OUT_3_EN_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
      break;
    case 3:
      HAL_GPIO_WritePin(OUT_4_EN_GPIO_Port, OUT_4_EN_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
      break;
    case 4:
      HAL_GPIO_WritePin(OUT_5_EN_GPIO_Port, OUT_5_EN_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
      break;
    case 5:
      HAL_GPIO_WritePin(OUT_6_EN_GPIO_Port, OUT_6_EN_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
      break;
    case 6:
      HAL_GPIO_WritePin(OUT_7_EN_GPIO_Port, OUT_7_EN_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
      break;
    case 7:
      HAL_GPIO_WritePin(OUT_8_EN_GPIO_Port, OUT_8_EN_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
      break;
    default:
      ULOG_CRITICAL("Invalid relay: %u", num);
      break;
  }
}

bool ll_relays_get_relay(uint8_t num)
{
  ASSERT_FATAL(num < 8, "Invalid relay num");

  switch (num)
  {
    case 0:
      return HAL_GPIO_ReadPin(OUT_1_EN_GPIO_Port, OUT_1_EN_Pin) != GPIO_PIN_RESET;
    case 1:
      return HAL_GPIO_ReadPin(OUT_2_EN_GPIO_Port, OUT_2_EN_Pin) != GPIO_PIN_RESET;
    case 2:
      return HAL_GPIO_ReadPin(OUT_3_EN_GPIO_Port, OUT_3_EN_Pin) != GPIO_PIN_RESET;
    case 3:
      return HAL_GPIO_ReadPin(OUT_4_EN_GPIO_Port, OUT_4_EN_Pin) != GPIO_PIN_RESET;
    case 4:
      return HAL_GPIO_ReadPin(OUT_5_EN_GPIO_Port, OUT_5_EN_Pin) != GPIO_PIN_RESET;
    case 5:
      return HAL_GPIO_ReadPin(OUT_6_EN_GPIO_Port, OUT_6_EN_Pin) != GPIO_PIN_RESET;
    case 6:
      return HAL_GPIO_ReadPin(OUT_7_EN_GPIO_Port, OUT_7_EN_Pin) != GPIO_PIN_RESET;
    case 7:
      return HAL_GPIO_ReadPin(OUT_8_EN_GPIO_Port, OUT_8_EN_Pin) != GPIO_PIN_RESET;
    default:
      ULOG_CRITICAL("Invalid relay: %u", num);
      break;
  }

  return false;
}
