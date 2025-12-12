/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h5xx_hal.h"

#include "stm32h5xx_ll_usart.h"
#include "stm32h5xx_ll_rcc.h"
#include "stm32h5xx_ll_system.h"
#include "stm32h5xx_ll_gpio.h"
#include "stm32h5xx_ll_exti.h"
#include "stm32h5xx_ll_bus.h"
#include "stm32h5xx_ll_cortex.h"
#include "stm32h5xx_ll_utils.h"
#include "stm32h5xx_ll_pwr.h"
#include "stm32h5xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "log.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/** Fatal assert macro.
 * Keep it on a single line so it's possible to track the caller line */
#define ASSERT_FATAL(condition, ...) if (!(condition)) { ULOG_CRITICAL(__VA_ARGS__); Error_Handler(); }

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RS485_TX_Pin LL_GPIO_PIN_2
#define RS485_TX_GPIO_Port GPIOE
#define OUT_I_EN_Pin LL_GPIO_PIN_3
#define OUT_I_EN_GPIO_Port GPIOE
#define OUT_I_A0_Pin LL_GPIO_PIN_4
#define OUT_I_A0_GPIO_Port GPIOE
#define OUT_I_A1_Pin LL_GPIO_PIN_5
#define OUT_I_A1_GPIO_Port GPIOE
#define OUT_I_A2_Pin LL_GPIO_PIN_6
#define OUT_I_A2_GPIO_Port GPIOE
#define ETH_nRST_Pin LL_GPIO_PIN_0
#define ETH_nRST_GPIO_Port GPIOA
#define ADC2_VOLTAGE_Pin LL_GPIO_PIN_0
#define ADC2_VOLTAGE_GPIO_Port GPIOB
#define ADC1_CURRENT_Pin LL_GPIO_PIN_1
#define ADC1_CURRENT_GPIO_Port GPIOB
#define OUT_V_EN_Pin LL_GPIO_PIN_7
#define OUT_V_EN_GPIO_Port GPIOE
#define OUT_V_A0_Pin LL_GPIO_PIN_8
#define OUT_V_A0_GPIO_Port GPIOE
#define OUT_V_A1_Pin LL_GPIO_PIN_9
#define OUT_V_A1_GPIO_Port GPIOE
#define OUT_V_A2_Pin LL_GPIO_PIN_10
#define OUT_V_A2_GPIO_Port GPIOE
#define LED_PWR_EN_Pin LL_GPIO_PIN_13
#define LED_PWR_EN_GPIO_Port GPIOE
#define LED_DATA_Pin LL_GPIO_PIN_10
#define LED_DATA_GPIO_Port GPIOB
#define BTN_8_Pin LL_GPIO_PIN_8
#define BTN_8_GPIO_Port GPIOD
#define BTN_7_Pin LL_GPIO_PIN_9
#define BTN_7_GPIO_Port GPIOD
#define BTN_6_Pin LL_GPIO_PIN_10
#define BTN_6_GPIO_Port GPIOD
#define BTN_5_Pin LL_GPIO_PIN_11
#define BTN_5_GPIO_Port GPIOD
#define BTN_4_Pin LL_GPIO_PIN_12
#define BTN_4_GPIO_Port GPIOD
#define BTN_3_Pin LL_GPIO_PIN_13
#define BTN_3_GPIO_Port GPIOD
#define BTN_2_Pin LL_GPIO_PIN_14
#define BTN_2_GPIO_Port GPIOD
#define BTN_1_Pin LL_GPIO_PIN_15
#define BTN_1_GPIO_Port GPIOD
#define OUT_8_EN_Pin LL_GPIO_PIN_15
#define OUT_8_EN_GPIO_Port GPIOA
#define OUT_7_EN_Pin LL_GPIO_PIN_10
#define OUT_7_EN_GPIO_Port GPIOC
#define OUT_6_EN_Pin LL_GPIO_PIN_11
#define OUT_6_EN_GPIO_Port GPIOC
#define OUT_5_EN_Pin LL_GPIO_PIN_12
#define OUT_5_EN_GPIO_Port GPIOC
#define OUT_4_EN_Pin LL_GPIO_PIN_0
#define OUT_4_EN_GPIO_Port GPIOD
#define OUT_3_EN_Pin LL_GPIO_PIN_1
#define OUT_3_EN_GPIO_Port GPIOD
#define OUT_2_EN_Pin LL_GPIO_PIN_2
#define OUT_2_EN_GPIO_Port GPIOD
#define OUT_1_EN_Pin LL_GPIO_PIN_3
#define OUT_1_EN_GPIO_Port GPIOD
#define SW_1_Pin LL_GPIO_PIN_5
#define SW_1_GPIO_Port GPIOD
#define SW_2_Pin LL_GPIO_PIN_6
#define SW_2_GPIO_Port GPIOD
#define SW_3_Pin LL_GPIO_PIN_7
#define SW_3_GPIO_Port GPIOD
#define SW_4_Pin LL_GPIO_PIN_3
#define SW_4_GPIO_Port GPIOB
#define SW_5_Pin LL_GPIO_PIN_4
#define SW_5_GPIO_Port GPIOB
#define SW_6_Pin LL_GPIO_PIN_5
#define SW_6_GPIO_Port GPIOB
#define SW_7_Pin LL_GPIO_PIN_6
#define SW_7_GPIO_Port GPIOB
#define SW_8_Pin LL_GPIO_PIN_7
#define SW_8_GPIO_Port GPIOB
#define RS485_DE_Pin LL_GPIO_PIN_8
#define RS485_DE_GPIO_Port GPIOB
#define RS485_RE_Pin LL_GPIO_PIN_9
#define RS485_RE_GPIO_Port GPIOB
#define RS485_RX_Pin LL_GPIO_PIN_0
#define RS485_RX_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
