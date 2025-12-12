/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    app_threadx.c
 * @author  MCD Application Team
 * @brief   ThreadX applicative file
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

/* Includes ------------------------------------------------------------------*/
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "watchdog.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// TODO: Fine-tune and adjust thread sizes and priorities:
#define FUSEBOARD_MAIN_STACK_SIZE 4096
#define FUSEBOARD_MAIN_THREAD_PRIO 10

#define FUSEBOARD_COMMS_STACK_SIZE 2048
#define FUSEBOARD_COMMS_THREAD_PRIO 10

#define FUSEBOARD_LEDS_STACK_SIZE 1024
#define FUSEBOARD_LEDS_THREAD_PRIO 10

#define FUSEBOARD_UI_STACK_SIZE 1024
#define FUSEBOARD_UI_THREAD_PRIO 10

#define FUSEBOARD_RELAYS_STACK_SIZE 1600
#define FUSEBOARD_RELAYS_THREAD_PRIO 10

#define FUSEBOARD_ADC_STACK_SIZE 1024
#define FUSEBOARD_ADC_THREAD_PRIO 10

#define LOG_STACK_SIZE 3072
#define LOG_THREAD_PRIO 10

#define CONFIG_THREAD_PRIO 10
#define CONFIG_STACK_SIZE 3072

#define WATCHDOG_STACK_SIZE 512
#define WATCHDOG_THREAD_PRIO 9

extern VOID Fuseboard_Main_Thread_Entry(ULONG);
extern VOID Fuseboard_Comms_Thread_Entry(ULONG);
extern VOID Fuseboard_LEDS_Thread_Entry(ULONG);
extern VOID Fuseboard_UI_Thread_Entry(ULONG);
extern VOID Fuseboard_Relays_Thread_Entry(ULONG);
extern VOID Fuseboard_Adc_Thread_Entry(ULONG);
extern VOID Log_Thread_Entry(ULONG);
extern VOID Config_Thread_Entry(ULONG);
extern VOID Watchdog_Thread_Entry(ULONG);

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
TX_THREAD fuseboard_main_thread;
TX_THREAD fuseboard_comms_thread;
TX_THREAD fuseboard_leds_thread;
TX_THREAD fuseboard_ui_thread;
TX_THREAD fuseboard_relays_thread;
TX_THREAD fuseboard_adc_thread;
TX_THREAD log_thread;
TX_THREAD watchdog_thread;

TX_THREAD config_thread;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;

  /* USER CODE BEGIN App_ThreadX_MEM_POOL */

  /* USER CODE END App_ThreadX_MEM_POOL */

  /* USER CODE BEGIN App_ThreadX_Init */
  TX_BYTE_POOL* byte_pool = (TX_BYTE_POOL*)memory_ptr;
  CHAR* pointer;

  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, FUSEBOARD_MAIN_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }
  if (tx_thread_create(&fuseboard_main_thread, "Fuseboard Main Thread", Fuseboard_Main_Thread_Entry, 0, pointer,
      FUSEBOARD_MAIN_STACK_SIZE, FUSEBOARD_MAIN_THREAD_PRIO, FUSEBOARD_MAIN_THREAD_PRIO,
      TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
  {
    return TX_THREAD_ERROR;
  }

  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, FUSEBOARD_COMMS_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }
  if (tx_thread_create(&fuseboard_comms_thread, "Fuseboard Comms thread", Fuseboard_Comms_Thread_Entry, 0, pointer,
      FUSEBOARD_COMMS_STACK_SIZE, FUSEBOARD_COMMS_THREAD_PRIO, FUSEBOARD_COMMS_THREAD_PRIO,
      TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
  {
    return TX_THREAD_ERROR;
  }

  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, FUSEBOARD_LEDS_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }
  if (tx_thread_create(&fuseboard_leds_thread, "Fuseboard LEDS thread", Fuseboard_LEDS_Thread_Entry, 0, pointer,
      FUSEBOARD_LEDS_STACK_SIZE, FUSEBOARD_LEDS_THREAD_PRIO, FUSEBOARD_LEDS_THREAD_PRIO,
      TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
  {
    return TX_THREAD_ERROR;
  }

  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, FUSEBOARD_UI_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }
  if (tx_thread_create(&fuseboard_ui_thread, "Fuseboard UI thread", Fuseboard_UI_Thread_Entry, 0, pointer,
      FUSEBOARD_UI_STACK_SIZE, FUSEBOARD_UI_THREAD_PRIO, FUSEBOARD_UI_THREAD_PRIO,
      TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
  {
    return TX_THREAD_ERROR;
  }


  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, FUSEBOARD_RELAYS_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }
  if (tx_thread_create(&fuseboard_relays_thread, "Fuseboard Relays thread", Fuseboard_Relays_Thread_Entry, 0, pointer,
      FUSEBOARD_RELAYS_STACK_SIZE, FUSEBOARD_RELAYS_THREAD_PRIO, FUSEBOARD_RELAYS_THREAD_PRIO,
      TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
  {
    return TX_THREAD_ERROR;
  }

  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, FUSEBOARD_ADC_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }
  if (tx_thread_create(&fuseboard_adc_thread, "Fuseboard ADC thread", Fuseboard_Adc_Thread_Entry, 0, pointer,
      FUSEBOARD_ADC_STACK_SIZE, FUSEBOARD_ADC_THREAD_PRIO, FUSEBOARD_ADC_THREAD_PRIO,
      TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
  {
    return TX_THREAD_ERROR;
  }

  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, LOG_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }
  if (tx_thread_create(&log_thread, "Log thread", Log_Thread_Entry, 0, pointer,
      LOG_STACK_SIZE, LOG_THREAD_PRIO, LOG_THREAD_PRIO,
      TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
  {
    return TX_THREAD_ERROR;
  }

  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, CONFIG_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }
  if (tx_thread_create(&config_thread, "Config thread", Config_Thread_Entry, 0, pointer,
      CONFIG_STACK_SIZE, CONFIG_THREAD_PRIO, CONFIG_THREAD_PRIO,
      TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
  {
    return TX_THREAD_ERROR;
  }

  if (tx_byte_allocate(byte_pool, (VOID**)&pointer, WATCHDOG_STACK_SIZE, TX_NO_WAIT) != TX_SUCCESS)
  {
    return TX_POOL_ERROR;
  }
  if (tx_thread_create(&watchdog_thread, "Watchdog Thread", Watchdog_Thread_Entry, 0, pointer,
      WATCHDOG_STACK_SIZE, WATCHDOG_THREAD_PRIO, WATCHDOG_THREAD_PRIO,
      TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
  {
    return TX_THREAD_ERROR;
  }


  /* USER CODE END App_ThreadX_Init */

  return ret;
}

  /**
  * @brief  Function that implements the kernel's initialization.
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN  Before_Kernel_Start */

  /* USER CODE END  Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN  Kernel_Start_Error */

  /* USER CODE END  Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
