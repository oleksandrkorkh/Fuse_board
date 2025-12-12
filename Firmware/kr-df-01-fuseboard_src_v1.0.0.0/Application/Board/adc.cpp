#include "common.h"
#include "relays.h"
#include "board.h"
#include "watchdog.h"
#include "ll_relays.h"
#include "tx_extension.h"
#include "util.h"
#include "main.h"

#include <tx_api.h>

/* Public variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

namespace adc
{
/* Constants -----------------------------------------------------------------*/

/* ADC resolution (12-bits) */
#define ADC_RESOLUTION 4096

/* Multiplier to convert MCU pin voltage to relay output voltage (V).
 * From HW-SW interface specification. */
#define ADC_RELAY_V_FACTOR(mV)  (mV * 11) //mv = adc * (0,000806f*11f)

/* Multiplier to convert MCU pin voltage to relay output current (A).
 * From HW-SW interface specification. */
#define ADC_RELAY_I_FACTOR(mV) (mV / 0.066) // ((ADC * 0.000806) / 0.066)

// Amount of adc values to save that the average is calculated from
#define ADC_VALUE_SAVE_COUNT 4U

#define COMPUTATION_DUALMODE_ADCMASTER_RESULT(DATA)                 \
  ((DATA) & 0x0000FFFF)
#define COMPUTATION_DUALMODE_ADCSLAVE_RESULT(DATA)                  \
  ((DATA) >> 16)

#define ADC_SAMPLING_TIMEOUT_MS  500

// ADC is muxxed toa relays with a TMUX1208, which is controlled using its A0,A1,A2 pins.
// If TMUX is enabled (EN pin) then the A pins work as follows:
// ---->A2
//  --->A1
//   -->A0
// 000 -> relay 1
// 001 -> relay 2
// 010 -> relay 3
// 011 -> relay 4
// 100 -> relay 5
// 101 -> relay 6
// 110 -> relay 7
// 111 -> relay 8
// Same muxxing logic applies to both voltage and current ADC. See SW-HW documentation for more.
// This macro uses bitwise operations to extract the pin state from the relay number:
#define TMUX_GET_A_STATE(num, A) ((num & (1 << A)) == 1 << A) ? GPIO_PIN_SET : GPIO_PIN_RESET

/* Private functions ---------------------------------------------------------*/
static void read_adc_results(uint8_t relay_num, uint32_t *adc_v, uint32_t *adc_i, uint32_t *adc_temp, uint32_t *adc_v_in,
                             uint32_t *adc_vref_of_v, uint32_t *adc_vref_of_i, uint32_t *adc_vref_of_v_in);

/* Private variables ---------------------------------------------------------*/
static TX_MUTEX mutex;

static float result_voltage_array[relays::RELAY_COUNT] = {0};
static float result_current_array[relays::RELAY_COUNT] = {0};
static float result_voltage_in  = 0;
static float result_temperature = 0;

static float adc2_voltage_data_collection[relays::RELAY_COUNT][ADC_VALUE_SAVE_COUNT] = {0};
static float adc2_current_data_collection[relays::RELAY_COUNT][ADC_VALUE_SAVE_COUNT] = {0};
static float adc2_input_voltage_data_collection[ADC_VALUE_SAVE_COUNT]                = {0};
static float adc1_temperature_data_collection[ADC_VALUE_SAVE_COUNT]                  = {0};

static bool adc1_done_flag  = false;
static uint32_t adc_data[4] = {0}; // holds values of adc1/2 (dual mode)

static void add_value_at_float_array_start(float new_value, float *data_array)
{
  for (uint8_t i = ADC_VALUE_SAVE_COUNT - 1; i > 0; i--) {
    data_array[i] = data_array[i - 1];
  }

  // Insert  new value at the start of the relay's data
  data_array[0] = new_value;
}

// Calculate average for float array
static float calculate_float_array_average(float *data_array)
{
  float sum = 0.0f;
  for (uint8_t i = 0; i < ADC_VALUE_SAVE_COUNT; i++) {
    sum += data_array[i];
  }
  return sum / ADC_VALUE_SAVE_COUNT;
}

/**
 * Converts adc value to mV
 */
static uint32_t convert_adc_value(uint32_t adc_raw, uint32_t vref)
{
  return __LL_ADC_CALC_DATA_TO_VOLTAGE(vref, adc_raw, LL_ADC_RESOLUTION_12B);
}

/**
 * Convert ADC raw digital value into relay voltage output
 * @param raw: ADC raw conversion value
 * @param vref: Vref value in mV
 * @return Volts (V) at relay output
 */
static float convert_adc_v(uint32_t raw, uint32_t vref)
{
  ASSERT_FATAL(raw < ADC_RESOLUTION, "Invalid ADC raw value");
  uint32_t converted_val = convert_adc_value(raw, vref);
  return ADC_RELAY_V_FACTOR(static_cast<float>(converted_val)) / 1000;  // convert to volts
}

/**
 * Convert ADC raw digital value into relay current value
 * @param raw:  ADC raw conversion value
 * @param vref: Vref value in mV
 * @return Current (A) through relay.
 */
static float convert_adc_i(uint32_t raw, uint32_t vref)
{
    ASSERT_FATAL(raw < ADC_RESOLUTION, "Invalid ADC raw value");  // 12-bit resolution check
    uint32_t converted_val = convert_adc_value(raw, vref);
    uint32_t v_mid = vref / 2;
    int32_t signed_voltage_diff = (int32_t)converted_val - (int32_t)v_mid;
    return ADC_RELAY_I_FACTOR(static_cast<float>(signed_voltage_diff)) / 1000; // convert to amps
}

static uint32_t convert_adc_temp(uint32_t raw, uint32_t vref)
{
  return __LL_ADC_CALC_TEMPERATURE(vref, raw, LL_ADC_RESOLUTION_12B);
}

/**
 * Converts vrefint data to mV
 */
static uint32_t calculate_vrefint_mv(uint32_t vrefint_data)
{
  if (vrefint_data == 0)
  {
    return 0;  // Prevent division by zero
  }

  uint32_t vrefint_mv = __LL_ADC_CALC_VREFANALOG_VOLTAGE(vrefint_data, LL_ADC_RESOLUTION_12B);
  return vrefint_mv;
}

static void handle_adc()
{
  for (relays::Relay rel_n : relays::RELAYS)
  {
    uint32_t adc_v            = 0;
    uint32_t adc_v_in         = 0;
    uint32_t adc_i            = 0;
    uint32_t adc_temp         = 0;
    uint32_t adc_vref_of_v    = 0;
    uint32_t adc_vref_of_i    = 0;
    uint32_t adc_vref_of_v_in = 0;
    read_adc_results(rel_n, &adc_v, &adc_i, &adc_temp, &adc_v_in,
                     &adc_vref_of_v, &adc_vref_of_i, &adc_vref_of_v_in);

    uint32_t vref_value_of_v = calculate_vrefint_mv(adc_vref_of_v);
    uint32_t vref_value_of_i = calculate_vrefint_mv(adc_vref_of_i);

    float converted_adc_v = convert_adc_v(adc_v, vref_value_of_v);
    float converted_adc_i = convert_adc_i(adc_i, vref_value_of_i);

    add_value_at_float_array_start(converted_adc_v, adc2_voltage_data_collection[rel_n]);
    add_value_at_float_array_start(converted_adc_i, adc2_current_data_collection[rel_n]);

    TX_MUTEX_SECTION(&mutex)
    {
      result_voltage_array[rel_n] = calculate_float_array_average(adc2_voltage_data_collection[rel_n]);
      result_current_array[rel_n] = calculate_float_array_average(adc2_current_data_collection[rel_n]);
    }

    if (rel_n == relays::RELAY_COUNT-1)
    {
      uint32_t vref_value_of_v_in = calculate_vrefint_mv(adc_vref_of_v_in);

      // unable to get the vref while reading temp adc as they are read from the same adc
      float converted_adc_temp = convert_adc_temp(adc_temp, vref_value_of_v_in);
      float converted_adc_v_in = convert_adc_v(adc_v_in, vref_value_of_v_in);

      add_value_at_float_array_start(converted_adc_temp, adc1_temperature_data_collection);
      add_value_at_float_array_start(converted_adc_v_in, adc2_input_voltage_data_collection);

      TX_MUTEX_SECTION(&mutex)
      {
        result_temperature = calculate_float_array_average(adc1_temperature_data_collection);
        result_voltage_in  = calculate_float_array_average(adc2_input_voltage_data_collection);
      }
    }
  }
}

// Get saved, averaged relay values
void get_adc_relay_result_values(uint8_t relay_num, float *adc_v, float *adc_i)
{
  ASSERT_FATAL(relay_num < relays::RELAY_COUNT, "Invalid relay num");

  TX_MUTEX_SECTION(&mutex)
  {
    *adc_v = result_voltage_array[relay_num];
    *adc_i = result_current_array[relay_num];
  }
}

// Get saved, averaged board values
void get_adc_board_result_values(float *adc_temp, float *adc_v_in)
{
  TX_MUTEX_SECTION(&mutex)
  {
    *adc_temp = result_temperature;
    *adc_v_in = result_voltage_in;
  }
}

/**
 * Used to wait for mux switching delay. Timing details in TIM3 init in main.c
 */
static void start_adc_trigger_timer()
{
  // Start timer used to trigger ADC1/ADC2
  HAL_TIM_Base_Stop_IT(&htim3);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  HAL_TIM_Base_Start_IT(&htim3);
}

// Start ADC
static void start_adc_multimode()
{
  bool status_adc1 = false;
  const uint8_t retry_count = 3;

  for (uint8_t attempt = 0; attempt < retry_count; attempt++)
  {
    HAL_StatusTypeDef err_adc1 = HAL_ADCEx_MultiModeStart_DMA(&hadc1, adc_data, COUNT_OF(adc_data));

    if (err_adc1 != HAL_OK)
    {
      ULOG_ERROR("ADC1 multimode DMA start failed, HAL error: %u", err_adc1);
      HAL_ADCEx_MultiModeStop_DMA(&hadc1);
      sleep_ms(1);
    }
    else
    {
      status_adc1 = true;
      break;
    }
  }

  if (!status_adc1)
  {
    ULOG_ERROR("ADC1 MULTIMODE DMA start failed, out of retries");
    Error_Handler();
  }

  return;
}

// ADC mux selection, setting, resetting
static void adc_set_mux()
{
  HAL_GPIO_WritePin(OUT_V_EN_GPIO_Port, OUT_V_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(OUT_I_EN_GPIO_Port, OUT_I_EN_Pin, GPIO_PIN_SET);
}

static void adc_reset_mux()
{
  HAL_GPIO_WritePin(OUT_V_EN_GPIO_Port, OUT_V_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(OUT_I_EN_GPIO_Port, OUT_I_EN_Pin, GPIO_PIN_RESET);
}

static void adc_config_mux(uint8_t relay_num)
{
  ASSERT_FATAL(relay_num < relays::RELAY_COUNT, "Invalid relay num");
  // Configure voltage ADC mux:
  uint32_t bsrr_v =
    ((TMUX_GET_A_STATE(relay_num, 0)) ? OUT_V_A0_Pin : (OUT_V_A0_Pin << 16)) |
    ((TMUX_GET_A_STATE(relay_num, 1)) ? OUT_V_A1_Pin : (OUT_V_A1_Pin << 16)) |
    ((TMUX_GET_A_STATE(relay_num, 2)) ? OUT_V_A2_Pin : (OUT_V_A2_Pin << 16));

  LL_GPIO_WriteReg(OUT_V_A0_GPIO_Port, BSRR, bsrr_v);

  // Configure current ADC mux:
  uint32_t bsrr_i =
    ((TMUX_GET_A_STATE(relay_num, 0)) ? OUT_I_A0_Pin : (OUT_I_A0_Pin << 16)) |
    ((TMUX_GET_A_STATE(relay_num, 1)) ? OUT_I_A1_Pin : (OUT_I_A1_Pin << 16)) |
    ((TMUX_GET_A_STATE(relay_num, 2)) ? OUT_I_A2_Pin : (OUT_I_A2_Pin << 16));

  LL_GPIO_WriteReg(OUT_I_A0_GPIO_Port, BSRR, bsrr_i);
}

// ADC conversion values reading
static void read_adc_results(uint8_t relay_num, uint32_t *adc_v, uint32_t *adc_i, uint32_t *adc_temp, uint32_t *adc_v_in,
                             uint32_t *adc_vref_of_v, uint32_t *adc_vref_of_i, uint32_t *adc_vref_of_v_in)
{
  static bool initial_adc_start_done = false;

  adc_set_mux();
  adc_config_mux(relay_num);

  if (!(initial_adc_start_done))
  {
    start_adc_multimode();
    initial_adc_start_done = true;
  }

  start_adc_trigger_timer();

  ULONG start_ticks = tx_time_get();

  while (!adc1_done_flag)
  {
    if ((tx_time_get() - start_ticks) >= TX_TICKS_MS(ADC_SAMPLING_TIMEOUT_MS))
    {
      HAL_TIM_Base_Stop_IT(&htim3);
      HAL_ADCEx_MultiModeStop_DMA(&hadc1);
      ULOG_ERROR("ADC1/2 never finished");
      break;
    }

    sleep_ms(1);
  }

  adc1_done_flag = false;

  adc_reset_mux();

  // adc 1 data
  *adc_vref_of_v    = COMPUTATION_DUALMODE_ADCMASTER_RESULT(adc_data[0]);
  *adc_vref_of_i    = COMPUTATION_DUALMODE_ADCMASTER_RESULT(adc_data[1]);
  *adc_vref_of_v_in = COMPUTATION_DUALMODE_ADCMASTER_RESULT(adc_data[2]);
  *adc_temp         = COMPUTATION_DUALMODE_ADCMASTER_RESULT(adc_data[3]);

  // adc 2 data
  *adc_v    = COMPUTATION_DUALMODE_ADCSLAVE_RESULT(adc_data[0]);
  *adc_i    = COMPUTATION_DUALMODE_ADCSLAVE_RESULT(adc_data[1]);
  *adc_v_in = COMPUTATION_DUALMODE_ADCSLAVE_RESULT(adc_data[2]);
  // 4th adc 2 value is a dummy, just to get adc1 temp data
}

void initialize_adc()
{
  // Create mutex for thread-safe data exchange
  uint32_t result = tx_mutex_create(&mutex, (char *)"ADC data mutex", TX_INHERIT);
  ASSERT_FATAL(result == TX_SUCCESS, "Mutex create error %u", result);
}

static void adc_thread_entry()
{
  uint32_t wake_time = tx_time_get();
  const uint32_t update_interval_ms = 100;

  watchdog_configure_thread(WATCHDOG_THREAD_ADC, update_interval_ms, update_interval_ms * 4);

  while (true)
  {
    handle_adc();
    board::handle_board_data();
    sleep_until_ms(&wake_time, update_interval_ms);
    watchdog_kick_thread(WATCHDOG_THREAD_ADC);
  }
}
}

// ADC conversion complete
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc == &hadc1)
  {
    adc::adc1_done_flag = true;
  }
}

extern "C"
{
VOID Fuseboard_Adc_Thread_Entry(ULONG)
{
  adc::adc_thread_entry();
}
}
