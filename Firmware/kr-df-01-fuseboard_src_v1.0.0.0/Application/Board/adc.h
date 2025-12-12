#pragma once

#include <stdint.h>

namespace adc
{
void initialize_adc();
void get_adc_relay_result_values(uint8_t relay_num, float *adc_v, float *adc_i);
void get_adc_board_result_values(float *adc_temp, float *adc_v_in);
}
