#ifndef INPUTS_LL_INPUTS_H_
#define INPUTS_LL_INPUTS_H_
#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

uint8_t ll_inputs_read_switch(uint8_t num);

uint8_t ll_inputs_read_button(uint8_t num);

#endif /* INPUTS_LL_INPUTS_H_ */

#ifdef __cplusplus
}
#endif
