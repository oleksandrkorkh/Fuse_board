#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

void ll_relays_set_relay(uint8_t num, bool state);
bool ll_relays_get_relay(uint8_t num);

#ifdef __cplusplus
}
#endif
