#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

void crc_gen_init();
uint32_t crc_gen_crc32(uint32_t *buffer, uint32_t buffer_length);

#ifdef __cplusplus
}
#endif
