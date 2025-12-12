#pragma once

#include "ulog.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define LOG_FLASH_MAX_LOG_LINE_SIZE 64
#define LOG_FLASH_MAX_LOG_LINES     20
#define LOG_FLASH_MAX_SIZE         (LOG_FLASH_MAX_LOG_LINE_SIZE * LOG_FLASH_MAX_LOG_LINES)

void log_flash_init();

#ifdef __cplusplus
}
#endif
