#pragma once

#include "nvm_cfg.h"
#include "tx_api.h"
#include "nvm_lfs_handle.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
  NVM_OP_APPEND = 0,
  NVM_OP_OVERWRITE,
  NVM_OP_COUNT
} NVM_OP_TYPE;

typedef enum
{
  NVM_READ_ONLY = 0,
  NVM_READ_CREATE,
  NVM_READ_COUNT
} NVM_READ_TYPE;

void nvm_init();
void nvm_reset_and_format();
bool nvm_is_mounted();
bool nvm_write(const char *filename, const uint8_t *data, const uint32_t data_size, NVM_OP_TYPE op_type);
bool nvm_read(const char *filename, uint8_t *data, const uint32_t data_size, NVM_READ_TYPE read_type);

#ifdef __cplusplus
}
#endif
