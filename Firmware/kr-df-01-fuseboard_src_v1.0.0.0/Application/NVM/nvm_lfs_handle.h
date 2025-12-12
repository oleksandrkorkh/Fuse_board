#pragma once

#include "nvm_cfg.h"
#include "lfs.h"
#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif

void nvm_lfs_handle_init_thread_mutex();

int nvm_lfs_handle_read (const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off, void *buf, lfs_size_t len);
int nvm_lfs_handle_prog (const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off, const void *buf, lfs_size_t len);
int nvm_lfs_handle_erase(const struct lfs_config *cfg, lfs_block_t block);
int nvm_lfs_handle_sync (const struct lfs_config *cfg);
int nvm_lfs_handle_mutex_lock  (const struct lfs_config *cfg);
int nvm_lfs_handle_mutex_unlock(const struct lfs_config *cfg);
bool nvm_lfs_handle_erase_edata();

#ifdef __cplusplus
}
#endif
