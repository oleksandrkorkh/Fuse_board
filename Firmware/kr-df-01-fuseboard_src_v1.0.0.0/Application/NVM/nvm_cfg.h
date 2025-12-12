#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <tx_api.h>
#include <string.h>
#include <assert.h>

#ifdef __cplusplus
extern "C"
{
#endif

// TODO: fix namings

// High-write cycle Flash sectors count per bank from MCU reference manual chapter 7.3.10.
#define NVM_EDATA_BANK_SECTOR_COUNT  8

// EDATA only allows for 16 bit programming, MCU reference manual chapter 7.3.10.
#define NVM_EDATA_PROG_WORD_SIZE     2

// Specifically EDATA sectors start from 120. MCU reference manual chapter 7.3.10.
#define NVM_EDATA_SECTOR_START       120

#define NVM_EDATA_BANK_IN_USE        FLASH_BANK_2

#define NVM_PAGE_OVERHEAD            4  // Space for CRC - https://github.com/littlefs-project/littlefs/issues/843#issuecomment-1599334056
#define NVM_EDATA_PAGE_SIZE          256
#define NVM_EDATA_BLOCK_SIZE         6144
#define NVM_EDATA_PAGES_PER_SECTOR  (NVM_EDATA_BLOCK_SIZE / NVM_EDATA_PAGE_SIZE)

#define NVM_LFS_OP_SIZE       (NVM_EDATA_PAGE_SIZE - NVM_PAGE_OVERHEAD)  // Size of one read/prog size
#define NVM_LFS_BLOCK_SIZE    (NVM_LFS_OP_SIZE * NVM_EDATA_PAGES_PER_SECTOR)
#define NVM_LFS_BLOCK_COUNT    8
#define NVM_LFS_CACHE_SIZE     NVM_LFS_OP_SIZE * 2
#define NVM_LFS_LOOKAHEAD_SIZE 256
#define NVM_LFS_BLOCK_CYCLES   500

#ifdef __cplusplus
}
#endif
