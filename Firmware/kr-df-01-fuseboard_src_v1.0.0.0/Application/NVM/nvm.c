/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <stm32h5xx_hal.h>
#include <tx_api.h>

#include "lfs.h"
#include "nvm.h"
#include "ulog.h"
#include "common.h"

/* Private define ------------------------------------------------------------*/

/* A key that is expected from format request no-init variable to make format */
#define FORMAT_REQUEST_KEYWORD 0xDEADBEEF

/* Private constants ---------------------------------------------------------*/

// Configuration of the LFS filesystem
static const struct lfs_config cfg =
{
  // block device operations
  .read   = nvm_lfs_handle_read,
  .prog   = nvm_lfs_handle_prog,
  .erase  = nvm_lfs_handle_erase,
  .sync   = nvm_lfs_handle_sync,
  .lock   = nvm_lfs_handle_mutex_lock,
  .unlock = nvm_lfs_handle_mutex_unlock,

  // block device configuration
  .read_size      = NVM_LFS_OP_SIZE,
  .prog_size      = NVM_LFS_OP_SIZE,
  .block_size     = NVM_LFS_BLOCK_SIZE,
  .block_count    = NVM_LFS_BLOCK_COUNT,
  .cache_size     = NVM_LFS_CACHE_SIZE,
  .lookahead_size = NVM_LFS_LOOKAHEAD_SIZE,
  .block_cycles   = NVM_LFS_BLOCK_CYCLES,
};

/* Private variables ---------------------------------------------------------*/
static lfs_t lfs = { 0 };
static bool is_mounted = false;

/* This variable is in the no-init RAM area. It will contain garbage
 * in normal situations, but it will be used to transfer format request
 * between MCU reset. */
static volatile uint32_t format_request_msg __attribute__ ((section (".noinit")));

/* Private function prototypes -----------------------------------------------*/
static void nvm_check_format_request();
static void nvm_mount();

/* Public functions ----------------------------------------------------------*/

/**
 * NVM initialization.
 */
void nvm_init()
{
  nvm_lfs_handle_init_thread_mutex();
  nvm_check_format_request();
  nvm_mount();
}

/**
 * Check file system format request from no-init variable.
 */
static void nvm_check_format_request()
{
  /* Check for request */
  if (format_request_msg != FORMAT_REQUEST_KEYWORD)
  {
    /* Clear garbage */
    format_request_msg = 0;
    return;
  }

  /* Clear request to avoid repeating format.
   * If it fails, then user must request again. */
  format_request_msg = 0;

  /* Erase EDATA */
  nvm_lfs_handle_mutex_lock(&cfg);
  bool erase_res = nvm_lfs_handle_erase_edata();
  nvm_lfs_handle_mutex_unlock(&cfg);
  if (!erase_res)
  {
    return;
  }

  /* Format file system */
  int format_res = lfs_format(&lfs, &cfg);
  if (format_res != LFS_ERR_OK)
  {
    ULOG_ERROR("LFS formatting error: %i", format_res);
    return;
  }

  ULOG_INFO("LFS formatted");
}

/**
 * LFS file system mounting.
 * @return true in case of success, false in case of error.
 */
static void nvm_mount()
{
  struct lfs_fsinfo info;

  /* By default, not mounted */
  is_mounted = false;

  /* Try to mount LFS */
  int res = lfs_mount(&lfs, &cfg);
  if (res != LFS_ERR_OK)
  {
    ULOG_ERROR("LFS mounting error: %i", res);
    return;
  }

  /* Try to get file system stats */
  res = lfs_fs_stat(&lfs, &info);
  if (res != LFS_ERR_OK)
  {
    ULOG_ERROR("LFS stats error: %i", res);
    return;
  }

  ULOG_INFO("LFS version 0x%X mounted, size %u x %u B", info.disk_version, info.block_count, info.block_size);
  is_mounted = true;
}

/**
 * Make a MCU reset and format at start-up.
 * @attention This function does not return!
 */
void nvm_reset_and_format()
{
  ULOG_INFO("Request NVM format and make MCU reset...");
  format_request_msg = FORMAT_REQUEST_KEYWORD;
  NVIC_SystemReset();
}

/**
 * Check if NVM is mounted.
 * @return true if mounted and accessible, false if not.
 */
bool nvm_is_mounted()
{
  return is_mounted;
}

// returns true on success
bool nvm_write(const char *filename, const uint8_t *data, const uint32_t data_size, NVM_OP_TYPE op_type)
{
  ASSERT_FATAL(op_type < NVM_OP_COUNT, "Write operation type not supported")

  lfs_file_t file;
  uint16_t flags = 0;
  int res;

  if (!is_mounted)
  {
    ULOG_ERROR("Can't write %s, file system not mounted", filename);
    return false;
  }

  if (op_type == NVM_OP_OVERWRITE)
  {
    flags = LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC;
  }
  else if (op_type == NVM_OP_APPEND)
  {
    flags = LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND;
  }

  res = lfs_file_open(&lfs, &file, filename, flags);

  if (res < LFS_ERR_OK)
  {
    ULOG_ERROR("File %s open error, code: %i", filename, res);
  }
  else
  {
    res = lfs_file_write(&lfs, &file, data, data_size);
    if (res < LFS_ERR_OK)
    {
      ULOG_ERROR("File %s write error, code: %i", filename, res);
    }

    int close_res = lfs_file_close(&lfs, &file);
    if (close_res < LFS_ERR_OK)
    {
      res = close_res;
      ULOG_ERROR("File %s close failed: %i", filename, res);
    }
  }

  bool res_stat = (res >= LFS_ERR_OK);
  return res_stat;
}

// returns true on success
bool nvm_read(const char *filename, uint8_t *data, const uint32_t data_size, NVM_READ_TYPE read_type)
{
  ASSERT_FATAL(read_type < NVM_READ_COUNT, "Read operation type not supported");

  lfs_file_t file;
  int res;

  if (!is_mounted)
  {
    ULOG_ERROR("Can't read %s, file system not mounted", filename);
    return false;
  }

  switch (read_type)
  {
    case NVM_READ_CREATE:
      res = lfs_file_open(&lfs, &file, filename, LFS_O_RDONLY | LFS_O_CREAT);
      break;

    case NVM_READ_ONLY:
      res = lfs_file_open(&lfs, &file, filename, LFS_O_RDONLY);
      break;

    default:
      ULOG_CRITICAL("Unhandled read type after assert");
      break;
  }

  if (res < LFS_ERR_OK)
  {
    ULOG_ERROR("File %s open error, code: %i", filename, res);
  }
  else
  {
    res = lfs_file_read(&lfs, &file, data, data_size);
    if (res < LFS_ERR_OK)
    {
      ULOG_ERROR("File %s read error, code: %i", filename, res);
    }

    int close_res = lfs_file_close(&lfs, &file);
    if (close_res < LFS_ERR_OK)
    {
      res = close_res;
      ULOG_ERROR("File %s close failed: %i", filename, close_res);
    }
  }

  bool res_stat = (res >= LFS_ERR_OK);
  return res_stat;
}
