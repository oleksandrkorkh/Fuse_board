#include "nvm_lfs_handle.h"
#include "main.h"
#include "stm32h5xx_hal.h"
#include "tx_api.h"

extern CRC_HandleTypeDef hcrc;

static TX_MUTEX nvm_mutex;

void nvm_lfs_handle_init_thread_mutex()
{
  uint32_t result = tx_mutex_create(&nvm_mutex, (char *)"NVM data mutex", TX_INHERIT);
  ASSERT_FATAL(result == TX_SUCCESS, "Mutex create error %u", result);
}

static uint32_t nvm_lfs_handle_calc_crc32(uint32_t *address, uint32_t length)
{
  const uint8_t word_size = sizeof(uint32_t);
  ASSERT_FATAL(length % word_size == 0, "Data size not aligned with word");

  uint16_t amount_of_words = length / word_size;
  return HAL_CRC_Calculate(&hcrc, address, amount_of_words);
}

/**
  * @brief  Gets the sector of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The sector of a given address
  */
static uint32_t nvm_lfs_handle_get_sector_edata(uint32_t address)
{
  uint32_t sector = 0;

  /* Check if the address is located in the FLASH high-cycle data area of BANK1 */
  if ((address >= FLASH_EDATA_BASE) && (address < FLASH_EDATA_BASE + (FLASH_EDATA_SIZE / 2)))
  {
    sector = (address & ~FLASH_EDATA_BASE) / NVM_EDATA_BLOCK_SIZE;
    sector += NVM_EDATA_SECTOR_START; // Details for define in nvm_cfg.h
  }

  /* Check if the address is located in the FLASH high-cycle data area of BANK2 */
  else if ((address >= FLASH_EDATA_BASE + (FLASH_EDATA_SIZE / 2)) && (address < FLASH_EDATA_BASE + FLASH_EDATA_SIZE))
  {
    sector = ((address & ~FLASH_EDATA_BASE) - (FLASH_EDATA_SIZE / 2)) / NVM_EDATA_BLOCK_SIZE;
    sector += NVM_EDATA_SECTOR_START;
  }
  else
  {
    sector = 0xFFFFFFFF; /* Address out of range */
  }

  return sector;
}

/**
  * @brief  Gets the bank of a given address in EDATA area
  * @param  Addr: Address of A given address in EDATA area
  * @retval The bank of a given address in EDATA area
  */
static uint32_t nvm_lfs_handle_get_bank_edata(uint32_t address)
{
  uint32_t bank = 0;

  /* (FLASH_EDATA_SIZE/2) is the size of high-cycle area of flash BANK1 */
  if ((address >= FLASH_EDATA_BASE) && (address < FLASH_EDATA_BASE + (FLASH_EDATA_SIZE / 2)))
  {
    bank = FLASH_BANK_1;
  }
  else if ((address >= FLASH_EDATA_BASE + (FLASH_EDATA_SIZE / 2)) && (address < FLASH_EDATA_BASE + FLASH_EDATA_SIZE))
  {
    bank = FLASH_BANK_2;
  }
  else
  {
    bank = 0xFFFFFFFF; /* Address out of range */
  }
  return bank;
}

int nvm_lfs_handle_mutex_lock(const struct lfs_config *cfg)
{
  UNUSED(cfg);

  UINT status;

  status = tx_mutex_get(&nvm_mutex, TX_WAIT_FOREVER);
  if (status != TX_SUCCESS)
  {
    ULOG_ERROR("NVM mutex lock failure: %u", status);
  }

  return status;
}

int nvm_lfs_handle_mutex_unlock(const struct lfs_config *cfg)
{
  UNUSED(cfg);

  UINT status;

  status = tx_mutex_put(&nvm_mutex);
  if (status != TX_SUCCESS)
  {
    ULOG_ERROR("NVM mutex unlock failure: %u", status);
  }

  return status;
}

int nvm_lfs_handle_read(const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off, void *buf, lfs_size_t len)
{
  UNUSED(cfg);
  ASSERT_FATAL(buf != NULL, "lfs buf NULL");
  ASSERT_FATAL((off % NVM_LFS_OP_SIZE) == 0, "off: %u, NVM_LFS_PROG_OP_SIZE: %u, result: %u", off, NVM_LFS_OP_SIZE, off % NVM_LFS_OP_SIZE);
  ASSERT_FATAL((len % NVM_LFS_OP_SIZE) == 0, "len: %u, NVM_LFS_PROG_OP_SIZE: %u, result: %u", len, NVM_LFS_OP_SIZE, len % NVM_LFS_OP_SIZE);

  int lfs_err = LFS_ERR_OK;
  uint8_t * buf_data_ptr = (uint8_t *)buf;
  uint8_t __ALIGNED(4) mem_read_buf[NVM_EDATA_PAGE_SIZE];

  /* Get the base address of the bank in use */
  lfs_block_t bank_base_address = (NVM_EDATA_BANK_IN_USE == FLASH_BANK_1) ? (FLASH_EDATA_BASE_NS) : (FLASH_EDATA_BASE_NS + (NVM_EDATA_BLOCK_SIZE * NVM_EDATA_BANK_SECTOR_COUNT));

  /* Calculate physical page address from LFS virtual page address.
   * Physical address starts from the currently used bank base address and because physical page is made of LFS page content plus 4 bytes of CRC,
   * address offset translation is required. */
  lfs_block_t address = bank_base_address + (block * NVM_EDATA_BLOCK_SIZE) + ((off / NVM_LFS_OP_SIZE) * NVM_EDATA_PAGE_SIZE);

  while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY))
  {
    // Wait for the flash controller to be ready
  }

  while (len > 0)
  {
    // Read entire page to buffer
    for (uint16_t i = 0; i < NVM_EDATA_PAGE_SIZE; i += NVM_EDATA_PROG_WORD_SIZE)
    {
      LL_SBS_FLASH_DisableECCNMI();
      __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

      /* If it fails here and memory view shows ??????? on addr,
         check the Flash Data bank X sectors option bytes       */
      *((uint16_t *)(mem_read_buf + i)) = *(volatile uint16_t*)(address + i);

      if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_ECCR_ERRORS))
      {
        uint16_t ecc_data = *(uint16_t *)READ_REG(FLASH->ECCDR);
        if (ecc_data != 0xFFFF) // In case it is just uninitialized flash
        {
           lfs_err = LFS_ERR_IO;
        }
      }

      __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ECCR_ERRORS);
      LL_SBS_FLASH_EnableECCNMI();

      if (lfs_err == LFS_ERR_IO)
      {
        ULOG_ERROR("Addr: %u - Double (or more) ECC error detected", address + i);
        break;
      }
    }

    if (lfs_err == LFS_ERR_OK)
    {
      // Check page's CRC validity if needed
      if (*((uint32_t*)(&mem_read_buf[NVM_LFS_OP_SIZE])) == 0xFFFFFFFF)
      {
          // No CRC checking as page is empty (0xFFFFFFFF), just get the FF's
      }
      else // If page is not deemed empty
      {
        uint32_t read_crc = *((uint32_t*)(&mem_read_buf[NVM_LFS_OP_SIZE]));
        uint32_t calc_crc = nvm_lfs_handle_calc_crc32((uint32_t*)mem_read_buf, NVM_LFS_OP_SIZE);

        // Check if data found in page is valid (crc check)
        if (read_crc != calc_crc)
        {
          ULOG_ERROR("Addr: %u - CRC error detected", address);
          lfs_err = LFS_ERR_IO;
          break;
        }
      }

      (void)memcpy(buf_data_ptr, &mem_read_buf, NVM_LFS_OP_SIZE);

      address      += NVM_EDATA_PAGE_SIZE;
      buf_data_ptr += NVM_LFS_OP_SIZE;
      len          -= NVM_LFS_OP_SIZE;
    }
    else
    {
      break;
    }
  }

  return lfs_err;
}

int nvm_lfs_handle_prog(const struct lfs_config *cfg, lfs_block_t block, lfs_off_t off, const void *buf, lfs_size_t len)
{
  UNUSED(cfg);
  ASSERT_FATAL(buf != NULL, "lfs buf NULL");
  ASSERT_FATAL((off % NVM_LFS_OP_SIZE) == 0, "off: %u, NVM_LFS_OP_SIZE: %u, result: %u", off, NVM_LFS_OP_SIZE, off % NVM_LFS_OP_SIZE);
  ASSERT_FATAL((len % NVM_LFS_OP_SIZE) == 0, "len: %u, NVM_LFS_OP_SIZE: %u, result: %u", len, NVM_LFS_OP_SIZE, off % NVM_LFS_OP_SIZE);

  HAL_StatusTypeDef hal_rc = HAL_OK;
  const uint8_t * buf_data_ptr = (const uint8_t *)buf;
  uint8_t __ALIGNED(4) mem_write_buf[NVM_EDATA_PAGE_SIZE];

  /* Get the base address of the bank in use */
  lfs_block_t bank_base_address = (NVM_EDATA_BANK_IN_USE == FLASH_BANK_1) ? (FLASH_EDATA_BASE_NS) : (FLASH_EDATA_BASE_NS + (NVM_EDATA_BLOCK_SIZE * NVM_EDATA_BANK_SECTOR_COUNT));

  /* Calculate physical page address from LFS virtual page address.
   * Physical address starts from the currently used bank base address and because physical page is made of LFS page content plus 4 bytes of CRC,
   * address offset translation is required. */
  lfs_block_t address = bank_base_address + (block * NVM_EDATA_BLOCK_SIZE) + ((off / NVM_LFS_OP_SIZE) * NVM_EDATA_PAGE_SIZE);

  while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY))
  {
    // Wait for the flash controller to be ready
  }
  // Check and clear all the error flags due to previous programming/erase operation
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

  HAL_FLASH_Unlock();

  while (len > 0)
  {
    if (hal_rc != HAL_OK)
    {
      break;
    }

    (void)memcpy(mem_write_buf, buf_data_ptr, NVM_LFS_OP_SIZE);
    uint32_t calc_crc = nvm_lfs_handle_calc_crc32((uint32_t*)mem_write_buf, NVM_LFS_OP_SIZE);
    *((uint32_t*)(&mem_write_buf[NVM_LFS_OP_SIZE])) = calc_crc;

    for (uint32_t i = 0; i < NVM_EDATA_PAGE_SIZE; i += NVM_EDATA_PROG_WORD_SIZE)
    {
      hal_rc = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD_EDATA, address + i, (uint32_t)&mem_write_buf[i]);
      if (hal_rc != HAL_OK)
      {
        ULOG_ERROR("Program Error at 0x%u, HAL_RC: %u", address, hal_rc);
        break;
      }
    }

    address      += NVM_EDATA_PAGE_SIZE;
    buf_data_ptr += NVM_LFS_OP_SIZE;
    len          -= NVM_LFS_OP_SIZE;
  }

  HAL_FLASH_Lock();
  return hal_rc == HAL_OK ? LFS_ERR_OK : LFS_ERR_IO; // If HAL_OK, return LFS_ERR_OK, else return LFS_ERR_IO
}

// returns 0 (HAL_OK == LFS_ERR_OK == 0) if no errors
int nvm_lfs_handle_erase(const struct lfs_config *cfg, lfs_block_t block)
{
  ASSERT_FATAL(block < cfg->block_count, "Block selected higher than exists");

  HAL_StatusTypeDef hal_rc;
  int status = LFS_ERR_OK;
  FLASH_EraseInitTypeDef erase_init_struct;
  uint32_t sector_error = 0;

  HAL_FLASH_Unlock();

  erase_init_struct.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase_init_struct.Banks = NVM_EDATA_BANK_IN_USE;
  erase_init_struct.Sector = NVM_EDATA_SECTOR_START + block;
  erase_init_struct.NbSectors = 1;
  hal_rc = HAL_FLASHEx_Erase(&erase_init_struct, &sector_error);

  HAL_FLASH_Lock();

  if (sector_error != 0xFFFFFFFF)
  {
    ULOG_ERROR("Sector erase error in sector: %u", sector_error);
    status = LFS_ERR_IO;
  }
  else
  {
    if (hal_rc != HAL_OK)
    {
      ULOG_ERROR("%s ERROR 0x%X", __func__, hal_rc);
      status = LFS_ERR_IO;
    }
    else
    {
      ULOG_DEBUG("%s SUCCESS", __func__);
    }
  }

  return status; // returns LFS ERR status
}

int nvm_lfs_handle_sync(const struct lfs_config *cfg)
{
  UNUSED(cfg);
  return LFS_ERR_OK;
}

/**
 * Erase whole current data bank
 * @return true in case of success, false if case of error.
 */
bool nvm_lfs_handle_erase_edata()
{
  FLASH_EraseInitTypeDef erase_init_struct;
  HAL_StatusTypeDef lock_res;
  HAL_StatusTypeDef erase_res;
  uint32_t sector_error = 0;

  lock_res = HAL_FLASH_Unlock();
  if (lock_res != HAL_OK)
  {
    ULOG_ERROR("EDATA unlock error %u", lock_res);
    return false;
  }

  erase_init_struct.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase_init_struct.Banks     = NVM_EDATA_BANK_IN_USE;
  erase_init_struct.Sector    = NVM_EDATA_SECTOR_START;
  erase_init_struct.NbSectors = NVM_EDATA_BANK_SECTOR_COUNT;
  erase_res = HAL_FLASHEx_Erase(&erase_init_struct, &sector_error);

  /* Lock asap, before checking and reporting erase error */
  lock_res = HAL_FLASH_Lock();

  if (erase_res == HAL_OK)
  {
    ULOG_INFO("EDATA bank %u erased", erase_init_struct.Banks);
  }
  else
  {
    ULOG_ERROR("EDATA bank %u erase error %u in sector %u", erase_init_struct.Banks, erase_res, sector_error);
  }

  if (lock_res != HAL_OK)
  {
    ULOG_ERROR("EDATA lock error %u", lock_res);
  }

  return (erase_res == HAL_OK) && (lock_res == HAL_OK);
}
