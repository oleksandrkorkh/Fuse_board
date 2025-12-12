#include "crc.h"
#include "main.h"
#include <tx_extension.h>

extern CRC_HandleTypeDef hcrc;
static TX_MUTEX crc_mutex;

void crc_gen_init()
{
  uint32_t result = tx_mutex_create(&crc_mutex, (char *)"CRC mutex", TX_INHERIT);
  ASSERT_FATAL(result == TX_SUCCESS, "CRC mutex create error %u", result);
}

uint32_t crc_gen_crc32(uint32_t *buffer, uint32_t buffer_length)
{
  uint32_t gen_crc = 0;

  TX_MUTEX_SECTION(&crc_mutex) // waits forever until mutex available
  {
    const uint8_t word_size = sizeof(uint32_t);
    ASSERT_FATAL(buffer_length % word_size == 0, "Data size not aligned with word");

    uint32_t amount_of_words = buffer_length / word_size;
    gen_crc = HAL_CRC_Calculate(&hcrc, buffer, amount_of_words);
  }

  return gen_crc;
}
