#include "adc.h"
#include "board.h"
#include "tx_extension.h"
#include "main.h"

namespace board
{

static TX_MUTEX mutex;
static BoardStatus board;

void handle_board_data()
{
  BoardStatus brd;
  TX_MUTEX_SECTION(&mutex)
  {
    brd = board;
  }

  float adc_temp;
  float adc_v_in;

  adc::get_adc_board_result_values(&adc_temp, &adc_v_in);
  brd.temperature = adc_temp;
  brd.v_in = adc_v_in;

  TX_MUTEX_SECTION(&mutex)
  {
    board.temperature = brd.temperature;
    board.v_in = brd.v_in;
  }
}

BoardStatus get_board_status()
{
  // Take a copy within critical section
  BoardStatus status;

  handle_board_data();

  TX_MUTEX_SECTION(&mutex)
  {
    status = board;
  }

  return status;
}

/**
 * Initialize board data
 */
void initialize_board_data()
{
  // Create mutex for thread-safe data exchange
  uint32_t result = tx_mutex_create(&mutex, (char *)"Board data mutex", TX_INHERIT);
  ASSERT_FATAL(result == TX_SUCCESS, "Mutex create error %u", result);
}
}
