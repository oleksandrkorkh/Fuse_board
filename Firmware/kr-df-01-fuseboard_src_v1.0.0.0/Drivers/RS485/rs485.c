/**  RS485 with RTU modbus
*
* UART messaging functionality with rs485 direction control and modbus RTU message formatting (nanomodbus)
**/

#include <stdbool.h>
#include <tx_api.h>

#include "main.h"
#include "nanomodbus.h"
#include "rs485.h"
#include "common.h"
#include "tx_extension.h"
#include "dp_info.h"

// TODO: create confirmation of data transmitted via rx

/* Private typedef -----------------------------------------------------------*/
typedef enum {
  TX_INCOMPLETE = 0U,
  TX_COMPLETE = 1U
} SERIAL_EVENT_TYPE;

typedef struct {
  SERIAL_EVENT_TYPE type;
} SERIAL_TX_EVENT;

/* Private define ------------------------------------------------------------*/
#define TX_QUEUE_SIZE           128
#define RX_QUEUE_SIZE           128
#define TX_EVENT_QUEUE_SIZE      16

/* Private variables ---------------------------------------------------------*/
static USART_TypeDef *uart_handle = NULL;

static TX_QUEUE tx_queue;
static uint8_t tx_queue_mem[TX_QUEUE_SIZE];

static TX_QUEUE tx_event_queue;
static SERIAL_TX_EVENT tx_event_queue_mem[TX_EVENT_QUEUE_SIZE];

static TX_QUEUE rx_queue;
static uint8_t rx_queue_mem[RX_QUEUE_SIZE];

static uint32_t rx_queue_write_error       = TX_SUCCESS;
static uint32_t tx_queue_read_error        = TX_SUCCESS;
static uint32_t tx_event_queue_write_error = TX_SUCCESS;

static bool transmit_occured = false;

/* Private function prototypes -----------------------------------------------*/
static void rs485_init_serial_queues();
static void rs485_set_receive_mode();
static void rs485_set_transmit_mode();

/**
 * Initialize RS-485 driver
 */
void rs485_init(USART_TypeDef *uart)
{
  ASSERT_FATAL(uart != NULL, "UART handle is NULL");
  ASSERT_FATAL(uart_handle == NULL, "UART handle already initialized");

  rs485_init_serial_queues();
  rs485_set_receive_mode();

  uart_handle = uart;

  /* Enable the UART receive and error interrupts.
  * Transmit buffer empty interrupt is enabled only when there's something to send */
  LL_USART_EnableIT_RXNE(uart_handle);
  LL_USART_EnableIT_ERROR(uart_handle);
}

static void rs485_init_serial_queues()
{
  tx_queue_create(&tx_queue, "RS485 TX Queue", sizeof(uint8_t), tx_queue_mem, TX_QUEUE_SIZE);
  tx_queue_create(&tx_event_queue, "RS485 TX Event Queue", sizeof(SERIAL_TX_EVENT), tx_event_queue_mem, TX_EVENT_QUEUE_SIZE);
  tx_queue_create(&rx_queue, "RS485 RX Queue", sizeof(uint8_t), rx_queue_mem, RX_QUEUE_SIZE);
}

/**
 * RS-485 UART interrupt service handler
 */
void rs485_uart_irq()
{
  ASSERT_FATAL(uart_handle != NULL, "UART handle is NULL in IRQ");

  uint8_t byte;

  /* Parity error ? */
  if (LL_USART_IsActiveFlag_PE(uart_handle))
  {
    LL_USART_ClearFlag_PE(uart_handle);
  }

  /* Frame error ? */
  if (LL_USART_IsActiveFlag_FE(uart_handle))
  {
    LL_USART_ClearFlag_FE(uart_handle);
  }

  /* Noise error ? */
  if (LL_USART_IsActiveFlag_NE(uart_handle))
  {
    LL_USART_ClearFlag_NE(uart_handle);
  }

  /* Overrun ? */
  if (LL_USART_IsActiveFlag_ORE(uart_handle))
  {
    LL_USART_ClearFlag_ORE(uart_handle);
  }

  /* Received byte ? */
  if (LL_USART_IsActiveFlag_RXNE(uart_handle))
  {
    /* Get received byte */
    byte = LL_USART_ReceiveData8(uart_handle);

    // Send the received byte to the RX queue
    uint32_t rx_result = tx_queue_send(&rx_queue, &byte, TX_NO_WAIT);

    /* Report error code outside IRQ handler */
    if ((rx_result != TX_SUCCESS) && (rx_queue_write_error == TX_SUCCESS))
    {
      rx_queue_write_error = rx_result;
    }
  }

  /* Sent byte ? */
  if (LL_USART_IsActiveFlag_TXE(uart_handle))
  {
    /* Send another byte if there is some in transmit FIFO */
    uint32_t tx_result = tx_queue_receive(&tx_queue, &byte, TX_NO_WAIT);
    if (tx_result != TX_SUCCESS)
    {
      if ((tx_queue_read_error == TX_SUCCESS) &&
          (tx_result != TX_QUEUE_EMPTY)) // Expecting queue to be empty
      {
        tx_queue_read_error = tx_result;
      }

      /* Once finished, disable transmit buffer empty interrupt
       * because it can't be cleared manually and it would trigger infinitely */
      LL_USART_DisableIT_TXE(uart_handle);

      // TODO: transmit_occured could be a workaround, figure out what causes TXE to set on its own
      if (transmit_occured)
      {
        transmit_occured = false;

        SERIAL_TX_EVENT tx_complete_event = { .type = TX_COMPLETE };
        uint32_t tx_res_result = tx_queue_send(&tx_event_queue, &tx_complete_event, TX_NO_WAIT);

        if ((tx_res_result != TX_SUCCESS) && (tx_event_queue_write_error == TX_SUCCESS))
        {
          tx_event_queue_write_error = tx_res_result;
        }
      }
    }
    else
    {
      transmit_occured = true;
      LL_USART_TransmitData8(uart_handle, byte);
    }
  }
}

/**
 * @brief Check and report errors that have been detected within IRQ handler.
 *        This keeps IRQ execution time short and maintains software real time operation.
 */
static void rs485_check_irq_errors()
{
  if (rx_queue_write_error != TX_SUCCESS)
  {
    ULOG_ERROR("RX queue write error: %u", rx_queue_write_error);
    rx_queue_write_error = TX_SUCCESS;
  }

  if (tx_queue_read_error != TX_SUCCESS)
  {
    ULOG_ERROR("TX queue read error: %u", tx_queue_read_error);
    tx_queue_read_error = TX_SUCCESS;
  }

  if (tx_event_queue_write_error != TX_SUCCESS)
  {
    ULOG_ERROR("TX event queue write error: %u", tx_event_queue_write_error);
    tx_event_queue_write_error = TX_SUCCESS;
  }
}

static void rs485_set_transmit_mode()
{
  tx_queue_flush(&tx_queue);
  tx_queue_flush(&tx_event_queue);

  uint8_t hw_revision = dp_get_hw_revision();

  // DE pin active high - Set High = Enable transmit, Set low = Disable transmit
  // RE pin active low -  Set High = Disable receive, Set low = Enable receive
  // Data from SN65HVD12 datasheet
  if (hw_revision >= 0x02)
  {
    // TODO: create code that check's via feedback if transmission was successful
    LL_GPIO_SetOutputPin(RS485_DE_GPIO_Port, RS485_DE_Pin | RS485_RE_Pin); // They are on the same port
  }
  else if (hw_revision == 0x01) // Revision 1 has RS485 RE and DE pin lines together
  {
    HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_SET);
  }
  else
  {
    ULOG_CRITICAL("Unexpected HW revision");
    Error_Handler();
  }
}

static void rs485_set_receive_mode()
{
  tx_queue_flush(&tx_queue); // Flush tx queue as nothing can be transmitted in rx mode anyway
  tx_queue_flush(&rx_queue);

  uint8_t hw_revision = dp_get_hw_revision();

  // DE pin active high - Set High = Enable transmit, Set low = Disable transmit
  // RE pin active low -  Set High = Disable receive, Set low = Enable receive
  // Data from SN65HVD12 datasheet
  if (hw_revision >= 0x02)
  {
    LL_GPIO_ResetOutputPin(RS485_DE_GPIO_Port, RS485_DE_Pin | RS485_RE_Pin);  // They are on the same port
  }
  else if (hw_revision == 0x01) // Revision 1 has RS485 RE and DE pin lines together
  {
    HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET);
  }
  else
  {
    ULOG_CRITICAL("Unexpected HW revision");
    Error_Handler();
  }
}

/**
 * Receive bytes from RS-485 interface.
 * @param buf: Pointer to receive buffer.
 * @param count: Number of bytes to read (maximum)
 * @param byte_timeout_ms: Inter-byte timeout in milliseconds
 *                         (user zero to read only what is already in buffer).
 */
int32_t rs485_receive(uint8_t *buf, uint16_t count, int32_t byte_timeout_ms)
{
  ASSERT_FATAL(uart_handle != NULL, "UART handle is NULL");
  ASSERT_FATAL(buf != NULL, "Buffer is NULL");
  ASSERT_FATAL(count > 0, "Receive byte count zero");

  uint16_t rx_count = 0;

  /* Read up to desired amount of data */
  while (rx_count < count)
  {
    uint32_t result = tx_queue_receive(&rx_queue, &buf[rx_count], TX_TICKS_MS(byte_timeout_ms));
    if (result == TX_SUCCESS)
    {
      rx_count++;
    }
    else if (result == TX_QUEUE_EMPTY)
    {
      /* It's an error only if read was expecting data (had non-zero timeout) */
      if (byte_timeout_ms > 0)
      {
        ULOG_WARNING("RX byte %u timeout", rx_count);
      }
      break;
    }
    else
    {
      ULOG_ERROR("RX queue read error: %u", result);
      break;
    }
  }

  /* Check and report any errors that may have happened during reception */
  rs485_check_irq_errors();

  return rx_count;
}

/**
 * Transmit bytes over RS-485 interface.
 * @param buf: Pointer to transmit buffer.
 * @param count: Number of bytes to write (maximum)
 * @param byte_timeout_ms: Inter-byte timeout in milliseconds. It shall not be zero.
 */
int32_t rs485_transmit(const uint8_t *buf, uint16_t count, int32_t byte_timeout_ms)
{
  ASSERT_FATAL(uart_handle != NULL, "UART handle is NULL");
  ASSERT_FATAL(buf != NULL, "Buffer is NULL");
  ASSERT_FATAL(count > 0, "Transmit byte count zero");
  ASSERT_FATAL(byte_timeout_ms > 0, "Transmit timeout zero");

  uint32_t result;

  rs485_set_transmit_mode();

  /* Need 1.75ms delay between transceiver mode change and first/last transmitted byte.
   * Specification from URL:
   * https://minimalmodbus.readthedocs.io/en/stable/serialcommunication.html */
  sleep_ms(2);

  /* Put data into the transmission queue */
  int32_t tx_count;
  for (tx_count = 0; tx_count < count; tx_count++)
  {
    result = tx_queue_send(&tx_queue, (void *)&buf[tx_count], TX_NO_WAIT);
    if (result != TX_SUCCESS)
    {
      ULOG_ERROR("TX queue byte %i send error: %u", tx_count, result);
      tx_count = 0;
      break;
    }
  }

  /* TX queuing succeeded ? */
  if (tx_count > 0)
  {
    LL_USART_EnableIT_TXE(uart_handle);

    /* Calculate total transmission timeout */
    int32_t total_timeout_ms = tx_count * byte_timeout_ms;

    /* Wait for the TX_COMPLETE event from the event queue */
    SERIAL_TX_EVENT tx_complete_event;
    result = tx_queue_receive(&tx_event_queue, &tx_complete_event, TX_TICKS_MS(total_timeout_ms));
    if (result == TX_SUCCESS)
    {
      if (tx_complete_event.type == TX_COMPLETE)
      {
        /* Success */
      }
      else
      {
        ULOG_ERROR("TX event queue bad type: %u", tx_complete_event.type);
        tx_count = 0;
      }
    }
    else if (result == TX_QUEUE_EMPTY)
    {
      ULOG_ERROR("TX timeout");
      tx_count = 0;
    }
    else
    {
      ULOG_ERROR("TX event queue error: %u", result);
      tx_count = 0;
    }
  }

  sleep_ms(2);
  rs485_set_receive_mode();

  /* Check and report any errors that may have happened during transmission */
  rs485_check_irq_errors();

  return tx_count;
}
