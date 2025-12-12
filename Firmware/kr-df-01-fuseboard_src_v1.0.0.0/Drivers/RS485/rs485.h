/**
 * @brief UART messaging functionality with rs485 direction control
 */

#pragma once

#include <stdint.h>
#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rs485_init(USART_TypeDef *uart);
void rs485_uart_irq();
int32_t rs485_receive(uint8_t *buf, uint16_t count, int32_t byte_timeout_ms);
int32_t rs485_transmit(const uint8_t *buf, uint16_t count, int32_t byte_timeout_ms);

#ifdef __cplusplus
}
#endif
