#ifndef BSP_UART_H_
#define BSP_UART_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"

void uart_bsp_init(void);
HAL_StatusTypeDef uart_bsp_write(const uint8_t *data, uint16_t length);
HAL_StatusTypeDef uart_bsp_write_str(const char *text);
uint32_t uart_bsp_rx_count(void);
uint8_t uart_bsp_read_byte(uint8_t *out);
void uart_bsp_rx_flush(void);
uint32_t uart_bsp_rx_overflow_count_get(void);
uint32_t uart_bsp_rx_total_get(void);
uint32_t uart_bsp_rx_error_count_get(void);
uint32_t uart_bsp_tx_drop_count_get(void);

#endif
