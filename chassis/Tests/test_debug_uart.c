#include "bsp_uart.h"
#include "bsp_gimbal_link.h"
#include "chassis_config.h"
#include "usart.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart2={USART2}, huart4={UART4}, huart5={UART5};
#if CHASSIS_AUTONOMOUS_ENABLE && CHASSIS_DEBUG_UART == 5
#define SELECTED (&huart5)
#define OTHER (&huart4)   /* neither the debug port nor the gimbal link (USART2) */
#else
#define SELECTED (&huart2)
#define OTHER (&huart5)
#endif
static uint8_t *rx_destination;
static const uint8_t *tx_data;
static uint16_t tx_length;
static unsigned tx_calls, rx_calls, gimbal_complete, gimbal_errors;
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);
void Error_Handler(void) { assert(0); }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *uart,uint8_t *data,uint16_t length) {
    assert(uart==SELECTED && length==1);
    rx_destination=data; rx_calls++; return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart,uint8_t *data,uint16_t length) {
    assert(uart==SELECTED && length>0);
    tx_data=data; tx_length=length; tx_calls++; return HAL_OK;
}
void gimbal_link_tx_complete(UART_HandleTypeDef *uart) { assert(uart==&GIMBAL_LINK_UART); gimbal_complete++; }
void gimbal_link_error(UART_HandleTypeDef *uart) { assert(uart==&GIMBAL_LINK_UART); gimbal_errors++; }
static unsigned gimbal_rx;
void gimbal_link_rx_complete(UART_HandleTypeDef *uart) { assert(uart==&GIMBAL_LINK_UART); gimbal_rx++; }
int main(void) {
    uint8_t out;
    uart_bsp_init(); assert(rx_calls==1 && uart_bsp_rx_count()==0);
    assert(uart_bsp_write_str("AUTO\n")==HAL_OK && tx_calls==1);
    assert(tx_length==5 && !memcmp(tx_data,"AUTO\n",5));
    assert(uart_bsp_write_str("STATUS\n")==HAL_OK && tx_calls==1);
    HAL_UART_TxCpltCallback(OTHER); assert(tx_calls==1);
    HAL_UART_TxCpltCallback(&GIMBAL_LINK_UART); assert(gimbal_complete==1 && tx_calls==1);
    assert(!memcmp(tx_data,"AUTO\n",5));
    HAL_UART_TxCpltCallback(SELECTED);
    assert(tx_calls==2 && tx_length==7 && !memcmp(tx_data,"STATUS\n",7));
    HAL_UART_TxCpltCallback(SELECTED);
    *rx_destination='S'; HAL_UART_RxCpltCallback(OTHER);
    assert(rx_calls==1 && uart_bsp_rx_count()==0);
    HAL_UART_RxCpltCallback(SELECTED);
    assert(rx_calls==2 && uart_bsp_read_byte(&out) && out=='S');
    assert(!uart_bsp_read_byte(&out));
    HAL_UART_RxCpltCallback(&GIMBAL_LINK_UART);           /* $G bytes go to the gimbal link */
    assert(gimbal_rx==1 && rx_calls==2 && uart_bsp_rx_count()==0);
    HAL_UART_ErrorCallback(OTHER); assert(rx_calls==2);
    HAL_UART_ErrorCallback(SELECTED); assert(rx_calls==3);
    HAL_UART_ErrorCallback(&GIMBAL_LINK_UART); assert(gimbal_errors==1 && rx_calls==3);
    /* Counters prove whether bytes reach the board: only the debug port counts. */
    assert(uart_bsp_rx_total_get()==1 && uart_bsp_rx_error_count_get()==1);
    for (unsigned i=0;i<260;i++) { *rx_destination='A'; HAL_UART_RxCpltCallback(SELECTED); }
    assert(uart_bsp_rx_count()==255 && uart_bsp_rx_overflow_count_get()==5);
    assert(uart_bsp_rx_total_get()==261);   /* counted even when the ring is full */
    uart_bsp_rx_flush(); assert(uart_bsp_rx_count()==0);
    puts("Debug UART tests passed: port routing, TX queue lifetime, RX rearm, overflow, gimbal isolation.");
    return 0;
}
