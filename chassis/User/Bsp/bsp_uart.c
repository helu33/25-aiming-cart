#include "bsp_uart.h"
#include "usart.h"
#include "chassis_config.h"
#include "bsp_gimbal_link.h"
#include <string.h>

#if CHASSIS_AUTONOMOUS_ENABLE && CHASSIS_DEBUG_UART == 5
#define DEBUG_UART_HANDLE huart5
#define DEBUG_UART_INSTANCE UART5
#else
#define DEBUG_UART_HANDLE huart2
#define DEBUG_UART_INSTANCE USART2
#endif

#define UART_RX_RING_SIZE 256U
#define UART_TX_RING_SIZE 512U

static uint8_t rx_ring[UART_RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static uint8_t rx_byte;
static uint8_t tx_ring[UART_TX_RING_SIZE];
static volatile uint16_t tx_head;
static volatile uint16_t tx_tail;
static volatile uint16_t tx_inflight;
static volatile uint8_t tx_active;
static volatile uint32_t rx_overflow_count;
/* Every byte / error seen on the debug port: shows in STATUS whether input arrives. */
static volatile uint32_t rx_total_count;
static volatile uint32_t rx_error_count;
static volatile uint32_t tx_drop_count;

static void rx_push(uint8_t byte)
{
    const uint16_t next = (uint16_t)((rx_head + 1U) % UART_RX_RING_SIZE);
    if (next == rx_tail) {
        rx_overflow_count++;
        return;
    }
    rx_ring[rx_head] = byte;
    rx_head = next;
}

static void tx_start_if_idle(void)
{
    uint16_t length;
    if ((tx_active != 0U) || (tx_head == tx_tail)) return;
    length = (tx_head > tx_tail) ? (uint16_t)(tx_head - tx_tail) :
                                  (uint16_t)(UART_TX_RING_SIZE - tx_tail);
    tx_inflight = length;
    tx_active = 1U;
    if (HAL_UART_Transmit_IT(&DEBUG_UART_HANDLE, &tx_ring[tx_tail], length) != HAL_OK) {
        tx_active = 0U;
        tx_inflight = 0U;
    }
}

void uart_bsp_init(void)
{
    rx_head = 0U;
    rx_tail = 0U;
    tx_head = 0U;
    tx_tail = 0U;
    tx_inflight = 0U;
    tx_active = 0U;
    rx_overflow_count = 0U;
    rx_total_count = 0U;
    rx_error_count = 0U;
    tx_drop_count = 0U;
    if (HAL_UART_Receive_IT(&DEBUG_UART_HANDLE, &rx_byte, 1U) != HAL_OK) Error_Handler();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == GIMBAL_LINK_INSTANCE) { gimbal_link_tx_complete(huart); return; }
    if (huart->Instance == DEBUG_UART_INSTANCE) {
        tx_tail = (uint16_t)((tx_tail + tx_inflight) % UART_TX_RING_SIZE);
        tx_inflight = 0U;
        tx_active = 0U;
        tx_start_if_idle();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == GIMBAL_LINK_INSTANCE) { gimbal_link_rx_complete(huart); return; }
    if (huart->Instance == DEBUG_UART_INSTANCE) {
        rx_total_count++;
        rx_push(rx_byte);
        (void)HAL_UART_Receive_IT(&DEBUG_UART_HANDLE, &rx_byte, 1U);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == GIMBAL_LINK_INSTANCE) { gimbal_link_error(huart); return; }
    if (huart->Instance == DEBUG_UART_INSTANCE) {
        rx_error_count++;
        (void)HAL_UART_Receive_IT(&DEBUG_UART_HANDLE, &rx_byte, 1U);
    }
}

HAL_StatusTypeDef uart_bsp_write(const uint8_t *data, uint16_t length)
{
    uint16_t used;
    uint16_t free_space;
    uint16_t i;
    uint32_t primask;
    if ((data == NULL) && (length != 0U)) return HAL_ERROR;
    if (length == 0U) return HAL_OK;
    primask = __get_PRIMASK();
    __disable_irq();
    used = (uint16_t)((tx_head - tx_tail + UART_TX_RING_SIZE) % UART_TX_RING_SIZE);
    free_space = (uint16_t)(UART_TX_RING_SIZE - 1U - used);
    if (length > free_space) {
        tx_drop_count++;
        if (primask == 0U) __enable_irq();
        return HAL_BUSY;
    }
    for (i = 0U; i < length; i++) {
        tx_ring[tx_head] = data[i];
        tx_head = (uint16_t)((tx_head + 1U) % UART_TX_RING_SIZE);
    }
    tx_start_if_idle();
    if (primask == 0U) __enable_irq();
    return HAL_OK;
}

HAL_StatusTypeDef uart_bsp_write_str(const char *text)
{
    if (text == NULL) return HAL_ERROR;
    return uart_bsp_write((const uint8_t *)text, (uint16_t)strlen(text));
}

uint32_t uart_bsp_rx_count(void)
{
    return (uint32_t)((rx_head - rx_tail + UART_RX_RING_SIZE) % UART_RX_RING_SIZE);
}

uint8_t uart_bsp_read_byte(uint8_t *out)
{
    if ((out == NULL) || (rx_head == rx_tail)) return 0U;
    *out = rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % UART_RX_RING_SIZE);
    return 1U;
}

void uart_bsp_rx_flush(void)
{
    rx_tail = rx_head;
}

uint32_t uart_bsp_rx_overflow_count_get(void) { return rx_overflow_count; }
uint32_t uart_bsp_rx_total_get(void) { return rx_total_count; }
uint32_t uart_bsp_rx_error_count_get(void) { return rx_error_count; }
uint32_t uart_bsp_tx_drop_count_get(void) { return tx_drop_count; }
