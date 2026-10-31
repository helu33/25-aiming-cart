#include "usart.h"
#include "chassis_config.h"

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

static void uart_common_init(UART_HandleTypeDef *huart, uint32_t baud)
{
    huart->Init.BaudRate = baud;
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    huart->Init.StopBits = UART_STOPBITS_1;
    huart->Init.Parity = UART_PARITY_NONE;
    huart->Init.Mode = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(huart) != HAL_OK) {
        Error_Handler();
    }
}

void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    /* 921600 when it carries the gimbal link (CHASSIS_GIMBAL_UART), else the 115200 port. */
    uart_common_init(&huart2, CHASSIS_GIMBAL_UART == 2 ? 921600U : 115200U);
}

void MX_UART4_Init(void)
{
    huart4.Instance = UART4;
    uart_common_init(&huart4, 921600U);
}

void MX_UART5_Init(void)
{
    huart5.Instance = UART5;
    uart_common_init(&huart5, 115200U);
}
