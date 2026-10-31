#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
typedef struct { void *Instance; } UART_HandleTypeDef;
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct { uint32_t Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;
typedef struct { uint32_t CYCCNT, CTRL; } TestDWT;
typedef struct { uint32_t DEMCR; } TestCoreDebug;
extern TestDWT test_dwt;
extern TestCoreDebug test_debug;
extern uint32_t SystemCoreClock;
#define DWT (&test_dwt)
#define CoreDebug (&test_debug)
#define CoreDebug_DEMCR_TRCENA_Msk 1U
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define GPIOB ((void *)2)
#define GPIOC ((void *)3)
#define GPIO_PIN_0 1U
#define GPIO_PIN_1 2U
#define GPIO_PIN_2 4U
#define GPIO_MODE_OUTPUT_PP 1U
#define GPIO_MODE_INPUT 2U
#define GPIO_SPEED_FREQ_LOW 0U
#define GPIO_PULLUP 1U
#define UART4 ((void *)4)
#define UART5 ((void *)5)
#define USART2 ((void *)6)
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
uint32_t HAL_GetTick(void);
void HAL_GPIO_WritePin(void *,uint32_t,GPIO_PinState);
void HAL_GPIO_Init(void *,GPIO_InitTypeDef *);
GPIO_PinState HAL_GPIO_ReadPin(void *,uint32_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *,uint8_t *,uint16_t);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *,uint8_t *,uint16_t);
void Error_Handler(void);
#endif
