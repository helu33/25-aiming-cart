#ifndef TEST_PLATFORM_H
#define TEST_PLATFORM_H
#include <stdint.h>
#include <stddef.h>
#define UNUSED(x) ((void)(x))
typedef uint32_t TickType_t;
typedef struct { unsigned id; } FDCAN_HandleTypeDef;
typedef struct { uint32_t Identifier, IdType, RxFrameType, FDFormat, DataLength; } FDCAN_RxHeaderTypeDef;
typedef struct { uint32_t RxState; unsigned id; uint32_t gState; } UART_HandleTypeDef;
extern FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
extern UART_HandleTypeDef huart1, huart2, huart7, huart10;
#define FDCAN_STANDARD_ID 0U
#define FDCAN_DATA_FRAME 0U
#define FDCAN_CLASSIC_CAN 0U
#define FDCAN_DLC_BYTES_8 8U
#define HAL_UART_STATE_READY 0U
#define HAL_OK 0
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define pdMS_TO_TICKS(x) (x)
#define __HAL_UART_CLEAR_OREFLAG(x) ((void)(x))
#define __HAL_UART_CLEAR_NEFLAG(x) ((void)(x))
#define __HAL_UART_CLEAR_FEFLAG(x) ((void)(x))
#define __HAL_UART_CLEAR_PEFLAG(x) ((void)(x))
uint32_t HAL_GetTick(void);
void osDelay(uint32_t ms);
TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *last, TickType_t delta);
void can_bsp_init(void);
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *, uint16_t, uint8_t *, uint32_t);
void BSP_Buzzer_Init(void);
void BSP_Buzzer_On(void);
void BSP_Buzzer_Off(void);
int HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
int HAL_UART_AbortReceive(UART_HandleTypeDef *);
int HAL_UART_Transmit_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
#endif
