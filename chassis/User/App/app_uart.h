#ifndef APP_UART_H_
#define APP_UART_H_

#include <stdint.h>
#include "bsp_imu.h"

#define CAR_FAULT_CMD_TIMEOUT   (1U << 0)
#define CAR_FAULT_IMU           (1U << 1)
#define CAR_FAULT_RX_CRC        (1U << 2)
#define CAR_FAULT_RX_OVERFLOW   (1U << 3)
#define CAR_FAULT_CONTROL_LATE  (1U << 4)

void car_uart_command_poll(void);
uint8_t car_uart_halted_get(void);
void car_uart_fault_set(uint16_t fault_mask, uint8_t active);
uint16_t car_uart_fault_get(void);
void car_uart_publish_telemetry(const imu_sample_t *imu, uint8_t imu_valid);

#endif
