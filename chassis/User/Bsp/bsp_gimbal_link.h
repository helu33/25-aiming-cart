#ifndef BSP_GIMBAL_LINK_H
#define BSP_GIMBAL_LINK_H
#include "stm32f1xx_hal.h"
#include "usart.h"
#include "chassis_config.h"
#if CHASSIS_GIMBAL_UART == 2
#define GIMBAL_LINK_UART      huart2
#define GIMBAL_LINK_INSTANCE  USART2
#else
#define GIMBAL_LINK_UART      huart4
#define GIMBAL_LINK_INSTANCE  UART4
#endif
void gimbal_link_init(void);
void gimbal_link_poll(uint8_t hold_vision);
/* Latest chassis gyro z (deg/s, CCW +) for the next frames; valid=0 sends imu_valid=0. */
void gimbal_link_set_gyro(uint8_t valid, float gz_deg_s);
/* Latest chassis forward speed from the encoders (m/s, clamped to +/-3) for the next frames. */
void gimbal_link_set_speed(float speed_m_s);
/* Circle task flag and run progress (1/1000 lap, clamped to 9999) for the next frames ($C v3). */
void gimbal_link_set_task(uint8_t circle, uint32_t permille);
void gimbal_link_tx_complete(UART_HandleTypeDef *uart);
void gimbal_link_error(UART_HandleTypeDef *uart);
uint8_t gimbal_link_healthy(void); /* Local TX completion only, NOT remote ACK. */
uint32_t gimbal_link_failures(void);
/* Gimbal -> chassis status ($G, chassis_decode_gimbal_status) on the same UART's RX. */
void gimbal_link_rx_complete(UART_HandleTypeDef *uart); /* HAL RX callback, one byte */
void gimbal_link_rx_poll(void);            /* main loop: assemble and parse received lines */
uint8_t gimbal_link_ready(void);           /* last $G (within CHASSIS_GIMBAL_STATUS_TIMEOUT_MS) READY */
uint8_t gimbal_link_status_flags(void);    /* flags of the last valid $G, 0 before any */
uint32_t gimbal_link_status_frames(void);  /* valid $G frames received */
uint32_t gimbal_link_rx_errors(void);
#endif
