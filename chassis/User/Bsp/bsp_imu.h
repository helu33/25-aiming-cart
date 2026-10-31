#ifndef BSP_IMU_H_
#define BSP_IMU_H_

#include <stdint.h>

typedef struct {
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t temperature_raw;
    float accel_m_s2[3];
    float gyro_rad_s[3];
    float temperature_c;
} imu_sample_t;

uint8_t imu_bsp_init(void);
uint8_t imu_bsp_calibrate(uint16_t sample_count, uint16_t sample_delay_ms);
uint8_t imu_bsp_read(imu_sample_t *sample);
uint8_t imu_bsp_ready_get(void);
uint8_t imu_bsp_who_am_i_get(void);

#endif
