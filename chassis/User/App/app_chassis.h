#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H
#include "bsp_imu.h"
#include "chassis_mission.h"
void app_chassis_init(uint8_t imu_calibrated);
void app_chassis_poll(void);
void app_chassis_control(float measurement_dt, float pid_dt, const imu_sample_t *imu, uint8_t valid);
uint8_t app_chassis_start(uint8_t laps);
void app_chassis_stop(void);
const chassis_mission_t *app_chassis_state(void);
#endif
