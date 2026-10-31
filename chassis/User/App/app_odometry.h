#ifndef APP_ODOMETRY_H_
#define APP_ODOMETRY_H_

#include <stdint.h>
#include "odometry.h"

void app_odometry_init(void);
void app_odometry_reset(float x_m, float y_m, float yaw_rad);
void app_odometry_update(float gyro_z_rad_s, uint8_t gyro_valid, float dt_sec);
const odometry_context_t *app_odometry_state_get(void);
void app_odometry_pose_fixed_get(int32_t *x_mm, int32_t *y_mm, int32_t *yaw_mrad);

#endif
