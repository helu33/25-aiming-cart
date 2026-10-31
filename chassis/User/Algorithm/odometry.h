#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <stdint.h>

typedef struct {
    float wheel_separation_m;
    float x_m;
    float y_m;
    float yaw_rad;
    float linear_velocity_m_s;
    float angular_velocity_rad_s;
    float left_distance_m;
    float right_distance_m;
} odometry_context_t;

/*
 * Coordinate convention:
 *   x: forward, y: left, yaw: counter-clockwise positive.
 * wheel_separation_m is the distance between the left and right wheel
 * contact-point centerlines.
 */
void odometry_init(odometry_context_t *ctx, float wheel_separation_m);
void odometry_reset(odometry_context_t *ctx, float x_m, float y_m, float yaw_rad);

/*
 * Integrate one sample of left/right wheel travel using exact arc integration.
 * Returns 1 when the sample was accepted, otherwise 0 and leaves the pose intact.
 */
uint8_t odometry_update(odometry_context_t *ctx,
                        float left_delta_m,
                        float right_delta_m,
                        float dt_sec);
uint8_t odometry_update_fused(odometry_context_t *ctx,
                              float left_delta_m,
                              float right_delta_m,
                              float gyro_z_rad_s,
                              float gyro_weight,
                              float dt_sec);

/* Fixed-point values recommended for UART transport. */
void odometry_pose_fixed_get(const odometry_context_t *ctx,
                             int32_t *x_mm,
                             int32_t *y_mm,
                             int32_t *yaw_mrad);

#endif
