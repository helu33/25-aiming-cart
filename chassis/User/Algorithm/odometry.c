#include "odometry.h"

#include <math.h>
#include <stddef.h>

#define ODOMETRY_PI_F              (3.14159265358979323846f)
#define ODOMETRY_TWO_PI_F          (2.0f * ODOMETRY_PI_F)
#define ODOMETRY_MIN_WHEELBASE_M   (1.0e-6f)
#define ODOMETRY_MIN_DELTA_YAW_RAD (1.0e-6f)
#define ODOMETRY_I32_MAX_SAFE_F    (2147483520.0f)
#define ODOMETRY_I32_MIN_F         (-2147483648.0f)

static float yaw_normalize(float yaw_rad)
{
    while (yaw_rad > ODOMETRY_PI_F) yaw_rad -= ODOMETRY_TWO_PI_F;
    while (yaw_rad < -ODOMETRY_PI_F) yaw_rad += ODOMETRY_TWO_PI_F;
    return yaw_rad;
}

static float absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int32_t float_to_i32_saturated(float value)
{
    if (value >= ODOMETRY_I32_MAX_SAFE_F) return INT32_MAX;
    if (value <= ODOMETRY_I32_MIN_F) return INT32_MIN;
    return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

void odometry_init(odometry_context_t *ctx, float wheel_separation_m)
{
    if (ctx == NULL) return;
    ctx->wheel_separation_m = wheel_separation_m;
    odometry_reset(ctx, 0.0f, 0.0f, 0.0f);
}

void odometry_reset(odometry_context_t *ctx, float x_m, float y_m, float yaw_rad)
{
    if (ctx == NULL) return;
    ctx->x_m = x_m;
    ctx->y_m = y_m;
    ctx->yaw_rad = yaw_normalize(yaw_rad);
    ctx->linear_velocity_m_s = 0.0f;
    ctx->angular_velocity_rad_s = 0.0f;
    ctx->left_distance_m = 0.0f;
    ctx->right_distance_m = 0.0f;
}

uint8_t odometry_update(odometry_context_t *ctx,
                        float left_delta_m,
                        float right_delta_m,
                        float dt_sec)
{
    return odometry_update_fused(ctx, left_delta_m, right_delta_m, 0.0f, 0.0f, dt_sec);
}

uint8_t odometry_update_fused(odometry_context_t *ctx,
                              float left_delta_m,
                              float right_delta_m,
                              float gyro_z_rad_s,
                              float gyro_weight,
                              float dt_sec)
{
    float center_delta_m;
    float yaw_delta_rad;
    float yaw_next_rad;

    if ((ctx == NULL) ||
        (ctx->wheel_separation_m <= ODOMETRY_MIN_WHEELBASE_M) ||
        (dt_sec <= 0.0f)) {
        return 0U;
    }

    center_delta_m = 0.5f * (left_delta_m + right_delta_m);
    yaw_delta_rad = (right_delta_m - left_delta_m) / ctx->wheel_separation_m;
    if (gyro_weight < 0.0f) gyro_weight = 0.0f;
    if (gyro_weight > 1.0f) gyro_weight = 1.0f;
    yaw_delta_rad = (1.0f - gyro_weight) * yaw_delta_rad +
                    gyro_weight * gyro_z_rad_s * dt_sec;
    yaw_next_rad = ctx->yaw_rad + yaw_delta_rad;

    if (absf(yaw_delta_rad) < ODOMETRY_MIN_DELTA_YAW_RAD) {
        const float yaw_mid_rad = ctx->yaw_rad + 0.5f * yaw_delta_rad;
        ctx->x_m += center_delta_m * cosf(yaw_mid_rad);
        ctx->y_m += center_delta_m * sinf(yaw_mid_rad);
    } else {
        const float radius_m = center_delta_m / yaw_delta_rad;
        ctx->x_m += radius_m * (sinf(yaw_next_rad) - sinf(ctx->yaw_rad));
        ctx->y_m += radius_m * (cosf(ctx->yaw_rad) - cosf(yaw_next_rad));
    }

    ctx->yaw_rad = yaw_normalize(yaw_next_rad);
    ctx->linear_velocity_m_s = center_delta_m / dt_sec;
    ctx->angular_velocity_rad_s = yaw_delta_rad / dt_sec;
    ctx->left_distance_m += left_delta_m;
    ctx->right_distance_m += right_delta_m;
    return 1U;
}

void odometry_pose_fixed_get(const odometry_context_t *ctx,
                             int32_t *x_mm,
                             int32_t *y_mm,
                             int32_t *yaw_mrad)
{
    if (ctx == NULL) return;
    if (x_mm != NULL) *x_mm = float_to_i32_saturated(ctx->x_m * 1000.0f);
    if (y_mm != NULL) *y_mm = float_to_i32_saturated(ctx->y_m * 1000.0f);
    if (yaw_mrad != NULL) *yaw_mrad = float_to_i32_saturated(ctx->yaw_rad * 1000.0f);
}
