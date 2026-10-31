#include "app_odometry.h"

#include "board_config.h"
#include "bsp_encoder.h"

static odometry_context_t odometry;
static int32_t previous_left_count;
static int32_t previous_right_count;
static float stationary_elapsed_sec;
static float stationary_gyro_sum_rad_s;
static float residual_gyro_bias_rad_s;
static uint16_t stationary_gyro_sample_count;
static uint8_t stationary_confirmed;

static float absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void stationary_detector_reset(void)
{
    stationary_elapsed_sec = 0.0f;
    stationary_gyro_sum_rad_s = 0.0f;
    stationary_gyro_sample_count = 0U;
    stationary_confirmed = 0U;
}

void app_odometry_init(void)
{
    odometry_init(&odometry, BOARD_WHEEL_SEPARATION_M);
    previous_left_count = encoder_bsp_pulse_count_get(ENCODER_SIDE_LEFT);
    previous_right_count = encoder_bsp_pulse_count_get(ENCODER_SIDE_RIGHT);
    residual_gyro_bias_rad_s = 0.0f;
    stationary_detector_reset();
}

void app_odometry_reset(float x_m, float y_m, float yaw_rad)
{
    odometry_reset(&odometry, x_m, y_m, yaw_rad);
    previous_left_count = encoder_bsp_pulse_count_get(ENCODER_SIDE_LEFT);
    previous_right_count = encoder_bsp_pulse_count_get(ENCODER_SIDE_RIGHT);
    stationary_detector_reset();
}

void app_odometry_update(float gyro_z_rad_s, uint8_t gyro_valid, float dt_sec)
{
    const int32_t left_count = encoder_bsp_pulse_count_get(ENCODER_SIDE_LEFT);
    const int32_t right_count = encoder_bsp_pulse_count_get(ENCODER_SIDE_RIGHT);
    const int32_t left_delta_count = left_count - previous_left_count;
    const int32_t right_delta_count = right_count - previous_right_count;
    const float left_delta_m = (float)left_delta_count * BOARD_METERS_PER_COUNT_LEFT;
    const float right_delta_m = (float)right_delta_count * BOARD_METERS_PER_COUNT_RIGHT;
    uint8_t stationary_candidate = 0U;

    previous_left_count = left_count;
    previous_right_count = right_count;

    if ((gyro_valid != 0U) &&
        (left_delta_count == 0) &&
        (right_delta_count == 0) &&
        (absf(gyro_z_rad_s) <= BOARD_ODOM_STATIONARY_GYRO_MAX_RAD_S)) {
        stationary_candidate = 1U;
    }

    if (stationary_candidate != 0U) {
        if (stationary_confirmed == 0U) {
            stationary_elapsed_sec += dt_sec;
            stationary_gyro_sum_rad_s += gyro_z_rad_s;
            if (stationary_gyro_sample_count < UINT16_MAX) {
                stationary_gyro_sample_count++;
            }

            if (stationary_elapsed_sec >= BOARD_ODOM_STATIONARY_CONFIRM_SEC) {
                if (stationary_gyro_sample_count != 0U) {
                    residual_gyro_bias_rad_s = stationary_gyro_sum_rad_s /
                        (float)stationary_gyro_sample_count;
                }
                stationary_confirmed = 1U;
            }
        } else {
            float bias_alpha = 1.0f;
            if (BOARD_ODOM_GYRO_BIAS_TAU_SEC > 0.0f) {
                bias_alpha = dt_sec /
                    (BOARD_ODOM_GYRO_BIAS_TAU_SEC + dt_sec);
            }
            residual_gyro_bias_rad_s += bias_alpha *
                (gyro_z_rad_s - residual_gyro_bias_rad_s);
        }

        if (stationary_confirmed != 0U) {
            /* Lock heading at rest and report exactly zero angular velocity. */
            (void)odometry_update(&odometry, left_delta_m, right_delta_m, dt_sec);
        } else {
            (void)odometry_update_fused(
                &odometry, left_delta_m, right_delta_m,
                gyro_z_rad_s - residual_gyro_bias_rad_s,
                BOARD_IMU_GYRO_FUSION_WEIGHT, dt_sec);
        }
    } else if (gyro_valid != 0U) {
        stationary_detector_reset();
        (void)odometry_update_fused(&odometry, left_delta_m, right_delta_m,
                                    gyro_z_rad_s - residual_gyro_bias_rad_s,
                                    BOARD_IMU_GYRO_FUSION_WEIGHT, dt_sec);
    } else {
        stationary_detector_reset();
        (void)odometry_update(&odometry, left_delta_m, right_delta_m, dt_sec);
    }
}

const odometry_context_t *app_odometry_state_get(void) { return &odometry; }

void app_odometry_pose_fixed_get(int32_t *x_mm, int32_t *y_mm, int32_t *yaw_mrad)
{
    odometry_pose_fixed_get(&odometry, x_mm, y_mm, yaw_mrad);
}
