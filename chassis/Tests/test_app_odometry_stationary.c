#include "app_odometry.h"
#include "board_config.h"
#include "bsp_encoder.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define TEST_DT_SEC                (0.01f)
#define TEST_RESIDUAL_BIAS_RAD_S   (-0.00452f)
#define TEST_STATIC_SAMPLES        (6000U)
#define TEST_ROTATION_COUNTS       (10)
#define TEST_STATIC_YAW_LIMIT_RAD  (0.001f)
#define TEST_ROTATION_ERROR_RAD    (0.0001f)

static int32_t encoder_counts[ENCODER_SIDE_MAX];

int32_t encoder_bsp_pulse_count_get(encoder_side_t side)
{
    if (side >= ENCODER_SIDE_MAX) return 0;
    return encoder_counts[side];
}

static void require_close(const char *name,
                          float actual,
                          float expected,
                          float tolerance)
{
    if (fabsf(actual - expected) > tolerance) {
        (void)fprintf(stderr, "%s: actual=%f expected=%f tolerance=%f\n",
                      name, actual, expected, tolerance);
        exit(EXIT_FAILURE);
    }
}

int main(void)
{
    const odometry_context_t *state;
    float yaw_before_rotation;
    float yaw_after_rotation;
    float yaw_after_second_stop;
    float expected_yaw_delta;
    float expected_gyro_rate;
    uint32_t sample;

    encoder_counts[ENCODER_SIDE_LEFT] = 0;
    encoder_counts[ENCODER_SIDE_RIGHT] = 0;
    app_odometry_init();

    for (sample = 0U; sample < TEST_STATIC_SAMPLES; sample++) {
        app_odometry_update(TEST_RESIDUAL_BIAS_RAD_S, 1U, TEST_DT_SEC);
    }

    state = app_odometry_state_get();
    require_close("static angular velocity",
                  state->angular_velocity_rad_s, 0.0f, 1.0e-7f);
    if (fabsf(state->yaw_rad) > TEST_STATIC_YAW_LIMIT_RAD) {
        (void)fprintf(stderr, "static yaw drift too large: %f rad\n",
                      state->yaw_rad);
        return EXIT_FAILURE;
    }

    yaw_before_rotation = state->yaw_rad;
    encoder_counts[ENCODER_SIDE_LEFT] -= TEST_ROTATION_COUNTS;
    encoder_counts[ENCODER_SIDE_RIGHT] += TEST_ROTATION_COUNTS;
    expected_yaw_delta =
        (2.0f * (float)TEST_ROTATION_COUNTS *
         BOARD_WHEEL_CIRCUMFERENCE_M /
         (float)BOARD_ENCODER_COUNTS_PER_WHEEL_REV) /
        BOARD_WHEEL_SEPARATION_M;
    expected_gyro_rate = expected_yaw_delta / TEST_DT_SEC;
    app_odometry_update(expected_gyro_rate + TEST_RESIDUAL_BIAS_RAD_S,
                        1U, TEST_DT_SEC);

    state = app_odometry_state_get();
    yaw_after_rotation = state->yaw_rad;
    require_close("rotation yaw delta",
                  yaw_after_rotation - yaw_before_rotation,
                  expected_yaw_delta,
                  TEST_ROTATION_ERROR_RAD);

    for (sample = 0U; sample < TEST_STATIC_SAMPLES; sample++) {
        app_odometry_update(TEST_RESIDUAL_BIAS_RAD_S, 1U, TEST_DT_SEC);
    }

    state = app_odometry_state_get();
    yaw_after_second_stop = state->yaw_rad;
    require_close("yaw after second stop",
                  yaw_after_second_stop,
                  yaw_after_rotation,
                  TEST_ROTATION_ERROR_RAD);
    require_close("second static angular velocity",
                  state->angular_velocity_rad_s, 0.0f, 1.0e-7f);

    (void)printf("stationary odometry test passed\n");
    return EXIT_SUCCESS;
}
