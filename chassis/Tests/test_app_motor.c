/* Wheel speed loop: feedforward near zero speed and correction authority.
 * Motivated by the 2026-10-18 field log: inner wheel kept rolling at 0.02-0.05 m/s
 * with a ~0.007 m/s target, and 0.03 m/s tracking errors produced <1% duty. */
#include "app_motor.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static float duty[MOTOR_MAX];
static float speed[ENCODER_SIDE_MAX];
void motor_bsp_init(void) {}
void motor_bsp_set_speed(motor_id_t motor, float duty_percent) { duty[motor] = duty_percent; }
void motor_bsp_set_dual_pwm(motor_id_t motor, float a, float b) { (void)motor; (void)a; (void)b; }
void motor_bsp_stop(motor_id_t motor, motor_dir_t mode) { (void)mode; duty[motor] = 0.0f; }
float encoder_bsp_velocity_m_s_get(encoder_side_t side) { return speed[side]; }

/* Run the 10 ms loop long enough for the 50 ms speed filter to settle. */
static void run(float left_target, float right_target, float left_meas, float right_meas, int cycles) {
    int i;
    app_motor_set_wheel_velocity_targets(left_target, right_target);
    speed[ENCODER_SIDE_LEFT] = left_meas; speed[ENCODER_SIDE_RIGHT] = right_meas;
    for (i = 0; i < cycles; i++) app_motor_speed_control_update(0.01f);
}

static void test_tiny_target_gets_tiny_feedforward(void) {
    /* Inner wheel of a 0.062 m turn at 0.10 m/s: target ~0.0074 m/s. The full 6.2%
     * start-up offset kept it rolling at several cm/s. */
    app_motor_init(); app_motor_pid_reset();
    run(0.0074f, 0.17f, 0.0074f, 0.17f, 1);
    assert(duty[MOTOR_LEFT] < 2.5f);
    /* Cruising targets keep the full measured offset + slope. */
    app_motor_init(); app_motor_pid_reset();
    run(0.20f, 0.20f, 0.20f, 0.20f, 40);   /* let the 50 ms speed filter settle */
    assert(fabsf(duty[MOTOR_LEFT] - (APP_MOTOR_DUTY_OFFSET_LEFT_PERCENT +
                                      0.20f/APP_MOTOR_SPEED_SLOPE_LEFT_M_S*100.0f)) < 0.5f);
}

static void test_tracking_error_has_real_authority(void) {
    /* Right wheel 0.03 m/s slow (field log): the loop must add clearly more than the
     * 0.75% the old Kp=25 gave, within a few cycles and more with time (integral). */
    float early, late;
    app_motor_init(); app_motor_pid_reset();
    run(0.20f, 0.20f, 0.20f, 0.20f, 30);
    run(0.20f, 0.20f, 0.20f, 0.17f, 10);
    early = duty[MOTOR_RIGHT] - duty[MOTOR_LEFT];
    run(0.20f, 0.20f, 0.20f, 0.17f, 50);
    late = duty[MOTOR_RIGHT] - duty[MOTOR_LEFT];
    printf("right wheel 30 mm/s slow: +%.1f%% duty after 0.1 s, +%.1f%% after 0.6 s\n", early, late);
    assert(early > 2.0f && late > early + 1.0f && late < 20.0f);
}

int main(void) {
    test_tiny_target_gets_tiny_feedforward();
    test_tracking_error_has_real_authority();
    puts("Motor speed loop tests passed: near-zero feedforward, tracking authority.");
    return 0;
}
