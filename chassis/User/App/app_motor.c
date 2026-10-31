#include "app_motor.h"
#include "board_config.h"
#include "filter.h"
#include "pid.h"
#include "bsp_motor.h"

static pid_context_t speed_pid_left;
static pid_context_t speed_pid_right;
static filter_lpf_1st_f32_t speed_lpf_left;
static filter_lpf_1st_f32_t speed_lpf_right;
static uint8_t speed_lpf_initialized;
static float last_speed_error_left;
static float last_speed_error_right;
static float filtered_speed_left;
static float filtered_speed_right;
static float target_speed_left;
static float target_speed_right;
static float last_duty_left;
static float last_duty_right;

//限制幅度在
static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static float absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float speed_feedforward_percent(float target_m_s,
                                       float speed_slope_m_s,
                                       float duty_offset_percent)
{
    float offset;
    if ((target_m_s == 0.0f) || (speed_slope_m_s <= 0.0f)) return 0.0f;
    /* Fade the start-up offset in over the first APP_MOTOR_FF_OFFSET_RAMP_M_S so a
     * near-zero target (inner wheel of a tight turn) really slows the wheel down. */
    offset = duty_offset_percent;
    if (absf(target_m_s) < APP_MOTOR_FF_OFFSET_RAMP_M_S) {
        offset *= absf(target_m_s) / APP_MOTOR_FF_OFFSET_RAMP_M_S;
    }
    if (target_m_s > 0.0f) {
        return offset + (target_m_s / speed_slope_m_s) * 100.0f;
    }
    return -offset + (target_m_s / speed_slope_m_s) * 100.0f;
}

static float turn_start_boost_percent(float target_m_s,
                                      float measured_m_s,
                                      float boost_percent)
{
    if (absf(target_m_s) < APP_MOTOR_TURN_START_MIN_TARGET_M_S) {
        return 0.0f;
    }
    if (absf(measured_m_s) >= APP_MOTOR_TURN_START_STILL_SPEED_M_S) {
        return 0.0f;
    }
    return (target_m_s > 0.0f) ?
        boost_percent :
        -boost_percent;
}

static void speed_filters_reset(void)
{
    speed_lpf_initialized = 0U;
    filtered_speed_left = 0.0f;
    filtered_speed_right = 0.0f;
    filter_lpf_1st_f32_reset(&speed_lpf_left, 0.0f);
    filter_lpf_1st_f32_reset(&speed_lpf_right, 0.0f);
}

void app_motor_init(void)
{
    motor_bsp_init();
    pid_init(&speed_pid_left, APP_MOTOR_SPEED_PID_KP_DEFAULT, APP_MOTOR_SPEED_PID_KI_DEFAULT,
             APP_MOTOR_SPEED_PID_KD_DEFAULT, APP_MOTOR_SPEED_PID_INTEGRAL_LIMIT,
             -APP_MOTOR_SPEED_PID_OUT_LIMIT, APP_MOTOR_SPEED_PID_OUT_LIMIT);
    pid_init(&speed_pid_right, APP_MOTOR_SPEED_PID_KP_DEFAULT, APP_MOTOR_SPEED_PID_KI_DEFAULT,
             APP_MOTOR_SPEED_PID_KD_DEFAULT, APP_MOTOR_SPEED_PID_INTEGRAL_LIMIT,
             -APP_MOTOR_SPEED_PID_OUT_LIMIT, APP_MOTOR_SPEED_PID_OUT_LIMIT);
    speed_filters_reset();
    target_speed_left = 0.0f;
    target_speed_right = 0.0f;
    last_duty_left = 0.0f;
    last_duty_right = 0.0f;
}

//设定对应的pid数值
void app_motor_set_speed_pid_gains(float kp, float ki, float kd, float integral_limit)
{
    speed_pid_left.kp = kp; speed_pid_left.ki = ki; speed_pid_left.kd = kd; speed_pid_left.integral_limit = integral_limit;
    speed_pid_right.kp = kp; speed_pid_right.ki = ki; speed_pid_right.kd = kd; speed_pid_right.integral_limit = integral_limit;
}

//重置pid
void app_motor_pid_reset(void)
{
    pid_reset(&speed_pid_left);
    pid_reset(&speed_pid_right);
    speed_filters_reset();
}

//停车并清零
void app_motor_stop(void)
{
    target_speed_left = 0.0f;
    target_speed_right = 0.0f;
    last_duty_left = 0.0f;
    last_duty_right = 0.0f;
    pid_reset(&speed_pid_left);
    pid_reset(&speed_pid_right);
    speed_filters_reset();
    motor_bsp_stop(MOTOR_LEFT, MOTOR_DIR_BRAKE);
    motor_bsp_stop(MOTOR_RIGHT, MOTOR_DIR_BRAKE);
}

void app_motor_set_differential_manual(float left, float right)
{
    last_duty_left = clampf(left, -100.0f, 100.0f);
    last_duty_right = clampf(right, -100.0f, 100.0f);
    motor_bsp_set_speed(MOTOR_LEFT, last_duty_left);
    motor_bsp_set_speed(MOTOR_RIGHT, last_duty_right);
}

void app_motor_set_wheel_velocity_targets(float left_m_s, float right_m_s)
{
    target_speed_left = clampf(left_m_s, -BOARD_MAX_WHEEL_SPEED_M_S, BOARD_MAX_WHEEL_SPEED_M_S);
    target_speed_right = clampf(right_m_s, -BOARD_MAX_WHEEL_SPEED_M_S, BOARD_MAX_WHEEL_SPEED_M_S);
}


void app_motor_set_chassis_velocity(float linear_x_m_s, float angular_z_rad_s)
{
    float left;
    float right;
    float peak;

    linear_x_m_s = clampf(linear_x_m_s, -BOARD_MAX_LINEAR_SPEED_M_S, BOARD_MAX_LINEAR_SPEED_M_S);
    angular_z_rad_s = clampf(angular_z_rad_s, -BOARD_MAX_ANGULAR_SPEED_RAD_S,
                             BOARD_MAX_ANGULAR_SPEED_RAD_S);
    left = linear_x_m_s - 0.5f * BOARD_WHEEL_SEPARATION_M * angular_z_rad_s;
    right = linear_x_m_s + 0.5f * BOARD_WHEEL_SEPARATION_M * angular_z_rad_s;
    peak = (left < 0.0f) ? -left : left;
    if (((right < 0.0f) ? -right : right) > peak) peak = (right < 0.0f) ? -right : right;
    if (peak > BOARD_MAX_WHEEL_SPEED_M_S) {
        const float scale = BOARD_MAX_WHEEL_SPEED_M_S / peak;
        left *= scale;
        right *= scale;
    }
    app_motor_set_wheel_velocity_targets(left, right);
}


//更新电机速度控制
void app_motor_speed_control_update(float dt_sec)
{
    float feedforward_left;
    float feedforward_right;
    float duty_left;
    float duty_right;
    uint8_t in_place_turn;

    if (dt_sec <= 0.0f) return;
    if (speed_lpf_initialized == 0U) {
        /* Start from the measured speed: starting at 0 fakes a tracking error after every
         * reset and winds up the integral (now that Ki is meaningful). */
        filter_lpf_1st_f32_init_tau(&speed_lpf_left, dt_sec, APP_MOTOR_SPEED_MEAS_LPF_TAU_SEC,
                                    encoder_bsp_velocity_m_s_get(ENCODER_SIDE_LEFT));
        filter_lpf_1st_f32_init_tau(&speed_lpf_right, dt_sec, APP_MOTOR_SPEED_MEAS_LPF_TAU_SEC,
                                    encoder_bsp_velocity_m_s_get(ENCODER_SIDE_RIGHT));
        speed_lpf_initialized = 1U;
    }
    //待会要看
    filtered_speed_left = filter_lpf_1st_f32_run(
        &speed_lpf_left, encoder_bsp_velocity_m_s_get(ENCODER_SIDE_LEFT));
    filtered_speed_right = filter_lpf_1st_f32_run(
        &speed_lpf_right, encoder_bsp_velocity_m_s_get(ENCODER_SIDE_RIGHT));

    if ((target_speed_left == 0.0f) && (target_speed_right == 0.0f)) {
        pid_reset(&speed_pid_left);
        pid_reset(&speed_pid_right);
        last_speed_error_left = 0.0f;
        last_speed_error_right = 0.0f;
        last_duty_left = 0.0f;
        last_duty_right = 0.0f;
        motor_bsp_set_speed(MOTOR_LEFT, 0.0f);
        motor_bsp_set_speed(MOTOR_RIGHT, 0.0f);
        return;
    }

    last_speed_error_left = target_speed_left - filtered_speed_left;
    last_speed_error_right = target_speed_right - filtered_speed_right;
    feedforward_left = speed_feedforward_percent(target_speed_left,
                                                 APP_MOTOR_SPEED_SLOPE_LEFT_M_S,
                                                 APP_MOTOR_DUTY_OFFSET_LEFT_PERCENT);
    feedforward_right = speed_feedforward_percent(target_speed_right,
                                                  APP_MOTOR_SPEED_SLOPE_RIGHT_M_S,
                                                  APP_MOTOR_DUTY_OFFSET_RIGHT_PERCENT);
    duty_left = feedforward_left +
        pid_compute(&speed_pid_left, last_speed_error_left, dt_sec);
    duty_right = feedforward_right +
        pid_compute(&speed_pid_right, last_speed_error_right, dt_sec);

    /*
     * In-place turns load the wheels more heavily than straight driving.
     * Add a small signed boost only while opposite wheel targets are
     * requested and the corresponding wheel is still stationary. The boost
     * is removed as soon as encoder feedback shows that the wheel is moving.
     */
    in_place_turn = (((target_speed_left > 0.0f) &&
                      (target_speed_right < 0.0f)) ||
                     ((target_speed_left < 0.0f) &&
                      (target_speed_right > 0.0f))) ? 1U : 0U;
    if (in_place_turn != 0U) {
        duty_left += turn_start_boost_percent(
            target_speed_left, filtered_speed_left,
            APP_MOTOR_TURN_START_BOOST_LEFT_PERCENT);
        duty_right += turn_start_boost_percent(
            target_speed_right, filtered_speed_right,
            APP_MOTOR_TURN_START_BOOST_RIGHT_PERCENT);
    }

    last_duty_left = clampf(duty_left, -100.0f, 100.0f);
    last_duty_right = clampf(duty_right, -100.0f, 100.0f);
    motor_bsp_set_speed(MOTOR_LEFT, last_duty_left);
    motor_bsp_set_speed(MOTOR_RIGHT, last_duty_right);
}

float app_motor_last_speed_err_left_get(void) { return last_speed_error_left; }
float app_motor_last_speed_err_right_get(void) { return last_speed_error_right; }
float app_motor_speed_meas_raw_m_s_get(encoder_side_t side) { return encoder_bsp_velocity_m_s_get(side); }
float app_motor_speed_meas_filtered_m_s_get(encoder_side_t side)
{
    if (side == ENCODER_SIDE_LEFT) return filtered_speed_left;
    if (side == ENCODER_SIDE_RIGHT) return filtered_speed_right;
    return 0.0f;
}
float app_motor_speed_target_m_s_get(encoder_side_t side)
{
    if (side == ENCODER_SIDE_LEFT) return target_speed_left;
    if (side == ENCODER_SIDE_RIGHT) return target_speed_right;
    return 0.0f;
}
float app_motor_last_duty_percent_get(encoder_side_t side)
{
    if (side == ENCODER_SIDE_LEFT) return last_duty_left;
    if (side == ENCODER_SIDE_RIGHT) return last_duty_right;
    return 0.0f;
}
