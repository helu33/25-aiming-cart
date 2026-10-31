#ifndef APP_MOTOR_H_
#define APP_MOTOR_H_

#include <stdint.h>
#include "bsp_encoder.h"

/* Originally 1.306 / 1.279, measured with the 0.204203 m circumference; rescaled
 * by the measured circumference ratio so the same duty gives the same wheel speed. */
#define APP_MOTOR_SPEED_SLOPE_LEFT_M_S         (1.4471f)
#define APP_MOTOR_SPEED_SLOPE_RIGHT_M_S        (1.4503f)
#define APP_MOTOR_DUTY_OFFSET_LEFT_PERCENT     (6.2f)
#define APP_MOTOR_DUTY_OFFSET_RIGHT_PERCENT    (7.625f)
/* Start-up offset fades in linearly from 0 to full over this target speed. */
#define APP_MOTOR_FF_OFFSET_RAMP_M_S           (0.05f)
#define APP_MOTOR_TURN_START_BOOST_LEFT_PERCENT  (2.0f)
#define APP_MOTOR_TURN_START_BOOST_RIGHT_PERCENT (2.5f)
#define APP_MOTOR_TURN_START_MIN_TARGET_M_S    (0.01f)
#define APP_MOTOR_TURN_START_STILL_SPEED_M_S   (0.01f)
/* 2026-10-18 field log: Kp 25 / Ki 20 left 25-29 mm/s wheel tracking error, enough to
 * swallow the steering differential. Loop gain ~0.36 -> ~1.2 (plant ~0.0145 m/s per %). */
#define APP_MOTOR_SPEED_PID_KP_DEFAULT        (80.0f)
#define APP_MOTOR_SPEED_PID_KI_DEFAULT        (100.0f)
#define APP_MOTOR_SPEED_PID_KD_DEFAULT        (0.0f)
#define APP_MOTOR_SPEED_PID_INTEGRAL_LIMIT    (0.12f)
#define APP_MOTOR_SPEED_PID_OUT_LIMIT         (22.0f)
#define APP_MOTOR_SPEED_MEAS_LPF_TAU_SEC      (0.05f)

void app_motor_init(void);
void app_motor_stop(void);
//设置电机的差速控制，输入为左轮和右轮的占空比百分比（-100%到100%）。
void app_motor_set_differential_manual(float left_percent, float right_percent);
void app_motor_set_wheel_velocity_targets(float left_m_s, float right_m_s);
void app_motor_set_chassis_velocity(float linear_x_m_s, float angular_z_rad_s);
void app_motor_speed_control_update(float dt_sec);
void app_motor_pid_reset(void);
void app_motor_set_speed_pid_gains(float kp, float ki, float kd, float integral_limit);
float app_motor_last_speed_err_left_get(void);
float app_motor_last_speed_err_right_get(void);
float app_motor_speed_meas_raw_m_s_get(encoder_side_t side);
float app_motor_speed_meas_filtered_m_s_get(encoder_side_t side);
float app_motor_speed_target_m_s_get(encoder_side_t side);
float app_motor_last_duty_percent_get(encoder_side_t side);

#endif
