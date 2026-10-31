#include "bsp_motor.h"
#include "board_config.h"
#include "tim.h"

typedef struct {
    TIM_HandleTypeDef *timer;
    uint32_t in2_channel;
    uint32_t in1_channel;
    uint8_t invert;
} motor_hw_t;

static const motor_hw_t motors[MOTOR_MAX] = {
    { &htim2, TIM_CHANNEL_1, TIM_CHANNEL_2, BOARD_MOTOR_LEFT_INVERT },
    { &htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, BOARD_MOTOR_RIGHT_INVERT }
};

static float clamp_percent(float value)
{
    if (value > 100.0f) return 100.0f;
    if (value < 0.0f) return 0.0f;
    return value;
}

static uint32_t percent_to_compare(const motor_hw_t *hw, float percent)
{
    const uint32_t period_counts = __HAL_TIM_GET_AUTORELOAD(hw->timer) + 1U;
    return (uint32_t)((clamp_percent(percent) * (float)period_counts) / 100.0f);
}

void motor_bsp_init(void)
{
    (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    motor_bsp_stop(MOTOR_LEFT, MOTOR_DIR_COAST);
    motor_bsp_stop(MOTOR_RIGHT, MOTOR_DIR_COAST);
}

void motor_bsp_set_dual_pwm(motor_id_t motor, float duty_in2_percent, float duty_in1_percent)
{
    const motor_hw_t *hw;
    if (motor >= MOTOR_MAX) return;
    hw = &motors[motor];
    __HAL_TIM_SET_COMPARE(hw->timer, hw->in2_channel, percent_to_compare(hw, duty_in2_percent));
    __HAL_TIM_SET_COMPARE(hw->timer, hw->in1_channel, percent_to_compare(hw, duty_in1_percent));
}

void motor_bsp_set_speed(motor_id_t motor, float duty_percent)
{
    const motor_hw_t *hw;
    if (motor >= MOTOR_MAX) return;
    hw = &motors[motor];
    if (hw->invert != 0U) duty_percent = -duty_percent;
    if (duty_percent > 100.0f) duty_percent = 100.0f;
    if (duty_percent < -100.0f) duty_percent = -100.0f;

    if (duty_percent >= 0.0f) {
        motor_bsp_set_dual_pwm(motor, 100.0f, 100.0f - duty_percent);
    } else {
        motor_bsp_set_dual_pwm(motor, 100.0f + duty_percent, 100.0f);
    }
}

void motor_bsp_stop(motor_id_t motor, motor_dir_t stop_mode)
{
    if (stop_mode == MOTOR_DIR_BRAKE) motor_bsp_set_dual_pwm(motor, 100.0f, 100.0f);
    else motor_bsp_set_dual_pwm(motor, 0.0f, 0.0f);
}
