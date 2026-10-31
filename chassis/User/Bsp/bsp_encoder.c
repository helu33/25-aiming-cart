#include "bsp_encoder.h"
#include "board_config.h"
#include "tim.h"

typedef struct {
    TIM_HandleTypeDef *timer;
    uint16_t previous_hw_count;
    int32_t total_count;
    int32_t velocity_previous_count;
    float velocity_m_s;
    int8_t sign;
    float meters_per_count;
} encoder_state_t;

static encoder_state_t encoders[ENCODER_SIDE_MAX] = {
    { &htim4, 0U, 0, 0, 0.0f, BOARD_ENCODER_LEFT_SIGN, BOARD_METERS_PER_COUNT_LEFT },
    { &htim8, 0U, 0, 0, 0.0f, BOARD_ENCODER_RIGHT_SIGN, BOARD_METERS_PER_COUNT_RIGHT }
};


static void encoder_refresh(encoder_state_t *state)
{
    const uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(state->timer);
    //读取编码器计数器的当前值，并计算与上一次读取的差值，更新总计数
    const int16_t delta = (int16_t)(now - state->previous_hw_count);
    state->previous_hw_count = now;
    //更新总计数，考虑编码器的方向
    state->total_count += (int32_t)delta * (int32_t)state->sign;
}

void encoder_bsp_init(void)
{
    uint32_t i;
    (void)HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    (void)HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim4, 0U);
    __HAL_TIM_SET_COUNTER(&htim8, 0U);
    for (i = 0U; i < ENCODER_SIDE_MAX; i++) {
        encoders[i].previous_hw_count = 0U;
        encoders[i].total_count = 0;
        encoders[i].velocity_previous_count = 0;
        encoders[i].velocity_m_s = 0.0f;
    }
}

//获取编码器的脉冲计数值
int32_t encoder_bsp_pulse_count_get(encoder_side_t side)
{
    if (side >= ENCODER_SIDE_MAX) return 0;
    encoder_refresh(&encoders[side]);
    return encoders[side].total_count;
}

//
void encoder_bsp_pulse_count_reset(encoder_side_t side)
{
    encoder_state_t *state;
    if (side >= ENCODER_SIDE_MAX) return;
    state = &encoders[side];
    __HAL_TIM_SET_COUNTER(state->timer, 0U);
    state->previous_hw_count = 0U;
    state->total_count = 0;
    state->velocity_previous_count = 0;
    state->velocity_m_s = 0.0f;
}


//
float encoder_bsp_distance_m_get(encoder_side_t side)
{
    if (side >= ENCODER_SIDE_MAX) return 0.0f;
    return (float)encoder_bsp_pulse_count_get(side) * encoders[side].meters_per_count;
}

void encoder_bsp_distance_reset(encoder_side_t side) { encoder_bsp_pulse_count_reset(side); }

//计算编码器的速度
void encoder_bsp_velocity_sample(float dt_sec)
{
    uint32_t i;
    if (dt_sec <= 1.0e-9f) return;
    for (i = 0U; i < ENCODER_SIDE_MAX; i++) {
        encoder_state_t *state = &encoders[i];
        int32_t delta;
        encoder_refresh(state);
        delta = state->total_count - state->velocity_previous_count;
        state->velocity_previous_count = state->total_count;
        state->velocity_m_s = ((float)delta * state->meters_per_count) / dt_sec;
    }
}

float encoder_bsp_velocity_m_s_get(encoder_side_t side)
{
    if (side >= ENCODER_SIDE_MAX) return 0.0f;
    return encoders[side].velocity_m_s;
}
