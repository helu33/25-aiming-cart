#ifndef BSP_ENCODER_H_
#define BSP_ENCODER_H_

#include <stdint.h>

typedef enum { ENCODER_SIDE_LEFT = 0, ENCODER_SIDE_RIGHT, ENCODER_SIDE_MAX } encoder_side_t;

void encoder_bsp_init(void);
int32_t encoder_bsp_pulse_count_get(encoder_side_t side);
void encoder_bsp_pulse_count_reset(encoder_side_t side);
float encoder_bsp_distance_m_get(encoder_side_t side);
void encoder_bsp_distance_reset(encoder_side_t side);
void encoder_bsp_velocity_sample(float dt_sec);
float encoder_bsp_velocity_m_s_get(encoder_side_t side);

#endif
