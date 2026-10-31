#ifndef GIMBAL_TUNE_H
#define GIMBAL_TUNE_H

#include <stdint.h>

/* Online-tunable parameters, defined with their field notes in gimbal_tune.c. */

#define VEH_CHASSIS_FORWARD_DEG 31.8f   /* yaw motor angle (absolute) with the laser along the chassis */
#define VISION_LATENCY_MAX_MS   200U

extern volatile uint32_t tune_vision_latency_ms;
extern volatile float tune_track_alpha;
extern volatile float tune_track_beta;
extern volatile float tune_track_ff;
extern volatile float tune_yaw_kp_scale;
extern volatile uint8_t tune_yaw_two_loop;
extern volatile float tune_yaw_kpos;
extern volatile float tune_yaw_krate;
extern volatile float tune_yaw_chassis_ff;
extern volatile float tune_yaw_pos_rate_max;
extern volatile float tune_yaw_rate_ki;
extern volatile float tune_yaw_rate_i_max;
extern volatile float tune_yaw_gz_lead_s;
extern volatile uint32_t tune_yaw_gz_tail_ms;
extern volatile uint8_t tune_telem_select;
extern volatile uint32_t tune_track_settle_ms;
extern volatile float tune_track_rate_clip_deg;
extern volatile uint8_t tune_turn_hold;
extern volatile float tune_veh_ff;
extern volatile float tune_veh_pitch_h;
extern volatile float tune_veh_speed_lead_s;
extern volatile float tune_yaw_fwd_deg;
extern volatile uint8_t tune_vision_use_age;
extern volatile uint32_t tune_vision_extra_ms;
extern volatile float g_pitch_kp;

#endif /* GIMBAL_TUNE_H */
