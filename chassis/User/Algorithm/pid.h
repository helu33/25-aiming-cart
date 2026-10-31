#ifndef PID_H_
#define PID_H_

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float integral_limit;
    float out_min;
    float out_max;
} pid_context_t;

void pid_init(pid_context_t *ctx, float kp, float ki, float kd,
              float integral_limit, float out_min, float out_max);
void pid_reset(pid_context_t *ctx);
float pid_compute(pid_context_t *ctx, float error, float dt_sec);

#endif
