#ifndef FILTER_H_
#define FILTER_H_

#include <stdint.h>

typedef struct {
    float *p_buf;
    uint32_t window_len;
    uint32_t idx;
    uint32_t fill;
    float sum;
} filter_ma_f32_t;

typedef struct {
    float alpha;
    float y;
} filter_lpf_1st_f32_t;

void filter_ma_f32_init(filter_ma_f32_t *ctx, float *storage, uint32_t window_len);
void filter_ma_f32_reset(filter_ma_f32_t *ctx);
float filter_ma_f32_run(filter_ma_f32_t *ctx, float x);
void filter_lpf_1st_f32_init_alpha(filter_lpf_1st_f32_t *ctx, float alpha, float y0);
void filter_lpf_1st_f32_init_tau(filter_lpf_1st_f32_t *ctx, float dt_sec, float tau_sec, float y0);
void filter_lpf_1st_f32_reset(filter_lpf_1st_f32_t *ctx, float y0);
float filter_lpf_1st_f32_run(filter_lpf_1st_f32_t *ctx, float x);

#endif
