#include "filter.h"
#include <stddef.h>

void filter_ma_f32_init(filter_ma_f32_t *ctx, float *storage, uint32_t window_len)
{
    if ((ctx == NULL) || (storage == NULL) || (window_len == 0U)) return;
    ctx->p_buf = storage;
    ctx->window_len = window_len;
    ctx->idx = 0U;
    ctx->fill = 0U;
    ctx->sum = 0.0f;
}

void filter_ma_f32_reset(filter_ma_f32_t *ctx)
{
    if (ctx == NULL) return;
    ctx->idx = 0U;
    ctx->fill = 0U;
    ctx->sum = 0.0f;
}

float filter_ma_f32_run(filter_ma_f32_t *ctx, float x)
{
    float oldest;
    if ((ctx == NULL) || (ctx->p_buf == NULL) || (ctx->window_len == 0U)) return x;
    if (ctx->fill < ctx->window_len) {
        ctx->p_buf[ctx->fill++] = x;
        ctx->sum += x;
        return ctx->sum / (float)ctx->fill;
    }
    oldest = ctx->p_buf[ctx->idx];
    ctx->sum += x - oldest;
    ctx->p_buf[ctx->idx] = x;
    ctx->idx = (ctx->idx + 1U) % ctx->window_len;
    return ctx->sum / (float)ctx->window_len;
}

static float clamp_alpha(float alpha)
{
    if (alpha > 1.0f) return 1.0f;
    if (alpha < 1.0e-9f) return 1.0e-9f;
    return alpha;
}

void filter_lpf_1st_f32_init_alpha(filter_lpf_1st_f32_t *ctx, float alpha, float y0)
{
    if (ctx == NULL) return;
    ctx->alpha = clamp_alpha(alpha);
    ctx->y = y0;
}

void filter_lpf_1st_f32_init_tau(filter_lpf_1st_f32_t *ctx, float dt_sec, float tau_sec, float y0)
{
    float alpha = 1.0f;
    if ((dt_sec > 0.0f) && (tau_sec > 0.0f)) alpha = dt_sec / (tau_sec + dt_sec);
    filter_lpf_1st_f32_init_alpha(ctx, alpha, y0);
}

void filter_lpf_1st_f32_reset(filter_lpf_1st_f32_t *ctx, float y0)
{
    if (ctx != NULL) ctx->y = y0;
}

float filter_lpf_1st_f32_run(filter_lpf_1st_f32_t *ctx, float x)
{
    if (ctx == NULL) return x;
    ctx->y = ctx->alpha * x + (1.0f - ctx->alpha) * ctx->y;
    return ctx->y;
}
