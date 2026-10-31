#ifndef TRACK_FILTER_H
#define TRACK_FILTER_H
/* Alpha-beta tracker of the target direction in a frame the gimbal motion does not change
 * (yaw: world bearing from the IMU; pitch: motor angle + IMU pitch). Each vision frame is a
 * measurement at its capture time; between frames the gimbal follows the prediction
 * x + v*(now - t), which also removes the vision latency, and v feeds the motor speed.
 * Optional known input u: the direction change the car's own motion explains (deg, integrated
 * by the caller; vehicle-speed feed-forward). Prediction adds its change since the last update,
 * so the filter only learns the residual rate. Pass 0 for no input.
 * Pure C, header-only (no Keil project change). Angles in deg, rates in deg/s, times in ms. */
#include <stdint.h>
#include <math.h>

#define TRACK_GATE_DEG          5.0f    /* residual beyond this restarts the track */
#define TRACK_MAX_RATE_DEG_S    90.0f
/* Times count from the capture instant, so they include the ~80 ms vision latency. 150/300 ms
 * (the first values) ended the coast and the track inside the 300-400 ms vision dropouts seen
 * while driving; the reacquired frame was then > TRACK_GATE_DEG off and the learnt rate was lost
 * (field result6: 8 deg after the start). A 400 ms dropout at 25 deg/s coasts ~11 deg. */
#define TRACK_COAST_MS          450U    /* extrapolate at most this far past the last frame */
#define TRACK_LOST_MS           500U    /* a frame after a longer gap starts a new track */

typedef struct {
    float x, v;             /* direction (deg) and residual rate (deg/s) at time t_ms */
    float u_ref;            /* integrated input at t_ms */
    float rate_clip;        /* > 0: residual used for the rate update clamped to +/- this, deg */
    uint32_t t_ms;
    uint8_t active;
    uint8_t wrap360;        /* 1: x lives on 0..360 (yaw bearing) */
} track_filter_t;

typedef enum { TRACK_UPDATED = 0, TRACK_RESET = 1 } track_result_t;

static inline float track_wrap180(float deg)
{
    deg = fmodf(deg + 180.0f, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    return deg - 180.0f;
}

static inline float track_normalise(const track_filter_t *f, float deg)
{
    if (!f->wrap360) return deg;
    deg = fmodf(deg, 360.0f);
    return deg < 0.0f ? deg + 360.0f : deg;
}

static inline void track_reset(track_filter_t *f, float z, uint32_t t_ms, float u_at)
{
    f->x = track_normalise(f, z);
    f->v = 0.0f;
    f->u_ref = u_at;
    f->t_ms = t_ms;
    f->active = 1U;
}

static inline void track_stop(track_filter_t *f)
{
    f->active = 0U;
    f->v = 0.0f;
}

/* One measurement z taken at t_ms; u_at = integrated input at t_ms. Restarts on the first
 * frame, after TRACK_LOST_MS without frames, or when the residual exceeds TRACK_GATE_DEG. */
static inline track_result_t track_update(track_filter_t *f, float z, uint32_t t_ms,
                                          float alpha, float beta, float u_at)
{
    int32_t gap = (int32_t)(t_ms - f->t_ms);
    if (!f->active || gap > (int32_t)TRACK_LOST_MS) {
        track_reset(f, z, t_ms, u_at);
        return TRACK_RESET;
    }
    float dt = (float)(gap > 0 ? gap : 0) / 1000.0f;
    float predicted = f->x + f->v * dt + (u_at - f->u_ref);
    float residual = z - predicted;
    if (f->wrap360) residual = track_wrap180(residual);
    if (fabsf(residual) > TRACK_GATE_DEG) {
        track_reset(f, z, t_ms, u_at);
        return TRACK_RESET;
    }
    f->x = track_normalise(f, predicted + alpha * residual);
    if (dt > 0.0f) {
        /* A single outlier frame (pose jitter at oblique views) must not kick the rate, which
         * feeds the motor directly; a real rate shows in several frames and is still learnt. */
        float rate_residual = residual;
        if (f->rate_clip > 0.0f) {
            if (rate_residual > f->rate_clip) rate_residual = f->rate_clip;
            if (rate_residual < -f->rate_clip) rate_residual = -f->rate_clip;
        }
        f->v += beta * rate_residual / dt;
        if (f->v > TRACK_MAX_RATE_DEG_S) f->v = TRACK_MAX_RATE_DEG_S;
        if (f->v < -TRACK_MAX_RATE_DEG_S) f->v = -TRACK_MAX_RATE_DEG_S;
        f->t_ms = t_ms;
        f->u_ref = u_at;
    }
    return TRACK_UPDATED;
}

/* Direction at now_ms given the integrated input u_now. *rate is the residual rate (add the
 * input rate for the feed-forward). Beyond TRACK_COAST_MS the residual part freezes and *rate
 * is 0; the input part keeps following u (it is measured, not extrapolated). */
static inline float track_predict(const track_filter_t *f, uint32_t now_ms, float u_now, float *rate)
{
    int32_t ahead = (int32_t)(now_ms - f->t_ms);
    if (ahead < 0) ahead = 0;
    float v = f->v;
    if (ahead >= (int32_t)TRACK_COAST_MS) {
        ahead = (int32_t)TRACK_COAST_MS;
        v = 0.0f;
    }
    if (rate) *rate = v;
    return track_normalise(f, f->x + f->v * (float)ahead / 1000.0f + (u_now - f->u_ref));
}
#endif
