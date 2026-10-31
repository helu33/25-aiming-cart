#ifndef GYRO_BIAS_H
#define GYRO_BIAS_H
/* Start-up gyro bias estimate that only accepts a window in which the board was still.
 * Pure C (header-only, so the Keil project needs no new source file).
 *
 * Field: touching the gimbal during the old fixed 1 s average baked the motion into the
 * bias, and yaw then crept away at that rate for the whole run (direction depended on
 * the push). Untouched it held still. */
#include <stdint.h>
#include <math.h>

#define GYRO_BIAS_WINDOW        500U     /* samples per window (2 ms each -> 1 s) */
/* Spread (max-min) allowed inside a window, rad/s (~2 deg/s). BMI088 noise at rest can
 * reach ~1 deg/s peak-to-peak over 500 samples; a hand push is well above 5 deg/s.
 * If the field shows endless restarts while untouched (dbg_imu_bias_restarts), raise it. */
#define GYRO_BIAS_MAX_SPAN      0.035f
/* Largest believable offset, rad/s (~2 deg/s; BMI088 spec is +/-1 deg/s). Catches a slow
 * steady turn that has no spread. */
#define GYRO_BIAS_MAX_OFFSET    0.035f

typedef struct {
    float sum[3], min[3], max[3];
    float bias[3];          /* valid once gyro_bias_feed returned 1 */
    uint32_t count;
    uint32_t restarts;      /* windows thrown away because the board moved */
    uint8_t relaxed;        /* caller sets after its timeout: accept any full window */
} gyro_bias_t;

static inline void gyro_bias_start_window(gyro_bias_t *b)
{
    for (unsigned a = 0; a < 3; ++a) {
        b->sum[a] = 0.0f;
        b->min[a] = INFINITY;
        b->max[a] = -INFINITY;
    }
    b->count = 0U;
}

static inline void gyro_bias_reset(gyro_bias_t *b)
{
    gyro_bias_start_window(b);
    for (unsigned a = 0; a < 3; ++a) b->bias[a] = 0.0f;
    b->restarts = 0U;
    b->relaxed = 0U;
}

/* Feed one raw sample (rad/s). Returns 1 when a window completes and bias[] is set. */
static inline uint8_t gyro_bias_feed(gyro_bias_t *b, const float g[3])
{
    for (unsigned a = 0; a < 3; ++a) {
        b->sum[a] += g[a];
        if (g[a] < b->min[a]) b->min[a] = g[a];
        if (g[a] > b->max[a]) b->max[a] = g[a];
        if (!b->relaxed && b->max[a] - b->min[a] > GYRO_BIAS_MAX_SPAN) {
            ++b->restarts;
            gyro_bias_start_window(b);
            return 0U;
        }
    }
    if (++b->count < GYRO_BIAS_WINDOW) return 0U;
    float mean[3];
    for (unsigned a = 0; a < 3; ++a) {
        mean[a] = b->sum[a] / (float)GYRO_BIAS_WINDOW;
        if (!b->relaxed && fabsf(mean[a]) > GYRO_BIAS_MAX_OFFSET) {
            ++b->restarts;
            gyro_bias_start_window(b);
            return 0U;
        }
    }
    for (unsigned a = 0; a < 3; ++a) b->bias[a] = mean[a];
    return 1U;
}

/* Heater warm-up before the bias window. The BMI088 gyro offset moves with temperature and
 * the heater only starts after power-on, so a bias taken cold drifts as the chip warms.
 * Settled once within GYRO_TEMP_BAND_C of the target, or once the temperature has changed
 * less than GYRO_TEMP_STEADY_C over GYRO_TEMP_STEADY_MS (heater too weak to reach it). */
#define GYRO_TEMP_BAND_C        2.0f    /* field: 1 C needed > 20 s; 38 C reached sooner */
#define GYRO_TEMP_STEADY_C      0.3f
#define GYRO_TEMP_STEADY_MS     5000U

typedef struct {
    float ref_c;
    uint32_t ref_ms;
    uint8_t started;
} gyro_temp_settle_t;

static inline uint8_t gyro_temp_settled(gyro_temp_settle_t *s, float temp_c, float target_c,
                                        uint32_t now_ms)
{
    if (fabsf(temp_c - target_c) <= GYRO_TEMP_BAND_C) return 1U;
    if (!s->started || (uint32_t)(now_ms - s->ref_ms) >= GYRO_TEMP_STEADY_MS) {
        uint8_t steady = s->started && fabsf(temp_c - s->ref_c) < GYRO_TEMP_STEADY_C;
        s->ref_c = temp_c;
        s->ref_ms = now_ms;
        s->started = 1U;
        return steady;
    }
    return 0U;
}
#endif
