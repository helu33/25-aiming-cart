/* Alpha-beta tracker of the target direction (deg, deg/s) used by the gimbal. */
#include "track_filter.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define ALPHA 0.5f
#define BETA  0.17f
#define FRAME_MS 44U

static void test_first_frame_initialises(void)
{
    track_filter_t f = {0};
    assert(track_update(&f, 12.0f, 1000U, ALPHA, BETA, 0.0f) == TRACK_RESET);
    float rate;
    assert(fabsf(track_predict(&f, 1100U, 0.0f, &rate) - 12.0f) < 1e-4f && rate == 0.0f);
}

static void test_constant_rate_is_learnt_and_predicted(void)
{
    /* Car passing the target: bearing moves 20 deg/s. After a second the filter knows
     * the rate, and predicting to "now" (80 ms after the frame) removes the lag. */
    track_filter_t f = {0};
    uint32_t t = 1000U;
    for (int k = 0; k < 25; ++k, t += FRAME_MS)
        track_update(&f, 20.0f * (float)(t - 1000U) / 1000.0f, t, ALPHA, BETA, 0.0f);
    t -= FRAME_MS;
    float rate;
    float now_pos = track_predict(&f, t + 80U, 0.0f, &rate);
    assert(fabsf(rate - 20.0f) < 0.5f);
    assert(fabsf(now_pos - 20.0f * (float)(t + 80U - 1000U) / 1000.0f) < 0.2f);
}

static void test_step_settles(void)
{
    track_filter_t f = {0};
    uint32_t t = 1000U;
    track_update(&f, 0.0f, t, ALPHA, BETA, 0.0f);
    float peak = 0.0f, rate;
    for (int k = 1; k < 40; ++k) {
        t += FRAME_MS;
        track_update(&f, 3.0f, t, ALPHA, BETA, 0.0f);
        float x = track_predict(&f, t, 0.0f, &rate);
        if (x > peak) peak = x;
    }
    assert(fabsf(track_predict(&f, t, 0.0f, &rate) - 3.0f) < 0.05f && fabsf(rate) < 0.2f);
    assert(peak < 3.0f * 1.35f);              /* alpha-beta overshoot stays modest */
}

static void test_noise_is_smoothed(void)
{
    track_filter_t f = {0};
    uint32_t t = 1000U;
    float rate, worst = 0.0f;
    for (int k = 0; k < 60; ++k, t += FRAME_MS) {
        track_update(&f, (k & 1) ? 0.3f : -0.3f, t, ALPHA, BETA, 0.0f);
        if (k > 20) {
            float x = track_predict(&f, t, 0.0f, &rate);
            if (fabsf(x) > worst) worst = fabsf(x);
        }
    }
    assert(worst < 0.3f);
}

static void test_jump_resets(void)
{
    /* Another object or a corner glitch: a jump beyond the gate restarts the track
     * instead of being learnt as a huge velocity. */
    track_filter_t f = {0};
    track_update(&f, 0.0f, 1000U, ALPHA, BETA, 0.0f);
    track_update(&f, 0.0f, 1044U, ALPHA, BETA, 0.0f);
    assert(track_update(&f, TRACK_GATE_DEG + 3.0f, 1088U, ALPHA, BETA, 0.0f) == TRACK_RESET);
    float rate;
    assert(fabsf(track_predict(&f, 1088U, 0.0f, &rate) - (TRACK_GATE_DEG + 3.0f)) < 1e-4f && rate == 0.0f);
}

static void test_coasting_is_bounded(void)
{
    /* Dropped frames: keep moving at the learnt rate, but only for TRACK_COAST_MS. */
    track_filter_t f = {0};
    uint32_t t = 1000U;
    for (int k = 0; k < 25; ++k, t += FRAME_MS)
        track_update(&f, 20.0f * (float)(t - 1000U) / 1000.0f, t, ALPHA, BETA, 0.0f);
    t -= FRAME_MS;
    float rate;
    float at_limit = track_predict(&f, t + TRACK_COAST_MS, 0.0f, &rate);
    float later = track_predict(&f, t + 1000U, 0.0f, &rate);
    assert(rate == 0.0f && fabsf(later - at_limit) < 1e-3f);
}

static void test_stale_track_restarts(void)
{
    track_filter_t f = {0};
    track_update(&f, 0.0f, 1000U, ALPHA, BETA, 0.0f);
    track_update(&f, 1.0f, 1044U, ALPHA, BETA, 0.0f);
    assert(track_update(&f, 2.0f, 1044U + TRACK_LOST_MS + 1U, ALPHA, BETA, 0.0f) == TRACK_RESET);
}

static void test_yaw_wraps(void)
{
    /* 358 -> 2 deg is a 4 deg move, not -356. */
    track_filter_t f = {.wrap360 = 1};
    uint32_t t = 1000U;
    float z = 350.0f, rate = 0.0f;
    for (int k = 0; k < 20; ++k, t += FRAME_MS, z += 0.88f) {   /* 20 deg/s */
        float wrapped = z >= 360.0f ? z - 360.0f : z;
        assert(track_update(&f, wrapped, t, ALPHA, BETA, 0.0f) != TRACK_RESET || k == 0);
    }
    float x = track_predict(&f, t - FRAME_MS, 0.0f, &rate);
    assert(x >= 0.0f && x < 360.0f && fabsf(rate - 20.0f) < 1.5f);
}

static void test_known_input_needs_no_learning(void)
{
    /* Vehicle feed-forward: the whole 20 deg/s is explained by the input u, so the filter
     * predicts it from the first frame on and its residual rate stays ~0 (no start lag). */
    track_filter_t f = {0};
    uint32_t t = 1000U;
    float rate = 1.0f;
    for (int k = 0; k < 10; ++k, t += FRAME_MS) {
        const float bearing = 20.0f * (float)(t - 1000U) / 1000.0f;
        if (k > 0) assert(fabsf(track_predict(&f, t, bearing, &rate) - bearing) < 1e-3f);
        assert(track_update(&f, bearing, t, ALPHA, BETA, bearing) == (k ? TRACK_UPDATED : TRACK_RESET));
    }
    assert(fabsf(f.v) < 1e-3f && fabsf(rate) < 1e-3f);
    /* Coasting freezes only the residual; the measured input keeps moving the prediction. */
    assert(fabsf(track_predict(&f, t + 1000U, 99.0f, &rate) - (f.x + 99.0f - f.u_ref)) < 1e-3f);
}

static void test_rate_clip_limits_one_outlier(void)
{
    /* Still target, one frame 1.5 deg off: the rate moves by at most beta * clip / dt. */
    track_filter_t f = {0}, g = {0};
    g.rate_clip = 0.4f;
    uint32_t t = 1000U;
    for (int k = 0; k < 10; ++k, t += 40U) {
        track_update(&f, 0.0f, t, ALPHA, BETA, 0.0f);
        track_update(&g, 0.0f, t, ALPHA, BETA, 0.0f);
    }
    track_update(&f, 1.5f, t, ALPHA, BETA, 0.0f);
    track_update(&g, 1.5f, t, ALPHA, BETA, 0.0f);
    assert(fabsf(f.v - BETA * 1.5f / 0.040f) < 0.05f);
    assert(fabsf(g.v - BETA * 0.4f / 0.040f) < 0.05f);
    /* A real 20 deg/s ramp is still learnt, just over more frames. */
    for (int k = 1; k <= 40; ++k)
        track_update(&g, 1.5f + 20.0f * 0.040f * (float)k, t + 40U * (uint32_t)k, ALPHA, BETA, 0.0f);
    assert(fabsf(g.v - 20.0f) < 1.0f);
}

int main(void)
{
    test_known_input_needs_no_learning();
    test_rate_clip_limits_one_outlier();
    test_first_frame_initialises();
    test_constant_rate_is_learnt_and_predicted();
    test_step_settles();
    test_noise_is_smoothed();
    test_jump_resets();
    test_coasting_is_bounded();
    test_stale_track_restarts();
    test_yaw_wraps();
    puts("Track filter tests passed: init, constant rate, step, noise, gate, coast, stale, wrap, input.");
    return 0;
}
