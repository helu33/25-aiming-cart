/* Start-up gyro bias: a window only counts when the board is still. */
#include "gyro_bias.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static const float BIAS[3] = {0.004f, -0.006f, 0.010f};   /* rad/s, typical BMI088 offset */

/* Small deterministic noise, about +/-0.003 rad/s like the real sensor at rest. */
static void still_sample(unsigned i, float g[3])
{
    for (unsigned a = 0; a < 3; ++a)
        g[a] = BIAS[a] + 0.003f * (float)((int)((i * 7U + a * 3U) % 5U) - 2) / 2.0f;
}

static unsigned feed_until_done(gyro_bias_t *b, unsigned start, unsigned limit)
{
    float g[3];
    for (unsigned i = start; i < limit; ++i) {
        still_sample(i, g);
        if (gyro_bias_feed(b, g)) return i + 1U;
    }
    return 0U;
}

static void test_still_board_gives_its_bias(void)
{
    gyro_bias_t b;
    gyro_bias_reset(&b);
    assert(feed_until_done(&b, 0, 10000) == GYRO_BIAS_WINDOW);
    for (unsigned a = 0; a < 3; ++a) assert(fabsf(b.bias[a] - BIAS[a]) < 0.001f);
    assert(b.restarts == 0);
}

static void test_touch_at_power_on_restarts_window(void)
{
    /* Field: nudging the gimbal while it powered up left a wrong bias and the yaw
     * crept away for ever. A hand push of a few deg/s must restart the window. */
    gyro_bias_t b;
    gyro_bias_reset(&b);
    float g[3];
    for (unsigned i = 0; i < 300; ++i) {
        still_sample(i, g);
        if (i >= 100 && i < 200) g[2] += 0.10f * sinf((float)(i - 100) * 0.0314f);  /* push */
        assert(!gyro_bias_feed(&b, g));
    }
    assert(b.restarts > 0);
    assert(feed_until_done(&b, 300, 10000) != 0);
    for (unsigned a = 0; a < 3; ++a) assert(fabsf(b.bias[a] - BIAS[a]) < 0.001f);
}

static void test_slow_steady_turn_is_rejected(void)
{
    /* A steady 3 deg/s turn has no spread, but no real gyro offset is that large. */
    gyro_bias_t b;
    gyro_bias_reset(&b);
    float g[3] = {0.0f, 0.0f, 0.052f};
    for (unsigned i = 0; i < 3 * GYRO_BIAS_WINDOW; ++i) assert(!gyro_bias_feed(&b, g));
    assert(b.restarts >= 2);
}

static void test_relaxed_accepts_any_full_window(void)
{
    /* Never still (e.g. vibrating table): after the timeout the caller relaxes the
     * checks so the gimbal still starts, with the plain average as before. */
    gyro_bias_t b;
    gyro_bias_reset(&b);
    b.relaxed = 1;
    float g[3] = {0.0f, 0.0f, 0.052f};
    unsigned done = 0;
    for (unsigned i = 0; i < GYRO_BIAS_WINDOW && !done; ++i) done = gyro_bias_feed(&b, g);
    assert(done && fabsf(b.bias[2] - 0.052f) < 1e-4f);
}

static void test_temperature_settle(void)
{
    /* Warming 25 -> 40 C at 0.5 C/s: not settled until within the band (2 C) of the target. */
    gyro_temp_settle_t s = {0};
    uint32_t t = 0;
    float temp = 25.0f;
    while (temp < 37.9f) {
        assert(!gyro_temp_settled(&s, temp, 40.0f, t));
        t += 100U; temp += 0.05f;
    }
    assert(gyro_temp_settled(&s, 38.2f, 40.0f, t));
    /* Weak heater plateaus at 33 C: settled after one steady 5 s window, not before. */
    gyro_temp_settle_t p = {0};
    unsigned done_at = 0;
    for (unsigned ms = 0; ms <= 12000U && !done_at; ms += 100U)
        if (gyro_temp_settled(&p, 33.0f, 40.0f, ms)) done_at = ms;
    assert(done_at >= GYRO_TEMP_STEADY_MS && done_at <= 2U * GYRO_TEMP_STEADY_MS);
}

int main(void)
{
    test_temperature_settle();
    test_still_board_gives_its_bias();
    test_touch_at_power_on_restarts_window();
    test_slow_steady_turn_is_rejected();
    test_relaxed_accepts_any_full_window();
    puts("Gyro bias tests passed: still window, restart on touch, steady turn, relaxed fallback, temperature settle.");
    return 0;
}
