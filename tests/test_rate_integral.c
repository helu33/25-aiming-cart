/* Host test for gimbal/App/inc/rate_integral.h. */
#include "rate_integral.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void test_off_clears_and_accumulates_when_on(void)
{
    assert(rate_integral_step(12.0f, 0.0f, 5.0f, 0.002f, 60.0f, 1, 0) == 0.0f);   /* ki 0: cleared */
    float s = 0.0f;
    for (int i = 0; i < 500; ++i) s = rate_integral_step(s, 10.0f, 2.0f, 0.002f, 60.0f, 1, 0);
    assert(fabsf(s - 20.0f) < 1e-3f);                       /* 10/s x 2 deg/s x 1 s */
}

static void test_limit_zone_and_anti_windup(void)
{
    float s = 0.0f;
    for (int i = 0; i < 5000; ++i) s = rate_integral_step(s, 50.0f, 10.0f, 0.002f, 60.0f, 1, 0);
    assert(s == 60.0f);                                     /* clamped */
    assert(rate_integral_step(30.0f, 10.0f, 5.0f, 0.002f, 60.0f, 0, 0) == 30.0f);  /* held out of zone */
    assert(rate_integral_step(30.0f, 10.0f, 5.0f, 0.002f, 60.0f, 1, 1) == 30.0f);  /* saturated + */
    assert(rate_integral_step(30.0f, 10.0f, -5.0f, 0.002f, 60.0f, 1, 1) < 30.0f);  /* may unwind */
    assert(rate_integral_step(-30.0f, 10.0f, -5.0f, 0.002f, 60.0f, 1, -1) == -30.0f);
    assert(rate_integral_step(30.0f, 10.0f, NAN, 0.002f, 60.0f, 1, 0) == 30.0f);   /* bad sample */
}

int main(void)
{
    test_off_clears_and_accumulates_when_on();
    test_limit_zone_and_anti_windup();
    puts("Rate integral tests passed: off, accumulate, limit, zone hold, anti-windup.");
    return 0;
}
