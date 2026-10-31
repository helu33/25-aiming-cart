/* DRIVE: straight, repeatable test run (gimbal tracking tests) - no line sensor needed. */
#include "drive_test.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define DT 0.01f
#define ACCEL 0.8f
#define DECEL 1.0f
#define KH 1.5f

/* Ideal wheels: body speed = command. Returns the time to finish. */
static float run(drive_test_t *d, float gyro_rad_s, float *peak)
{
    float t = 0.0f;
    *peak = 0.0f;
    while (d->active && t < 30.0f) {
        const float step = d->v * DT;
        drive_test_step(d, DT, step, step, gyro_rad_s, 1U);
        if (d->v > *peak) *peak = d->v;
        t += DT;
    }
    return t;
}

static void test_rejects_bad_requests(void)
{
    drive_test_t d;
    assert(!drive_test_start(&d, 0.0f, 1.0f, ACCEL, DECEL, KH));
    assert(!drive_test_start(&d, 0.80f, 1.0f, ACCEL, DECEL, KH));      /* above DRIVE_MAX_M_S */
    assert(!drive_test_start(&d, 0.20f, 0.0f, ACCEL, DECEL, KH));
    assert(!drive_test_start(&d, 0.20f, 5.0f, ACCEL, DECEL, KH));      /* above DRIVE_MAX_M */
    assert(!d.active);
}

static void test_trapezoid_reaches_distance(void)
{
    drive_test_t d;
    float peak;
    assert(drive_test_start(&d, 0.35f, 1.00f, ACCEL, DECEL, KH));
    const float t = run(&d, 0.0f, &peak);
    assert(!d.active && !d.timed_out);
    assert(fabsf(d.travelled - 1.00f) < 0.01f);
    assert(fabsf(peak - 0.35f) < 1e-4f);
    /* 1 m at 0.35 m/s with 0.8/1.0 ramps: ~3.2 s; the slow final creep adds a little. */
    assert(t > 3.0f && t < 3.8f);
    assert(d.v == 0.0f && d.w == 0.0f);
}

static void test_short_run_never_reaches_cruise(void)
{
    drive_test_t d;
    float peak;
    assert(drive_test_start(&d, 0.35f, 0.05f, ACCEL, DECEL, KH));
    (void)run(&d, 0.0f, &peak);
    assert(peak < 0.25f && fabsf(d.travelled - 0.05f) < 0.01f);
}

static void test_heading_hold_steers_back(void)
{
    drive_test_t d;
    assert(drive_test_start(&d, 0.20f, 1.0f, ACCEL, DECEL, KH));
    for (int i = 0; i < 50; ++i) drive_test_step(&d, DT, 0.001f, 0.001f, 0.2f, 1U);   /* drifting left */
    assert(d.heading > 0.09f && d.w < 0.0f);                       /* turn right */
    drive_test_step(&d, DT, 0.001f, 0.001f, 0.0f, 0U);             /* gyro invalid: no steering */
    assert(d.w == 0.0f);
}

static void test_stalled_wheels_time_out(void)
{
    drive_test_t d;
    assert(drive_test_start(&d, 0.20f, 0.50f, ACCEL, DECEL, KH));
    float t = 0.0f;
    while (d.active && t < 30.0f) { drive_test_step(&d, DT, 0.0f, 0.0f, 0.0f, 1U); t += DT; }
    assert(!d.active && d.timed_out && t < 10.0f);
}

static void test_stop_is_immediate(void)
{
    drive_test_t d;
    assert(drive_test_start(&d, 0.20f, 1.0f, ACCEL, DECEL, KH));
    drive_test_step(&d, DT, 0.0f, 0.0f, 0.0f, 1U);
    drive_test_stop(&d);
    assert(!d.active && d.v == 0.0f && d.w == 0.0f);
}

int main(void)
{
    test_rejects_bad_requests();
    test_trapezoid_reaches_distance();
    test_short_run_never_reaches_cruise();
    test_heading_hold_steers_back();
    test_stalled_wheels_time_out();
    test_stop_is_immediate();
    puts("Drive test passed: limits, trapezoid, short run, heading hold, stall timeout, stop.");
    return 0;
}
