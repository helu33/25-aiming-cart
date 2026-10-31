#include "drive_test.h"
#include <math.h>
#include <string.h>

uint8_t drive_test_start(drive_test_t *d, float speed, float distance,
                         float accel, float decel, float kh)
{
    memset(d, 0, sizeof(*d));
    if (!(speed > 0.0f && speed <= DRIVE_MAX_M_S) || !(distance > 0.0f && distance <= DRIVE_MAX_M) ||
        !(accel > 0.0f) || !(decel > 0.0f)) return 0U;
    d->speed = speed; d->distance = distance; d->accel = accel; d->decel = decel; d->kh = kh;
    d->max_time = distance / speed + speed / accel + speed / decel + DRIVE_EXTRA_TIME_S;
    d->active = 1U;
    return 1U;
}

void drive_test_stop(drive_test_t *d)
{
    d->active = 0U;
    d->v = d->w = 0.0f;
}

void drive_test_step(drive_test_t *d, float dt, float left_m, float right_m,
                     float gyro_z, uint8_t gyro_valid)
{
    if (!d->active || !(dt > 0.0f)) return;
    d->travelled += 0.5f * (left_m + right_m);
    if (gyro_valid) d->heading += gyro_z * dt;
    d->elapsed += dt;
    if (d->elapsed > d->max_time) d->timed_out = 1U;

    const float remaining = d->distance - d->travelled;
    float target = 0.0f;
    if (!d->timed_out && remaining > DRIVE_DONE_M) {
        /* Latest braking that still stops at the mark, never below the creep speed. */
        target = sqrtf(2.0f * d->decel * remaining);
        if (target > d->speed) target = d->speed;
        if (target < DRIVE_MIN_M_S) target = DRIVE_MIN_M_S;
    }
    if (target > d->v) {
        d->v += d->accel * dt;
        if (d->v > target) d->v = target;
    } else {
        d->v -= d->decel * dt;
        if (d->v < target) d->v = target;
    }

    if (target == 0.0f && d->v == 0.0f) {
        drive_test_stop(d);
        return;
    }
    d->w = gyro_valid ? -d->kh * d->heading : 0.0f;
    if (d->w > DRIVE_MAX_W_RAD_S) d->w = DRIVE_MAX_W_RAD_S;
    if (d->w < -DRIVE_MAX_W_RAD_S) d->w = -DRIVE_MAX_W_RAD_S;
}
