#ifndef DRIVE_TEST_H
#define DRIVE_TEST_H
#include <stdint.h>

/* DRIVE: drive straight for a set distance and stop - a repeatable motion for gimbal
 * tracking tests, without the line sensor or the mission. Pure C: metres, seconds, rad. */
#define DRIVE_MAX_M_S       0.50f
#define DRIVE_MAX_M         3.00f
#define DRIVE_MIN_M_S       0.03f   /* final creep: below this the wheels may not turn */
#define DRIVE_DONE_M        0.005f
#define DRIVE_MAX_W_RAD_S   1.0f
#define DRIVE_EXTRA_TIME_S  2.0f    /* timeout margin over the ideal trapezoid time */

typedef struct {
    float speed, distance, accel, decel, kh;
    float v, w;             /* commands */
    float travelled, heading, elapsed, max_time;
    uint8_t active, timed_out;
} drive_test_t;

/* 0 (and inactive) for speed outside (0, DRIVE_MAX_M_S] or distance outside (0, DRIVE_MAX_M]. */
uint8_t drive_test_start(drive_test_t *d, float speed, float distance,
                         float accel, float decel, float kh);
/* left/right: wheel travel this step (m); gyro_z: rad/s, CCW +. Holds the start heading. */
void drive_test_step(drive_test_t *d, float dt, float left_m, float right_m,
                     float gyro_z, uint8_t gyro_valid);
void drive_test_stop(drive_test_t *d);
#endif
