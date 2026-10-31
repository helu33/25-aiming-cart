#ifndef RATE_INTEGRAL_H
#define RATE_INTEGRAL_H
/* Integral of a rate-loop error with conditional integration (pure C, header-only, host-tested).
 * Used by the yaw two-loop inner loop ("ratei"): it adds what the motor is missing until the
 * stage turns at the wanted rate. Field 2026-10-28: the yaw motor lags its command ~150 ms, so
 * the integral helps only a little there; a faster driver speed loop is the real fix.
 *   integral   state, in the units of the loop (deg/s)
 *   ki         1/s; <= 0 switches it off and clears the state
 *   in_zone    0 holds the state (large position error: reacquiring a target)
 *   saturated  +1 / -1 when the last output hit its limit in that direction, else 0:
 *              the state is not pushed further into the limit (anti-windup)
 *   limit      |state| bound, same units */
static inline float rate_integral_step(float integral, float ki, float error, float dt,
                                       float limit, int in_zone, int saturated)
{
    if (!(ki > 0.0f) || !(limit > 0.0f) || !(dt > 0.0f)) return 0.0f;
    if (!in_zone || error != error) return integral;
    if ((saturated > 0 && error > 0.0f) || (saturated < 0 && error < 0.0f)) return integral;
    integral += ki * error * dt;
    if (integral > limit) integral = limit;
    if (integral < -limit) integral = -limit;
    return integral;
}
#endif
