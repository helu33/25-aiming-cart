"""Yaw two-loop through a chassis corner with a slow yaw motor, with and without the inner-loop
integral (gimbal_task.c, "ratei"), and what a faster driver speed loop would buy.

Field 2026-10-28 (telemetry channel 9, measured motor rpm): measured / commanded rpm runs
0.67 early in a corner and 1.2-1.3 late, the signature of a lag, not of a lower gain. A
first-order fit over 34 corners and 56 straight chunks (run_report.py) gives tau 130-200 ms,
gain 0.8-0.9: the JC4010 speed loop (factory Kp 0.05 / Ki 0.5) is slow with the stage on it.
With that motor the model gives the field's corner error (peak 1.47 vs 1.2-1.8 deg per
corner); the integral barely helps (ratei 20: -4 %, 50: -27 % and a worse straight), while a
faster motor does (tau 60 ms: 0.64 deg).

Model, 2 ms steps:
- body yaw rate: first-order lag (tau 0.12 s) on a 129 deg/s command for 0.7 s, then back to 0
  (the field body peaked 110-118 deg/s); $C gz every 20 ms, 40 ms old;
- target bearing turns at pre_rate deg/s (the tracker rate, assumed known);
- gimbal as flashed: w* = pre_rate + clamp(kpos*e, 40); rpm = (w* - gz + krate*(w* - gyro)
  + I) / 6, clamped to 50; I = rate_integral_step (zone 3 deg, anti-windup);
- yaw motor: first-order (motor_tau) towards motor_gain x command, one cycle of CAN delay.
Also a straight with a 4 Hz +-15 deg/s body wobble (seen on the straights) to check that the
integral does not make that worse.

Run: py -3.13 gimbal/tools/rate_integral_sim.py
"""
import math

DT = 0.002
KPOS, KRATE, POS_MAX, OUT_LIMIT, ZONE = 8.0, 7.0, 40.0, 50.0, 3.0


def integral_step(i, ki, err, limit, in_zone, saturated):
    """Mirror of rate_integral_step in gimbal/App/inc/rate_integral.h."""
    if ki <= 0 or limit <= 0:
        return 0.0
    if not in_zone or (saturated > 0 and err > 0) or (saturated < 0 and err < 0):
        return i
    return max(-limit, min(limit, i + ki * err * DT))


def run(ki, motor_gain=0.85, motor_tau=0.15, pre_rate=5.0, imax=60.0, corner=True, seconds=2.0,
        wobble_hz=4.0, wobble_deg_s=15.0):
    body_cmd = body = stage = 0.0
    target = 0.0
    motor = pre_rate / 6.0
    seen_gz, frames, next_frame = 0.0, [], 0.0
    integ, saturated, last_cmd = 0.0, 0, motor
    t, peak, sq, n = 0.0, 0.0, 0.0, 0
    while t < seconds:
        if corner:
            body_cmd = 129.0 if 0.3 <= t < 1.0 else 0.0
            body += (body_cmd - body) * DT / 0.12
        else:
            body = wobble_deg_s * math.sin(2 * math.pi * wobble_hz * t)
        if t >= next_frame:
            frames.append((t + 0.040, body))
            next_frame += 0.020
        while frames and frames[0][0] <= t:
            seen_gz = frames.pop(0)[1]
        target += pre_rate * DT
        stage_rate = body + motor * 6.0
        e = target - stage
        w_star = pre_rate + max(-POS_MAX, min(POS_MAX, KPOS * e))
        rate_err = w_star - stage_rate
        integ = integral_step(integ, ki, rate_err, imax, abs(e) < ZONE, saturated)
        cmd = (w_star - seen_gz + KRATE * rate_err + integ) / 6.0
        saturated = 1 if cmd >= OUT_LIMIT else -1 if cmd <= -OUT_LIMIT else 0
        cmd = max(-OUT_LIMIT, min(OUT_LIMIT, cmd))
        motor += (motor_gain * last_cmd - motor) * DT / motor_tau
        last_cmd = cmd
        stage += (body + motor * 6.0) * DT
        if t > 0.3:
            peak = max(peak, abs(e))
            sq += e * e
            n += 1
        t += DT
    return peak, math.sqrt(sq / max(n, 1))


def main():
    print("fitted motor (tau 150 ms, gain 0.85); field corner peaks 1.2-1.8 deg:")
    for ki in (0, 20, 50):
        p, r = run(ki)
        pw, _ = run(ki, corner=False)
        print(f"  ratei {ki:3d}: corner peak {p:4.2f} deg rms {r:4.2f} | straight wobble peak {pw:4.2f}")
    print("faster driver speed loop (raise its speed Kp/Ki in Motor tool), ratei 0:")
    for tau, gain in ((0.15, 0.85), (0.10, 0.9), (0.06, 0.95), (0.03, 1.0)):
        p, r = run(0, motor_gain=gain, motor_tau=tau)
        pw, _ = run(0, motor_gain=gain, motor_tau=tau, corner=False)
        print(f"  motor tau {tau * 1000:3.0f} ms: corner peak {p:4.2f} deg rms {r:4.2f} | straight wobble {pw:4.2f}")


if __name__ == "__main__":
    main()
