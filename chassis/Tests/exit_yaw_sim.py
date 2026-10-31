"""Line following just after a corner exit, with a gyro yaw-rate loop on the wheel differential.

Field log 2026-10-24 (3 laps, LOG ON): at every exit the body still turned +0.3 rad/s while
the command was already 0, the left (inner) wheel lagged while both wheels accelerated
0.14 -> 0.35 m/s, then ran ~3 % fast; the line went +11 mm then -22 mm and took ~2 s to
centre, so a corner that came early met an off-centre line and was missed (line lost).

Model (mirrors chassis_mission.c LINE state): 8-probe binary array 95 mm ahead, error and
derivative low-passes, w = -kp*e - kd*de - kh*heading, angular-acceleration limit, speed
ramp at the configured acceleration. Each wheel follows its target with a first-order lag;
the left wheel's real speed is (1 + mismatch) x its measured one. Yaw loop: wheel
differential = w + kg * (w - gyro), gyro one control period old.
Run: py -3.13 chassis/Tests/exit_yaw_sim.py
"""
import math

PITCH_MM, LINE_HALF_MM, AHEAD_M = 11.14, 11.0, 0.095
B = 0.115                          # wheel separation, m
DT = 0.01
KP, KD, KH = 0.020, 0.002, 1.5
ACCEL, ALPHA, CRUISE = 0.8, 8.0, 0.35


def scan(lateral_mm):
    return sum(1 << i for i in range(8) if abs((i - 3.5) * PITCH_MM - lateral_mm) <= LINE_HALF_MM)


def line_error(mask):
    bits = [i for i in range(8) if mask >> i & 1]
    if not bits or len(bits) > 3 or bits[-1] - bits[0] + 1 != len(bits):
        return None
    return sum((i - 3.5) * PITCH_MM for i in bits) / len(bits)


def run(kg, wheel_tau=0.10, mismatch=0.03, w0=0.30, heading0_deg=-1.5, seconds=3.0):
    """Returns (max |line error| mm, time to stay within 6 mm, max |w - gyro|), or None if lost."""
    phi = math.radians(heading0_deg)          # body heading vs line, + = CCW
    y = 0.0                                   # axle lateral offset, + = left of the line, m
    vl = 0.14 - 0.5 * B * w0                  # measured wheel speeds at the exit
    vr = 0.14 + 0.5 * B * w0
    v_cmd, w_cmd = 0.14, 0.0
    err_f = deriv = 0.0
    init, lost, gyro_prev = False, 0.0, w0
    errors, rate_err = [], 0.0
    t = 0.0
    while t < seconds:
        real_l, real_r = vl * (1 + mismatch), vr
        v_body = 0.5 * (real_l + real_r)
        w_body = (real_r - real_l) / B
        lateral_mm = (y + AHEAD_M * math.sin(phi)) * 1000.0   # probes left of the line -> line right = +
        e = line_error(scan(lateral_mm))
        target_w = w_cmd
        if e is not None:
            prev = err_f
            if not init:
                err_f = prev = e
                init = True
            else:
                err_f += DT / (0.02 + DT) * (e - err_f)
            deriv += DT / (0.03 + DT) * ((err_f - prev) / DT - deriv)
            target_w = max(-1.5, min(1.5, -KP * err_f - KD * deriv - KH * phi))
            lost = 0.0
        else:
            lost += DT
            if lost > 0.2:
                return None
        w_cmd += max(-ALPHA * DT, min(ALPHA * DT, target_w - w_cmd))
        v_cmd = min(CRUISE, v_cmd + ACCEL * DT)
        w_wheels = w_cmd + kg * (w_cmd - gyro_prev)
        gyro_prev = w_body
        tl, tr = v_cmd - 0.5 * B * w_wheels, v_cmd + 0.5 * B * w_wheels
        vl += (tl - vl) * DT / wheel_tau
        vr += (tr - vr) * DT / wheel_tau
        phi += w_body * DT
        y += v_body * math.sin(phi) * DT
        errors.append(err_f)
        rate_err = max(rate_err, abs(w_cmd - w_body)) if t > 0.3 else rate_err
        t += DT
    settle = None
    for k in range(len(errors)):
        if all(abs(x) <= 6.0 for x in errors[k:]):
            settle = k * DT
            break
    return max(abs(x) for x in errors), settle, rate_err


def main():
    fmt = lambda r: "  LOST" if r is None else f"{r[0]:5.1f} mm  settle {'  -  ' if r[1] is None else f'{r[1]:4.2f} s'}  |w-gyro| {r[2]:.2f}"
    print("kg   | field-like (tau 0.10, 3% mismatch)      | slow wheels tau 0.15          | no mismatch")
    for kg in (0.0, 0.5, 1.0, 1.5, 2.0, 3.0, 5.0):
        print(f"{kg:3.1f}  | {fmt(run(kg))} | {fmt(run(kg, wheel_tau=0.15))} | {fmt(run(kg, mismatch=0.0))}")


if __name__ == "__main__":
    main()
