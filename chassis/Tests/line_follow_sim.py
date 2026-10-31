"""Kinematic model of straight-line following, used to choose CHASSIS_LINE_KP/KD.

Mirrors chassis_mission.c: binary 8-probe array, error low-pass (20 ms), derivative
low-pass (30 ms), w clamp and angular-acceleration limit. The body yaw rate follows
the command with a first-order lag. Run: python chassis/Tests/line_follow_sim.py
"""
import math

PITCH_MM = 11.14
LINE_HALF_MM = 9.0 + 2.0      # 18 mm tape plus a little probe spot
AHEAD_MM = 95.0
DT = 0.01
SCAN_S = 0.008


def scan(lateral_mm):
    """lateral_mm: line position relative to array centre, + = right."""
    mask = 0
    for i in range(8):
        if abs((i - 3.5) * PITCH_MM - lateral_mm) <= LINE_HALF_MM:
            mask |= 1 << i
    return mask


def line_error(mask):
    bits = [i for i in range(8) if mask >> i & 1]
    if not bits or len(bits) > 3 or bits[-1] - bits[0] + 1 != len(bits):
        return None
    return sum((i - 3.5) * PITCH_MM for i in bits) / len(bits)


def run(kp, kd, v, lag, alpha, sensor_mm, heading_deg, seconds=4.0, kh=0.0):
    """Start with the line sensor_mm right of the array centre and the body rotated
    heading_deg counter-clockwise from the line direction."""
    phi = math.radians(heading_deg)   # + = rotated CCW (left) relative to line direction
    y = sensor_mm - AHEAD_MM * math.sin(phi)   # axle offset, + = axle left of line
    w_cmd = w_body = 0.0
    err_f = deriv = 0.0
    init = False
    lost = 0.0
    t = scan_t = 0.0
    mask = 0
    trace = []
    while t < seconds:
        if t >= scan_t:
            lateral = y + AHEAD_MM * math.sin(phi)   # line relative to probes, + = right
            mask = scan(lateral)
            scan_t += SCAN_S
        e = line_error(mask)
        target = w_cmd
        if e is not None:
            prev = err_f
            if not init:
                err_f = prev = e
                init = True
            else:
                err_f += DT / (0.02 + DT) * (e - err_f)
            deriv += DT / (0.03 + DT) * ((err_f - prev) / DT - deriv)
            target = max(-1.5, min(1.5, -kp * err_f - kd * deriv - kh * phi))
            lost = 0.0
        else:
            lost += DT
            if lost > 0.2:
                return None, trace
        step = alpha * DT
        w_cmd += max(-step, min(step, target - w_cmd))
        w_body += (w_cmd - w_body) * DT / lag
        phi += w_body * DT
        y += v * 1000 * math.sin(phi) * DT       # rotated left -> drifts left
        t += DT
        trace.append(y)
    tail = trace[int(len(trace) * 0.5):]
    return max(abs(x) for x in tail), trace


def main():
    lag, alpha = 0.12, 8.0
    print("kp     kd     | sway mm @0.20 | after-turn (line +35mm, 22deg) @0.20 | @0.12")
    for kp in (0.010, 0.015, 0.020):
        for kd in (0.002, 0.004, 0.006, 0.008, 0.010):
            sway, _ = run(kp, kd, 0.20, lag, alpha, 15, 3)
            rec, _ = run(kp, kd, 0.20, lag, alpha, 35, 22)
            slow, _ = run(kp, kd, 0.12, lag, alpha, 35, 22)
            fmt = lambda r: "LOST" if r is None else f"{r:5.1f}"
            print(f"{kp:.3f}  {kd:.3f}  |      {fmt(sway)}       |          {fmt(rec)}         |  {fmt(slow)}")


if __name__ == "__main__":
    main()
