"""Yaw tracking loop of the gimbal while the car moves past a fixed target.

Replicates the firmware (gimbal_task.c, track_filter.h): 2 ms control cycle, angle history
+ latency compensation, alpha-beta tracker, yaw PID, gyro feed-forward and the tracker
rate feed-forward. The control board (IMU) rides on the yaw stage, so the gyro sees the
stage's own world rotation, not only the chassis'.

Plant: JC4010 in speed mode = first-order speed lag after a CAN delay. Vision: 23 fps,
error measured at the capture instant, delivered `latency` later, with noise.

Run: py -3.13 gimbal/tools/track_sim.py
"""
import math
import random

CTRL_DT = 0.002
SUB = 4                       # physics sub-steps per control cycle
DEG_S_PER_RPM = 6.0

# firmware constants (gimbal_task.c)
KP, KI, KD = 2.8, 0.05, 0.10
INT_LIMIT, OUT_LIMIT = 600.0, 50.0
INT_DECAY, INT_DECAY_BAND = 0.90, 1.0
GYRO_FF_GAIN = 7.0            # g_yaw_ff_gain, sign -1
GATE, MAX_RATE, COAST, LOST = 5.0, 90.0, 0.450, 0.500


class Tracker:
    """track_filter.h (no wrap needed here)."""

    def __init__(self):
        self.active = False
        self.x = self.v = self.t = self.u_ref = 0.0

    def update(self, z, t, alpha, beta, u_at=0.0):
        """u_at: integrated vehicle-explained bearing change (deg) at the capture time t."""
        gap = t - self.t
        if not self.active or gap > LOST:
            self.active, self.x, self.v, self.t, self.u_ref = True, z, 0.0, t, u_at
            return
        dt = max(gap, 0.0)
        pred = self.x + self.v * dt + (u_at - self.u_ref)
        r = z - pred
        if abs(r) > GATE:
            self.x, self.v, self.t, self.u_ref = z, 0.0, t, u_at
            return
        self.x = pred + alpha * r
        if dt > 0:
            self.v = max(-MAX_RATE, min(MAX_RATE, self.v + beta * r / dt))
            self.t, self.u_ref = t, u_at

    def predict(self, now, u_now=0.0, u_rate=0.0):
        """Direction now and the feed-forward rate (residual + vehicle-explained)."""
        ahead = max(0.0, now - self.t)
        vehicle = u_now - self.u_ref
        if ahead >= COAST:
            return self.x + self.v * COAST + vehicle, u_rate
        return self.x + self.v * ahead + vehicle, self.v + u_rate


def push_profile(distance, duration):
    """Hand push: smooth (cosine) speed bump, rest before and after."""
    def x_at(t):
        if t <= 0:
            return 0.0
        if t >= duration:
            return distance
        u = t / duration
        return distance * (u - math.sin(2 * math.pi * u) / (2 * math.pi))
    return x_at


def drive_profile(speed, distance, accel=0.8, decel=1.0):
    """Chassis straight run: trapezoid with the firmware's ramps."""
    t1 = speed / accel
    d1 = 0.5 * accel * t1 * t1
    t3 = speed / decel
    d3 = 0.5 * decel * t3 * t3
    t2 = max(0.0, (distance - d1 - d3) / speed)

    def x_at(t):
        if t <= 0:
            return 0.0
        if t < t1:
            return 0.5 * accel * t * t
        if t < t1 + t2:
            return d1 + speed * (t - t1)
        if t < t1 + t2 + t3:
            s = t - t1 - t2
            return d1 + speed * t2 + speed * s - 0.5 * decel * s * s
        return d1 + speed * t2 + d3
    return x_at


def simulate(x_at, *, start_x, target_range, ff=1.0, ff_scale=1.0, kp=KP,
             alpha=0.5, beta=0.17, latency=0.080, latency_est=0.080, fps=23.0,
             motor_tau=0.020, can_delay=0.002, noise=0.03, gyro_noise=0.05,
             t_end=3.0, t_motion=0.5, seed=1, veh_ff=0.0, v_period=0.020, v_delay=0.020,
             range_err=0.0, angle_err=0.0):
    """Returns (true error trace [(t, deg)], vision-reported errors [(t, deg)]).
    veh_ff: scale of the vehicle-speed feed-forward u = v * cos(gimbal bearing) / range, with
    the car speed sampled every v_period and v_delay old (chassis $C frames), the range off by
    range_err (fraction, as the K230 would report it) and the gimbal-vs-chassis angle off by
    angle_err (deg, forward-offset calibration).
    Error = target bearing - laser bearing. The target sits target_range ahead and its
    bearing grows as x_at grows, so a laser lagging behind gives a positive error."""
    rng = random.Random(seed)
    theta = 0.0                    # stage world yaw (deg) = motor angle (chassis does not turn)
    motor_rpm = 0.0
    cmd_queue = [0.0] * max(1, round(can_delay / CTRL_DT))
    history = []                   # (t, theta)
    tracker = Tracker()
    target_yaw = None
    integral = prev = 0.0
    frames_due = []                # (arrival_t, capture_t, err)
    next_frame = rng.uniform(0, 1 / fps)
    trace, reported = [], []

    def bearing(t):
        x = start_x + x_at(t - t_motion)
        return math.degrees(math.atan2(x, target_range))

    def car_speed(t):
        return (x_at(t - t_motion + 0.001) - x_at(t - t_motion - 0.001)) / 0.002

    def distance(t):
        return math.hypot(start_x + x_at(t - t_motion), target_range)

    u_cum = 0.0                    # integrated vehicle-explained bearing change, deg
    v_seen, next_v, v_samples = 0.0, 0.0, []
    range_seen = None

    theta = bearing(0.0)           # laser on the target before the car moves
    t = 0.0
    while t < t_end:
        # camera shutter
        while next_frame <= t:
            err = bearing(next_frame) - theta_at(history, next_frame, theta) + rng.gauss(0, noise)
            frames_due.append((next_frame + latency, next_frame, err))
            next_frame += 1 / fps
        # frames arriving this cycle
        arrived = [f for f in frames_due if f[0] <= t]
        frames_due = [f for f in frames_due if f[0] > t]
        for arrival, captured, err in arrived:
            cap = arrival - latency_est
            z = theta_at(history, cap, theta) + err
            tracker.update(z, cap, alpha, beta, u_at(history, cap, u_cum))
            reported.append((t, err))
            range_seen = distance(captured) * (1.0 + range_err)
        # chassis speed as the $C frames deliver it
        if t >= next_v:
            v_samples.append((t + v_delay, car_speed(t)))
            next_v += v_period
        while v_samples and v_samples[0][0] <= t:
            v_seen = v_samples.pop(0)[1]
        u_rate = 0.0
        if veh_ff and range_seen:
            u_rate = veh_ff * math.degrees(v_seen * math.cos(math.radians(theta + angle_err)) / range_seen)
        u_cum += u_rate * CTRL_DT
        # control cycle (firmware order)
        history.append((t, theta, u_cum))
        if len(history) > 200:
            history.pop(0)
        rate = 0.0
        if tracker.active and t - tracker.t > LOST:
            tracker.active = False
            tracker.v = 0.0
        if tracker.active:
            target_yaw, rate = tracker.predict(t, u_cum, u_rate)
        elif target_yaw is None:
            target_yaw = theta
        e = target_yaw - theta
        if (e > 0 and prev < 0) or (e < 0 and prev > 0):
            integral = 0.0
        integral = max(-INT_LIMIT, min(INT_LIMIT, integral + e))
        out = kp * e + KI * integral + KD * (e - prev)
        prev = e
        out = max(-OUT_LIMIT, min(OUT_LIMIT, out))
        gyro = motor_rpm * DEG_S_PER_RPM + rng.gauss(0, gyro_noise)
        out += -1.0 * gyro / DEG_S_PER_RPM * GYRO_FF_GAIN
        out += ff * ff_scale * rate / DEG_S_PER_RPM
        if abs(e) < INT_DECAY_BAND:
            integral *= INT_DECAY
        out = max(-OUT_LIMIT, min(OUT_LIMIT, out))
        cmd_queue.append(out)
        cmd = cmd_queue.pop(0)
        # plant
        for _ in range(SUB):
            h = CTRL_DT / SUB
            motor_rpm += (cmd - motor_rpm) * h / motor_tau
            theta += motor_rpm * DEG_S_PER_RPM * h
        t += CTRL_DT
        trace.append((t, bearing(t) - theta))
    return trace, reported


def simulate_corner(*, structure="current", kp_scale=1.0, kpos=8.0, krate=GYRO_FF_GAIN,
                    chassis_ff=1.0, gz_period=0.020, gz_delay=0.020, motor_tau=0.020,
                    can_delay=0.002, turn_deg=90.0, w_max=2.26, w_accel=8.0, gyro_noise=0.05,
                    seed=1):
    """A chassis corner with vision held (as the firmware does): the gimbal must keep its
    world heading while the body turns turn_deg. Returns the peak world error (deg).

    structure "current": PID (kp_scale x Kp) - K/6 * stage gyro (as flashed).
    structure "two_loop": w* = kpos * e; u = (w* - chassis_ff * gz + krate * (w* - gyro)) / 6,
    gz = chassis gyro from the $C frames: sampled every gz_period, gz_delay old."""
    rng = random.Random(seed)
    w_max, w_accel = math.degrees(w_max), math.degrees(w_accel)
    heading = rate = 0.0               # chassis
    motor_rpm = theta_motor = 0.0      # gimbal motor angle relative to the chassis
    cmd_queue = [0.0] * max(1, round(can_delay / CTRL_DT))
    gz_samples = []                    # (time available, value)
    gz_seen = 0.0
    integral = prev = 0.0
    peak, t = 0.0, 0.0
    next_ctrl, next_gz = 0.0, 0.0
    h = CTRL_DT / SUB
    while t < 2.5:
        remaining = turn_deg - heading
        want = 0.0 if remaining <= 0 else min(w_max, math.sqrt(2 * w_accel * remaining))
        rate += max(-w_accel * h, min(w_accel * h, want - rate))
        heading += rate * h
        world = heading + theta_motor              # target world heading is 0
        if t >= next_gz:
            gz_samples.append((t + gz_delay, rate))
            next_gz += gz_period
        while gz_samples and gz_samples[0][0] <= t:
            gz_seen = gz_samples.pop(0)[1]
        if t >= next_ctrl:
            next_ctrl += CTRL_DT
            e = -world
            gyro = rate + motor_rpm * DEG_S_PER_RPM + rng.gauss(0, gyro_noise)
            if structure == "current":
                if (e > 0 and prev < 0) or (e < 0 and prev > 0):
                    integral = 0.0
                integral = max(-INT_LIMIT, min(INT_LIMIT, integral + e))
                out = KP * kp_scale * e + KI * integral + KD * (e - prev)
                prev = e
                out = max(-OUT_LIMIT, min(OUT_LIMIT, out))
                out += -GYRO_FF_GAIN * gyro / DEG_S_PER_RPM
                if abs(e) < INT_DECAY_BAND:
                    integral *= INT_DECAY
            else:
                w_star = kpos * e
                out = (w_star - chassis_ff * gz_seen + krate * (w_star - gyro)) / DEG_S_PER_RPM
            cmd_queue.append(max(-OUT_LIMIT, min(OUT_LIMIT, out)))
            cmd = cmd_queue.pop(0)
        motor_rpm += (cmd - motor_rpm) * h / motor_tau
        theta_motor += motor_rpm * DEG_S_PER_RPM * h
        peak = max(peak, abs(world))
        t += h
    return peak


def theta_at(history, when, current):
    for sample in reversed(history):
        if sample[0] <= when:
            return sample[1]
    return history[0][1] if history else current


def u_at(history, when, current):
    for sample in reversed(history):
        if sample[0] <= when:
            return sample[2]
    return history[0][2] if history else current


def summary(trace, reported, t_motion, t_stop):
    moving = [e for t, e in trace if t_motion <= t <= t_stop]
    after = [e for t, e in trace if t > t_stop]
    rep = [e for _, e in reported]
    return (max(moving), min(after) if after else 0.0,
            max(rep), min(rep))


def run_case(name, x_at, t_stop, **kw):
    trace, reported = simulate(x_at, t_motion=0.5, **kw)
    lag, over, rmax, rmin = summary(trace, reported, 0.5, t_stop)
    print(f"{name:34s} lag {lag:5.2f}  overshoot {over:6.2f}   vision max {rmax:5.2f} min {rmin:6.2f}")


def main():
    """Findings (2026-10-21):
    - The stage gyro feed-forward (-7/6 * own world rate) is a rate damper: any rpm command
      only yields 1/(1+K) = 1/8 of its rate. The tracker rate feed-forward as first flashed
      (v/6 rpm) therefore removed only ~1/8 of the lag, and the PID's effective Kp is
      2.8 rpm/deg * 6/8 = 2.1 deg/s per deg (tau ~0.5 s): the yaw axis is sluggish.
    - Scaling the rate feed-forward by (1+K) roughly halves the lag; raising Kp (2-8x) and
      beta 0.3 cut lag and stop overshoot further. Hard stops still overshoot because the
      tracker needs a few frames to see the rate drop (chassis-speed feed-forward fixes that).
    The model matches the field ff-off run (3.98 deg) with a ~0.3 m/s hand push; the single
    ff-on run (5.16 deg) is not reproduced and is likely push-to-push spread.
    Firmware since then: rate feed-forward x (1+K) by default ("ff x (1+K)" rows), Kp via
    tune_yaw_kp_scale, beta via tune_track_beta."""
    damping = 1.0 + GYRO_FF_GAIN      # the stage gyro feed-forward divides rate commands by this
    push = push_profile(0.3, 2.0)     # ~0.3 m/s peak: matches the field 3.98 deg with ff off
    drive = drive_profile(0.35, 0.9)  # chassis cruise 0.35 m/s past the target
    cases = [
        ("ff off (field: 3.98)", dict(ff=0.0)),
        ("ff on, as flashed (field: 5.16)", dict(ff=1.0)),
        ("ff x (1+K)", dict(ff=1.0, ff_scale=damping)),
        ("ff x (1+K), kp x2", dict(ff=1.0, ff_scale=damping, kp=2 * KP)),
        ("ff x (1+K), kp x4", dict(ff=1.0, ff_scale=damping, kp=4 * KP)),
        ("ff x (1+K), kp x4, beta 0.3", dict(ff=1.0, ff_scale=damping, kp=4 * KP, beta=0.3)),
        ("ff x (1+K), kp x8, beta 0.3", dict(ff=1.0, ff_scale=damping, kp=8 * KP, beta=0.3)),
        ("  same, true latency 110 ms", dict(ff=1.0, ff_scale=damping, kp=8 * KP, beta=0.3,
                                             latency=0.110)),
    ]
    drive_stop = 0.5 + 0.35 / 0.8 + (0.9 - 0.35**2 / 1.6 - 0.35**2 / 2) / 0.35 + 0.35
    for title, x_at, dist, t_stop in (("hand push 0.3 m / 2 s", push, 0.3, 2.5),
                                      ("chassis 0.35 m/s, 0.9 m", drive, 0.9, drive_stop)):
        print(f"\n== {title}: true error (deg), lag = behind the target, overshoot = past it ==")
        for name, kw in cases:
            run_case(name, x_at, t_stop, start_x=-dist / 2, target_range=0.8,
                     t_end=t_stop + 1.5, **kw)
    print("\nmotor lag sensitivity (hand push, ff x(1+K), kp x8, beta 0.3):")
    for tau in (0.010, 0.020, 0.040):
        run_case(f"motor tau {tau * 1000:.0f} ms", push, 2.5, start_x=-0.15, target_range=0.8,
                 ff=1.0, ff_scale=damping, kp=8 * KP, beta=0.3, motor_tau=tau, t_end=4.0)

    print("\n== DRIVE 20 60, target 0.48 m to the side (field result7 geometry): vehicle feed-forward ==")
    drive20 = drive_profile(0.20, 0.60)
    t_stop = 0.5 + 0.25 + (0.60 - 0.025 - 0.02) / 0.20 + 0.20
    base = dict(start_x=-0.06, target_range=0.48, t_end=t_stop + 1.5, ff=1.0, ff_scale=damping,
                fps=20.0, noise=0.05)
    for name, kw in (("as flashed (field result7: max 3.7)", dict()),
                     ("beta 0.3, kp x4 (tuning only)", dict(beta=0.3, kp=4 * KP)),
                     ("  + vehicle ff ($C 50 Hz, 20 ms old)", dict(beta=0.3, kp=4 * KP, veh_ff=1.0)),
                     ("  + range 15% off, angle 5 deg off", dict(beta=0.3, kp=4 * KP, veh_ff=1.0,
                                                               range_err=0.15, angle_err=5.0)),
                     ("vehicle ff, no other tuning", dict(veh_ff=1.0)),
                     ("vehicle ff with the wrong sign", dict(beta=0.3, kp=4 * KP, veh_ff=-1.0))):
        run_case(name, drive20, t_stop, **dict(base, **kw))

    print("\n== chassis 90 deg corner, vision held: peak laser swing (deg) ==")
    for scale in (1, 4):
        print(f"current structure, kp x{scale}            {simulate_corner(kp_scale=scale):5.2f}")
    for kpos in (4.0, 8.0, 16.0):
        print(f"two-loop kpos {kpos:4.1f}, no chassis ff     "
              f"{simulate_corner(structure='two_loop', kpos=kpos, chassis_ff=0.0):5.2f}")
        print(f"two-loop kpos {kpos:4.1f}, chassis gz ff     "
              f"{simulate_corner(structure='two_loop', kpos=kpos):5.2f}"
              f"   (gz ideal: {simulate_corner(structure='two_loop', kpos=kpos, gz_period=0.002, gz_delay=0.0):4.2f})")
    print("two-loop kpos 8, gz 40 ms old          "
          f"{simulate_corner(structure='two_loop', gz_delay=0.040):5.2f}")


if __name__ == "__main__":
    main()
