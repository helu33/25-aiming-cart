"""Two-loop yaw tracking with the K230's variable frame latency.

Field 2026-10-24 (lap r10_ph_pos3, lap 2): on a side edge the yaw tracker rate swung +26 /
-16 deg/s, the motor saturated and the gimbal oscillated at ~4.5 Hz for ~1 s (3.8 deg). The
K230 needs 37-86 ms per frame (log ms=/max_ms=), so a frame is 45-120 ms old when it arrives,
while the gimbal assumes a fixed `latency` (80 ms) to place it in its angle history and to
time-stamp it for the tracker. Two frames arriving 40 ms apart may have been captured 80 ms
apart (or the other way round): the tracker's rate update beta*r/dt is off by up to 2x, and
at 40 deg/s every 10 ms of latency error is 0.4 deg of bearing error.

Model: firmware loop (2 ms): angle history, alpha-beta tracker (settle after a reset), vehicle
feed-forward, two-loop yaw (w* = ff*rate + clamp(kpos*e), rpm = (w* + krate*(w* - gyro))/6),
JC4010 speed lag. K230: the sensor streams at 30 fps; each loop takes the newest frame,
processes it for p ms (80 %: ~38 ms, 20 %: ~80 ms) and sends it with age p. True latency =
p + how long the frame had waited in the sensor + 5 ms exposure middle.
Scene: the car passes a target 0.45 m to the side at 0.35 m/s (bearing rate up to 45 deg/s).

Run: py -3.13 gimbal/tools/jitter_sim.py
"""
import math
import random

DT = 0.002
KPOS, KRATE, POS_MAX, OUT_LIMIT = 8.0, 7.0, 40.0, 50.0
GATE, MAX_RATE, COAST, LOST = 5.0, 90.0, 0.450, 0.500


def run(seed, *, use_age=False, latency=0.080, extra=0.022, alpha=0.5, beta=0.17, ff=1.0,
        settle=0.300, veh_err=0.10, speed=0.35, rng_range=0.45, seconds=4.0, motor_tau=0.020,
        slow_share=0.2, fast_share=0.0, noise=0.03, outlier=0.0, rate_clip=None):
    rng = random.Random(seed)
    t_pass = 2.0                                   # closest approach time
    bearing = lambda t: math.degrees(math.atan2(speed * (t - t_pass), rng_range))
    bearing_rate = lambda t: math.degrees(speed * rng_range / (rng_range ** 2 + (speed * (t - t_pass)) ** 2))

    theta = bearing(0.0)
    rpm = 0.0
    history = []                                   # (t, theta, u_cum)
    x = v = t_upd = u_ref = 0.0
    active, settle_until = False, -1.0
    target = theta
    u_cum = 0.0
    frames = []                                    # (arrival, capture, error)
    loop_free = 0.0                                # K230 loop ready for the next snapshot
    peak_gyro = peak_err = 0.0
    resets = 0
    t = 0.0
    while t < seconds:
        # K230 loop: snapshot the newest sensor frame (30 fps stream), process, send
        if t >= loop_free:
            sensor_t = math.floor(t * 30.0) / 30.0           # newest frame start
            capture = sensor_t + 0.005                        # exposure middle
            roll = rng.random()
            p = (rng.gauss(0.080, 0.004) if roll < slow_share else
                 rng.gauss(0.024, 0.003) if roll < slow_share + fast_share else rng.gauss(0.038, 0.002))
            arrival = t + p + 0.001
            err = bearing(capture) - angle_at(history, capture, theta) + rng.gauss(0, noise)
            if rng.random() < outlier:                         # pose jitter at oblique views
                err += rng.choice((-1, 1)) * rng.uniform(0.5, 1.5)
            frames.append((arrival, capture, err, p))
            loop_free = arrival
        for f in [f for f in frames if f[0] <= t]:
            frames.remove(f)
            arrival, capture, err, age = f
            lat = (age + extra) if use_age else latency
            cap_est = arrival - lat
            z = angle_at(history, cap_est, theta) + err
            u_at = u_hist_at(history, cap_est, u_cum)
            gap = cap_est - t_upd
            if not active or gap > LOST:
                active, x, v, t_upd, u_ref = True, z, 0.0, cap_est, u_at
                settle_until = cap_est + settle
                resets += 1
            else:
                dt = max(gap, 0.0)
                pred = x + v * dt + (u_at - u_ref)
                r = z - pred
                if abs(r) > GATE:
                    x, v, t_upd, u_ref = z, 0.0, cap_est, u_at
                    settle_until = cap_est + settle
                    resets += 1
                else:
                    x = pred + alpha * r
                    if dt > 0:
                        rr = r if rate_clip is None else max(-rate_clip, min(rate_clip, r))
                        v = max(-MAX_RATE, min(MAX_RATE, v + beta * rr / dt))
                        t_upd, u_ref = cap_est, u_at
                if cap_est < settle_until:
                    v = 0.0
        # vehicle feed-forward (chassis speed known, 10 % scale error)
        u_rate = bearing_rate(t) * (1.0 + veh_err)
        u_cum += u_rate * DT
        history.append((t, theta, u_cum))
        if len(history) > 200:
            history.pop(0)
        rate = 0.0
        if active and t - t_upd > LOST:
            active = False
        if active:
            ahead = max(0.0, t - t_upd)
            if ahead >= COAST:
                target, rate = x + v * COAST + (u_cum - u_ref), u_rate
            else:
                target, rate = x + v * ahead + (u_cum - u_ref), v + u_rate
        e = target - theta
        pos = max(-POS_MAX, min(POS_MAX, KPOS * e))
        w_star = ff * rate + pos
        gyro = rpm * 6.0
        cmd = max(-OUT_LIMIT, min(OUT_LIMIT, (w_star + KRATE * (w_star - gyro)) / 6.0))
        for _ in range(4):
            rpm += (cmd - rpm) * (DT / 4) / motor_tau
            theta += rpm * 6.0 * DT / 4
        peak_gyro = max(peak_gyro, abs(rpm * 6.0 - bearing_rate(t)))
        if t > 0.5:
            peak_err = max(peak_err, abs(bearing(t) - theta))
        t += DT
    return peak_err, peak_gyro, resets


def angle_at(history, when, current):
    for k in range(len(history) - 1, -1, -1):
        if history[k][0] <= when:
            return history[k][1]
    return history[0][1] if history else current


def u_hist_at(history, when, current):
    for k in range(len(history) - 1, -1, -1):
        if history[k][0] <= when:
            return history[k][2]
    return history[0][2] if history else current


def sweep(label, n=200, **kw):
    res = [run(seed, **kw) for seed in range(n)]
    errs = sorted(r[0] for r in res)
    osc = sum(1 for r in res if r[1] > 40.0) / n           # gimbal rate off the target rate by > 40 deg/s
    print(f"{label:44s} err p50 {errs[n // 2]:4.2f}  p95 {errs[int(n * .95)]:4.2f}  max {errs[-1]:5.2f} deg | "
          f"oscillation {osc:5.1%} | resets/run {sum(r[2] for r in res) / n:4.1f}")


def main():
    field = dict(fast_share=0.25, noise=0.08, outlier=0.03)     # r10: 25 % of frames 20-30 ms apart
    sweep("field-like, fixed latency 80 ms (default)", **field)
    sweep("field-like, per-frame age (useage 1)", use_age=True, **field)
    sweep("field-like, fixed 80 ms, beta 0.10", beta=0.10, **field)
    sweep("field-like, fixed 80 ms, alpha 0.35", alpha=0.35, **field)
    sweep("field-like, per-frame age, alpha 0.35", use_age=True, alpha=0.35, **field)
    sweep("field-like, rate residual clip 0.5 deg", rate_clip=0.5, **field)
    sweep("field-like, rate clip 0.3 deg", rate_clip=0.3, **field)
    sweep("field-like, rate clip 0.5, per-frame age", rate_clip=0.5, use_age=True, **field)
    sweep("field-like, beta 0.10, per-frame age", beta=0.10, use_age=True, **field)
    sweep("no outliers, rate clip 0.5 (cost check)", rate_clip=0.5)
    sweep("no outliers, beta 0.10 (cost check)", beta=0.10)
    sweep("field-like, fixed 80 ms, closer target 0.30 m", rng_range=0.30, **field)
    sweep("field-like, per-frame age, closer target 0.30 m", use_age=True, rng_range=0.30, **field)
    sweep("fixed latency 80 ms (field default)")
    sweep("per-frame age + 22 ms (useage 1)", use_age=True)
    sweep("fixed 80 ms, beta 0.10", beta=0.10)
    sweep("fixed 80 ms, alpha 0.35", alpha=0.35)
    sweep("per-frame age, alpha 0.35", use_age=True, alpha=0.35)
    sweep("fixed 80 ms, all frames fast (no jitter)", slow_share=0.0, latency=0.060)


if __name__ == "__main__":
    main()
