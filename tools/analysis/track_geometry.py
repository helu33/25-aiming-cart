"""Reconstruct the track geometry: chassis odometry + gimbal target bearing -> target position.

    py -3.13 tools/analysis/track_geometry.py logs/2026-10/final [sign]

Reads RUN.csv and RUN_chassis.txt. The chassis log gives the wheel speed and gz, integrated into
an odometry track (x, y, heading). The gimbal telemetry gives, for every fresh vision frame, the
bearing of the target in the gimbal IMU frame (integrated stage gyro minus the vision yaw error,
at the capture time L_VIS earlier). Model: the gimbal yaw axis sits df metres ahead of the
wheel-axle centre, and

    bearing = atan2(T - P_gimbal) + c0 + c1 * t

(c0: frame offset, c1: gimbal yaw drift). The fit per lap gives the target position T in
odometry metres and the residual (deg). face_error_cm.py and circle_fit.py build on load() and
fit(). sign = -1 flips the vision error sign (to check the convention).
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "gimbal" / "tools"))
import run_report as R  # noqa: E402
import telemetry_log as T  # noqa: E402

COL = {n: i for i, n in enumerate(T.COLUMNS)}
L_VIS = 0.075      # vision capture latency, s (fitted on the 2026-10 runs)
LINK = 0.02        # $C link delay, s (fitted on the 2026-10 runs)


def load(stem):
    """Telemetry rows, chassis odometry, vision frames (t, bearing), clock offset, corner spans."""
    rows = [r for r in T.read_csv(stem + '.csv') if T.plausible(r)]
    ch = R.read_chassis_log(stem + '_chassis.txt')
    off = R.chassis_offset(rows, COL, ch, R.channel10_is_gz(stem))
    # chassis odometry (chassis seconds)
    ct = np.array([c['t_ms'] / 1000.0 for c in ch])
    v = np.array([c['vmeas'] / 1000.0 for c in ch])
    gz = np.array([c['gz'] / 1000.0 for c in ch])
    psi = np.concatenate([[0.0], np.cumsum(0.5 * (gz[1:] + gz[:-1]) * np.diff(ct))])
    x = np.concatenate([[0.0], np.cumsum(0.5 * (v[1:] * np.cos(psi[1:]) + v[:-1] * np.cos(psi[:-1])) * np.diff(ct))])
    y = np.concatenate([[0.0], np.cumsum(0.5 * (v[1:] * np.sin(psi[1:]) + v[:-1] * np.sin(psi[:-1])) * np.diff(ct))])
    odo = dict(t=ct, v=v, gz=gz, psi=psi, x=x, y=y, state=np.array([c['state'] for c in ch]))
    # gimbal: stage world angle, target bearing at capture time (gimbal seconds)
    gt = np.array([r[0] for r in rows])
    gyro = np.array([r[COL['gyro_z']] for r in rows])
    th = np.concatenate([[0.0], np.cumsum(0.5 * (gyro[1:] + gyro[:-1]) * np.diff(gt))])
    frames, last = [], None
    for i, r in enumerate(rows):
        if not int(r[3]) & T.FLAG_VALID:
            last = None
            continue
        if last is not None and r[1] == last:
            continue                                   # held value, not a new frame
        last = r[1]
        tc = r[0] - L_VIS
        frames.append((tc, np.interp(tc, gt, th) - r[1], i))
    fr = np.array([(a, b) for a, b, _ in frames])
    idx = [i for _, _, i in frames]
    spans = [(a + off, b + off) for a, b in R.chassis_corners(ch)]
    return rows, odo, fr, idx, off, spans, gt


def odo_at(odo, tch):
    return {k: np.interp(tch, odo['t'], odo[k]) for k in ('v', 'gz', 'psi', 'x', 'y')}


def bearing(p, o, tch):
    tx, ty, c0, c1, df = p
    gx = o['x'] + df * np.cos(o['psi'])
    gy = o['y'] + df * np.sin(o['psi'])
    return np.degrees(np.arctan2(ty - gy, tx - gx)) + c0 + c1 * tch


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def fit(theta, tch, o, p0, free=(0, 1, 2, 3, 4), iters=30):
    """Gauss-Newton fit of p = (tx, ty, c0, c1, df); only the indices in free move."""
    p = np.array(p0, float)
    for _ in range(iters):
        model = bearing(p, o, tch)
        res = wrap(theta - model)
        J = np.zeros((len(res), len(free)))          # d(model)/dp
        for k, j in enumerate(free):
            dp = np.zeros(5)
            dp[j] = 1e-5
            J[:, k] = wrap(bearing(p + dp, o, tch) - model) / 1e-5
        step, *_ = np.linalg.lstsq(J, res, rcond=None)
        for k, j in enumerate(free):
            p[j] += step[k]
        if np.max(np.abs(step)) < 1e-7:
            break
    res = wrap(theta - bearing(p, o, tch))
    return p, res


def init_guess(theta, tch, o):
    """Grid search of the target position (+-3 m, 0.1 m) with the best constant offset c0."""
    best = None
    for tx in np.arange(-3.0, 3.01, 0.1):
        for ty in np.arange(-3.0, 3.01, 0.1):
            b = np.degrees(np.arctan2(ty - o['y'], tx - o['x']))
            d = np.radians(wrap(theta - b))
            c0 = math.degrees(math.atan2(np.mean(np.sin(d)), np.mean(np.cos(d))))
            rms = np.sqrt(np.mean(wrap(theta - b - c0) ** 2))
            if best is None or rms < best[0]:
                best = (rms, tx, ty, c0)
    return best


def laps(odo, spans, off):
    """(start, end) of every full lap in chassis seconds: 4 corner ends per lap."""
    ends = [b - off for _, b in spans]
    return [(ends[k - 1] if k else odo['t'][0], ends[k + 3]) for k in range(0, len(ends) - 3, 4)]


if __name__ == '__main__':
    stem = sys.argv[1]
    sign = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    rows, odo, fr, idx, off, spans, gt = load(stem)
    tch = fr[:, 0] - (off - LINK)
    keep = (tch > odo['t'][0]) & (tch < odo['t'][-1])
    tch, theta = tch[keep], sign * fr[keep, 1]
    o = odo_at(odo, tch)
    print(f"{stem}: {len(tch)} frames, odometry heading total {math.degrees(odo['psi'][-1]):.1f} deg")
    for a, b in laps(odo, spans, off):
        m = (tch >= a) & (tch < b)
        ow = {k: v[m] for k, v in o.items()}
        g = init_guess(theta[m], tch[m], ow)
        p, res = fit(theta[m], tch[m], ow, [g[1], g[2], g[3], 0.0, 0.0])
        print(f"  lap {a:5.1f}-{b:5.1f}s: grid rms {g[0]:.2f}  fit T=({p[0]:+.3f},{p[1]:+.3f}) m c0 {p[2]:+.1f} "
              f"c1 {p[3]:+.4f} deg/s df {p[4]*100:+.1f} cm  rms {np.sqrt(np.mean(res**2)):.3f} deg")
