"""Was the circle (task 3) really drawn during a lap? Fit the gimbal bearing three ways.

    py -3.13 tools/analysis/circle_fit.py logs/2026-10/t3a

Reads RUN.csv and RUN_chassis.txt of a 1-lap circle run (CIRCLE mode / PB5 key). The run
progress p (0..1) comes from the chassis wheel distance between RUN and DONE.
  1. fixed point: the bearing to one target point (track_geometry.py model, df = 0);
  2. + circle -sin(2 pi p): the target point plus a horizontal offset s(t) * (a, b);
  3. + free phase: offset sin(2 pi p) * u + cos(2 pi p) * w, reports amplitude and phase.
A drawn circle shows an amplitude near 60 mm (the circle radius) and a clearly lower rms than
the fixed point. Also the laser error against the commanded circle point, in cm on the face
(3D range with the target H above the camera, see face_error_cm.py), and every frame over 1.5 cm.
"""
import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "gimbal" / "tools"))
import track_geometry as G  # noqa: E402
import run_report as R  # noqa: E402

H = 0.182          # target centre above the camera, m

stem = sys.argv[1]
rows, odo, fr, idx, off, spans, gt = G.load(stem)
ch = R.read_chassis_log(stem + '_chassis.txt')
ct = np.array([c['t_ms'] / 1000.0 for c in ch])
v = np.array([c['vmeas'] / 1000.0 for c in ch])
dist = np.concatenate([[0.0], np.cumsum(0.5 * (v[1:] + v[:-1]) * np.diff(ct))])
running = np.array([c['state'] not in (0, 6) for c in ch])
t_run0 = ct[np.argmax(running)]
t_done = ct[len(running) - 1 - np.argmax(running[::-1])]
total = np.interp(t_done, ct, dist) - np.interp(t_run0, ct, dist)

tch = fr[:, 0] - (off - G.LINK)
m = (tch >= t_run0) & (tch <= t_done)
tch, theta = tch[m], fr[m, 1]
o = G.odo_at(odo, tch)
prog = np.clip((np.interp(tch, ct, dist) - np.interp(t_run0, ct, dist)) / total, 0.0, 1.0)
ang = 2 * math.pi * prog


def bearing(p, extra):
    tx, ty, c0, c1 = p[:4]
    gx, gy = o['x'], o['y']                     # gimbal at the axle centre (df = 0)
    ox, oy = extra(p)
    return np.degrees(np.arctan2(ty + oy - gy, tx + ox - gx)) + c0 + c1 * tch


def fit(p0, extra, iters=40):
    p = np.array(p0, float)
    for _ in range(iters):
        model = bearing(p, extra)
        res = G.wrap(theta - model)
        J = np.zeros((len(res), len(p)))
        for j in range(len(p)):
            dp = np.zeros(len(p))
            dp[j] = 1e-6
            J[:, j] = G.wrap(bearing(p + dp, extra) - model) / 1e-6
        step, *_ = np.linalg.lstsq(J, res, rcond=None)
        p += step
        if np.max(np.abs(step)) < 1e-8:
            break
    return p, G.wrap(theta - bearing(p, extra))


def rms(r):
    return math.sqrt(np.mean(r ** 2))


g = G.init_guess(theta, tch, o)
p0, r0 = fit([g[1], g[2], g[3], 0.0], lambda p: (0.0, 0.0))
p1, r1 = fit(list(p0) + [0.0, 0.0], lambda p: (-np.sin(ang) * p[4], -np.sin(ang) * p[5]))
p2, r2 = fit(list(p0) + [0.0, 0.0, 0.0, 0.0],
             lambda p: (np.sin(ang) * p[4] + np.cos(ang) * p[6], np.sin(ang) * p[5] + np.cos(ang) * p[7]))
rng = np.hypot(np.hypot(p0[0] - o['x'], p0[1] - o['y']), H)
print(f"{os.path.basename(stem)}: {len(tch)} frames, run {t_done - t_run0:.2f} s, wheel distance {total:.2f} m, "
      f"range {rng.min():.2f}-{rng.max():.2f} m")
print(f"  fixed point          rms {rms(r0):.3f} deg  T=({p0[0]:+.3f},{p0[1]:+.3f})")
print(f"  + circle -sin(2pi p) rms {rms(r1):.3f} deg  amplitude {math.hypot(p1[4], p1[5]) * 1000:5.1f} mm "
      f"axis {math.degrees(math.atan2(p1[5], p1[4])):+.0f} deg")
u, w = np.array(p2[4:6]), np.array(p2[6:8])
# horizontal displacement = A sin(ang + phi) along one axis: project both on the dominant axis
axis = u / (np.linalg.norm(u) + 1e-12) if np.linalg.norm(u) >= np.linalg.norm(w) else w / (np.linalg.norm(w) + 1e-12)
a_s, a_c = float(u @ axis), float(w @ axis)
print(f"  + free phase         rms {rms(r2):.3f} deg  |sin| {np.linalg.norm(u) * 1000:5.1f} mm |cos| "
      f"{np.linalg.norm(w) * 1000:5.1f} mm  amplitude {math.hypot(a_s, a_c) * 1000:5.1f} mm "
      f"phase {math.degrees(math.atan2(a_c, a_s)):+.0f} deg")
# laser vs the commanded circle point (K230 vision error, valid frames), cm on the face
err = np.array([rng[k] * math.tan(math.radians(math.hypot(rows[idx[i]][1], rows[idx[i]][2]))) * 100
                for k, i in enumerate(np.nonzero(m)[0])])
print(f"  error on the face: p50 {np.percentile(err, 50):.2f} p95 {np.percentile(err, 95):.2f} "
      f"p99 {np.percentile(err, 99):.2f} max {err.max():.2f} cm, over 2 cm {np.mean(err > 2) * 100:.1f} % "
      f"({int(np.sum(err > 2))} frames)")
for k in np.nonzero(err > 1.5)[0]:
    print(f"    t {tch[k] - t_run0:5.2f} s  progress {prog[k]:.3f}  range {rng[k]:.2f} m  err {err[k]:.2f} cm")
