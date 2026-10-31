"""Tracking error on the target face in cm (the contest measures D1 there: <= 2 cm).

    py -3.13 tools/analysis/face_error_cm.py logs/2026-10/final logs/2026-10/final2

Reads RUN.csv and RUN_chassis.txt. A per-lap geometry fit (track_geometry.py) gives the
horizontal range camera -> target for every vision frame; the 3D range adds the target height
above the camera (H, measured on the 2026-10 setup: change it for another build). The angular
error from the K230 (yaw, pitch) becomes cm on the face: range * tan(error). Prints p50 / p95 /
p99 / max, the share over 2 cm and 1.5 cm, and p99 / max per range band.
"""
import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import track_geometry as G  # noqa: E402

H = 0.182          # target centre above the camera, m

for stem in sys.argv[1:]:
    rows, odo, fr, idx, off, spans, gt = G.load(stem)
    tch_all = fr[:, 0] - (off - G.LINK)
    ends = [b - off for _, b in spans]
    errs, ranges, deg, tags = [], [], [], []
    for a, b in G.laps(odo, spans, off):
        m = (tch_all >= a) & (tch_all < b)
        tch, theta = tch_all[m], fr[m, 1]
        o = G.odo_at(odo, tch)
        g = G.init_guess(theta, tch, o)
        p, res = G.fit(theta, tch, o, [g[1], g[2], g[3], 0.0, 0.0], free=(0, 1, 2, 3))
        for k, i in enumerate(np.nonzero(m)[0]):
            r = rows[idx[i]]
            rh = math.hypot(p[0] - o['x'][k], p[1] - o['y'][k])
            rng = math.hypot(rh, H)
            e = math.hypot(r[1], r[2])
            errs.append(rng * math.tan(math.radians(e)) * 100.0)
            ranges.append(rng)
            deg.append(e)
            tags.append(sum(1 for x in ends if x <= tch[k]) % 4)
    E = np.array(errs)
    Rg = np.array(ranges)
    print(f"{os.path.basename(stem)}: range {Rg.min():.2f}-{Rg.max():.2f} m | error on the face cm: "
          f"p50 {np.percentile(E,50):.2f} p95 {np.percentile(E,95):.2f} p99 {np.percentile(E,99):.2f} "
          f"max {E.max():.2f} | > 2 cm {100*np.mean(E>2):.2f}% , > 1.5 cm {100*np.mean(E>1.5):.2f}%")
    for lo in (0.3, 0.7, 1.1, 1.5):
        mm = (Rg >= lo) & (Rg < lo + 0.4)
        if mm.sum():
            print(f"     range {lo:.1f}-{lo+0.4:.1f} m: n {mm.sum():4d} p99 {np.percentile(E[mm],99):.2f} max {E[mm].max():.2f} cm"
                  f"  (deg p99 {np.percentile(np.array(deg)[mm],99):.2f})")
