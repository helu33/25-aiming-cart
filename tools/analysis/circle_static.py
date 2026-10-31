"""Static circle test (chassis command CIRCLE <s>, car standing): gimbal motion over time.

    py -3.13 tools/analysis/circle_static.py logs/2026-10/c20f

Reads RUN.csv recorded with tsel 1 (telemetry channel 10 = pitch track rate, deg/s). Integrates
the stage yaw gyro and the pitch track rate from the start and prints them once a second with
the vision errors, target area (%), flags and vision code: during the test the yaw/pitch columns
should trace one circle of about atan(60 mm / range) in <s> seconds.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "gimbal" / "tools"))
import telemetry_log as T  # noqa: E402

PITCH_TRACK_RATE = 10      # channel index with tsel 1

col = {n: i for i, n in enumerate(T.COLUMNS)}
rows = [r for r in T.read_csv(sys.argv[1] + '.csv') if T.plausible(r)]
t0 = rows[0][0]
yaw = pit = 0.0
track = []
for a, b in zip(rows, rows[1:]):
    dt = b[0] - a[0]
    yaw += 0.5 * (a[col['gyro_z']] + b[col['gyro_z']]) * dt
    pit += 0.5 * (a[PITCH_TRACK_RATE] + b[PITCH_TRACK_RATE]) * dt
    track.append((b[0] - t0, yaw, pit, b))
print(" t    yaw_deg  pitch_deg  vis_yaw vis_pitch  area  flags code")
for k in range(0, len(track), 50):
    t, y, p, r = track[k]
    print(f"{t:5.1f} {y:8.2f} {p:9.2f}  {r[1]:7.2f} {r[2]:8.2f} {r[col['vis_area']]:5.1f} "
          f"{int(r[3]):5d} {int(r[col['vis_code']]):4d}")
