"""Gimbal error in the first second after each corner, per corner number (c1..c4, all laps).

    py -3.13 tools/analysis/corner_exit.py logs/2026-10/final logs/2026-10/e100

Reads RUN.csv and RUN_chassis.txt. For each corner: yaw rms / max, the yaw max split into
0-0.4 s and 0.4-1 s after the corner end, and pitch rms / max (deg, valid vision frames only).
The exit of corner 4 was the worst section in the 2026-10 runs (the target is close there).
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "gimbal" / "tools"))
import run_report as R  # noqa: E402
import telemetry_log as T  # noqa: E402

col = {n: i for i, n in enumerate(T.COLUMNS)}


def rms(x):
    return math.sqrt(sum(y * y for y in x) / len(x)) if x else float('nan')


for s in sys.argv[1:]:
    rows = [r for r in T.read_csv(s + '.csv') if T.plausible(r)]
    ch = R.read_chassis_log(s + '_chassis.txt')
    spans = R.run_spans(rows, col, ch, R.channel10_is_gz(s))
    out = []
    for k in range(1, 5):
        y, p, y1, y2 = [], [], [], []
        for i in range(k - 1, len(spans), 4):
            b = spans[i][1]
            for r in rows:
                if b <= r[0] < b + 1.0 and int(r[3]) & T.FLAG_VALID:
                    y.append(abs(r[1]))
                    p.append(abs(r[2]))
                    (y1 if r[0] < b + 0.4 else y2).append(abs(r[1]))
        out.append(f"c{k}: yaw {rms(y):.2f}/{max(y):.2f} [0-0.4s max {max(y1):.2f}, 0.4-1s max {max(y2):.2f}] "
                   f"pit {rms(p):.2f}/{max(p):.2f}")
    print(f"{s:4s} " + " | ".join(out))
