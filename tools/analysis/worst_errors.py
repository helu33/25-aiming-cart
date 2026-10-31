"""The 5 largest gimbal errors of a run, with the track section and the vision state before them.

    py -3.13 tools/analysis/worst_errors.py logs/2026-10/final logs/2026-10/final2

Reads RUN.csv and RUN_chassis.txt. Only frames with valid vision count (flag 1), from 2.5 s
before the first corner to 1.5 s after the last; picks at least 0.5 s apart. "invalid in prior
0.5 s" is the share of frames with a vision error code just before, and which codes (see
run_report.VISION_CODES) - a large error right after a dropout is a re-acquisition, not a
control error.
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "gimbal" / "tools"))
import run_report as R  # noqa: E402
import telemetry_log as T  # noqa: E402

col = {n: i for i, n in enumerate(T.COLUMNS)}
for s in sys.argv[1:]:
    rows = [r for r in T.read_csv(s + '.csv') if T.plausible(r)]
    ch = R.read_chassis_log(s + '_chassis.txt')
    spans = R.run_spans(rows, col, ch, R.channel10_is_gz(s))
    lo, hi = spans[0][0] - 2.5, spans[-1][1] + 1.5
    valid = [r for r in rows if lo <= r[0] <= hi and int(r[3]) & T.FLAG_VALID]

    def ctx(t):
        prev = [r for r in rows if t - 0.5 <= r[0] < t]
        lost = sum(1 for r in prev if r[col['vis_code']] > 0) / max(1, len(prev))
        codes = sorted(set(int(r[col['vis_code']]) for r in prev if r[col['vis_code']] > 0))
        return lost, codes

    top = sorted(valid, key=lambda r: -math.hypot(r[1], r[2]))
    picked = []
    for r in top:
        if all(abs(r[0] - p[0]) > 0.5 for p in picked):
            picked.append(r)
        if len(picked) == 5:
            break
    print(f"== {s}: 5 largest errors")
    for r in picked:
        lost, codes = ctx(r[0])
        print(f"   {math.hypot(r[1], r[2]):.2f} deg (yaw {r[1]:+.2f} pitch {r[2]:+.2f})  "
              f"{R.section_of(r[0], spans):32s} invalid in prior 0.5 s: {100 * lost:3.0f}% {codes}")
