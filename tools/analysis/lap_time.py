"""Lap time from the chassis log: time from RUN to DONE, wheel distance, corner trigger times.

    py -3.13 tools/analysis/lap_time.py logs/2026-10/final logs/2026-10/t3a

Reads RUN_chassis.txt (the chassis LOG ON lines, "time since RUN" clock), as written by
gimbal/tools/telemetry_log.py run --chassis-log. A corner trigger is the first PRE_TURN row of a
corner. The contest limit is 20 s per lap.
"""
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "gimbal" / "tools"))
import run_report as R  # noqa: E402

DONE = 6

for s in sys.argv[1:]:
    ch = R.read_chassis_log(s + '_chassis.txt')
    t = [c['t_ms'] / 1000 for c in ch]
    v = [c['vmeas'] / 1000 for c in ch]
    d = [0.0]
    for i in range(1, len(ch)):
        d.append(d[-1] + 0.5 * (v[i] + v[i - 1]) * (t[i] - t[i - 1]))
    done = next((t[i] for i in range(len(ch)) if ch[i]['state'] == DONE), None)
    trig = [round(t[i], 2) for i in range(1, len(ch))
            if ch[i]['state'] == R.PRE_TURN and ch[i - 1]['state'] != R.PRE_TURN]
    print(f"{os.path.basename(s)}: RUN->DONE {done} s, distance {d[-1]:.2f} m, "
          f"corner triggers at {trig}, rows {len(ch)}")
