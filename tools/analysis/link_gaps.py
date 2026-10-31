"""Chassis link losses seen by the gimbal: telemetry channel chassis_gz exactly 0 for >= 0.3 s.

    py -3.13 tools/analysis/link_gaps.py logs/2026-10/final logs/2026-10/link_check

Reads RUN.csv (gimbal telemetry). Only runs recorded with tsel 0 carry the chassis gz in channel
10 (RUN_params.txt says which); others are skipped. Gaps in the first second are left out; each
gap is printed as [start s, length s] with "turn" when the turning flag was set.
"""
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "gimbal" / "tools"))
import run_report as R  # noqa: E402
import telemetry_log as T  # noqa: E402

GAP_S = 0.3

col = {n: i for i, n in enumerate(T.COLUMNS)}
for s in sys.argv[1:]:
    if not os.path.exists(s + '.csv'):
        continue
    if not R.channel10_is_gz(s):
        print(f"{os.path.basename(s):8s} (tsel != 0, channel 10 is not gz)")
        continue
    rows = [r for r in T.read_csv(s + '.csv') if T.plausible(r)]
    gaps, start = [], None
    for r in rows:
        zero = r[col['chassis_gz']] == 0.0
        if zero and start is None:
            start = r
        elif not zero and start is not None:
            if r[0] - start[0] >= GAP_S:
                gaps.append((start[0], r[0] - start[0], int(start[3]) & T.FLAG_TURNING))
            start = None
    moving = [g for g in gaps if g[0] > rows[0][0] + 1.0]
    print(f"{os.path.basename(s):8s} {len(rows):5d} rows, link gaps >= {GAP_S} s: {len(moving)}  " +
          " ".join(f"[{t - rows[0][0]:.1f}s {d:.2f}s{' turn' if tf else ''}]" for t, d, tf in moving[:8]))
