"""Why the K230 drops frames, by track position: its per-frame log (deploy.ps1 -FrameLog,
captures/framelog_<ticks>.csv) against a lap run's gimbal telemetry and chassis log.

    py -3.13 vision/tools/framelog_report.py framelog_12345.csv f1
        (f1 = the -o name of telemetry_log.py run: f1.csv and f1_chassis.txt)
    py -3.13 vision/tools/framelog_report.py framelog_12345.csv     # summary only

The K230 clock is put on the gimbal clock by matching the K230 valid flag with telemetry
flag 1 (the gimbal copies each frame's valid bit): onsets of invalid stretches give a coarse
offset, the agreement of the two sequences refines it. Corners come from the chassis log
(gimbal/tools/run_report.py).
"""
import csv
import math
import sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "gimbal" / "tools"))
import run_report as R          # noqa: E402
import telemetry_log as T       # noqa: E402

ONSET_BIN_S = 0.02
REFINE_S, REFINE_STEP_S = 0.15, 0.005
section_of = R.section_of


def read_framelog(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            if r.get("t_ms", "t_ms") == "t_ms" or not r.get("t_ms"):
                continue                     # repeated header after a reboot
            try:
                r["t"] = int(r["t_ms"]) / 1000.0
                r["valid"] = r["valid"] == "1"
            except ValueError:
                continue
            for key in ("area_pct", "tilt_deg", "reproj_px", "depth_mm", "thr", "cand"):
                try:
                    r[key] = float(r[key]) if r.get(key) not in (None, "") else None
                except ValueError:
                    r[key] = None
            rows.append(r)
    return rows


def outcome(frame):
    """One label per frame: what stopped it, in pipeline order."""
    if frame["valid"]:
        return "valid"
    reason = frame.get("reason", "")
    if frame.get("det") in ("no_candidates", "association_rejected", "confirming"):
        return frame["det"]
    if reason.startswith("pose reprojection"):
        return "pose_reprojection"
    return reason or "unknown"


def invalid_onsets(times, valid):
    return [t for t, v, pv in zip(times[1:], valid[1:], valid[:-1]) if pv and not v]


def align_offset(frames, gimbal_rows, col):
    """gimbal_t = k230_t + offset, or None."""
    g = [(r[0], bool(int(r[col["flags"]]) & T.FLAG_VALID)) for r in gimbal_rows]
    k_on = invalid_onsets([f["t"] for f in frames], [f["valid"] for f in frames])
    g_on = invalid_onsets([t for t, _ in g], [v for _, v in g])
    if not k_on or not g_on:
        return None
    votes = Counter(round((gt - kt) / ONSET_BIN_S) for kt in k_on for gt in g_on)
    coarse = votes.most_common(1)[0][0] * ONSET_BIN_S
    times = [f["t"] for f in frames]

    def k230_valid(t):
        lo, hi = 0, len(times) - 1
        if t < times[0] or t > times[-1]:
            return None
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if times[mid] <= t:
                lo = mid
            else:
                hi = mid - 1
        return frames[lo]["valid"]

    best = None
    steps = int(REFINE_S / REFINE_STEP_S)
    for k in range(-steps, steps + 1):
        offset = coarse + k * REFINE_STEP_S
        agree = n = 0
        for t, v in g[::3]:
            kv = k230_valid(t - offset)
            if kv is not None:
                agree += kv == v
                n += 1
        if n > 100 and (best is None or agree / n > best[1]):
            best = (offset, agree / n)
    return best


def median(values):
    values = sorted(v for v in values if v is not None)
    return values[len(values) // 2] if values else float("nan")


def summarise(frames, label_of=None):
    groups = {}
    for f in frames:
        key = label_of(f) if label_of else "all"
        if key is not None:
            groups.setdefault(key, []).append(f)
    lines = []
    for key in sorted(groups):
        g = groups[key]
        counts = Counter(outcome(f) for f in g)
        valid = counts.pop("valid", 0)
        top = ", ".join("{} {}".format(name, n) for name, n in counts.most_common(4))
        lines.append("{:<34s} frames {:4d}  valid {:5.1f} %  area valid/invalid {:.2f}/{:.2f} %  {}".format(
            key, len(g), 100.0 * valid / len(g),
            median(f["area_pct"] for f in g if f["valid"]),
            median(f["area_pct"] for f in g if not f["valid"]), top))
    return lines


def report(framelog_path, run=None, out=print):
    frames = read_framelog(framelog_path)
    if not frames:
        out("no frames in " + str(framelog_path))
        return
    for line in summarise(frames):
        out(line)
    if not run:
        return
    col = {name: i for i, name in enumerate(T.COLUMNS)}
    gimbal = [r for r in T.read_csv(run + ".csv") if T.plausible(r)]
    fit = align_offset(frames, gimbal, col)
    if not fit:
        out("could not align the K230 log with " + run + ".csv (no invalid stretches in both?)")
        return
    offset, agreement = fit
    out("K230 -> gimbal clock: +{:.3f} s, valid flags agree {:.1f} %".format(offset, 100 * agreement))
    chassis = R.read_chassis_log(run + "_chassis.txt") if Path(run + "_chassis.txt").exists() else []
    spans = R.run_spans(gimbal, col, chassis or None)
    if not spans:
        out("no corners found in the run")
        return
    lo, hi = spans[0][0] - 2.5, spans[-1][1] + 1.5
    for line in summarise(frames, lambda f: section_of(f["t"] + offset, spans)
                          if lo <= f["t"] + offset <= hi else None):
        out(line)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    report(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
