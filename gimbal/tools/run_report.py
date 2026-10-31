"""Side-by-side summary of lap runs: chassis smoothness (its LOG ON lines), gimbal error on the
straights and in the corners, vision coverage, and how fast the yaw motor follows its command.

    py -3.13 gimbal/tools/run_report.py s5_motor s8_smooth      # reads RUN.csv (+ RUN_chassis.txt)
    py -3.13 gimbal/tools/telemetry_log.py report s5_motor s8_smooth

Definitions (as used for the 2026-10-28 tuning):
- corners come from the chassis log (TURN start until the state leaves TURN/REACQUIRE),
  moved onto the gimbal clock by matching the chassis_gz channel to the chassis' own gz;
  without a chassis log, from telemetry flag 16. A straight sample lies outside
  [corner start - 0.1 s, corner end + 0.5 s];
- gimbal errors use frames with fresh vision only and leave out the first second after each
  track (re)start, so a vision dropout does not count as a control error (dropouts are
  counted separately);
- chassis "kicks": the body yaw rate crossing 12 deg/s on a straight (LINE state, after its
  first 0.5 s); exit error: largest line error from TURN start until 2.4 s later;
- yaw motor lag: first-order fit (time constant, gain) of the measured motor rpm to the
  commanded one over all gap-free stretches (needs the motor_rpm channel, firmware from
  2026-10-28). Field 2026-10-28 with the factory driver speed loop: tau 130-200 ms, gain
  0.8-0.9; that lag, not a low gain, is what leaves the stage ~1 deg behind in corners.
"""
import math
import sys
from pathlib import Path

import telemetry_log as T

CHASSIS_FIELDS = ("t_ms", "state", "mask", "err_raw", "err_f", "v", "vmeas", "w", "gz", "hdg",
                  "l_tgt", "l_meas", "r_tgt", "r_meas", "duty_l", "duty_r")
LINE, PRE_TURN, TURN, REACQUIRE = 1, 2, 3, 4
NO_ERROR = -9999
MRAD_TO_DEG = 180.0 / math.pi / 1000.0
KICK_DEG_S = 12.0
LINE_SETTLE_S = 0.5
EXIT_WINDOW_S = 2.4
CORNER_BEFORE_S, CORNER_AFTER_S = 0.1, 0.5
RESTART_EXCLUDE_S = 1.0
MOTOR_TAUS_S = (0.02, 0.04, 0.06, 0.08, 0.10, 0.13, 0.16, 0.20, 0.25, 0.30)
MOTOR_GAINS = (0.6, 0.7, 0.8, 0.9, 1.0, 1.1)
FIT_GAP_S = 0.15            # stretches for the motor fit are split at longer telemetry gaps
CONTINUOUS_S = 0.1          # telemetry steps longer than this are gaps
MERGE_S = 0.5               # turning spans closer than this are one corner
MIN_CORNER_S = 0.3


def read_chassis_log(path):
    """D, lines of the chassis LOG ON stream as dicts (other lines and torn ones skipped)."""
    rows = []
    for line in Path(path).read_text(encoding="ascii", errors="replace").splitlines():
        if not line.startswith("D,"):
            continue
        parts = line.split(",")[1:]
        if len(parts) != len(CHASSIS_FIELDS):
            continue
        try:                                   # the probe mask is printed as %02X
            rows.append({k: int(p, 16 if k == "mask" else 10) for k, p in zip(CHASSIS_FIELDS, parts)})
        except ValueError:
            continue
    return rows


def rms(values):
    values = list(values)
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else float("nan")


def percentile(values, share):
    values = sorted(values)
    return values[min(len(values) - 1, int(share * len(values)))] if values else float("nan")


def chassis_metrics(rows):
    straight, segment = [], []
    for r in rows + [{"state": -1, "t_ms": 0}]:
        if r["state"] == LINE:
            segment.append(r)
            continue
        if segment:
            start = segment[0]["t_ms"]
            straight += [s for s in segment if s["t_ms"] - start >= LINE_SETTLE_S * 1000]
        segment = []
    gz = [r["gz"] * MRAD_TO_DEG for r in straight]
    err = [r["err_f"] / 10.0 for r in straight if r["err_f"] != NO_ERROR]
    kicks = sum(1 for a, b in zip(gz, gz[1:]) if abs(b) > KICK_DEG_S >= abs(a))
    span_s = sum(min(b["t_ms"] - a["t_ms"], 100) for a, b in zip(straight, straight[1:])) / 1000.0
    exits = []
    for i in range(1, len(rows)):
        if rows[i]["state"] == TURN and rows[i - 1]["state"] != TURN:
            t0 = rows[i]["t_ms"]
            window = [r["err_f"] / 10.0 for r in rows[i:] if r["t_ms"] - t0 <= EXIT_WINDOW_S * 1000
                      and r["state"] in (LINE, REACQUIRE) and r["err_f"] != NO_ERROR]
            if window:
                exits.append(max(abs(e) for e in window))
    return {
        "body yaw rms on straights, deg/s": rms(gz),
        "body yaw max on straights, deg/s": max((abs(x) for x in gz), default=float("nan")),
        "kicks > 12 deg/s per minute": kicks / span_s * 60.0 if span_s else float("nan"),
        "line error rms on straights, mm": rms(err),
        "exit line error mean, mm": sum(exits) / len(exits) if exits else float("nan"),
        "exit line error max, mm": max(exits, default=float("nan")),
        "corners (chassis)": float(len(exits)),
    }


def corner_spans(rows, col):
    """(start, end) of each chassis turn. A telemetry gap must not hide a corner (the lap
    position of every later corner would shift), so flag changes count across gaps; spans
    closer than MERGE_S are one corner and blips shorter than MIN_CORNER_S are dropped."""
    flags = col["flags"]
    spans, start = [], None
    for prev, r in zip(rows, rows[1:]):
        turning, was = int(r[flags]) & T.FLAG_TURNING, int(prev[flags]) & T.FLAG_TURNING
        if turning and not was:
            start = r[0]
        elif was and not turning and start is not None:
            spans.append([start, prev[0]])
            start = None
    merged = []
    for s, e in spans:
        if merged and s - merged[-1][1] < MERGE_S:
            merged[-1][1] = e
        else:
            merged.append([s, e])
    return [(s, e) for s, e in merged if e - s >= MIN_CORNER_S]


def chassis_corners(chassis):
    """(start, end) in chassis seconds: TURN start until the state leaves TURN/REACQUIRE."""
    spans, start = [], None
    for prev, r in zip(chassis, chassis[1:]):
        inside, was = r["state"] in (TURN, REACQUIRE), prev["state"] in (TURN, REACQUIRE)
        if inside and not was:
            start = r["t_ms"] / 1000.0
        elif was and not inside and start is not None:
            spans.append((start, prev["t_ms"] / 1000.0))
            start = None
    return spans


def hold_starts(chassis):
    """Chassis seconds where "hold vision" begins (PRE_TURN, TURN or REACQUIRE entered from
    another state): what the gimbal's flag 16 shows, one link delay later."""
    hold = (PRE_TURN, TURN, REACQUIRE)
    return [r["t_ms"] / 1000.0 for prev, r in zip(chassis, chassis[1:])
            if r["state"] in hold and prev["state"] not in hold]


GZ_MATCH_MAX_RMS = 10.0     # deg/s; a worse gz match means channel 10 is not the chassis gz


def chassis_offset(rows, col, chassis, use_gz=True):
    """Seconds to add to chassis time to get gimbal time. With the chassis_gz channel: the
    offset that best matches it to the chassis' own gz (+-1.5 s around the first corner);
    otherwise - also when channel 10 carries something else (tsel != 0, or the match stays
    worse than GZ_MATCH_MAX_RMS) - the median distance between flag-16 starts and the
    nearest hold start (PRE_TURN, ~0.15-0.2 s before TURN; matching TURN made the chassis
    look that much early)."""
    flag_starts = [s for s, _ in corner_spans(rows, col)]
    turns = hold_starts(chassis)
    if not flag_starts or not turns:
        return None
    guess = flag_starts[0] - turns[0]

    def by_flags():
        near = sorted(min(((f - guess) - t for t in turns if abs((f - guess) - t) < 1.0), key=abs) + guess
                      for f in flag_starts if any(abs((f - guess) - t) < 1.0 for t in turns))
        return near[len(near) // 2] if near else guess

    if not use_gz or not any(abs(r[col["chassis_gz"]]) > 1.0 for r in rows):
        return by_flags()
    gz = [(c["t_ms"] / 1000.0, c["gz"] * MRAD_TO_DEG) for c in chassis]
    sample = rows[::5]
    best = None
    for k in range(-150, 151):
        offset = guess + k * 0.01
        j, err, n = 0, 0.0, 0
        for r in sample:
            t = r[0] - offset
            while j < len(gz) - 1 and gz[j + 1][0] <= t:
                j += 1
            if gz[j][0] <= t and j < len(gz) - 1 and gz[j + 1][0] - gz[j][0] < 0.05:
                err += (r[col["chassis_gz"]] - gz[j][1]) ** 2
                n += 1
        if n > 200 and (best is None or err / n < best[1]):
            best = (offset, err / n)
    if best is None or best[1] > GZ_MATCH_MAX_RMS ** 2:
        return by_flags()
    return best[0]


def gimbal_metrics(rows, chassis=None, use_gz=True):
    col = {name: i for i, name in enumerate(T.COLUMNS)}
    rows = [r for r in rows if T.plausible(r)]
    spans = run_spans(rows, col, chassis, use_gz)
    if not spans:
        return {}
    lo, hi = spans[0][0] - 2.5, spans[-1][1] + 1.5
    rows = [r for r in rows if lo <= r[0] <= hi]
    restarts = [r[0] for p, r in zip(rows, rows[1:]) if r[0] - p[0] <= CONTINUOUS_S
                and int(r[col["flags"]]) & T.FLAG_TRACKING and not int(p[col["flags"]]) & T.FLAG_TRACKING]

    def corner_of(t):
        for k, (s, e) in enumerate(spans):
            if s - CORNER_BEFORE_S <= t <= e + CORNER_AFTER_S:
                return k
        return None

    def clean(r):
        return int(r[col["flags"]]) & T.FLAG_VALID and not any(0 <= r[0] - s <= RESTART_EXCLUDE_S for s in restarts)

    out = {}
    for name, keep in (("straight", lambda t: corner_of(t) is None), ("corner", lambda t: corner_of(t) is not None)):
        part = [r for r in rows if keep(r[0]) and clean(r)]
        total = [math.hypot(r[col["vis_yaw"]], r[col["vis_pitch"]]) for r in part]
        out[f"{name} yaw rms, deg"] = rms(r[col["vis_yaw"]] for r in part)
        out[f"{name} pitch rms, deg"] = rms(r[col["vis_pitch"]] for r in part)
        out[f"{name} p99, deg"] = percentile(total, 0.99)
        out[f"{name} max, deg"] = max(total, default=float("nan"))
    out["track restarts (vision lost)"] = float(len(restarts))
    coded = [int(r[col["vis_code"]]) for r in rows if r[col["vis_code"]] >= 0]
    if coded:
        for code in (1, 2, 3, 4):
            out["vision invalid: {}, % of time".format(VISION_CODES[code])] = 100.0 * coded.count(code) / len(coded)
        out["vision invalid: other, % of time"] = 100.0 * sum(1 for c in coded if c >= 5) / len(coded)
    for lap_corner in range(1, 5):
        inside = [r for r in rows if any(s <= r[0] <= e for k, (s, e) in enumerate(spans) if k % 4 == lap_corner - 1)]
        if inside:
            valid = sum(1 for r in inside if int(r[col["flags"]]) & T.FLAG_VALID)
            out[f"vision valid in corner {lap_corner}, %"] = 100.0 * valid / len(inside)
    ends = [e for _, e in spans]
    for edge in (1, 2, 3, 0):
        part = [r for r in rows if corner_of(r[0]) is None and clean(r)
                and sum(1 for e in ends if e <= r[0]) % 4 == edge and r[0] > spans[0][0]]
        if part:
            name = f"after corner {edge or 4}"
            out[f"pitch mean on edge {name}, deg"] = sum(r[col["vis_pitch"]] for r in part) / len(part)
    if any(abs(r[col["motor_rpm"]]) > 0.5 for r in rows):
        fit = fit_motor_lag(rows, col)
        if fit:
            out["yaw motor lag tau (fit), ms"] = fit[0] * 1000.0
            out["yaw motor gain (fit)"] = fit[1]
            out["yaw motor fit residual, rpm"] = fit[2]
    return out


def fit_motor_lag(rows, col):
    """(tau_s, gain, rms_rpm) of motor' = (gain * command - motor) / tau over the gap-free
    stretches, by grid search; None without enough data."""
    stretches, current = [], [rows[0]] if rows else []
    for prev, r in zip(rows, rows[1:]):
        if r[0] - prev[0] > FIT_GAP_S:
            stretches.append(current)
            current = []
        current.append(r)
    stretches = [s for s in stretches + [current] if len(s) > 40]
    if not stretches:
        return None
    cmd, motor = col["yaw_rpm"], col["motor_rpm"]
    best = None
    for tau in MOTOR_TAUS_S:
        for gain in MOTOR_GAINS:
            err, n = 0.0, 0
            for s in stretches:
                m = s[0][motor]
                for a, b in zip(s, s[1:]):
                    m += (gain * a[cmd] - m) * min(1.0, (b[0] - a[0]) / tau)
                    err += (m - b[motor]) ** 2
                    n += 1
            if best is None or err / n < best[2]:
                best = (tau, gain, err / n)
    return best[0], best[1], math.sqrt(best[2])


VISION_CODES = ("valid", "no rectangle", "association", "confirming", "reprojection",
                "tilt", "depth", "angle", "too old", "other")   # vision/protocol.py CODE_*
SECTION_PARTS_S = (1.0, 2.0)    # edge sections: 0-1 s, 1-2 s, 2+ s after the corner


def section_of(t, spans):
    """'corner N' (lap position) or 'edge after corner N, a-b s' for gimbal time t."""
    for k, (s, e) in enumerate(spans):
        if s <= t <= e:
            return "corner {}".format(k % 4 + 1)
    before = [k for k, (_, e) in enumerate(spans) if e < t]
    if not before:
        return "start edge"
    k = before[-1]
    since = t - spans[k][1]
    edges = (0.0,) + SECTION_PARTS_S + (float("inf"),)
    for lo, hi in zip(edges, edges[1:]):
        if lo <= since < hi:
            part = "{:.0f}+ s".format(lo) if hi == float("inf") else "{:.0f}-{:.0f} s".format(lo, hi)
            return "edge after corner {}, {}".format(k % 4 + 1, part)
    return "edge"


def channel10_is_gz(stem):
    """False when the run's saved parameters say telemetry channel 10 held something else."""
    path = Path(str(stem) + "_params.txt")
    if not path.exists():
        return True
    text = path.read_text(errors="replace")
    return "tsel=" not in text or text.split("tsel=")[1].split()[0] in ("0", "0.0")


def run_spans(rows, col, chassis, use_gz=True):
    spans = corner_spans(rows, col)
    offset = chassis_offset(rows, col, chassis, use_gz) if chassis else None
    if offset is not None:
        spans = [(s + offset, e + offset) for s, e in chassis_corners(chassis)]
    return spans


def vision_sections(stem, out=print):
    """Per track section: share of time with a valid K230 frame, what stopped the others
    (telemetry vis_code, $V version 3) and the tape rectangle's median area."""
    stem = str(stem)[:-4] if str(stem).endswith(".csv") else str(stem)
    col = {name: i for i, name in enumerate(T.COLUMNS)}
    rows = [r for r in T.read_csv(stem + ".csv") if T.plausible(r) and r[col["vis_code"]] >= 0]
    if not rows:
        out(stem + ": no vis_code (needs the $V version 3 K230 and gimbal firmware from 2026-10-28)")
        return
    path = Path(stem + "_chassis.txt")
    spans = run_spans(rows, col, read_chassis_log(path) if path.exists() else None, channel10_is_gz(stem))
    groups = {}
    for r in rows:
        if spans and spans[0][0] - 2.5 <= r[0] <= spans[-1][1] + 1.5:
            groups.setdefault(section_of(r[0], spans), []).append(r)
    out("{:<34s} {:>6s} {:>7s}  {:>11s}  {}".format("section", "n", "valid%", "area v/inv%", "invalid, share of the section"))
    for key in sorted(groups):
        g = groups[key]
        codes = [int(r[col["vis_code"]]) for r in g]
        counts = sorted(((codes.count(c), c) for c in set(codes) if c != 0), reverse=True)
        areas_valid = sorted(r[col["vis_area"]] for r in g if int(r[col["vis_code"]]) == 0)
        areas_invalid = sorted(r[col["vis_area"]] for r in g if int(r[col["vis_code"]]) != 0)
        med = lambda a: a[len(a) // 2] if a else float("nan")
        why = ", ".join("{} {:.0f}%".format(VISION_CODES[c] if c < len(VISION_CODES) else c, 100.0 * n / len(g))
                        for n, c in counts[:4])
        out("{:<34s} {:6d} {:6.1f}%  {:5.2f}/{:5.2f}  {}".format(
            key, len(g), 100.0 * codes.count(0) / len(g), med(areas_valid), med(areas_invalid), why))


def run_metrics(stem):
    stem = str(stem)[:-4] if str(stem).endswith(".csv") else str(stem)
    path = Path(stem + "_chassis.txt")
    chassis = read_chassis_log(path) if path.exists() else []
    metrics = gimbal_metrics(T.read_csv(stem + ".csv"), chassis or None, channel10_is_gz(stem))
    if chassis:
        metrics.update(chassis_metrics(chassis))
    return metrics


def format_report(names, results):
    keys = []
    for result in results:
        keys += [k for k in result if k not in keys]
    width = max(len(k) for k in keys) if keys else 10
    lines = [" " * width + "".join(f"{n[-12:]:>13s}" for n in names)]
    for k in keys:
        cells = "".join(f"{results[i].get(k, float('nan')):13.2f}" for i in range(len(names)))
        lines.append(f"{k:<{width}s}{cells}")
    return "\n".join(lines)


def report(stems, out=print):
    out(format_report([Path(str(s)).name for s in stems], [run_metrics(s) for s in stems]))


if __name__ == "__main__":
    report(sys.argv[1:])
