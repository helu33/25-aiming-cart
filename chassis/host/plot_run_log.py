"""Parse and plot the chassis LOG ON stream (D, lines at 50 Hz) saved from the debug UART.

    py -3.13 chassis/host/plot_run_log.py run.txt              # summary + run_N.png next to it
    py -3.13 chassis/host/plot_run_log.py run.txt --out plots

Non-data lines (AUTO/HW/TRACE, Bluetooth noise) are ignored. A new run starts at each
"H," header or when the time goes backwards. Units in the stream: mm/s, mrad/s, mrad,
line error in 0.1 mm; -9999 = no normal line pattern.
"""
import argparse
from pathlib import Path

FIELDS = ("t_ms", "state", "mask", "err_raw", "err_f", "v", "vmeas", "w", "gz", "hdg",
          "l_tgt", "l_meas", "r_tgt", "r_meas", "duty_l", "duty_r")
STATE_NAMES = {0: "IDLE", 1: "LINE", 2: "PRE_TURN", 3: "TURN", 4: "REACQUIRE",
               5: "FINISH", 6: "DONE", 7: "FAULT"}
NO_ERROR = -9999
SWING_BAND_DMM = 30        # a swing = error crossing from beyond +3 mm to beyond -3 mm
SETTLED_DMM = 30
SETTLED_HOLD_S = 0.5
CORNER_WINDOW_S = 2.5      # look this long after each corner exit


def read_lines_from_text(text):
    """Plain serial capture, or a serial-app JSON export: a list of {"type": "read"|"write",
    "data": chunk, "timestamp": ms}. Chunks are split anywhere, so join the received
    ones in time order before splitting into lines."""
    stripped = text.lstrip("﻿ \t\r\n")
    if stripped.startswith("["):
        import json
        try:
            records = json.loads(stripped)
        except ValueError:
            records = None
        if isinstance(records, list):
            received = [r for r in records if isinstance(r, dict) and r.get("type") == "read"]
            received.sort(key=lambda r: r.get("timestamp", 0))
            return "".join(str(r.get("data", "")) for r in received).splitlines()
    return text.splitlines()


def parse(lines):
    runs, current, last_t = [], None, None
    for raw in lines:
        text = raw.strip()
        if text.startswith("H,"):
            current, last_t = [], None
            runs.append(current)
            continue
        if not text.startswith("D,"):
            continue
        parts = text.split(",")[1:]
        if len(parts) != len(FIELDS):
            continue
        try:
            row = {name: (int(value, 16) if name == "mask" else int(value))
                   for name, value in zip(FIELDS, parts)}
        except ValueError:
            continue          # corrupted line (e.g. dropped bytes over Bluetooth)
        if current is None or (last_t is not None and row["t_ms"] < last_t):
            current = []
            runs.append(current)
        row["t"] = row["t_ms"] / 1000.0
        row["err_raw"] = None if row["err_raw"] == NO_ERROR else row["err_raw"]
        current.append(row)
        last_t = row["t_ms"]
    return [run for run in runs if run]


def _swings(values):
    count, side = 0, 0
    for value in values:
        if value > SWING_BAND_DMM:
            count += side == -1
            side = 1
        elif value < -SWING_BAND_DMM:
            count += side == 1
            side = -1
    return count


def _settle_time(rows, start):
    for i, row in enumerate(rows):
        later = [r for r in rows[i:] if r["t"] - row["t"] <= SETTLED_HOLD_S]
        if all(abs(r["err_f"]) < SETTLED_DMM for r in later):
            return row["t"] - start
    return None


def _rms(values):
    values = list(values)
    return (sum(v*v for v in values)/len(values))**0.5 if values else None


def summarize(run):
    exits = [i for i in range(1, len(run))
             if run[i-1]["state"] == 4 and run[i]["state"] in (1, 5)]
    corners = []
    for i in exits:
        start = run[i]["t"]
        window = [r for r in run[i:] if r["t"] - start <= CORNER_WINDOW_S and r["state"] in (1, 5)]
        raw = [abs(r["err_raw"]) for r in window if r["err_raw"] is not None]
        corners.append({"t": start, "peak_mm": max(raw)/10.0 if raw else None,
                        "swings": _swings(r["err_f"] for r in window),
                        "settle_s": _settle_time(window, start),
                        "heading_deg_at_exit": run[i]["hdg"]*0.0573})
    exit_times = [run[i]["t"] for i in exits]
    straight = [r for r in run if r["state"] == 1 and r["t"] > 1.0 and
                all(r["t"] - t > CORNER_WINDOW_S for t in exit_times if t <= r["t"])]
    duration = straight[-1]["t"] - straight[0]["t"] if len(straight) > 1 else 0.0
    line_rows = [r for r in run if r["state"] in (1, 5)]
    return {
        "duration_s": run[-1]["t"], "end_state": STATE_NAMES.get(run[-1]["state"], run[-1]["state"]),
        "corner_exits": corners,
        "straight_rms_mm": None if not straight else _rms(r["err_f"] for r in straight)/10.0,
        "straight_swings_per_s": None if duration <= 0 else _swings(r["err_f"] for r in straight)/duration,
        "wheel_rms_mm_s": (_rms(r["l_meas"]-r["l_tgt"] for r in line_rows),
                           _rms(r["r_meas"]-r["r_tgt"] for r in line_rows)) if line_rows else None,
    }


def print_summary(index, summary):
    print("run {}: {:.1f} s, ended {}".format(index, summary["duration_s"], summary["end_state"]))
    for n, corner in enumerate(summary["corner_exits"], 1):
        settle = "not within {:.1f} s".format(CORNER_WINDOW_S) if corner["settle_s"] is None \
            else "{:.2f} s".format(corner["settle_s"])
        peak = "-" if corner["peak_mm"] is None else "{:.1f} mm".format(corner["peak_mm"])
        print("  corner {} exit @{:.1f}s: heading {:+.1f} deg, peak offset {}, swings {}, settled {}".format(
            n, corner["t"], corner["heading_deg_at_exit"], peak, corner["swings"], settle))
    if summary["straight_rms_mm"] is not None:
        print("  straight (>{:.1f}s after corners): rms {:.1f} mm, {:.2f} swings/s".format(
            CORNER_WINDOW_S, summary["straight_rms_mm"], summary["straight_swings_per_s"] or 0.0))
    if summary["wheel_rms_mm_s"]:
        print("  wheel speed tracking rms: left {:.0f} mm/s, right {:.0f} mm/s".format(*summary["wheel_rms_mm_s"]))


def plot(run, out_path, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = [r["t"] for r in run]
    fig, axes = plt.subplots(7, 1, figsize=(14, 16), sharex=True)
    colours = {2: "#fff3c4", 3: "#ffd8b0", 4: "#d8ecff", 5: "#e3f7e3", 7: "#ffd0d0"}
    for ax in axes:
        start = 0
        for i in range(1, len(run)+1):
            if i == len(run) or run[i]["state"] != run[start]["state"]:
                colour = colours.get(run[start]["state"])
                if colour:
                    ax.axvspan(run[start]["t"], run[i-1]["t"], color=colour, lw=0)
                start = i
    image = [[(r["mask"] >> bit) & 1 for r in run] for bit in range(8)]
    axes[0].imshow(image, aspect="auto", cmap="Greys", interpolation="nearest",
                   extent=(t[0], t[-1], 7.5, -0.5))
    axes[0].set_ylabel("probe\n(0=left)")
    axes[1].plot(t, [None if r["err_raw"] is None else r["err_raw"]/10 for r in run], ".", ms=2, label="raw")
    axes[1].plot(t, [r["err_f"]/10 for r in run], label="filtered")
    axes[1].set_ylabel("line err mm\n(+ = line right)")
    axes[2].plot(t, [r["hdg"]*0.0573 for r in run])
    axes[2].set_ylabel("heading vs\nedge deg")
    axes[3].plot(t, [r["w"]/1000 for r in run], label="w cmd")
    axes[3].plot(t, [r["gz"]/1000 for r in run], label="gyro z")
    axes[3].set_ylabel("rad/s")
    axes[4].plot(t, [r["v"]/1000 for r in run], label="v cmd")
    axes[4].plot(t, [r["vmeas"]/1000 for r in run], label="v meas")
    axes[4].set_ylabel("m/s")
    for side, name in (("l", "left"), ("r", "right")):
        axes[5].plot(t, [r[side+"_tgt"]/1000 for r in run], "--", label=name+" target")
        axes[5].plot(t, [r[side+"_meas"]/1000 for r in run], label=name+" meas")
    axes[5].set_ylabel("wheel m/s")
    axes[6].plot(t, [r["duty_l"] for r in run], label="left")
    axes[6].plot(t, [r["duty_r"] for r in run], label="right")
    axes[6].set_ylabel("duty %")
    axes[6].set_xlabel("time since RUN (s)   shading: yellow PRE_TURN, orange TURN, blue REACQUIRE")
    for ax in axes[1:]:
        ax.grid(alpha=0.3)
        if ax.get_legend_handles_labels()[0]:
            ax.legend(loc="upper right", fontsize=8)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out_path, dpi=90)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log")
    parser.add_argument("--out", help="folder for PNGs (default: next to the log)")
    args = parser.parse_args()
    log = Path(args.log)
    runs = parse(read_lines_from_text(log.read_text(encoding="utf-8", errors="replace")))
    if not runs:
        raise SystemExit("No D, lines found. Send LOG ON before RUN and save the serial output.")
    out_dir = Path(args.out) if args.out else log.parent
    out_dir.mkdir(parents=True, exist_ok=True)
    for index, run in enumerate(runs, 1):
        print_summary(index, summarize(run))
        png = out_dir / "{}_run{}.png".format(log.stem, index)
        plot(run, png, "{} run {}".format(log.name, index))
        print("  plot:", png)


if __name__ == "__main__":
    main()
