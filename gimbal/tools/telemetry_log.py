"""Record and analyse the gimbal Bluetooth telemetry (VOFA+ JustFloat, see
gimbal/App/inc/telemetry.h): time + 8 channels per frame, tail 00 00 80 7F.

  capture  read the COM port into a CSV (live stream, or --replay the on-board
           60 s recording after a run), then print the analysis
  analyze  metrics (and optionally a plot) from a CSV written by capture
  set      change gimbal tune_* values over Bluetooth (not stored: a reset restores them)
  get      print the gimbal tune_* values
  run      one test: set values, start recording, send DRIVE on the chassis port, analyse

  py -3.13 gimbal/tools/telemetry_log.py capture COM7 --seconds 30 -o run1.csv
  py -3.13 gimbal/tools/telemetry_log.py capture COM7 --replay -o run1.csv
  py -3.13 gimbal/tools/telemetry_log.py analyze run1.csv --plot run1.png
  py -3.13 gimbal/tools/telemetry_log.py set COM7 beta=0.3 kpscale=2
  py -3.13 gimbal/tools/telemetry_log.py run COM7 --chassis COM10 --set beta=0.3 -o r8

Close VOFA+ first: only one program can hold the COM port. Bluetooth ports often fail to
connect at the first try (WinError 121): the tool retries, waits until data flows and resends
unanswered commands.
"""
import argparse
import csv
import math
import re
import struct
import sys
import time

COLUMNS = ["t", "vis_yaw", "vis_pitch", "flags", "track_rate", "yaw_err", "yaw_rpm",
           "gyro_z", "pitch_err", "motor_rpm", "chassis_gz", "vis_code", "vis_area"]
OLD_WIDTHS = (9, 11)       # older CSVs: before motor_rpm / chassis_gz, before vis_code / vis_area
OLD_FLOATS = OLD_WIDTHS[0]
VIS_CODE_NONE = -1.0       # vis_code before the first $V version-3 frame (and in old files)
FLOATS = len(COLUMNS)
TAIL = b"\x00\x00\x80\x7f"
FRAME = FLOATS * 4 + len(TAIL)
FLAG_VALID, FLAG_SUSPENDED, FLAG_WAITING, FLAG_TRACKING, FLAG_TURNING = 1, 2, 4, 8, 16
CORNER = FLAG_SUSPENDED | FLAG_TURNING   # vision held, or the chassis turning (turnhold 0)
ON_TARGET_DEG = 0.5
RPM_LIMIT = 50.0
OPEN_ATTEMPTS, OPEN_RETRY_S = 3, 2.0
LINK_WAIT_S = 3.0          # after opening, until the first bytes arrive (link is through)
COMMAND_ATTEMPTS = 3
TELEMETRY_HZ = 50.0        # live frames per second over the HC-05 (TELEM_LINK_USE); replay is 100
LIVE_SHORT_FRACTION = 0.8  # fewer live frames than this share of the run: fetch the recorder
DRIVE_RESEND_S = 1.5      # no "AUTO DRIVE ..." answer by then: send DRIVE once more
BLUETOOTH_HINT = ("  module powered? not held by VOFA+, a serial terminal or a phone? "
                  "close those, wait 5 s, retry; still failing: Bluetooth off/on in Windows")


class JustFloatParser:
    """Bytes in, complete frames (lists of floats) out; resynchronises on the tail."""

    def __init__(self):
        self.buf = bytearray()
        self.dropped = 0

    def feed(self, data):
        self.buf += data
        frames = []
        while True:
            i = self.buf.find(TAIL)
            if i < 0:
                if len(self.buf) > 4 * FRAME:          # no tail for a long time: garbage
                    self.dropped += len(self.buf) - FRAME
                    del self.buf[:-FRAME]
                return frames
            if i >= FLOATS * 4:
                values = list(struct.unpack("<%df" % FLOATS, bytes(self.buf[i - FLOATS * 4:i])))
                if plausible(values):
                    frames.append(values)
                else:
                    self.dropped += FRAME
                self.dropped += i - FLOATS * 4
            else:
                self.dropped += i + len(TAIL)              # partial frame before the tail
            del self.buf[:i + len(TAIL)]


def plausible(values):
    """A frame resynchronised on a tail that happened to sit inside other data decodes to
    huge numbers or a non-integer flags field (2026-10-28: 'gyro peak 1.8e37')."""
    flags = values[COLUMNS.index("flags")]
    return (all(math.isfinite(v) for v in values) and 0.0 <= values[0] < 1e6 and
            all(abs(v) < 1e4 for v in values[1:]) and flags == int(flags) and 0 <= flags < 64)


def pack_frame(values):
    """Reference encoder (same layout as telem_frame in telemetry.h); used by the tests."""
    return struct.pack("<%df" % FLOATS, *values) + TAIL


def write_csv(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(COLUMNS)
        for r in rows:
            w.writerow(["%.4f" % v for v in r])


def read_csv(path):
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        width = next((w for w in (FLOATS,) + OLD_WIDTHS[::-1] if header[:w] == COLUMNS[:w]
                      and len(header) >= w and (w == FLOATS or len(header) == w)), None)
        if width is None:
            raise SystemExit(f"{path}: expected columns {','.join(COLUMNS)}")
        pad = [0.0] * (FLOATS - width)
        if width <= COLUMNS.index("vis_code"):
            pad[COLUMNS.index("vis_code") - width] = VIS_CODE_NONE
        return [[float(x) for x in row[:width]] + pad for row in reader if row]


def analyze(rows):
    """Metrics of one run. Vision errors only count frames flagged valid (the channel holds
    the last value otherwise); turns are the spans the chassis turned or held vision."""
    if not rows:
        return {"samples": 0}
    col = {name: i for i, name in enumerate(COLUMNS)}
    flags = [int(r[col["flags"]]) for r in rows]
    t = [r[col["t"]] for r in rows]
    valid = [r for r, f in zip(rows, flags) if f & FLAG_VALID]
    free = [f for f in flags if not f & CORNER]
    out = {
        "samples": len(rows),
        "duration_s": t[-1] - t[0],
        "valid_ratio": (sum(1 for f in free if f & FLAG_VALID) / len(free)) if free else 0.0,
        "rpm_saturated_ratio": sum(1 for r in rows if abs(r[col["yaw_rpm"]]) >= RPM_LIMIT - 0.5) / len(rows),
        "gyro_peak_deg_s": max(abs(r[col["gyro_z"]]) for r in rows),
        "turns": [],
    }
    for name in ("vis_yaw", "vis_pitch"):
        values = [r[col[name]] for r in valid]
        if values:
            out[name + "_max"] = max(values)
            out[name + "_min"] = min(values)
            out[name + "_rms"] = math.sqrt(sum(v * v for v in values) / len(values))
    # Turns: corner spans (errors of the valid frames inside), then how long until the laser
    # is back on the target.
    k = 0
    while k < len(rows):
        if not flags[k] & CORNER:
            k += 1
            continue
        start = k
        while k < len(rows) and flags[k] & CORNER:
            k += 1
        inside = [rows[i] for i in range(start, k) if flags[i] & FLAG_VALID]
        turn = {"start_s": t[start], "held_s": t[k - 1] - t[start] if k > start else 0.0,
                "valid_during": len(inside) / (k - start),
                "peak_during_deg": max((max(abs(r[col["vis_yaw"]]), abs(r[col["vis_pitch"]]))
                                        for r in inside), default=0.0)}
        end_t = t[k] if k < len(rows) else None
        peak, back = 0.0, None
        j = k
        while j < len(rows) and not flags[j] & CORNER and t[j] - (end_t or 0) <= 2.0:
            if flags[j] & FLAG_VALID:
                e = max(abs(rows[j][col["vis_yaw"]]), abs(rows[j][col["vis_pitch"]]))
                if t[j] - end_t <= 1.0:
                    peak = max(peak, e)
                if back is None and e < ON_TARGET_DEG:
                    back = t[j] - end_t
            j += 1
        turn["peak_after_deg"] = peak
        turn["back_on_target_s"] = back
        out["turns"].append(turn)
    return out


def print_report(m):
    if not m.get("samples"):
        print("no samples")
        return
    print(f"samples {m['samples']}  duration {m['duration_s']:.2f} s  "
          f"valid frames {100 * m['valid_ratio']:.0f}% (outside turns)")
    for name, label in (("vis_yaw", "yaw  "), ("vis_pitch", "pitch")):
        if name + "_max" in m:
            print(f"vision {label} error  max {m[name + '_max']:+6.2f}  min {m[name + '_min']:+6.2f}"
                  f"  rms {m[name + '_rms']:5.2f} deg   (1 deg = 1.4 cm at 80 cm)")
    print(f"yaw rpm at the 50 rpm limit {100 * m['rpm_saturated_ratio']:.1f}% of samples, "
          f"gyro peak {m['gyro_peak_deg_s']:.0f} deg/s")
    for i, turn in enumerate(m["turns"], 1):
        back = turn["back_on_target_s"]
        back_text = f"{back:.2f} s" if back is not None else "not within 2 s"
        print(f"turn {i} at {turn['start_s']:.2f} s: {turn['held_s']:.2f} s, valid during "
              f"{100 * turn['valid_during']:.0f}% peak {turn['peak_during_deg']:.2f} deg; "
              f"error peak {turn['peak_after_deg']:.2f} deg in the 1 s after, back on target {back_text}")


def plot(rows, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = [r[0] for r in rows]
    c = {name: [r[i] for r in rows] for i, name in enumerate(COLUMNS)}
    valid = [int(f) & FLAG_VALID for f in c["flags"]]
    held = [int(f) & CORNER for f in c["flags"]]
    fig, ax = plt.subplots(4, 1, sharex=True, figsize=(11, 9))
    for a in ax:
        a.fill_between(t, 0, 1, where=held, transform=a.get_xaxis_transform(),
                       color="0.85", step="post", label="_turn")
        a.grid(alpha=0.3)
    mask = lambda v: [x if ok else float("nan") for x, ok in zip(v, valid)]
    ax[0].plot(t, mask(c["vis_yaw"]), label="vision yaw error")
    ax[0].plot(t, mask(c["vis_pitch"]), label="vision pitch error")
    ax[0].set_ylabel("deg")
    ax[1].plot(t, c["yaw_err"], label="yaw target - IMU")
    ax[1].plot(t, c["pitch_err"], label="pitch loop error")
    ax[1].set_ylabel("deg")
    ax[2].plot(t, c["track_rate"], label="tracker yaw rate")
    ax[2].plot(t, c["gyro_z"], label="gyro z (stage)")
    ax[2].set_ylabel("deg/s")
    ax[3].plot(t, c["yaw_rpm"], label="yaw command")
    ax[3].step(t, [10 * v for v in valid], where="post", label="valid x10")
    ax[3].set_ylabel("rpm")
    ax[3].set_xlabel("time (s), grey = turn (chassis turning or vision held)")
    for a in ax:
        a.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print("plot saved:", path)


# The gimbal sends "\n#P ...\n" (printable ASCII) between frames; frame bytes can contain
# "#P" by chance, so the whole line must be printable and enclosed in line feeds.
REPLY = re.compile(rb"\n(#(?:OK|ERR|P) [\x20-\x7e]*)\r?\n")


def find_reply(buffer):
    """Text reply ("#OK ...", "#ERR ...", "#P ...") among the binary frames, or None."""
    m = REPLY.search(buffer)
    return m.group(1).decode("ascii").strip() if m else None


def parse_assignments(items):
    """["beta=0.3", "kpscale=2"] -> [("beta", 0.3), ("kpscale", 2.0)]."""
    out = []
    for item in items:
        name, sep, value = item.partition("=")
        if not sep or not name:
            raise SystemExit(f"expected name=value, got {item!r}")
        out.append((name, float(value)))
    return out


def port_names():
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    return sorted(p.device for p in list_ports.comports())


def open_port(port, baud, timeout=0.05, factory=None, attempts=OPEN_ATTEMPTS,
              retry_s=OPEN_RETRY_S, sleep=time.sleep):
    """Open a serial port; Bluetooth ports that time out connecting are retried."""
    if factory is None:
        import serial
        factory = serial.Serial
    for attempt in range(1, attempts + 1):
        try:
            return factory(port, baud, timeout=timeout)
        except OSError as e:                      # serial.SerialException is an OSError
            if "FileNotFoundError" in str(e):
                raise SystemExit(f"{port} does not exist; ports here: {', '.join(port_names()) or 'none'}")
            if attempt == attempts:
                raise SystemExit(f"could not open {port}: {e}\n{BLUETOOTH_HINT}")
            print(f"{port}: Bluetooth did not connect ({attempt}/{attempts}), retrying in {retry_s:g} s")
            sleep(retry_s)


def wait_for_data(link, seconds=LINK_WAIT_S, until=None):
    """Bytes received within `seconds` (empty: nothing came). Stops at the first bytes, or once
    the regex `until` matches. A line sent before the link is through may be lost."""
    data, end = b"", time.time() + seconds
    while time.time() < end:
        data += link.read(4096)
        if data and (until is None or re.search(until, data)):
            break
    return data


def connect_gimbal(port, baud):
    link = open_port(port, baud)
    if not wait_for_data(link):
        link.write(b"L\n")                      # live stream may be paused by an unfinished replay
        if not wait_for_data(link):
            print(f"{port}: no telemetry yet (gimbal running? right port?)")
    return link


HW_LINE = rb"HW [^\r\n]*"


def chassis_rx_bytes(data):
    """Bytes the chassis has received on its command port (rx_bytes= in its HW status line)."""
    found = re.findall(rb"rx_bytes=(\d+)", data)
    return int(found[-1]) if found else None


def chassis_receives(link, wait_s=2.5):
    """The chassis prints its status every second; STATUS must raise its rx_bytes. The
    Bluetooth bridge sometimes passes chassis -> PC but not PC -> chassis (2026-10-24)."""
    before = chassis_rx_bytes(wait_for_data(link, 3.0, until=HW_LINE + rb"\r?\n"))
    link.write(b"STATUS\r\n")
    data, end = b"", time.time() + wait_s
    while time.time() < end:
        data += link.read(4096)
        after = chassis_rx_bytes(data)
        if before is not None and after is not None and after > before:
            hw = re.findall(HW_LINE, data)
            return hw[-1].decode("ascii", "replace")
    return None


def connect_chassis(port, baud, attempts=3, open_link=None, wait_s=2.5, sleep=time.sleep):
    """Chassis port that verifiably receives commands (reopened when it does not)."""
    open_link = open_link or (lambda: open_port(port, baud))
    for attempt in range(1, attempts + 1):
        link = open_link()
        status = chassis_receives(link, wait_s)
        if status:
            print("chassis:", status)
            return link
        link.close()
        print(f"{port}: the chassis did not receive STATUS ({attempt}/{attempts}), reconnecting")
        sleep(1.0)
    raise SystemExit(f"{port}: the chassis receives nothing (rx_bytes does not rise): DRIVE would be lost; "
                     "power-cycle the chassis Bluetooth or retry in a minute")


def command(link, text, timeout=1.0, attempts=COMMAND_ATTEMPTS):
    """Send one command line, return the gimbal's text reply (None if it never answered).
    Resent when unanswered (commands are idempotent)."""
    for _ in range(attempts):
        link.write((text + "\n").encode("ascii"))
        buffer, end = b"", time.time() + timeout
        while time.time() < end:
            buffer += link.read(4096)
            reply = find_reply(buffer)
            if reply:
                return reply
    return None


def set_values(link, assignments):
    for name, value in assignments:
        reply = command(link, f"S {name} {value:g}")
        print(f"S {name} {value:g} -> {reply or 'no reply (old firmware? wrong port?)'}")
        if not reply or not reply.startswith("#OK"):
            raise SystemExit("setting failed; nothing recorded")


def record(link, seconds, replay=False, on_time=None):
    """Frames from an open link: live for `seconds`, or the replayed recording.
    on_time(elapsed_s) is called on every read (used to send DRIVE)."""
    parser = JustFloatParser()
    rows = []
    if replay:
        link.write(b"P\n")           # stop live frames so only the recording comes back
        time.sleep(0.2)
        link.reset_input_buffer()
        link.write(b"D\n")
    start = last_data = time.time()
    while True:
        data = link.read(4096)
        now = time.time()
        if on_time:
            on_time(now - start)
        if data:
            last_data = now
            rows += parser.feed(data)
        if replay and rows and now - last_data > 1.5:
            break
        if not replay and now - start >= seconds:
            break
        if replay and not rows and now - start > 5.0:
            raise SystemExit("no replay received: check the port, baud and that the gimbal runs")
    if replay:
        link.write(b"L\n")           # live stream back on
    return rows, parser.dropped


def save(rows, dropped, out):
    write_csv(out, rows)
    print(f"{len(rows)} frames -> {out}  (bytes dropped while syncing: {dropped})")


def capture(port, baud, seconds, replay, out):
    out = out if out.endswith(".csv") else out + ".csv"
    with connect_gimbal(port, baud) as link:
        rows, dropped = record(link, seconds, replay)
    save(rows, dropped, out)
    return rows


def chassis_ack(line):
    """What the chassis prints once it has taken `line` (None: nothing to check)."""
    laps = re.fullmatch(r"LAPS ([1-5])", line)
    if laps:
        return ("STATUS", rb"AUTO armed=\d+ state=\d+ fault=\d+ laps=" + laps.group(1).encode())
    if line.startswith("TURN "):
        return (None, rb"AUTO TURN turn_mm_s=")
    if line.upper().startswith("SET "):
        return (None, rb"AUTO SET (?!REJECTED)[^\r\n]*\r?\n")
    if line == "LOG ON":
        return (None, rb"AUTO LOG ON")
    return None


def send_chassis_checked(link, line, answer, attempts=COMMAND_ATTEMPTS, wait_s=0.8):
    """Send a setup command until the chassis confirms it: the Bluetooth bridge sometimes
    drops bytes towards the chassis (2026-10-28: LAPS 3 lost, the car ran 1 lap)."""
    ack = chassis_ack(line)
    for attempt in range(1, attempts + 1):
        start = len(answer)
        link.write((line + "\r\n").encode("ascii"))
        if ack and ack[0]:
            time.sleep(0.2)
            link.write((ack[0] + "\r\n").encode("ascii"))
        end = time.time() + wait_s
        while time.time() < end:
            answer.extend(link.read(link.in_waiting or 1))
            got = bytes(answer[start:])
            rejected = re.search(rb"AUTO (?:SET|TURN) REJECTED[^\r\n]*", got)
            if rejected:
                raise SystemExit(f"chassis rejected {line!r}: {rejected.group().decode('ascii', 'replace')}")
            confirmed = ack and re.search(ack[1], got)
            if confirmed:
                print(f"{line!r} confirmed by the chassis: {confirmed.group().decode('ascii', 'replace').strip()}")
                return
        if not ack:
            print(f"{line!r} sent to the chassis (no confirmation to check)")
            return
        print(f"{line!r} not confirmed ({attempt}/{attempts}), resending")
    raise SystemExit(f"chassis did not confirm {line!r}: check the chassis Bluetooth link")


def run_test(args):
    """Set values, record, send DRIVE on the chassis Bluetooth port after --delay, analyse."""
    out = args.out if args.out.endswith(".csv") else args.out + ".csv"
    plot_path = args.plot or out[:-4] + ".png"
    with connect_gimbal(args.port, args.baud) as link:
        set_values(link, parse_assignments(args.set or []))
        current = command(link, "G")
        print("gimbal:", current or "no reply")
        chassis = connect_chassis(args.chassis, args.chassis_baud) if args.chassis else None
        answer = bytearray()
        for line in (args.chassis_pre or []) if chassis else []:
            send_chassis_checked(chassis, line, answer)
        sent = {"at": None, "count": 0}
        # DRIVE answers "AUTO DRIVE ..."; RUN only prints its status line, so it is sent once
        # (a second RUN while running is rejected anyway).
        is_drive = args.drive.upper().startswith("DRIVE")
        max_sends = 2 if is_drive else 1

        def on_time(elapsed):
            if not chassis:
                return
            answer.extend(chassis.read(chassis.in_waiting))
            if b"AUTO DRIVE" in answer or elapsed < args.delay or sent["count"] >= max_sends:
                return
            if sent["at"] is None or elapsed - sent["at"] > DRIVE_RESEND_S:
                chassis.write((args.drive + "\r\n").encode("ascii"))
                sent["at"], sent["count"] = elapsed, sent["count"] + 1
                print(f"{args.drive!r} sent at {elapsed:.2f} s")
        try:
            if not chassis:
                print(f"no --chassis port: send {args.drive!r} yourself now")
            rows, dropped = record(link, args.seconds, on_time=on_time)
        finally:
            if chassis:
                answer.extend(chassis.read(chassis.in_waiting))
                replies = re.findall(rb"AUTO (?:DRIVE|RUN|TURN) [^\r\n]*", bytes(answer))
                for line in replies:
                    print("chassis:", line.decode("ascii", "replace"))
                if args.chassis_log:
                    with open(args.chassis_log, "wb") as f:
                        f.write(bytes(answer))
                    print(f"chassis output ({len(answer)} bytes) -> {args.chassis_log}")
                if not replies and is_drive:
                    print("chassis: no answer to DRIVE (wrong port, or the Bluetooth link is not through)")
                chassis.close()
    save(rows, dropped, out)
    with open(out[:-4] + "_params.txt", "w") as f:
        f.write((current or "") + "\n")
    if len(rows) < LIVE_SHORT_FRACTION * args.seconds * TELEMETRY_HZ:
        # The live link dropped: fetch the on-board recorder now, before idle frames overwrite it.
        print("live telemetry short: fetching the on-board recording (last 60 s)")
        try:
            replay = capture(args.port, args.baud, 0, True, out[:-4] + "_replay.csv")
            if len(replay) > len(rows):
                rows = replay
            else:
                print(f"replay has {len(replay)} frames, fewer than live: analysing the live ones")
        except SystemExit as e:
            print("replay failed:", e)
    return rows, plot_path


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("capture")
    c.add_argument("port")
    c.add_argument("--baud", type=int, default=115200)
    c.add_argument("--seconds", type=float, default=30.0)
    c.add_argument("--replay", action="store_true", help="fetch the on-board recording instead")
    c.add_argument("-o", "--out", default="telemetry.csv")
    c.add_argument("--plot")
    a = sub.add_parser("analyze")
    a.add_argument("csv")
    a.add_argument("--plot")
    rp = sub.add_parser("report", help="runs side by side (RUN.csv + RUN_chassis.txt), see run_report.py")
    rp.add_argument("runs", nargs="+", metavar="RUN")
    vs = sub.add_parser("vision", help="why K230 frames were invalid, per track section (RUN.csv)")
    vs.add_argument("run", metavar="RUN")
    st = sub.add_parser("set")
    st.add_argument("port")
    st.add_argument("assignments", nargs="+", metavar="name=value")
    st.add_argument("--baud", type=int, default=115200)
    gt = sub.add_parser("get")
    gt.add_argument("port")
    gt.add_argument("--baud", type=int, default=115200)
    r = sub.add_parser("run")
    r.add_argument("port", help="gimbal Bluetooth COM port")
    r.add_argument("--baud", type=int, default=115200)
    r.add_argument("--chassis", help="chassis Bluetooth COM port, e.g. COM10 (DRIVE is sent there)")
    r.add_argument("--chassis-baud", type=int, default=115200)
    r.add_argument("--drive", default="DRIVE 20 60")
    r.add_argument("--delay", type=float, default=1.0, help="seconds of recording before DRIVE")
    r.add_argument("--chassis-pre", nargs="*", metavar="CMD",
                   help='chassis commands sent before recording, e.g. "LAPS 3" "LOG ON"')
    r.add_argument("--chassis-log", help="save everything the chassis sent (LOG ON lines) to this file")
    r.add_argument("--set", nargs="*", metavar="name=value")
    r.add_argument("--seconds", type=float, default=7.0)
    r.add_argument("-o", "--out", required=True)
    r.add_argument("--plot")
    args = ap.parse_args(argv)
    if args.cmd in ("set", "get"):
        with connect_gimbal(args.port, args.baud) as link:
            if args.cmd == "set":
                set_values(link, parse_assignments(args.assignments))
            print(command(link, "G") or "no reply (old firmware? wrong port?)")
        return 0
    if args.cmd == "report":
        import run_report
        run_report.report(args.runs)
        return 0
    if args.cmd == "vision":
        import run_report
        run_report.vision_sections(args.run)
        return 0
    if args.cmd == "run":
        rows, plot_path = run_test(args)
        print_report(analyze(rows))
        plot(rows, plot_path)
        out = args.out[:-4] if args.out.endswith(".csv") else args.out
        if args.chassis_log and args.chassis_log == out + "_chassis.txt":
            import run_report
            run_report.report([out])
            run_report.vision_sections(out)
        return 0
    rows = (capture(args.port, args.baud, args.seconds, args.replay, args.out)
            if args.cmd == "capture" else read_csv(args.csv))
    print_report(analyze(rows))
    if args.plot:
        plot(rows, args.plot)


if __name__ == "__main__":
    sys.exit(main())
