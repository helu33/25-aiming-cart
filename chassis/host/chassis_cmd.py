"""Send commands to the chassis debug port (UART5 / Bluetooth, 115200 8N1) and print the replies.

    py -3.13 chassis/host/chassis_cmd.py COM10 STATUS PARAMS
    py -3.13 chassis/host/chassis_cmd.py COM10 "SET decel 0.6" "LAPS 3"
    py -3.13 chassis/host/chassis_cmd.py COM10 "CIRCLE 20" --expect "AUTO CIRCLE"

Without --expect each command is sent once and every reply line within 1.5 s is printed
(the 50 Hz "D," log lines are left out). With --expect TEXT the single command is resent, up to
5 times, until a line containing TEXT comes back (the Bluetooth link drops bytes now and then);
the exit code is 1 when it never does.

Any known command cancels the power-up run (see docs/GIMBAL_TUNING.md, section 2): to watch a
power-up run without touching it, use listen_chassis.py. Commands: chassis/AUTONOMOUS_CHASSIS.md.
Needs pyserial; close any other program that holds the port first.
"""
import argparse
import sys
import time

import serial

BAUD = 115200
OPEN_TRIES = 6
SEND_TRIES = 5


def open_port(port):
    """Bluetooth ports often refuse the first connection: retry for about 12 s."""
    for attempt in range(OPEN_TRIES):
        try:
            return serial.Serial(port, BAUD, timeout=0.2)
        except OSError as e:
            print("open retry", attempt + 1, e)
            time.sleep(2)
    raise SystemExit("could not open " + port)


def send_until(link, cmd, expect):
    """Resend cmd until a reply line contains expect; True when confirmed."""
    for attempt in range(1, SEND_TRIES + 1):
        link.reset_input_buffer()
        link.write((cmd + "\r\n").encode("ascii"))
        end, buf = time.time() + 2.0, b""
        while time.time() < end:
            buf += link.read(4096)
            if expect.encode() in buf:
                line = [l for l in buf.decode("ascii", "replace").splitlines() if expect in l][0]
                print(f"confirmed on attempt {attempt}: {line.strip()} at {time.strftime('%H:%M:%S')}")
                return True
        print(f"no '{expect}' yet (attempt {attempt}), resending")
    return False


def send_each(link, cmds):
    """Send each command once and print its reply lines."""
    for c in cmds:
        link.write((c + "\r\n").encode("ascii"))
        end = time.time() + 1.5
        buf = b""
        while time.time() < end:
            buf += link.read(4096)
        for line in buf.decode("ascii", "replace").splitlines():
            if line.strip() and not line.startswith("D,"):
                print(c, "->", line.strip()[:240])


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("port", help="chassis debug/Bluetooth COM port, e.g. COM10")
    ap.add_argument("commands", nargs="+", help="commands, e.g. STATUS or \"LAPS 3\"")
    ap.add_argument("--expect", help="resend the single command until a reply contains this text")
    args = ap.parse_args()
    if args.expect and len(args.commands) != 1:
        ap.error("--expect takes exactly one command")
    link = open_port(args.port)
    try:
        if args.expect:
            time.sleep(1.0)
            if not send_until(link, args.commands[0], args.expect):
                sys.exit(f"'{args.commands[0]}' not confirmed")
        else:
            time.sleep(0.5)
            send_each(link, args.commands)
    finally:
        link.close()


if __name__ == "__main__":
    main()
