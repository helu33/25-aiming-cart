"""Record the chassis debug port (UART5 / Bluetooth, 115200 8N1) without sending anything.

    py -3.13 chassis/host/listen_chassis.py COM10 60 logs/2026-10/auto1_chassis_listen.txt

Any command on that port cancels the power-up run, so this is the way to log one: start it
before (or right after) power-up, keep your hands off the car. Every line goes to OUTFILE with
the seconds since the start in front; all but the 50 Hz "D," log lines (LOG ON) are also
printed. The port is retried for about 20 s, so it can be started before the module pairs.
Needs pyserial; close any other program that holds the port first.
"""
import argparse
import time

import serial

BAUD = 115200
OPEN_TRIES = 10


def open_port(port):
    for attempt in range(OPEN_TRIES):
        try:
            return serial.Serial(port, BAUD, timeout=0.2)
        except OSError as e:
            print(f"{port}: not connected yet ({attempt + 1}/{OPEN_TRIES}): {e}", flush=True)
            time.sleep(2)
    raise SystemExit(f"{port}: could not open")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("port", help="chassis debug/Bluetooth COM port, e.g. COM10")
    ap.add_argument("seconds", type=float, help="how long to listen")
    ap.add_argument("outfile", help="text file for the time-stamped lines")
    args = ap.parse_args()
    link = open_port(args.port)
    start = time.time()
    buf = b""
    with open(args.outfile, "w", encoding="ascii", errors="replace") as f:
        while time.time() - start < args.seconds:
            buf += link.read(4096)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("ascii", "replace").strip()
                stamp = f"{time.time() - start:7.2f}"
                f.write(f"{stamp} {text}\n")
                if not text.startswith("D,"):
                    print(f"{stamp} {text}", flush=True)
    link.close()


if __name__ == "__main__":
    main()
