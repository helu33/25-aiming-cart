#!/usr/bin/env python3

import argparse

import serial


def main():
    parser = argparse.ArgumentParser(
        description="Print ASCII hardware-test telemetry from the STM32."
    )
    parser.add_argument("--port", default="/dev/ttyS6")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    print(f"Opening {args.port} at {args.baud} baud. Press Ctrl+C to stop.")
    try:
        with serial.Serial(
            port=args.port,
            baudrate=args.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
            exclusive=True,
        ) as uart:
            while True:
                line = uart.readline()
                if line:
                    print(line.decode("ascii", errors="replace").rstrip(), flush=True)
    except KeyboardInterrupt:
        print("\nStopped.")
    except serial.SerialException as exc:
        raise SystemExit(f"Serial error: {exc}") from exc


if __name__ == "__main__":
    main()
