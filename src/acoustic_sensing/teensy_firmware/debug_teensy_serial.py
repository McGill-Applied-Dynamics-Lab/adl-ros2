#!/usr/bin/env python3
"""Inspect raw Teensy serial traffic and optionally send command bytes."""

from __future__ import annotations

import argparse
import time

import serial


DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 3000000
CMD_START = 0x43
CMD_FINISH = 0x45


def _parse_byte(value: str) -> int:
    if value.lower().startswith("0x"):
        parsed = int(value, 16)
    elif len(value) == 1 and not value.isdigit():
        parsed = ord(value)
    else:
        parsed = int(value)

    if not 0 <= parsed <= 255:
        raise argparse.ArgumentTypeError(f"byte must be in [0, 255], got {parsed}")
    return parsed


def _format_chunk(chunk: bytes) -> str:
    ints = " ".join(str(b) for b in chunk)
    hexes = " ".join(f"{b:02X}" for b in chunk)
    text = chunk.decode("ascii", errors="replace").replace("\n", "\\n").replace("\r", "\\r")
    return f"len={len(chunk):04d} dec=[{ints}] hex=[{hexes}] ascii='{text}'"


def _run_burst_test(ser: serial.Serial, chunk_size: int) -> None:
    ser.reset_input_buffer()
    ser.write(bytes([CMD_START]))
    print(f"{time.monotonic():.6f} TX dec={CMD_START} hex=0x{CMD_START:02X} ascii='C'")
    print("Waiting for frame data, then end byte 69 / 0x45 / 'E'...")

    saw_frame_data = False
    while True:
        chunk = ser.read(min(max(ser.in_waiting, 1), chunk_size))
        if not chunk:
            continue

        if not saw_frame_data:
            cleaned = chunk.replace(bytes([CMD_FINISH]), b"")
            if b"S0" in cleaned or b"S1" in cleaned or b"S2" in cleaned or b"S3" in cleaned:
                saw_frame_data = True
                print(f"{time.monotonic():.6f} RX frame data started")
            continue

        if CMD_FINISH in chunk:
            print(f"{time.monotonic():.6f} RX received end byte 69 / 0x45 / 'E'")
            return


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Print raw Teensy serial traffic and optionally send command bytes."
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"serial port, default {DEFAULT_PORT}")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"baud rate, default {DEFAULT_BAUD}")
    parser.add_argument(
        "--send",
        type=_parse_byte,
        action="append",
        default=[],
        help="byte to send once after opening, e.g. 67, 69, 0x43, or C. Can be repeated.",
    )
    parser.add_argument(
        "--send-interval",
        type=float,
        default=0.0,
        help="if set > 0, repeatedly send the last --send byte every N seconds",
    )
    parser.add_argument(
        "--chunk-size",
        type=int,
        default=256,
        help="maximum bytes to read per print, default 256",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.05,
        help="serial read timeout in seconds, default 0.05",
    )
    parser.add_argument(
        "--reset-input",
        action="store_true",
        help="discard buffered Teensy bytes immediately after opening",
    )
    parser.add_argument(
        "--burst-test",
        action="store_true",
        help="send 67 once, wait for RF frame data, then print when 69 is received and exit",
    )
    args = parser.parse_args()

    with serial.Serial(args.port, args.baud, timeout=args.timeout) as ser:
        if args.reset_input:
            ser.reset_input_buffer()

        print(f"Opened {args.port} at {args.baud} baud")
        print("Press Ctrl-C to stop.")

        if args.burst_test:
            _run_burst_test(ser, args.chunk_size)
            return

        for value in args.send:
            ser.write(bytes([value]))
            print(f"{time.monotonic():.6f} TX dec={value} hex=0x{value:02X} ascii={chr(value)!r}")

        repeat_byte = args.send[-1] if args.send and args.send_interval > 0 else None
        next_send = time.monotonic() + args.send_interval if repeat_byte is not None else None

        try:
            while True:
                now = time.monotonic()
                if repeat_byte is not None and next_send is not None and now >= next_send:
                    ser.write(bytes([repeat_byte]))
                    print(f"{now:.6f} TX dec={repeat_byte} hex=0x{repeat_byte:02X} ascii={chr(repeat_byte)!r}")
                    next_send = now + args.send_interval

                waiting = ser.in_waiting
                chunk = ser.read(min(max(waiting, 1), args.chunk_size))
                if chunk:
                    print(f"{time.monotonic():.6f} RX {_format_chunk(chunk)}")
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
