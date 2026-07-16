"""Run bounded RTU timing and recovery checks against a serial slave."""

from __future__ import annotations

import argparse
import time

import serial


REQUEST = bytes.fromhex("01 03 00 00 00 01 84 0A")
EXPECTED = bytes.fromhex("01 03 02 05 63 FB 3D")


def busy_wait(seconds: float) -> None:
    deadline = time.perf_counter() + seconds
    while time.perf_counter() < deadline:
        pass


def read_reply(port: serial.Serial) -> bytes:
    return port.read(len(EXPECTED))


def exchange(port: serial.Serial) -> bytes:
    port.reset_input_buffer()
    port.write(REQUEST)
    port.flush()
    return read_reply(port)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--baud", type=int, required=True)
    parser.add_argument("--count", type=int, default=100)
    parser.add_argument("--t15-gap-ms", type=float)
    parser.add_argument("--t35-gap-ms", type=float)
    parser.add_argument("--normal-only", action="store_true")
    args = parser.parse_args()

    if args.baud > 19200:
        t15 = 0.000750
        t35 = 0.001750
    else:
        char_time = 10.0 / args.baud
        t15 = 1.5 * char_time
        t35 = 3.5 * char_time

    with serial.Serial(args.port, args.baud, timeout=0.08) as port:
        first = exchange(port)
        print("baseline", first.hex(" ").upper())
        if first != EXPECTED:
            return 1

        started = time.perf_counter()
        passed = 0
        for _ in range(args.count):
            if exchange(port) == EXPECTED:
                passed += 1
        elapsed = time.perf_counter() - started
        print(f"stress {passed}/{args.count} elapsed={elapsed:.3f}s")
        if passed != args.count:
            return 2
        if args.normal_only:
            return 0

        port.reset_input_buffer()
        port.write(REQUEST[:4])
        port.flush()
        illegal_gap = ((t15 + t35) / 2.0 if args.t15_gap_ms is None
                       else args.t15_gap_ms / 1000.0)
        busy_wait(illegal_gap)
        port.write(REQUEST[4:])
        port.flush()
        illegal_gap_reply = read_reply(port)
        print("t15_to_t35_reply", illegal_gap_reply.hex(" ").upper() or "<none>")
        if illegal_gap_reply:
            return 3

        recovery = exchange(port)
        print("recovery_after_t15", recovery.hex(" ").upper())
        if recovery != EXPECTED:
            return 4

        port.reset_input_buffer()
        port.write(REQUEST[:4])
        port.flush()
        split_gap = (t35 + 0.0007 if args.t35_gap_ms is None
                     else args.t35_gap_ms / 1000.0)
        busy_wait(split_gap)
        port.write(REQUEST[4:])
        port.flush()
        split_reply = read_reply(port)
        print("t35_split_reply", split_reply.hex(" ").upper() or "<none>")
        if split_reply:
            return 5

        recovery = exchange(port)
        print("recovery_after_t35", recovery.hex(" ").upper())
        return 0 if recovery == EXPECTED else 6


if __name__ == "__main__":
    raise SystemExit(main())
