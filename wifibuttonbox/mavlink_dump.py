#!/usr/bin/env python3
"""
MAVLink serial dump.

Connects to a serial MAVLink link (e.g. the buttonbox's ArduPilotSerial UART,
tapped via a USB-serial adapter) and prints every message received, one line
per message. Handy for checking what's actually flowing on the wire without
digging through the ESP32's own on-screen log.

Requires:
    pip install pymavlink

Usage:
    python3 mavlink_dump.py
    python3 mavlink_dump.py --device /dev/ttyUSB1 --baud 57600
    python3 mavlink_dump.py --type HEARTBEAT --type ATTITUDE
    python3 mavlink_dump.py --exclude BAD_DATA --exclude RADIO_STATUS
    python3 mavlink_dump.py --raw
"""

import argparse
import time

from pymavlink import mavutil


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    p.add_argument("--device", default="/dev/ttyUSB0",
                    help="Serial device (default: /dev/ttyUSB0)")
    p.add_argument("--baud", type=int, default=115200,
                    help="Baud rate (default: 115200, matches the buttonbox's ArduPilotSerial)")
    p.add_argument("--type", action="append", dest="types", metavar="MSGTYPE",
                    help="Only show these message types (repeatable). Default: all.")
    p.add_argument("--exclude", action="append", dest="excludes", metavar="MSGTYPE",
                    help="Hide these message types (repeatable).")
    p.add_argument("--raw", action="store_true",
                    help="Print a full field dump instead of a compact one-liner")
    p.add_argument("--count", type=int, default=None,
                    help="Stop after this many messages (default: run until Ctrl+C)")
    return p.parse_args()


def format_message(msg, raw):
    t = time.strftime("%H:%M:%S", time.localtime())
    prefix = f"[{t}] sys{msg.get_srcSystem()}/comp{msg.get_srcComponent()}"
    if raw:
        return f"{prefix} {msg.get_type()}\n{msg}"
    fields = msg.to_dict()
    fields.pop("mavpackettype", None)
    field_str = " ".join(f"{k}={v}" for k, v in fields.items())
    return f"{prefix} {msg.get_type()}: {field_str}"


def main():
    args = parse_args()
    types = set(args.types) if args.types else None
    excludes = set(args.excludes) if args.excludes else set()

    print(f"Connecting to {args.device} @ {args.baud} baud...")
    conn = mavutil.mavlink_connection(args.device, baud=args.baud, dialect="ardupilotmega")

    print("Waiting for messages (Ctrl+C to stop)...")
    count = 0
    try:
        while True:
            msg = conn.recv_match(blocking=True, timeout=5)
            if msg is None:
                continue
            msg_type = msg.get_type()
            if types is not None and msg_type not in types:
                continue
            if msg_type in excludes:
                continue

            print(format_message(msg, args.raw))
            count += 1
            if args.count is not None and count >= args.count:
                break
    except KeyboardInterrupt:
        pass
    print(f"\nStopped after {count} message(s).")


if __name__ == "__main__":
    main()
