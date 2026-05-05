#!/usr/bin/env python3
"""
Host-side controller for x8rl1-pico.

Talks to the Pico over USB CDC (a serial port that appears as
/dev/cu.usbmodem* on macOS, /dev/ttyACM* on Linux, COM* on Windows).
Sends text commands and reads back framed binary data, writing it
to X1EMU-format .TAP files.

Examples:
    receiver.py --port /dev/cu.usbmodem1234 read tape.tap
    receiver.py --port /dev/cu.usbmodem1234 read --rate 16000 --max 30 tape.tap
    receiver.py --port /dev/cu.usbmodem1234 write tape.tap
    receiver.py --port /dev/cu.usbmodem1234 cmd 02       # PLAY
    receiver.py --port /dev/cu.usbmodem1234 status
"""

import argparse
import struct
import sys
import time
from pathlib import Path

import serial


CZ8RL1_CMD_NAMES = {
    0x00: "EJECT", 0x01: "STOP",  0x02: "PLAY",  0x03: "FF",
    0x04: "REW",   0x05: "AFF",   0x06: "AREW",  0x0A: "REC",
    0x80: "STATUS", 0x81: "SENSOR",
}


class PicoLink:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 5.0):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        # Flush any lingering bytes from a previous session.
        time.sleep(0.1)
        self.ser.reset_input_buffer()

    def send_line(self, line: str) -> None:
        self.ser.write((line + "\n").encode())
        self.ser.flush()

    def read_line(self) -> str:
        raw = self.ser.readline()
        if not raw:
            raise TimeoutError("no response from Pico")
        return raw.decode(errors="replace").rstrip("\r\n")

    def read_exact(self, n: int) -> bytes:
        chunks = []
        remaining = n
        while remaining > 0:
            chunk = self.ser.read(remaining)
            if not chunk:
                raise TimeoutError(f"short read: got {n - remaining}/{n}")
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def wait_for_ready(self, attempts: int = 20) -> None:
        # Send a PING and look for PONG; some hosts buffer the boot READY.
        for _ in range(attempts):
            self.send_line("PING")
            try:
                line = self.read_line()
            except TimeoutError:
                continue
            if line.endswith("PONG") or line == "PONG":
                return
        raise RuntimeError("Pico did not respond to PING")


def read_x1emu_tap(path: Path) -> tuple[int, bytes]:
    raw = path.read_bytes()
    if len(raw) < 4:
        raise ValueError(f"{path} is too short to be a .TAP file")
    rate = struct.unpack("<I", raw[:4])[0]
    if not (1000 <= rate <= 200000):
        raise ValueError(f"{path}: invalid sampling rate {rate} Hz "
                         f"(only X1EMU format is supported here)")
    return rate, raw[4:]


def cmd_read(link: PicoLink, args: argparse.Namespace) -> int:
    out = Path(args.output)
    if out.exists() and not args.force:
        print(f"refusing to overwrite {out} (use --force)", file=sys.stderr)
        return 1

    print(f"sampling at {args.rate} Hz, max {args.max}s, "
          f"idle-stop {args.idle}s ...")
    link.send_line(f"READ {args.rate} {args.max} {args.idle}")

    line = link.read_line()
    if not line.startswith("OK"):
        print(f"unexpected response: {line!r}", file=sys.stderr)
        return 1

    # Stream CHUNK frames straight to disk; loop ends when we see STATS.
    print("waiting for tape... (Ctrl-C to abort the host side)")
    total = 0
    stats_line = None
    with out.open("wb") as f:
        f.write(struct.pack("<I", args.rate))
        while True:
            try:
                line = link.read_line()
            except TimeoutError:
                # No data yet -- tape may still be spinning up; keep waiting.
                continue
            if line.startswith("CHUNK "):
                length = int(line.split()[1])
                f.write(link.read_exact(length))
                total += length
                # Lightweight progress indicator.
                sys.stdout.write(f"\r  {total} bytes received")
                sys.stdout.flush()
            elif line.startswith("STATS "):
                stats_line = line
                break
            elif line.startswith("ERR "):
                print(f"\ndevice error: {line}", file=sys.stderr)
                return 1
            else:
                # Unknown line -- log and keep going.
                print(f"\nunexpected: {line!r}", file=sys.stderr)

    print()
    if stats_line:
        print(stats_line)
    end = link.read_line()
    if end != "END":
        print(f"warning: expected END, got {end!r}", file=sys.stderr)

    print(f"saved {total} bytes to {out}")
    return 0


def cmd_write(link: PicoLink, args: argparse.Namespace) -> int:
    rate, data = read_x1emu_tap(Path(args.input))
    if args.rate is not None:
        rate = args.rate
    print(f"sending {len(data)} bytes at {rate} Hz")

    link.send_line(f"WRITE {rate} {len(data)}")
    line = link.read_line()
    if not line.startswith("OK"):
        print(f"unexpected response: {line!r}", file=sys.stderr)
        return 1

    link.ser.write(data)
    link.ser.flush()

    result = link.read_line()
    end = link.read_line()
    print(result)
    if end != "END":
        print(f"warning: expected END, got {end!r}", file=sys.stderr)
    return 0


def cmd_cmd(link: PicoLink, args: argparse.Namespace) -> int:
    code = int(args.code, 16)
    name = CZ8RL1_CMD_NAMES.get(code, "?")
    print(f"sending command {code:#04x} ({name})")
    link.send_line(f"CMD {code:02x}")
    line = link.read_line()
    print(line)
    return 0 if line.startswith("STS") else 1


def cmd_status(link: PicoLink, _args: argparse.Namespace) -> int:
    link.send_line("STATUS")
    print(link.read_line())
    link.send_line("SENSOR")
    print(link.read_line())
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", required=True, help="serial device for the Pico")
    p.add_argument("--baud", type=int, default=115200)

    sub = p.add_subparsers(dest="cmd", required=True)

    pr = sub.add_parser("read", help="record tape into a .TAP file")
    pr.add_argument("output")
    pr.add_argument("--rate", type=int, default=8000)
    pr.add_argument("--max",  type=int, default=60, help="max seconds")
    pr.add_argument("--idle", type=int, default=15, help="idle-stop seconds")
    pr.add_argument("--force", action="store_true")
    pr.set_defaults(func=cmd_read)

    pw = sub.add_parser("write", help="play a .TAP file to the CZ8RL1")
    pw.add_argument("input")
    pw.add_argument("--rate", type=int, default=None,
                    help="override the rate stored in the file")
    pw.set_defaults(func=cmd_write)

    pc = sub.add_parser("cmd", help="send a raw CZ8RL1 command (hex)")
    pc.add_argument("code", help="command byte in hex (e.g. 02 = PLAY)")
    pc.set_defaults(func=cmd_cmd)

    ps = sub.add_parser("status", help="query device status + sensor")
    ps.set_defaults(func=cmd_status)

    args = p.parse_args()
    link = PicoLink(args.port, args.baud)
    link.wait_for_ready()
    return args.func(link, args)


if __name__ == "__main__":
    sys.exit(main())
