#!/usr/bin/env python3
"""
Host-side controller for x8rl1-pico.

Talks to the Pico over USB CDC (a serial port that appears as
/dev/cu.usbmodem* on macOS, /dev/ttyACM* on Linux, COM* on Windows).
Sends text commands and reads back framed binary data, writing it
to X1EMU-format .TAP files.

Examples:
    x8rl1-pico.py --port /dev/cu.usbmodem1234 read tape.tap
    x8rl1-pico.py --port /dev/cu.usbmodem1234 read --rate 16000 --max 30 tape.tap
    x8rl1-pico.py --port /dev/cu.usbmodem1234 write tape.tap
    x8rl1-pico.py --port /dev/cu.usbmodem1234 cmd 02       # PLAY
    x8rl1-pico.py --port /dev/cu.usbmodem1234 status
"""

from __future__ import annotations

import argparse
import re
import struct
import sys
import time
from pathlib import Path

import serial
from serial.tools import list_ports


CZ8RL1_CMD_NAMES = {
    0x00: "EJECT", 0x01: "STOP",  0x02: "PLAY",  0x03: "FF",
    0x04: "REW",   0x05: "AFF",   0x06: "AREW",  0x0A: "REC",
    0x80: "STATUS", 0x81: "SENSOR",
}

# Mirrors CZ8RL1_STS_AUTOSTOP in src/cz8rl1.h (0x101).
CZ8RL1_STS_AUTOSTOP = 0x101

# Raspberry Pi VID, plus the PID assigned to pico_stdlib's stdio_usb CDC.
PICO_VID = 0x2E8A
PICO_STDIO_PID = 0x000A


def find_pico_port() -> str | None:
    """Return the /dev path of a Pico running stdio_usb, or None."""
    matches = [p for p in list_ports.comports()
               if p.vid == PICO_VID and p.pid == PICO_STDIO_PID]
    if not matches:
        return None
    if len(matches) > 1:
        names = ", ".join(p.device for p in matches)
        print(f"warning: found {len(matches)} Pico devices ({names}); "
              f"using {matches[0].device}", file=sys.stderr)
    return matches[0].device


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

    def wait_for_ready(self, attempts: int = 20) -> str:
        """PING the device, return the identifying PONG line."""
        for _ in range(attempts):
            self.send_line("PING")
            try:
                line = self.read_line()
            except TimeoutError:
                continue
            if line.startswith("PONG"):
                # Earlier retries may have queued extra PINGs that the
                # Pico is still echoing back; drain them so the next
                # read_line() returns the real command response.
                time.sleep(0.1)
                self.ser.reset_input_buffer()
                return line
        raise RuntimeError("Pico did not respond to PING")


"""
.TAP file formats:

X1EMU:
    [4]  rate_hz (LE)
    [..] raw sample bytes (MSB-first within each byte)

XMillennium T-tune:
    [0x00, 4]  magic "TAPE" (0x45504154 LE)
    [0x04,17]  name (ASCIIZ, NUL-padded)
    [0x15, 5]  reserved (zeros)
    [0x1A, 1]  protect  (0x10 = write protected)
    [0x1B, 1]  format   (0x01 = sampling)
    [0x1C, 4]  frequency (Hz, LE)
    [0x20, 4]  datasize  (in BITS, LE)
    [0x24, 4]  position  (in BITS, LE)
    [0x28,..]  raw sample bytes
"""
TTUNE_MAGIC = b"TAPE"
TTUNE_HEADER_SIZE = 0x28


def read_tap(path: Path) -> tuple[int, bytes]:
    """Read an X1EMU or T-tune .TAP file. Returns (rate_hz, data_bytes)."""
    raw = path.read_bytes()
    if len(raw) < 4:
        raise ValueError(f"{path} is too short to be a .TAP file")

    if raw[:4] == TTUNE_MAGIC:
        if len(raw) < TTUNE_HEADER_SIZE:
            raise ValueError(f"{path}: truncated T-tune header")
        fmt = raw[0x1B]
        if fmt != 0x01:
            raise ValueError(f"{path}: unsupported T-tune format byte 0x{fmt:02x}")
        rate  = struct.unpack("<I", raw[0x1C:0x20])[0]
        nbits = struct.unpack("<I", raw[0x20:0x24])[0]
        nbytes = (nbits + 7) // 8
        if not (1000 <= rate <= 200000):
            raise ValueError(f"{path}: invalid T-tune frequency {rate} Hz")
        end = TTUNE_HEADER_SIZE + nbytes
        if end > len(raw):
            raise ValueError(f"{path}: T-tune datasize {nbits} bits exceeds file")
        return rate, raw[TTUNE_HEADER_SIZE:end]

    rate = struct.unpack("<I", raw[:4])[0]
    if not (1000 <= rate <= 200000):
        raise ValueError(f"{path}: not X1EMU or T-tune (first 4 bytes don't decode)")
    return rate, raw[4:]


def write_ttune_header(f, rate: int, name: str, protect: bool,
                       data_bits: int = 0, position_bits: int = 0) -> None:
    """Write a 40-byte T-tune header. Caller can patch datasize/position
    later by seeking to offset 0x20 / 0x24."""
    name_b = name.encode("ascii", errors="replace")[:17]
    name_b = name_b + b"\x00" * (17 - len(name_b))
    f.write(TTUNE_MAGIC)                                 # 0x00
    f.write(name_b)                                      # 0x04
    f.write(b"\x00" * 5)                                 # 0x15 reserve
    f.write(bytes([0x10 if protect else 0x00]))          # 0x1A protect
    f.write(bytes([0x01]))                               # 0x1B format
    f.write(struct.pack("<I", rate))                     # 0x1C frequency
    f.write(struct.pack("<I", data_bits))                # 0x20 datasize (bits)
    f.write(struct.pack("<I", position_bits))            # 0x24 position (bits)


def cmd_read(link: PicoLink, args: argparse.Namespace) -> int:
    out = Path(args.output)
    if out.exists() and not args.force:
        print(f"refusing to overwrite {out} (use --force)", file=sys.stderr)
        return 1

    max_sec = args.max * 60
    print(f"sampling at {args.rate} Hz, max {args.max} min, "
          f"idle-stop {args.idle}s, keep {args.keep}s, jitter 50/50%")

    # Match the original cmt_read_tape: send PLAY, give the transport
    # 200 ms to settle, then start sampling.
    link.send_line("CMD 02")
    resp = link.read_line()
    if not resp.startswith("STS"):
        print(f"PLAY failed: {resp}", file=sys.stderr)
        return 1
    time.sleep(0.2)

    link.send_line(f"READ {args.rate} {max_sec} {args.idle}")

    line = link.read_line()
    if not line.startswith("OK"):
        print(f"unexpected response: {line!r}", file=sys.stderr)
        return 1

    # Stream CHUNK frames straight to disk; loop ends when we see STATS.
    print("waiting for tape... (Ctrl-C to abort the host side)")
    total = 0
    stats_line = None
    with out.open("wb") as f:
        if args.format == "ttune":
            # datasize/position get patched after the trim step
            write_ttune_header(f, args.rate, args.name, args.protect)
        else:
            f.write(struct.pack("<I", args.rate))
        header_size = TTUNE_HEADER_SIZE if args.format == "ttune" else 4
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

    # If the run terminated via idle-detect, trim the trailing silence
    # so the .TAP file only retains `--keep` seconds of padding. Matches
    # the original X8RL1 behaviour (cmt_read_tape post-processing).
    trimmed = 0
    if stats_line and args.idle > args.keep:
        m = re.search(r"result=(-?\d+)", stats_line)
        if m and int(m.group(1)) == CZ8RL1_STS_AUTOSTOP:
            trim_bytes = (args.idle - args.keep) * args.rate // 8
            trim_bytes = min(trim_bytes, total)
            if trim_bytes > 0:
                with out.open("r+b") as f:
                    f.truncate(header_size + total - trim_bytes)
                trimmed = trim_bytes
                print(f"trimmed {trimmed} bytes of trailing silence "
                      f"(kept {args.keep}s padding)")

    # Patch the T-tune header's datasize field now that we know the
    # final byte count (post-trim).
    if args.format == "ttune":
        final_bytes = total - trimmed
        with out.open("r+b") as f:
            f.seek(0x20)
            f.write(struct.pack("<I", final_bytes * 8))

    # Mirror the original: send STOP after sampling completes.
    link.send_line("CMD 01")
    resp = link.read_line()
    if not resp.startswith("STS"):
        print(f"warning: STOP response: {resp}", file=sys.stderr)

    print(f"saved {total - trimmed} bytes to {out}")
    return 0


def cmd_write(link: PicoLink, args: argparse.Namespace) -> int:
    rate, data = read_tap(Path(args.input))
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


def cmd_pins(link: PicoLink, args: argparse.Namespace) -> int:
    link.send_line(f"PINS {args.duration}")
    print(link.read_line())
    return 0


def cmd_level(link: PicoLink, _args: argparse.Namespace) -> int:
    link.send_line("LEVEL")
    print(link.read_line())
    return 0


def cmd_poke(link: PicoLink, args: argparse.Namespace) -> int:
    link.send_line(f"POKE {args.pin} {args.level}")
    print(link.read_line())
    return 0


def cmd_raw(link: PicoLink, args: argparse.Namespace) -> int:
    link.send_line(args.line)
    print(link.read_line())
    return 0


def _format_pin_state(s: int) -> str:
    return f"BUSY={s & 1} STATUS={(s >> 1) & 1} RDATA={(s >> 2) & 1}"


def cmd_trace(link: PicoLink, args: argparse.Namespace) -> int:
    code = int(args.code, 16)
    name = CZ8RL1_CMD_NAMES.get(code, "?")
    print(f"tracing command {code:#04x} ({name}) for {args.duration} us")
    link.send_line(f"TRACE {code:02x} {args.duration}")

    header = link.read_line()
    if not header.startswith("TRACE "):
        print(f"unexpected: {header!r}", file=sys.stderr)
        return 1
    print(header)
    fields = dict(kv.split("=", 1) for kv in header.split()[1:])
    initial = int(fields.get("init", "0"))
    print(f"  initial: {_format_pin_state(initial)}")

    prev_state = initial
    while True:
        line = link.read_line()
        if line == "TRACE_END":
            break
        if not line.startswith("EVT "):
            print(f"unexpected: {line!r}", file=sys.stderr)
            continue
        _, t_us, state_s = line.split()
        state = int(state_s)
        changed = []
        for bit, label in ((0, "BUSY"), (1, "STATUS"), (2, "RDATA")):
            old = (prev_state >> bit) & 1
            new = (state >> bit) & 1
            if old != new:
                changed.append(f"{label}:{old}->{new}")
        print(f"  t={int(t_us):>8} us  {_format_pin_state(state)}  ({', '.join(changed)})")
        prev_state = state
    return 0


def cmd_ping(link: PicoLink, _args: argparse.Namespace) -> int:
    link.send_line("PING")
    line = link.read_line()
    print(line)
    if "x8rl1-pico" in line:
        return 0
    print("warning: device did not identify as x8rl1-pico", file=sys.stderr)
    return 1


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", help="serial device for the Pico "
                                  "(auto-detected by USB VID:PID if omitted)")
    p.add_argument("--baud", type=int, default=115200)

    sub = p.add_subparsers(dest="cmd", required=True)

    pr = sub.add_parser("read", help="record tape into a .TAP file")
    pr.add_argument("output")
    pr.add_argument("--rate", type=int, default=8000)
    pr.add_argument("--max",  type=int, default=60, help="max recording time (minutes)")
    pr.add_argument("--idle", type=int, default=15, help="idle-stop seconds")
    pr.add_argument("--keep", type=int, default=3,
                    help="seconds of trailing silence to keep after idle-stop")
    pr.add_argument("--format", choices=["x1emu", "ttune"], default="ttune",
                    help="output .TAP variant (default ttune)")
    pr.add_argument("--name", default="x8rl1-pico",
                    help="tape name embedded in T-tune header (max 16 chars)")
    pr.add_argument("--protect", action="store_true",
                    help="set the write-protect flag in T-tune header")
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

    pp = sub.add_parser("ping", help="check that the device is x8rl1-pico")
    pp.set_defaults(func=cmd_ping)

    ppins = sub.add_parser("pins", help="sample input pins for N ms and report HIGH counts")
    ppins.add_argument("--duration", type=int, default=10,
                       help="sample window in ms (1..1000, default 10)")
    ppins.set_defaults(func=cmd_pins)

    plvl = sub.add_parser("level", help="instantaneous read of all input pins")
    plvl.set_defaults(func=cmd_level)

    ppoke = sub.add_parser("poke", help="drive an output pin to a level (for multimeter probing)")
    ppoke.add_argument("pin", choices=["wdata", "strobe", "cmd"])
    ppoke.add_argument("level", type=int, choices=[0, 1])
    ppoke.set_defaults(func=cmd_poke)

    praw = sub.add_parser("raw", help="send a raw line to the Pico and print the reply")
    praw.add_argument("line", help="exact text to send (e.g. \"CMD 80\")")
    praw.set_defaults(func=cmd_raw)

    ptrace = sub.add_parser("trace",
        help="send a CZ8RL1 command and capture every BUSY/STATUS/RDATA edge in firmware")
    ptrace.add_argument("code", help="command byte in hex (e.g. 80 = STATUS, 01 = STOP)")
    ptrace.add_argument("--duration", type=int, default=100000,
                        help="trace window in microseconds (1000..2000000, default 100000)")
    ptrace.set_defaults(func=cmd_trace)

    args = p.parse_args()
    if args.port is None:
        args.port = find_pico_port()
        if args.port is None:
            print("could not auto-detect a Pico on USB; pass --port",
                  file=sys.stderr)
            return 1
        print(f"using auto-detected Pico at {args.port}")
    link = PicoLink(args.port, args.baud)
    pong = link.wait_for_ready()
    if "x8rl1-pico" not in pong:
        print(f"warning: device at {args.port} did not identify as x8rl1-pico "
              f"(got {pong!r})", file=sys.stderr)
    return args.func(link, args)


if __name__ == "__main__":
    sys.exit(main())
