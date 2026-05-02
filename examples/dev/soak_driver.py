#!/usr/bin/env python3
"""TCP echo soak driver for `embassy_tcp_soak.rs`.

Opens four parallel TCP streams to the device, sends a deterministic
pseudo-random byte stream on each, reads the echo back, and verifies
byte-exact match. Reports per-stream MB throughput, total bytes echoed,
and any verification mismatches every 30 seconds.

Run on the host while the chip is flashed with `embassy_tcp_soak.elf`:

    python3 soak_driver.py --host 192.168.0.50 --duration 3600

Killable with Ctrl+C; partial stats are printed on exit.
"""

import argparse
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass, field

DEFAULT_HOST = "192.168.0.50"
DEFAULT_PORTS = [7780, 7781, 7782, 7783]
CHUNK_SIZE = 4096
DEFAULT_DURATION = 3600
SEND_TIMEOUT = 30.0


def stream_bytes(seed: int, length: int) -> bytes:
    """Deterministic pseudo-random byte stream — xorshift32 seeded by `seed`."""
    state = seed if seed != 0 else 0xDEADBEEF
    out = bytearray(length)
    pos = 0
    while pos < length:
        # xorshift32
        state ^= (state << 13) & 0xFFFFFFFF
        state ^= (state >> 17)
        state ^= (state << 5) & 0xFFFFFFFF
        state &= 0xFFFFFFFF
        chunk = struct.pack("<I", state)
        take = min(4, length - pos)
        out[pos : pos + take] = chunk[:take]
        pos += take
    return bytes(out)


@dataclass
class StreamStats:
    port: int
    bytes_sent: int = 0
    bytes_received: int = 0
    mismatches: int = 0
    errors: int = 0
    last_error: str = ""
    reconnects: int = 0


def stream_worker(host: str, port: int, stop_event: threading.Event,
                  stats: StreamStats) -> None:
    """Send a continuous pseudo-random stream and verify the echo."""
    seed_base = port * 0x9E3779B1  # different seed per port
    while not stop_event.is_set():
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(SEND_TIMEOUT)
                s.connect((host, port))
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                # Pre-generate 1 MB of pseudo-random bytes; cycle through it.
                pattern = stream_bytes(seed_base, 1024 * 1024)
                pat_len = len(pattern)
                pat_pos = 0
                while not stop_event.is_set():
                    chunk = pattern[pat_pos : pat_pos + CHUNK_SIZE]
                    if len(chunk) < CHUNK_SIZE:
                        chunk += pattern[: CHUNK_SIZE - len(chunk)]
                    pat_pos = (pat_pos + CHUNK_SIZE) % pat_len

                    s.sendall(chunk)
                    stats.bytes_sent += CHUNK_SIZE

                    # Read exactly len(chunk) bytes back, verify byte-exact.
                    received = bytearray()
                    while len(received) < CHUNK_SIZE:
                        part = s.recv(CHUNK_SIZE - len(received))
                        if not part:
                            raise ConnectionResetError("peer closed during recv")
                        received.extend(part)
                    stats.bytes_received += CHUNK_SIZE

                    if bytes(received) != chunk:
                        stats.mismatches += 1
                        # Find the first diverging byte for the error log.
                        for i, (a, b) in enumerate(zip(chunk, received)):
                            if a != b:
                                stats.last_error = (
                                    f"mismatch at offset {i}: "
                                    f"sent=0x{a:02x} got=0x{b:02x}"
                                )
                                break
        except Exception as e:
            stats.errors += 1
            stats.last_error = f"{type(e).__name__}: {e}"
            stats.reconnects += 1
            time.sleep(1.0)


def fmt_bytes(n: int) -> str:
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024:
            return f"{n:.1f} {unit}"
        n /= 1024
    return f"{n:.1f} TB"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default=DEFAULT_HOST,
                        help=f"chip IP (default: {DEFAULT_HOST})")
    parser.add_argument("--ports", nargs="+", type=int, default=DEFAULT_PORTS,
                        help=f"listener ports (default: {DEFAULT_PORTS})")
    parser.add_argument("--duration", type=int, default=DEFAULT_DURATION,
                        help=f"run for N seconds, then stop (default: {DEFAULT_DURATION})")
    parser.add_argument("--report-period", type=int, default=30,
                        help="stats print interval in seconds (default: 30)")
    args = parser.parse_args()

    stop = threading.Event()
    stats = [StreamStats(port=p) for p in args.ports]
    threads = []
    for s in stats:
        t = threading.Thread(target=stream_worker,
                             args=(args.host, s.port, stop, s),
                             daemon=True, name=f"port-{s.port}")
        t.start()
        threads.append(t)

    print(f"[soak] driving {len(stats)} stream(s) to {args.host} for {args.duration}s")
    start = time.monotonic()
    last_print = start
    last_bytes = [0] * len(stats)

    try:
        while time.monotonic() - start < args.duration:
            time.sleep(0.5)
            now = time.monotonic()
            if now - last_print >= args.report_period:
                elapsed = now - start
                print(f"\n[t={elapsed:>6.0f}s]")
                for i, s in enumerate(stats):
                    bytes_now = s.bytes_received
                    delta = bytes_now - last_bytes[i]
                    rate = delta / (now - last_print) / 1024 / 1024
                    last_bytes[i] = bytes_now
                    print(f"  port {s.port}: rx={fmt_bytes(s.bytes_received)} "
                          f"({rate:.2f} MB/s)  "
                          f"mismatch={s.mismatches}  err={s.errors}  "
                          f"reconns={s.reconnects}")
                    if s.last_error and (s.errors or s.mismatches):
                        print(f"            last: {s.last_error}")
                last_print = now
    except KeyboardInterrupt:
        print("\n[soak] Ctrl+C received, stopping ...")

    stop.set()
    for t in threads:
        t.join(timeout=5.0)

    print("\n[soak] FINAL STATS:")
    total_rx = 0
    total_mismatch = 0
    for s in stats:
        total_rx += s.bytes_received
        total_mismatch += s.mismatches
        print(f"  port {s.port}: sent={fmt_bytes(s.bytes_sent)} "
              f"recv={fmt_bytes(s.bytes_received)}  "
              f"mismatches={s.mismatches}  errors={s.errors}  "
              f"reconnects={s.reconnects}")
    elapsed = time.monotonic() - start
    print(f"\nTotal RX: {fmt_bytes(total_rx)} in {elapsed:.0f}s "
          f"({total_rx / elapsed / 1024 / 1024:.2f} MB/s avg)")
    print(f"Mismatches: {total_mismatch}")

    return 0 if total_mismatch == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
