"""The ZED Mini IMU's reader process: its hidraw reports to stdout, with the time of each read.

`ZedImuNode` runs it as `python -m ultraskelly.core.bot.sensory.zed_imu_reader <device>`. In a
process of its own it is never kept waiting by the bot's other threads: as a thread of the
busy bot it lost 9% of the samples, whenever it was held off longer than the 80 ms of
reports the kernel keeps. Each read writes a header, the time.monotonic() it was made and
how many reports follow, then the 64-byte reports. Standard library only, so it starts fast.
The report format and the start command are those of Stereolabs' zed-open-capture.
"""
import os
import select
import signal
import struct
import sys
import time
from pathlib import Path
from typing import BinaryIO, Callable, Protocol

REPORT_SIZE = 64
STREAM_STATUS = 0x32  # Feature report: 1 starts the IMU's reports, 0 stops them
HEADER = struct.Struct("<dI")  # A read's time on time.monotonic()'s clock, and how many reports follow
READ_SECONDS = 0.02  # Between reads: each takes every report waiting (the kernel keeps 64, 80 ms)
QUIET_SECONDS = 1.0  # Without a report for this long, the stream is started again
REPORT_SECONDS = 30.0  # At most one complaint this often


def hid_set_feature(length: int) -> int:
    """Linux's HIDIOCSFEATURE(length) ioctl number: _IOC(read | write, 'H', 0x06, length)."""
    return (3 << 30) | (length << 16) | (ord("H") << 8) | 0x06


class ImuDevice(Protocol):
    def set_streaming(self, on: bool) -> None: ...

    def read(self, timeout: float) -> tuple[bytes, float]: ...

    def close(self) -> None: ...


class HidrawImu:
    """The IMU's HID interface through Linux hidraw: reads and one ioctl, no library."""

    def __init__(self, fd: int) -> None:
        self.fd = fd

    @classmethod
    def open(cls, path: Path) -> "HidrawImu":
        return cls(os.open(path, os.O_RDWR | os.O_NONBLOCK))

    def set_streaming(self, on: bool) -> None:
        """Start or stop the reports (zed-open-capture's REP_ID_SENSOR_STREAM_STATUS)."""
        import fcntl  # Linux only, so imported here: importing this module opens nothing

        report = bytearray([STREAM_STATUS, 1 if on else 0])
        fcntl.ioctl(self.fd, hid_set_feature(len(report)), report)

    def read(self, timeout: float) -> tuple[bytes, float]:
        """Every report waiting, after up to `timeout` seconds for the first, and the time they were read."""
        reports = []
        if select.select([self.fd], [], [], timeout)[0]:
            while True:
                try:
                    report = os.read(self.fd, REPORT_SIZE)
                except BlockingIOError:
                    break
                if len(report) != REPORT_SIZE:
                    break
                reports.append(report)
        return b"".join(reports), time.monotonic()

    def close(self) -> None:
        os.close(self.fd)


def stream(device: ImuDevice, out: BinaryIO, stopping: Callable[[], bool]) -> None:
    """Start the IMU's reports and write each read to `out` until `stopping()`."""
    device.set_streaming(True)
    heard_at, complained_at = time.monotonic(), float("-inf")
    while not stopping():
        data, read_at = device.read(timeout=0.25)
        if data:
            heard_at = read_at
            out.write(HEADER.pack(read_at, len(data) // REPORT_SIZE) + data)
            out.flush()
        elif read_at - heard_at > QUIET_SECONDS:
            if read_at - complained_at >= REPORT_SECONDS:
                print("The ZED's IMU went quiet; starting its stream again", file=sys.stderr, flush=True)
                complained_at = read_at
            device.set_streaming(True)
            heard_at = read_at
        time.sleep(READ_SECONDS)


def main(path: str) -> int:
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))  # So the stream is stopped on the way out
    device = HidrawImu.open(Path(path))
    try:
        stream(device, sys.stdout.buffer, lambda: False)
    except (BrokenPipeError, KeyboardInterrupt):
        pass  # The bot has gone
    finally:
        try:
            device.set_streaming(False)
        except OSError:
            pass  # Unplugged already
        device.close()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
