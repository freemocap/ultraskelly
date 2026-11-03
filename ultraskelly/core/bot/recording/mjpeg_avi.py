"""A Motion JPEG AVI written from JPEG bytes, without decoding or re-encoding them.

The recorder gets each camera frame as JPEG already: the stream encodes it once, or the
camera does (an OAK's hardware encoder). This stores those bytes in an ordinary AVI 1.0
file with an idx1 index, which OpenCV, FFmpeg and media players read. AVI 1.0 counts
sizes in 32 bits, so a frame that would take the file past 4 GiB, roughly an hour and a
half of 640x480 video, raises instead of being written.
"""
import math
import struct
from pathlib import Path

AVIF_HASINDEX = 0x10
AVIIF_KEYFRAME = 0x10
MAX_RIFF_SIZE = 0xFFFF_FFFF


def _chunk(tag: bytes, data: bytes) -> bytes:
    return tag + struct.pack("<I", len(data)) + data + (b"\0" if len(data) % 2 else b"")


def _list(kind: bytes, content: bytes) -> bytes:
    return b"LIST" + struct.pack("<I", 4 + len(content)) + kind + content


class MjpegAviWriter:
    """Writes JPEG frames of one size to an AVI file; `release()` completes its headers and index."""

    def __init__(self, path: Path, size: tuple[int, int], fps: float) -> None:
        self.size, self.fps = size, fps
        self.index: list[tuple[int, int]] = []  # Each frame chunk's offset from the 'movi' tag, and its length
        self.largest = 0
        self.file = open(path, "wb")
        self.file.write(self._headers(frames=0, riff_size=0, movi_size=0))  # Rewritten by release()
        self.movi_tag = self.file.tell() - 4

    def write(self, jpeg: bytes) -> None:
        """Append one frame's JPEG as it is."""
        position = self.file.tell()
        chunk = _chunk(b"00dc", jpeg)
        index_size = 8 + 16 * (len(self.index) + 1)
        if position + len(chunk) + index_size - 8 > MAX_RIFF_SIZE:
            raise OSError(f"{self.file.name} reached the AVI size limit of 4 GiB after {len(self.index)} frames")
        self.file.write(chunk)
        self.index.append((position - self.movi_tag, len(jpeg)))
        self.largest = max(self.largest, len(jpeg))

    def release(self) -> None:
        """Write the index and the final headers, and close the file."""
        if self.file.closed:
            return
        movi_end = self.file.tell()
        entries = b"".join(
            b"00dc" + struct.pack("<III", AVIIF_KEYFRAME, offset, length) for offset, length in self.index
        )
        self.file.write(_chunk(b"idx1", entries))
        end = self.file.tell()
        self.file.seek(0)
        self.file.write(self._headers(frames=len(self.index), riff_size=end - 8, movi_size=movi_end - self.movi_tag))
        self.file.close()

    def _headers(self, *, frames: int, riff_size: int, movi_size: int) -> bytes:
        """RIFF and hdrl, then the 'movi' list's header; the same length whatever the counts."""
        width, height = self.size
        scale = 1000
        rate = round(self.fps * scale)
        common = math.gcd(scale, rate) or 1
        main_header = struct.pack(
            "<14I",
            round(1e6 / self.fps),  # Microseconds per frame
            math.ceil(self.largest * self.fps),  # Most bytes a second, roughly
            0, AVIF_HASINDEX, frames, 0, 1, self.largest, width, height, 0, 0, 0, 0,
        )
        stream_header = struct.pack(
            "<4s4sIHHIIIIIIII4h",
            b"vids", b"MJPG", 0, 0, 0, 0, scale // common, rate // common, 0, frames, self.largest,
            0xFFFF_FFFF, 0, 0, 0, width, height,
        )
        bitmap_header = struct.pack("<IiiHH4sIiiII", 40, width, height, 1, 24, b"MJPG", width * height * 3, 0, 0, 0, 0)
        stream_list = _list(b"strl", _chunk(b"strh", stream_header) + _chunk(b"strf", bitmap_header))
        header_list = _list(b"hdrl", _chunk(b"avih", main_header) + stream_list)
        return (b"RIFF" + struct.pack("<I", riff_size) + b"AVI " + header_list
                + b"LIST" + struct.pack("<I", movi_size) + b"movi")
