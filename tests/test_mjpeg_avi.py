import struct

import numpy as np
import pytest

from ultraskelly.core.bot.recording import mjpeg_avi
from ultraskelly.core.bot.recording.mjpeg_avi import MjpegAviWriter


def chunks(data: bytes, start: int, end: int) -> list[tuple[bytes, int, int]]:
    """Each chunk between `start` and `end`: its tag, where its data starts, and its length."""
    found = []
    while start < end:
        tag, length = data[start:start + 4], struct.unpack_from("<I", data, start + 4)[0]
        found.append((tag, start + 8, length))
        start += 8 + length + length % 2
    return found


def test_frames_are_stored_as_they_came_with_an_index_and_complete_headers(tmp_path):
    jpegs = [b"\xff\xd8one\xff\xd9", b"\xff\xd8second\xff\xd9", b"\xff\xd8 3\xff\xd9"]  # Odd and even lengths
    path = tmp_path / "oak.avi"
    writer = MjpegAviWriter(path, (640, 480), 30.0)
    for jpeg in jpegs:
        writer.write(jpeg)
    writer.release()
    data = path.read_bytes()

    assert data[:4] == b"RIFF" and data[8:12] == b"AVI " and struct.unpack_from("<I", data, 4)[0] == len(data) - 8
    hdrl, movi, idx1 = chunks(data, 12, len(data))
    assert (hdrl[0], data[hdrl[1]:hdrl[1] + 4], movi[0], data[movi[1]:movi[1] + 4], idx1[0]) == (
        b"LIST", b"hdrl", b"LIST", b"movi", b"idx1",
    )
    (avih, avih_at, _), (_, strl_at, strl_length) = chunks(data, hdrl[1] + 4, hdrl[1] + hdrl[2])
    microseconds, _, _, flags, frames = struct.unpack_from("<5I", data, avih_at)
    assert avih == b"avih" and (microseconds, frames, flags & 0x10) == (33333, 3, 0x10)
    assert struct.unpack_from("<2I", data, avih_at + 32) == (640, 480)
    (strh, strh_at, _), (strf, strf_at, _) = chunks(data, strl_at + 4, strl_at + strl_length)
    assert struct.unpack_from("<4s4s", data, strh_at) == (b"vids", b"MJPG")
    assert struct.unpack_from("<3I", data, strh_at + 20) == (1, 30, 0)  # Scale and rate: 30 fps; start
    assert struct.unpack_from("<I", data, strh_at + 32)[0] == 3  # Length in frames
    assert struct.unpack_from("<4s", data, strf_at + 16)[0] == b"MJPG"

    stored = chunks(data, movi[1] + 4, movi[1] + movi[2])
    assert [data[at:at + length] for _, at, length in stored] == jpegs
    for number, jpeg in enumerate(jpegs):
        tag, flags, offset, length = struct.unpack_from("<4sIII", data, idx1[1] + 16 * number)
        assert (tag, flags, length) == (b"00dc", 0x10, len(jpeg))
        assert data[movi[1] + offset:movi[1] + offset + 4] == b"00dc"  # Offsets count from the 'movi' tag
        assert data[movi[1] + offset + 8:movi[1] + offset + 8 + length] == jpeg


def test_a_frame_that_would_pass_the_avi_size_limit_is_refused(tmp_path, monkeypatch):
    monkeypatch.setattr(mjpeg_avi, "MAX_RIFF_SIZE", 400)
    path = tmp_path / "oak.avi"
    writer = MjpegAviWriter(path, (64, 48), 30.0)
    writer.write(b"x" * 100)
    with pytest.raises(OSError, match="4 GiB"):
        writer.write(b"x" * 200)
    writer.release()  # Still a valid file with the frame that fitted
    assert struct.unpack_from("<4sIII", path.read_bytes(), len(path.read_bytes()) - 16)[3] == 100


def test_opencv_reads_the_recorded_frames_back(tmp_path):
    cv2 = pytest.importorskip("cv2", reason="OpenCV comes with the hardware extra")
    images = [np.full((48, 64, 3), value, dtype=np.uint8) for value in (20, 120, 220)]
    path = tmp_path / "oak.avi"
    writer = MjpegAviWriter(path, (64, 48), 30.0)
    for image in images:
        writer.write(cv2.imencode(".jpg", image)[1].tobytes())
    writer.release()

    video = cv2.VideoCapture(str(path))
    read = []
    while (frame := video.read())[0]:
        read.append(frame[1])
    video.release()
    assert len(read) == 3 and read[0].shape == (48, 64, 3)
    assert [float(image.mean()) for image in read] == pytest.approx([20, 120, 220], abs=3)
