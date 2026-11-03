import threading
import time

import numpy as np

from ultraskelly.core.pubsub.bot_topics import FrameMessage


class CountingEncoder:
    def __init__(self, delay=0.0):
        self.calls, self.delay = [], delay

    def __call__(self, image, quality):
        self.calls.append(quality)
        time.sleep(self.delay)  # Long enough for a second consumer to arrive meanwhile
        return b"JPEG@%d" % quality


def message(**fields):
    return FrameMessage(frame=np.zeros((4, 6, 3), dtype=np.uint8), **fields)


def test_a_frame_is_encoded_once_per_quality_however_many_consumers_ask():
    frame, encode = message(), CountingEncoder()
    assert frame.jpeg_bytes(80, encode) == frame.jpeg_bytes(80, encode) == b"JPEG@80"
    assert frame.jpeg_bytes(95, encode) == b"JPEG@95"
    assert encode.calls == [80, 95]


def test_consumers_on_two_threads_share_one_encoding():
    frame, encode = message(), CountingEncoder(delay=0.05)  # The stream's worker and the recorder's writer
    results = []
    threads = [threading.Thread(target=lambda: results.append(frame.jpeg_bytes(80, encode))) for _ in range(2)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert results == [b"JPEG@80", b"JPEG@80"] and encode.calls == [80]


def test_a_cameras_own_jpeg_is_used_as_it_came():
    frame, encode = message(jpeg=b"from the camera"), CountingEncoder()
    assert frame.jpeg_bytes(80, encode) == b"from the camera" and encode.calls == []
