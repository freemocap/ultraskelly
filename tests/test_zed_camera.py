import asyncio
import queue
import subprocess
import sys

import numpy as np
import pytest

from ultraskelly.core.bot.sensory.zed_camera_node import (
    FramePeriodCounter,
    ZedCameraNode,
    ZedCameraNodeParams,
    ZedResolution,
    find_zed_device,
    has_lost_rows,
)
from ultraskelly.core.pubsub.bot_topics import StereoFrameMessage, StereoFrameTopic
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

HEIGHT, EYE_WIDTH = 8, 12


def yuyv_pair(left_luma: int, right_luma: int) -> np.ndarray:
    """A side-by-side YUYV frame whose eyes have uniform brightness and neutral color."""
    frame = np.full((HEIGHT, 2 * EYE_WIDTH, 2), 128, np.uint8)
    frame[:, :EYE_WIDTH, 0], frame[:, EYE_WIDTH:, 0] = left_luma, right_luma
    return frame


class FakeSource:
    """Hands the node whatever the test queues: (frame, time), or None for a camera that stopped."""

    def __init__(self) -> None:
        self.frames: queue.Queue = queue.Queue()
        self.closed = False

    def read(self):
        return self.frames.get(timeout=5)

    def close(self) -> None:
        self.closed = True


def test_params_offer_only_the_cameras_modes():
    assert ZedCameraNodeParams().resolution is ZedResolution.VGA
    assert ZedCameraNodeParams(resolution="VGA", fps=100).fps == 100
    with pytest.raises(ValueError, match="HD720 runs at 60, 30, 15 fps, not 100"):
        ZedCameraNodeParams(resolution=ZedResolution.HD720, fps=100)


def test_the_zed_is_found_by_its_capture_nodes_name(tmp_path):
    def video(number: int, name: str, index: int) -> None:
        node = tmp_path / f"video{number}"
        node.mkdir()
        (node / "name").write_text(name + "\n")
        (node / "index").write_text(f"{index}\n")

    assert find_zed_device(tmp_path) is None
    video(2, "pispbe", 0)
    video(10, "ZED-M: ZED-M", 0)
    video(11, "ZED-M: ZED-M", 1)  # Its metadata node
    assert find_zed_device(tmp_path) == "/dev/video10"


def test_rows_lost_in_transfer_are_detected_but_black_is_not():
    limited_black = np.zeros((40, 64, 2), np.uint8)
    limited_black[..., 0], limited_black[..., 1] = 16, 128
    full_range_black = limited_black.copy()
    full_range_black[..., 0] = 0
    lost = limited_black.copy()
    lost[20:28] = 0

    assert not has_lost_rows(limited_black)
    assert not has_lost_rows(full_range_black)
    assert has_lost_rows(lost)


def test_frame_numbers_count_camera_periods_so_drops_skip_numbers():
    counter = FramePeriodCounter(30)
    times = [10.0, 10.034, 10.0665, 10.1333, 10.168]  # One frame missing after the third
    assert [counter.advance(t) for t in times] == [0, 1, 2, 4, 5]


def test_node_publishes_eye_pairs_drops_lost_frames_and_reopens_a_stopped_camera():
    async def scenario():
        bus = PubSubTopicManager.create()
        sources = [FakeSource(), FakeSource()]
        opened = iter(sources[1:])
        node = ZedCameraNode(
            pubsub=bus, params=ZedCameraNodeParams(), open_source=lambda: next(opened),
        )
        node.source = sources[0]
        pairs = bus.get_subscription(StereoFrameTopic)
        running = asyncio.create_task(node.run())

        sources[0].frames.put((yuyv_pair(40, 200), 5.0))
        first = await asyncio.wait_for(pairs.get(), timeout=5)
        lost = yuyv_pair(40, 200)
        lost[2:6] = 0
        sources[0].frames.put((lost, 5.0 + 1 / 30))
        sources[0].frames.put((yuyv_pair(41, 201), 5.0 + 2 / 30))
        second = await asyncio.wait_for(pairs.get(), timeout=5)
        sources[0].frames.put(None)  # The camera stops; the node closes it and opens it again
        sources[1].frames.put((yuyv_pair(42, 202), 9.0))
        third = await asyncio.wait_for(pairs.get(), timeout=5)

        node.stop()
        sources[1].frames.put((yuyv_pair(43, 203), 9.1))  # Unblocks the read in progress
        await asyncio.wait_for(running, timeout=5)
        bus.close()
        return node, sources, first, second, third

    node, sources, first, second, third = asyncio.run(scenario())

    assert first.yuyv.shape == (HEIGHT, 2 * EYE_WIDTH, 2) and first.eye_size == (EYE_WIDTH, HEIGHT)
    assert (first.grey(0)[0, 0], first.grey(1)[0, 0]) == (40, 200)  # Left eye is the frame's left half
    assert (first.frame_number, first.timestamp) == (0, 5.0)
    assert (second.grey(0)[0, 0], second.frame_number) == (41, 2)  # The lost frame kept its number
    assert (third.grey(1)[0, 0], third.timestamp) == (202, 9.0)
    assert node.frames_lost == 1 and node.frames_published == 3
    assert sources[0].closed and sources[1].closed and node.source is None


def test_a_pair_hands_out_each_eye_as_it_came_and_its_grey_without_converting():
    frame = yuyv_pair(40, 200)
    frame[:, EYE_WIDTH + 1, 1] = 99  # A chroma byte of the right eye
    pair = StereoFrameMessage(yuyv=frame)

    assert pair.eye(1).base is frame or np.shares_memory(pair.eye(1), frame)  # A view
    assert pair.eye(1)[0, 1, 1] == 99 and pair.eye(0).shape == (HEIGHT, EYE_WIDTH, 2)
    grey = pair.grey(1)
    assert grey.shape == (HEIGHT, EYE_WIDTH) and grey.flags.c_contiguous and np.all(grey == 200)


def test_importing_the_zed_node_loads_no_camera_library():
    code = """
import sys
import ultraskelly.core.bot.sensory.zed_camera_node
assert 'cv2' not in sys.modules
"""
    subprocess.run([sys.executable, "-c", code], check=True, timeout=10)


def test_each_eye_is_converted_once_straight_from_yuyv():
    cv2 = pytest.importorskip("cv2")
    from ultraskelly.core.bot.sensory.zed_camera_node import yuyv_to_rgb_with_opencv
    from ultraskelly.ui.runtime_server import encode_jpeg_with_opencv

    reddish = np.zeros((16, 2 * 32, 2), np.uint8)  # Y 120, U 100 (less blue), V 200 (more red)
    reddish[..., 0], reddish[:, 0::2, 1], reddish[:, 1::2, 1] = 120, 100, 200
    pair = StereoFrameMessage(yuyv=reddish)
    rgb = yuyv_to_rgb_with_opencv(pair.eye(0))
    decoded = cv2.imdecode(np.frombuffer(encode_jpeg_with_opencv(pair.eye(1), 95), np.uint8), cv2.IMREAD_COLOR)

    assert rgb.shape == (16, 32, 3) and rgb[0, 0, 0] > rgb[0, 0, 1] > rgb[0, 0, 2]  # Red, then green, then blue
    assert decoded.shape == (16, 32, 3)
    assert np.abs(decoded[8, 16].astype(int) - rgb[8, 16, ::-1]).max() <= 4  # The JPEG's BGR matches the RGB
