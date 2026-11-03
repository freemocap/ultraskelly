import asyncio
import struct

import cbor2
import numpy as np
import pytest
from aiohttp.test_utils import TestClient, TestServer

from ultraskelly.core.bot.sensory.stereo_depth_node import StereoDepthNode, StereoDepthNodeParams, depth_millimetres
from ultraskelly.core.bot.sensory.stereo_rectification import StereoRectification
from ultraskelly.core.pubsub.bot_topics import (
    DepthFrameMessage,
    DepthFrameTopic,
    StereoFrameMessage,
    StereoFrameTopic,
)
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager
from ultraskelly.ui.runtime_server import RuntimeServerNode, RuntimeServerParams

SHARED = StereoRectification(width=8, height=6, focal_length=340.0, cx=4.0, cy=3.0, baseline=0.063,
                             left_rotation=((1, 0, 0), (0, 1, 0), (0, 0, 1)))
ONE_METRE = 340.0 * 0.063  # Disparity of a point 1 m away, pixels


class ConstantMatcher:
    name = "constant"
    search_range = 64
    color = False

    def __init__(self, disparity: float) -> None:
        self.value = disparity
        self.closed = False

    def disparity(self, left, right):
        assert left.ndim == right.ndim == 2  # Matched in grey
        return np.full(left.shape, self.value, np.float32)

    def close(self):
        self.closed = True


class Rectifier:
    rectification = SHARED

    def __init__(self) -> None:
        self.calls = 0

    def rectify_eye(self, eye, image):
        self.calls += 1
        return image


def pair(value: int, *, rectified: bool = False) -> StereoFrameMessage:
    image = np.full((6, 8, 3), value, np.uint8)
    return StereoFrameMessage(left=image, right=image.copy(), frame_number=12, timestamp=3.5,
                              rectification=SHARED if rectified else None)


def test_depth_is_whole_millimetres_and_zero_where_unknown_or_out_of_range():
    disparity = np.array([[ONE_METRE, ONE_METRE / 2, 0.0, -1.0, ONE_METRE * 10, ONE_METRE / 20]], np.float32)
    depth = depth_millimetres(disparity, SHARED, min_depth=0.3, max_depth=8.0)

    assert depth.dtype == np.uint16
    assert depth.tolist() == [[1000, 2000, 0, 0, 0, 0]]  # 0.1 m is too near, 20 m too far
    assert depth_millimetres(np.array([0.1]), SHARED, 0.3, 1000.0).tolist() == [65535]  # Saturates, never wraps
    # Matches within 4 px of the end of a 64-pixel search are dropped
    assert depth_millimetres(np.array([59.9, 60.0, 63.5]), SHARED, 0.3, 8.0, search_range=64).tolist() == [358, 0, 0]


def test_params_need_a_multiple_of_16_disparities_and_an_odd_window():
    with pytest.raises(ValueError, match="multiple of 16"):
        StereoDepthNodeParams(num_disparities=70)
    with pytest.raises(ValueError, match="odd"):
        StereoDepthNodeParams(block_size=6)


def run_node(pairs: list[StereoFrameMessage], rectifier: Rectifier) -> list[DepthFrameMessage]:
    async def scenario():
        bus = PubSubTopicManager.create()
        node = StereoDepthNode(
            pubsub=bus, params=StereoDepthNodeParams(max_hz=30), rectifier=rectifier,
            matcher=ConstantMatcher(ONE_METRE / 2), to_grey=lambda image: image[..., 0],
            pairs=bus.get_subscription(StereoFrameTopic),
        )
        depths = bus.get_subscription(DepthFrameTopic)
        running = asyncio.create_task(node.run())
        results = []
        for stereo_pair in pairs:
            await bus.publish(StereoFrameTopic, stereo_pair)
            results.append(await asyncio.wait_for(depths.get(), timeout=5))
        node.stop()
        await bus.publish(StereoFrameTopic, pairs[-1])  # Lets the loop see the stop
        await asyncio.wait_for(running, timeout=5)
        bus.close()
        return results

    return asyncio.run(scenario())


def test_the_node_turns_each_pair_into_depth_aligned_with_its_rectified_left_image():
    rectifier = Rectifier()
    raw, already_rectified = run_node([pair(7), pair(9, rectified=True)], rectifier)

    assert raw.depth.shape == (6, 8) and int(raw.depth[0, 0]) == 2000
    assert raw.color[0, 0, 0] == 7 and raw.rectification is SHARED and raw.method == "constant"
    assert (raw.frame_number, raw.timestamp) == (12, 3.5)
    assert already_rectified.color[0, 0, 0] == 9
    assert rectifier.calls == 2  # Both eyes of the raw pair; the rectified pair is used as it is


def test_a_depth_frame_streams_as_its_rectified_image_and_its_depth():
    encoded = []

    def fake_encode(image, quality):
        encoded.append((image.dtype.name, image.shape))
        return b"JPEG"

    async def scenario():
        bus = PubSubTopicManager.create()
        server = RuntimeServerNode.create(
            pubsub=bus, params=RuntimeServerParams(state_hz=50), cameras={}, recorder=None,
            motors_simulated=True, node_status=lambda: [], encode_jpeg=fake_encode,
            stereo_cameras={"zed": StereoFrameTopic}, depth_cameras={"zed": DepthFrameTopic},
        )
        pump = asyncio.create_task(server.pump())
        async with TestClient(TestServer(server.build_app())) as client:
            socket = await client.ws_connect("/stream")
            while server.clients == 0:
                await asyncio.sleep(0.005)
            depth = np.full((6, 8), 1500, np.uint16)
            await bus.publish(DepthFrameTopic, DepthFrameMessage(
                depth=depth, color=np.zeros((6, 8, 3), np.uint8), rectification=SHARED, frame_number=4, timestamp=1.0,
            ))
            received = {}
            while len(received) < 2:
                message = cbor2.loads((await asyncio.wait_for(socket.receive(), timeout=5)).data)
                if message["kind"] in ("frame", "depth_frame"):
                    received[message["kind"]] = message
            await socket.close()
        server.stop()
        await pump
        bus.close()
        return received

    received = asyncio.run(scenario())

    frame = received["frame"]
    assert [(camera["id"], camera["index"]) for camera in frame["cameras"]] == [("zed_rectified", 2), ("zed_depth", 3)]
    assert struct.unpack_from("<i", frame["image"], 16)[0] == 2  # Two images in the payload
    assert encoded[:2] == [("uint8", (6, 8, 3)), ("uint16", (6, 8))]  # The JPEG encoder colours the depth
    depth = received["depth_frame"]
    assert (depth["frame_number"], depth["camera"]["fx"], depth["depth"]["step"]) == (4, 340.0, 2)
    assert depth["color"]["data"] == b"JPEG" and depth["camera_to_head"]["status"] == "nominal"
