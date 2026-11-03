"""What a viewer shows (docs/stream.md, "What a viewer draws"): the server encodes only the
feeds someone shows, sends each viewer only its own views, and robot_state always."""
import asyncio
import json
import struct

import cbor2
import numpy as np
import pytest
from aiohttp import WSMsgType
from aiohttp.test_utils import TestClient, TestServer

from ultraskelly.core.bot.sensory.stereo_rectification import StereoRectification
from ultraskelly.core.pubsub.bot_topics import (
    DepthFrameMessage,
    DepthFrameTopic,
    FrameMessage,
    FrameTopic,
    StereoFrameMessage,
    StereoFrameTopic,
)
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager
from ultraskelly.ui.runtime_server import JpegCache, RuntimeServerNode, RuntimeServerParams, requested_views

SHARED = StereoRectification(width=8, height=6, focal_length=340.0, cx=4.0, cy=3.0, baseline=0.063,
                             left_rotation=((1, 0, 0), (0, 1, 0), (0, 0, 1)))
PAIR, PREVIEW = ("zed_left", "zed_right"), ("zed_rectified", "zed_depth")


class Encoder:
    """Stands in for the JPEG encoder: names each image by its first value and remembers it."""

    def __init__(self):
        self.images = []

    def __call__(self, image, quality):
        self.images.append(int(image.flat[0]))
        return b"JPEG%d" % int(image.flat[0])


def oak(value, **fields):
    return FrameMessage(frame=np.full((4, 6, 3), value, np.uint8), **fields)


def zed(value):
    return StereoFrameMessage(left=np.full((4, 6, 3), value, np.uint8), right=np.full((4, 6, 3), value + 1, np.uint8))


def depth(value):
    """A depth frame whose colour image is `value` everywhere and whose depth is 1500 mm."""
    return DepthFrameMessage(depth=np.full((6, 8), 1500, np.uint16), color=np.full((6, 8, 3), value, np.uint8),
                             rectification=SHARED)


def make_server(bus, encoder, **options):
    return RuntimeServerNode.create(
        pubsub=bus, params=RuntimeServerParams(state_hz=50), cameras={"oak": FrameTopic}, recorder=None,
        motors_simulated=True, encode_jpeg=encoder, stereo_cameras={"zed": StereoFrameTopic},
        depth_cameras={"zed": DepthFrameTopic}, **{"node_status": lambda: [], **options},
    )


class Inbox:
    """Everything one viewer's socket receives, read in the background until it closes."""

    def __init__(self, socket, viewer):
        self.socket, self.viewer, self.messages, self.texts = socket, viewer, [], []
        self.reading = asyncio.create_task(self._read())

    async def _read(self):
        async for message in self.socket:
            if message.type == WSMsgType.BINARY:
                self.messages.append(cbor2.loads(message.data))
            elif message.type == WSMsgType.TEXT:
                self.texts.append(message.data)

    def kinds(self):
        return [message["kind"] for message in self.messages]

    def frames(self):
        """The camera IDs of each frame message, in the order they came."""
        return [tuple(camera["id"] for camera in message["cameras"])
                for message in self.messages if message["kind"] == "frame"]

    async def show(self, views):
        """Send a views message and wait until the server has applied it."""
        await self.socket.send_str(json.dumps({"kind": "views", "views": views}))
        await until(lambda: self.viewer.views == frozenset(views))

    async def close(self):
        await self.socket.close()
        await self.reading


async def until(condition, timeout=5.0):
    async with asyncio.timeout(timeout):
        while not condition():
            await asyncio.sleep(0.005)


async def connect(client, server, views=None):
    """A new viewer; with `views`, once the server has applied them."""
    count = len(server.viewers) + 1
    socket = await client.ws_connect("/stream")
    await until(lambda: len(server.viewers) == count)
    inbox = Inbox(socket, server.viewers[-1])
    if views is not None:
        await inbox.show(views)
    return inbox


def run(scenario, **options):
    """Run `scenario(bus, server, client, encoder)` against a pumping server; return its result."""
    async def hosted():
        bus, encoder = PubSubTopicManager.create(), Encoder()
        server = make_server(bus, encoder, **options)
        pump = asyncio.create_task(server.pump())
        try:
            async with TestClient(TestServer(server.build_app())) as client:
                return await scenario(bus, server, client, encoder)
        finally:
            server.stop()
            await pump
            bus.close()

    return asyncio.run(hosted())


def test_each_viewer_gets_only_the_views_it_shows_and_always_the_robot_state():
    async def scenario(bus, server, client, encoder):
        oak_viewer = await connect(client, server, ["oak"])
        zed_viewer = await connect(client, server, ["zed_right", "no_such_camera"])  # One eye sends the pair
        await bus.publish(FrameTopic, oak(10))
        await bus.publish(StereoFrameTopic, zed(20))
        await until(lambda: ("oak",) in oak_viewer.frames() and PAIR in zed_viewer.frames())
        await until(lambda: "robot_state" in oak_viewer.kinds() and "robot_state" in zed_viewer.kinds())
        await asyncio.sleep(0.1)  # Time for anything sent to the wrong viewer to arrive
        await oak_viewer.close()
        await zed_viewer.close()
        return oak_viewer.frames(), zed_viewer.frames(), sorted(encoder.images)

    oak_frames, zed_frames, encoded = run(scenario)

    assert set(oak_frames) == {("oak",)} and set(zed_frames) == {PAIR}
    assert encoded == [10, 20, 21]  # Each image once


def test_a_viewer_that_shows_nothing_gets_robot_state_and_nothing_is_encoded():
    async def scenario(bus, server, client, encoder):
        hidden = await connect(client, server, [])
        await bus.publish(FrameTopic, oak(10))
        await bus.publish(StereoFrameTopic, zed(20))
        await bus.publish(DepthFrameTopic, depth(30))
        await until(lambda: all(feed.frames_seen == 1 for feed in server.feeds.values()))
        await until(lambda: hidden.kinds().count("robot_state") >= 3)
        await asyncio.sleep(0.05)
        await hidden.close()
        return hidden.kinds(), encoder.images

    kinds, encoded = run(scenario)

    assert set(kinds) == {"robot_state"} and encoded == []


def test_depth_shown_in_3d_sends_no_previews_and_encodes_only_the_colour_image():
    async def scenario(bus, server, client, encoder):
        viewer = await connect(client, server, ["depth_frame"])
        await bus.publish(DepthFrameTopic, depth(30))
        await until(lambda: "depth_frame" in viewer.kinds())
        await asyncio.sleep(0.05)
        await viewer.close()
        return viewer.frames(), encoder.images

    frames, encoded = run(scenario)

    assert frames == []  # Neither the rectified image nor the depth drawn in colour
    assert encoded == [30]  # The colour image the depth_frame carries; the depth is never coloured


def test_a_depth_preview_alone_sends_no_depth_frame():
    async def scenario(bus, server, client, encoder):
        viewer = await connect(client, server, ["zed_depth"])
        await bus.publish(DepthFrameTopic, depth(30))
        await until(lambda: PREVIEW in viewer.frames())
        await asyncio.sleep(0.05)
        await viewer.close()
        return viewer.kinds(), encoder.images

    kinds, encoded = run(scenario)

    assert "depth_frame" not in kinds and encoded == [30, 1500]


def test_a_view_asked_for_later_gets_the_newest_frame_without_waiting_for_another():
    async def scenario(bus, server, client, encoder):
        viewer = await connect(client, server, [])
        await bus.publish(DepthFrameTopic, depth(30))  # Depth comes at 5 Hz or less: worth not waiting for
        await until(lambda: server.feeds["zed_depth"].frames_seen == 1)
        await asyncio.sleep(0.02)
        unshown = list(encoder.images)
        await viewer.show(["depth_frame"])
        await until(lambda: "depth_frame" in viewer.kinds())
        await viewer.close()
        return unshown, encoder.images

    unshown, encoded = run(scenario)

    assert unshown == [] and encoded == [30]


def test_only_a_views_message_changes_what_a_viewer_gets():
    assert requested_views('{"kind": "views", "views": ["oak", "depth_frame", 3]}') == {"oak", "depth_frame"}
    assert requested_views('{"kind": "views", "views": [], "displayImageSizes": {}}') == frozenset()
    for text in ('{"frameNumber": 3, "displayImageSizes": {"oak": {"width": 640, "height": 480}}, "kinds": []}',
                 '{"kind": "views", "views": "oak"}', '{"kind": "other", "views": ["oak"]}', '["oak"]', "not json"):
        assert requested_views(text) is None

    async def scenario(bus, server, client, encoder):
        viewer = await connect(client, server)
        await viewer.socket.send_str('{"frameNumber": 3, "displayImageSizes": {}, "kinds": []}')
        await viewer.socket.send_str("ping")  # Answered once the message before it was read
        await until(lambda: "pong" in viewer.texts)
        await bus.publish(FrameTopic, oak(10))
        await until(lambda: ("oak",) in viewer.frames())
        await viewer.close()
        return viewer.viewer.views

    assert run(scenario) is None  # Still every view


def test_robot_state_is_published_only_while_someone_watches():
    async def scenario(bus, server, client, encoder):
        viewer = await connect(client, server)
        await until(lambda: viewer.kinds().count("robot_state") >= 2)
        await viewer.close()
        await until(lambda: server.clients == 0)
        published = server.state_sequence
        await asyncio.sleep(0.1)  # Five periods at 50 Hz
        return published, server.state_sequence

    published, later = run(scenario)

    assert published >= 2 and later == published


def test_a_failing_part_stops_the_pump_with_its_error():
    def broken():
        raise RuntimeError("node list broke")

    async def scenario():
        bus = PubSubTopicManager.create()
        server = make_server(bus, Encoder(), node_status=broken)
        pump = asyncio.create_task(server.pump())
        async with TestClient(TestServer(server.build_app())) as client:
            socket = await client.ws_connect("/stream")  # Robot state is built while someone watches
            with pytest.raises(RuntimeError, match="node list broke"):
                await asyncio.wait_for(pump, timeout=5)
            await socket.close()
        bus.close()

    asyncio.run(scenario())


def read_jpeg(payload: bytes) -> bytes:
    """The first image's JPEG, read straight from SkellyCam's layout."""
    length = struct.unpack_from("<i", payload, 24 + 48)[0]
    return payload[24 + 56:24 + 56 + length]


def test_a_cameras_own_jpeg_streams_as_it_came():
    async def scenario(bus, server, client, encoder):
        viewer = await connect(client, server, ["oak"])
        await bus.publish(FrameTopic, oak(10, jpeg=b"from the camera"))
        await until(lambda: ("oak",) in viewer.frames())
        await viewer.close()
        return next(message for message in viewer.messages if message["kind"] == "frame"), encoder.images

    sent, encoded = run(scenario)

    assert read_jpeg(sent["image"]) == b"from the camera" and encoded == []


def test_the_stream_and_the_recorder_share_a_frames_encoding():
    encoder, message = Encoder(), oak(10)
    streamed = JpegCache(encoder, message).encode(message.frame, 80)
    assert streamed == message.jpeg_bytes(80, encoder) == b"JPEG10"  # As the recorder asks for it
    assert encoder.images == [10]
