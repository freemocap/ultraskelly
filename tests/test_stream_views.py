"""What a viewer draws (docs/stream.md, "What a viewer draws"): from each viewer's display
feedback the server encodes only the feeds someone draws, sends each viewer only what it
draws and the kinds it wants, and robot_state always."""
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
    ImuMessage,
    ImuTopic,
    PoseDataMessage,
    PoseDataTopic,
    StereoFrameMessage,
    StereoFrameTopic,
    TargetLocationMessage,
)
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager
from ultraskelly.ui.runtime_server import (
    Display,
    JpegCache,
    RuntimeServerNode,
    RuntimeServerParams,
    TrackingOverlay,
    display_feedback,
    resize_with_opencv,
)

SHARED = StereoRectification(width=8, height=6, focal_length=340.0, cx=4.0, cy=3.0, baseline=0.063,
                             left_rotation=((1, 0, 0), (0, 1, 0), (0, 0, 1)))
PAIR, PREVIEW = ("zed_left", "zed_right"), ("zed_rectified", "zed_depth")


class Encoder:
    """Stands in for the JPEG encoder: names each image by its first value and remembers it and its size."""

    def __init__(self):
        self.images, self.sizes = [], []

    def __call__(self, image, quality):
        self.images.append(int(image.flat[0]))
        self.sizes.append((image.shape[1], image.shape[0]))
        return b"JPEG%d" % int(image.flat[0])


def nearest(image, size):
    """Stands in for the resize: nearest pixels, without OpenCV."""
    width, height = size
    return image[np.arange(height) * image.shape[0] // height][:, np.arange(width) * image.shape[1] // width]


def oak(value, **fields):
    return FrameMessage(frame=np.full((4, 6, 3), value, np.uint8), **fields)


def stereo(value, size=(6, 4)):
    """A ZED pair as the camera sends it, YUYV, its eyes' brightness `value` and `value + 1`."""
    width, height = size
    yuyv = np.full((height, 2 * width, 2), 128, np.uint8)
    yuyv[:, :width, 0], yuyv[:, width:, 0] = value, value + 1
    return StereoFrameMessage(yuyv=yuyv)


def zed(value):
    return stereo(value)


def depth(value):
    """A depth frame whose colour image is `value` everywhere and whose depth is 1500 mm."""
    return DepthFrameMessage(depth=np.full((6, 8), 1500, np.uint16), color=np.full((6, 8, 3), value, np.uint8),
                             rectification=SHARED)


def imu_batch(first_sample, count=4):
    times = 200.0 + 0.00126 * np.arange(first_sample, first_sample + count)
    return ImuMessage(timestamps=times, imu_times=times - 50.0, acceleration=np.zeros((count, 3), np.float32),
                      angular_velocity=np.ones((count, 3), np.float32), first_sample=first_sample, temperature=52.5)


def make_server(bus, encoder, *, previews=False, **options):
    return RuntimeServerNode.create(
        pubsub=bus, params=RuntimeServerParams(state_hz=50, depth_previews=previews), cameras={"oak": FrameTopic},
        recorder=None, motors_simulated=True, encode_jpeg=encoder, stereo_cameras={"zed": StereoFrameTopic},
        depth_cameras={"zed": DepthFrameTopic}, **{"node_status": lambda: [], **options},
    )


def feedback(cameras, kinds=(), size=(640, 480)):
    """Display feedback as the dashboard sends it: each camera drawn at `size`, and `kinds`.
    `cameras` may also map each camera ID to its own size."""
    drawn = cameras if isinstance(cameras, dict) else dict.fromkeys(cameras, size)
    sizes = {camera: {"width": width, "height": height} for camera, (width, height) in drawn.items()}
    return {"frameNumber": 7, "displayImageSizes": sizes, "kinds": list(kinds)}


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

    async def show(self, cameras, kinds=()):
        """Send display feedback and wait until the server has applied it."""
        await self.socket.send_str(json.dumps(feedback(cameras, kinds)))
        await until(lambda: self.viewer.display is not None and set(self.viewer.display.sizes) == set(cameras)
                    and self.viewer.display.kinds == frozenset(kinds))

    async def close(self):
        await self.socket.close()
        await self.reading


async def until(condition, timeout=5.0):
    async with asyncio.timeout(timeout):
        while not condition():
            await asyncio.sleep(0.005)


async def connect(client, server, cameras=None, kinds=()):
    """A new viewer; with `cameras`, once the server has applied its feedback."""
    count = len(server.viewers) + 1
    socket = await client.ws_connect("/stream")
    await until(lambda: len(server.viewers) == count)
    inbox = Inbox(socket, server.viewers[-1])
    if cameras is not None:
        await inbox.show(cameras, kinds)
    return inbox


def run(scenario, **options):
    """Run `scenario(bus, server, client, encoder)` against a pumping server; return its result."""
    async def hosted():
        bus, encoder = PubSubTopicManager.create(), Encoder()
        server = make_server(bus, encoder, **options)
        server.resize = nearest
        pump = asyncio.create_task(server.pump())
        try:
            async with TestClient(TestServer(server.build_app())) as client:
                return await scenario(bus, server, client, encoder)
        finally:
            server.stop()
            await pump
            bus.close()

    return asyncio.run(hosted())


def test_each_viewer_gets_only_the_cameras_it_draws_and_always_the_robot_state():
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


def test_a_hidden_page_gets_robot_state_and_nothing_is_encoded():
    async def scenario(bus, server, client, encoder):
        hidden = await connect(client, server, [])  # What a hidden page sends: no cameras, no kinds
        await bus.publish(FrameTopic, oak(10))
        await bus.publish(StereoFrameTopic, zed(20))
        await bus.publish(DepthFrameTopic, depth(30))
        await bus.publish(ImuTopic, imu_batch(0))
        await until(lambda: all(feed.frames_seen == 1 for feed in server.feeds.values()) and "imu" in server.latest)
        await until(lambda: hidden.kinds().count("robot_state") >= 3)
        await asyncio.sleep(0.05)
        await hidden.close()
        return hidden.kinds(), encoder.images, server.imu_sequence

    kinds, encoded, imu_batches = run(scenario)

    assert set(kinds) == {"robot_state"} and encoded == [] and imu_batches == 0


def test_depth_shown_in_3d_encodes_only_the_colour_image():
    async def scenario(bus, server, client, encoder):
        viewer = await connect(client, server, [], ["depth_frame"])
        await bus.publish(DepthFrameTopic, depth(30))
        await until(lambda: "depth_frame" in viewer.kinds())
        await asyncio.sleep(0.05)
        await viewer.close()
        return viewer.frames(), encoder.images

    frames, encoded = run(scenario, previews=True)

    assert frames == []  # Neither the rectified image nor the depth drawn in colour
    assert encoded == [30]  # The colour image the depth_frame carries; the depth is never coloured


def test_a_drawn_depth_preview_sends_no_depth_frame():
    async def scenario(bus, server, client, encoder):
        viewer = await connect(client, server, ["zed_depth"])
        await bus.publish(DepthFrameTopic, depth(30))
        await until(lambda: PREVIEW in viewer.frames())
        await asyncio.sleep(0.05)
        await viewer.close()
        return viewer.kinds(), encoder.images

    kinds, encoded = run(scenario, previews=True)

    assert "depth_frame" not in kinds and encoded == [30, 1500]


def test_without_depth_previews_a_drawn_preview_is_never_encoded():
    async def scenario(bus, server, client, encoder):
        viewer = await connect(client, server, ["zed_rectified", "zed_depth"])
        await bus.publish(DepthFrameTopic, depth(30))
        await until(lambda: server.feeds["zed_depth"].frames_seen == 1)
        await asyncio.sleep(0.05)
        await viewer.close()
        return viewer.frames(), encoder.images

    assert run(scenario) == ([], [])


def test_a_camera_asked_for_later_gets_its_newest_frame_without_waiting_for_another():
    async def scenario(bus, server, client, encoder):
        viewer = await connect(client, server, [])
        await bus.publish(DepthFrameTopic, depth(30))  # Depth comes at 10 Hz or less: worth not waiting for
        await until(lambda: server.feeds["zed_depth"].frames_seen == 1)
        await asyncio.sleep(0.02)
        unshown = list(encoder.images)
        await viewer.show([], ["depth_frame"])
        await until(lambda: "depth_frame" in viewer.kinds())
        await viewer.close()
        return unshown, encoder.images

    unshown, encoded = run(scenario)

    assert unshown == [] and encoded == [30]


def test_imu_batches_go_only_to_viewers_that_want_the_kind():
    async def scenario(bus, server, client, encoder):
        plotting = await connect(client, server, [], ["imu"])
        drawing = await connect(client, server, ["oak"])
        await bus.publish(ImuTopic, imu_batch(0))
        await until(lambda: "imu" in plotting.kinds())
        await asyncio.sleep(0.05)
        await plotting.close()
        await drawing.close()
        return plotting.kinds(), drawing.kinds()

    plotted, drawn = run(scenario)

    assert plotted.count("imu") == 1 and "imu" not in drawn


EYES = (672, 376)  # The ZED's eyes at VGA, width and height


def eyes(value):
    return stereo(value, EYES)


def sent_sizes(inbox):
    """Each frame message's declared image sizes, by camera ID."""
    return [{camera["id"]: tuple(camera["image_size"]) for camera in message["cameras"]}
            for message in inbox.messages if message["kind"] == "frame"]


def stream_eyes(viewers, **options):
    """Publish one ZED pair to viewers that send the given feedback (None: none); return what the
    first one got and the sizes encoded."""
    async def scenario(bus, server, client, encoder):
        inboxes = []
        for drawn in viewers:
            inbox = await connect(client, server)
            if drawn is not None:
                await inbox.socket.send_str(json.dumps(feedback(drawn)))
                await until(lambda: inbox.viewer.display is not None)
            inboxes.append(inbox)
        await bus.publish(StereoFrameTopic, eyes(20))
        await until(lambda: PAIR in inboxes[0].frames())
        for inbox in inboxes:
            await inbox.close()
        return sent_sizes(inboxes[0]), encoder.sizes

    return run(scenario, **options)


def test_eyes_drawn_small_are_encoded_at_the_drawn_size():
    sent, encoded = stream_eyes([{"zed_left": (317, 178), "zed_right": (317, 178)}])

    assert sent == [{"zed_left": (318, 178), "zed_right": (318, 178)}]  # 4.5 times fewer pixels each
    assert encoded == [(318, 178), (318, 178)]


def test_one_drawn_eye_sets_the_pairs_size():
    sent, _ = stream_eyes([{"zed_left": (317, 178)}, {"zed_right": (160, 90)}])

    assert sent == [{"zed_left": (318, 178), "zed_right": (318, 178)}]  # The larger, for both eyes


def test_an_enlarged_eye_or_a_viewer_without_sizes_gets_the_camera_size():
    enlarged, _ = stream_eyes([{"zed_left": (317, 178)}, {"zed_left": (1238, 734)}])
    unsized, _ = stream_eyes([{"zed_left": (317, 178)}, None])
    nearly_whole, _ = stream_eyes([{"zed_left": (560, 313)}])  # 83%: resizing would cost about what it saves

    assert enlarged == unsized == nearly_whole == [{"zed_left": EYES, "zed_right": EYES}]


def test_the_oak_shrinks_while_its_keypoints_and_target_stay_in_camera_pixels():
    async def scenario(bus, server, client, encoder):
        viewer = await connect(client, server, {"oak": (160, 120)})
        camera_frame = FrameMessage(frame=np.full((480, 640, 3), 10, np.uint8), frame_number=5)
        await bus.publish(FrameTopic, camera_frame)
        keypoints = np.zeros((1, 17, 3), np.float32)
        keypoints[0, 0] = [330, 200, 0.9]
        await bus.publish(PoseDataTopic, PoseDataMessage(
            keypoints=keypoints, scores=np.array([0.9]), frame=camera_frame,
            target=TargetLocationMessage(x=330, y=200, angle=85.0), target_person=0))
        await until(lambda: any(message["trackers"] for message in viewer.messages if message["kind"] == "frame"))
        await viewer.close()
        return next(m for m in viewer.messages if m["kind"] == "frame" and m["trackers"]), encoder.sizes

    posed, encoded = run(scenario, overlays={"oak": TrackingOverlay()}, pose_camera="oak")

    assert posed["cameras"][0]["image_size"] == [160, 120] and (160, 120) in encoded  # A sixteenth of the pixels
    keypoint_channel, target_channel = posed["trackers"][0]["channels"]
    assert keypoint_channel["image_size"] == target_channel["image_size"] == [640, 480]  # The camera's, as before
    assert np.frombuffer(keypoint_channel["data"], "<f4").reshape(17, 3)[0].tolist() == pytest.approx([330, 200, 0.9])


def test_a_cameras_own_jpeg_goes_out_whole_even_when_drawn_small():
    async def scenario(bus, server, client, encoder):
        viewer = await connect(client, server, {"oak": (160, 120)})
        await bus.publish(FrameTopic, FrameMessage(frame=np.full((480, 640, 3), 10, np.uint8), jpeg=b"from the camera"))
        await until(lambda: ("oak",) in viewer.frames())
        await viewer.close()
        sent = next(message for message in viewer.messages if message["kind"] == "frame")
        return sent["cameras"][0]["image_size"], read_jpeg(sent["image"]), encoder.images

    assert run(scenario) == ([640, 480], b"from the camera", [])  # Shrinking it would mean decoding it


def test_the_resize_averages_colour_and_keeps_depth_values_exact():
    cv2 = pytest.importorskip("cv2", reason="OpenCV comes with the hardware extra")
    colour = np.zeros((376, 672, 3), np.uint8)
    colour[:, ::2] = 200  # Alternate columns: averaging gives about 100
    depth_image = np.zeros((376, 672), np.uint16)
    depth_image[:, ::2] = 1500
    yuyv = np.full((376, 672, 2), 128, np.uint8)  # A grey eye as the ZED sends it: no colour
    yuyv[:, ::2, 0] = 200  # Brightness 200 and 128 in alternate columns, 164 on average
    averaged = np.full((2, 2, 2), 128, np.uint8)
    averaged[..., 0] = 164

    small_colour, small_depth = resize_with_opencv(colour, (336, 188)), resize_with_opencv(depth_image, (336, 188))
    small_eye = resize_with_opencv(stereo(0, (672, 376)).eye(0), (336, 188))  # A view into the pair's frame
    grey_eye = resize_with_opencv(yuyv, (336, 188))

    assert small_colour.shape == (188, 336, 3) and abs(int(small_colour.mean()) - 100) <= 1
    assert small_depth.dtype == np.uint16 and set(np.unique(small_depth)) <= {0, 1500}
    assert small_eye.shape == grey_eye.shape == (188, 336, 3)  # Eyes come back as RGB
    assert np.ptp(grey_eye.reshape(-1, 3).mean(axis=0)) < 2  # Still grey
    assert abs(grey_eye.mean() - cv2.cvtColor(averaged, cv2.COLOR_YUV2RGB_YUYV).mean()) <= 1  # Averaged


def test_display_feedback_is_read_in_freemocaps_shape_with_kinds():
    shown = display_feedback(json.dumps({
        "frameNumber": 3, "displayImageSizes": {"oak": {"width": 640, "height": 480}, "zed_left": {"width": "wide"},
                                                "zed_right": {"width": 316.6, "height": 178.2}},
        "kinds": ["depth_frame", 3],
    }))
    assert shown == Display(sizes={"oak": (640, 480), "zed_left": None, "zed_right": (317, 178)},
                            kinds=frozenset({"depth_frame"}))
    assert display_feedback('{"displayImageSizes": {}, "kinds": []}') == Display(sizes={}, kinds=frozenset())
    assert display_feedback('{"displayImageSizes": {"oak": {"width": 1, "height": 1}}}').kinds == frozenset()
    for text in ('{"kind": "views", "views": ["oak"]}', '{"frameNumber": 3}', '{"displayImageSizes": ["oak"]}',
                 '["oak"]', "not json"):
        assert display_feedback(text) is None


def test_other_text_changes_nothing():
    async def scenario(bus, server, client, encoder):
        viewer = await connect(client, server)
        await viewer.socket.send_str('{"kind": "views", "views": []}')  # Not display feedback
        await viewer.socket.send_str("ping")  # Answered once the message before it was read
        await until(lambda: "pong" in viewer.texts)
        await bus.publish(FrameTopic, oak(10))
        await until(lambda: ("oak",) in viewer.frames())
        await viewer.close()
        return viewer.viewer.display

    assert run(scenario) is None  # Still everything


def test_the_stream_goes_uncompressed_even_when_a_browser_offers_deflate():
    async def scenario(bus, server, client, encoder):
        socket = await client.ws_connect("/stream", compress=15)  # Offers permessage-deflate, as browsers do
        negotiated = socket.compress
        await socket.close()
        return negotiated

    assert run(scenario) == 0


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
