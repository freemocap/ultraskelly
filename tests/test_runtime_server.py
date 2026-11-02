import asyncio

import numpy as np
from aiohttp.test_utils import TestClient, TestServer

from ultraskelly.core.bot.motor.simulated_actuators import CommandRecorder
from ultraskelly.core.pubsub.bot_topics import (
    FrameMessage,
    FrameTopic,
    PoseDataMessage,
    PoseDataTopic,
    ServoStateMessage,
    ServoStateTopic,
    TargetLocationMessage,
    TargetLocationTopic,
)
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager
from ultraskelly.ui.runtime_server import RuntimeServerNode, RuntimeServerParams, TrackingOverlay


def make_server(bus, *, encoded, **overlay_options):
    recorder = CommandRecorder()
    recorder.record("pan", 100.0)

    def fake_encode(frame, quality):
        encoded.append(int(frame[0, 0, 0]))
        return b"JPEG%d" % int(frame[0, 0, 0])

    return RuntimeServerNode.create(
        pubsub=bus,
        params=RuntimeServerParams(state_hz=50),
        cameras={"oak": FrameTopic},
        recorder=recorder,
        motors_simulated=True,
        node_status=lambda: [{"name": "FakeNode", "state": "running"}],
        encode_jpeg=fake_encode,
        **overlay_options,
    )


def frame(value):
    return np.full((480, 640, 3), value, dtype=np.uint8)


def test_state_reports_motor_commands_bot_state_and_nodes():
    async def scenario():
        bus = PubSubTopicManager.create()
        server = make_server(bus, encoded=[])
        pump = asyncio.create_task(server.pump())
        await bus.publish(
            ServoStateTopic,
            ServoStateMessage(
                pan_angle=100, tilt_angle=80, roll_angle=90,
                is_locked_x=True, is_locked_y=False, is_locked_roll=False,
            ),
        )
        await bus.publish(TargetLocationTopic, TargetLocationMessage(x=330, y=200, angle=85.0))
        keypoints = np.zeros((1, 17, 3), np.float32)
        keypoints[0, 0] = [330, 200, 0.9]
        await bus.publish(PoseDataTopic, PoseDataMessage(keypoints=keypoints, scores=np.array([0.8])))
        await bus.publish(FrameTopic, FrameMessage(frame=frame(1)))
        await asyncio.sleep(0.05)
        async with TestClient(TestServer(server.build_app())) as client:
            state = await (await client.get("/api/state")).json()
        server.stop()
        await pump
        bus.close()
        return state

    state = asyncio.run(scenario())

    assert state["motors"]["simulated"] is True
    assert state["motors"]["commands"]["pan"]["value"] == 100.0
    assert state["servo_state"]["pan"] == 100 and state["servo_state"]["locked_x"] is True
    assert (state["target"]["x"], state["target"]["y"]) == (330, 200)
    assert state["pose"]["keypoints"][0][0] == [330.0, 200.0, 0.9]
    assert state["pose"]["scores"] == [0.8]
    assert (state["cameras"]["oak"]["width"], state["cameras"]["oak"]["height"]) == (640, 480)
    assert state["nodes"] == [{"name": "FakeNode", "state": "running"}]


def test_video_overlay_draws_latest_pose_and_target_on_a_copy():
    async def scenario():
        bus = PubSubTopicManager.create()
        drawn = []

        def fake_draw(frame, *, keypoints, target, aim, jaw_zone):
            drawn.append((None if keypoints is None else keypoints.shape, target, aim, jaw_zone))
            annotated = frame.copy()
            annotated[0, 0, 0] = 99
            return annotated

        server = make_server(
            bus,
            encoded=[],
            overlays={"oak": TrackingOverlay(aim=(300, 200), jaw_zone=80)},
            draw_overlay=fake_draw,
        )
        pump = asyncio.create_task(server.pump())
        await bus.publish(TargetLocationTopic, TargetLocationMessage(x=330, y=210, angle=0.0))
        await bus.publish(
            PoseDataTopic, PoseDataMessage(keypoints=np.zeros((2, 17, 3), np.float32), scores=np.array([0.9, 0.8]))
        )
        published = frame(2)
        async with TestClient(TestServer(server.build_app())) as client:
            response = await client.get("/video/oak.mjpg")
            await asyncio.sleep(0.02)
            await bus.publish(FrameTopic, FrameMessage(frame=published))
            header = await response.content.readuntil(b"\r\n\r\n")
            length = int(header.split(b"Content-Length: ")[1].split(b"\r\n")[0])
            part = await response.content.readexactly(length)
            response.close()
        server.stop()
        await pump
        bus.close()
        return part, drawn, published

    part, drawn, published = asyncio.run(scenario())

    assert part == b"JPEG99"
    assert drawn == [((2, 17, 3), (330, 210), (300, 200), 80)]
    assert published[0, 0, 0] == 2  # Other subscribers' frame is unchanged


def test_dashboard_assets_are_served_locally_with_explicit_types():
    async def scenario():
        bus = PubSubTopicManager.create()
        server = make_server(bus, encoded=[])
        served = {}
        async with TestClient(TestServer(server.build_app())) as client:
            for path in ["/", "/dashboard.js", "/dashboard.css", "/style.css", "/skeleton.js",
                         "/vendor/three.module.js", "/vendor/three.core.js", "/vendor/OrbitControls.js"]:
                response = await client.get(path)
                served[path] = (response.status, response.content_type, len(await response.read()))
        bus.close()
        return served

    served = asyncio.run(scenario())

    assert all(status == 200 and size > 100 for status, _, size in served.values())
    assert served["/"][1] == "text/html"
    assert {served[path][1] for path in served if path.endswith(".js")} == {"text/javascript"}


def test_websocket_streams_fresh_snapshots():
    async def scenario():
        bus = PubSubTopicManager.create()
        server = make_server(bus, encoded=[])
        async with TestClient(TestServer(server.build_app())) as client:
            websocket = await client.ws_connect("/ws")
            first = await websocket.receive_json()
            second = await websocket.receive_json()
            await websocket.close()
        server.stop()
        bus.close()
        return first, second

    first, second = asyncio.run(scenario())

    assert second["time"] > first["time"]
    assert first["motors"]["commands"]["pan"]["value"] == 100.0


def test_video_streams_newest_frames_and_encodes_only_while_watched():
    async def scenario():
        bus = PubSubTopicManager.create()
        encoded = []
        server = make_server(bus, encoded=encoded)
        pump = asyncio.create_task(server.pump())
        await bus.publish(FrameTopic, FrameMessage(frame=frame(1)))
        await asyncio.sleep(0.05)
        encoded_unwatched = list(encoded)
        async with TestClient(TestServer(server.build_app())) as client:
            response = await client.get("/video/oak.mjpg")
            content_type = response.headers["Content-Type"]
            parts = []
            for value in (2, 3):
                await bus.publish(FrameTopic, FrameMessage(frame=frame(value)))
                header = await response.content.readuntil(b"\r\n\r\n")
                length = int(header.split(b"Content-Length: ")[1].split(b"\r\n")[0])
                parts.append(await response.content.readexactly(length))
                await response.content.readexactly(2)
            response.close()
            missing_status = (await client.get("/video/nope.mjpg")).status
        server.stop()
        await pump
        bus.close()
        return encoded_unwatched, encoded, content_type, parts, missing_status

    encoded_unwatched, encoded, content_type, parts, missing_status = asyncio.run(scenario())

    assert encoded_unwatched == []
    assert content_type.startswith("multipart/x-mixed-replace")
    assert parts == [b"JPEG2", b"JPEG3"]
    assert encoded == [2, 3]
    assert missing_status == 404
