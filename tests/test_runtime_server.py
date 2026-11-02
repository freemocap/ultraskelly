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
from ultraskelly.ui.runtime_server import RuntimeServerNode, RuntimeServerParams


def make_server(bus, *, encoded):
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
