import asyncio
import socket

import aiohttp
import cbor2

from ultraskelly.replay import scripted_target, serve


def free_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def test_scripted_target_cycles_center_left_right_and_missing():
    assert [scripted_target(step).x for step in (0, 10, 20, 30, 40)] == [320, 0, 640, None, 320]
    assert scripted_target(30).y is None


def test_serve_streams_controller_state_driven_by_scripted_targets():
    async def scenario():
        port = free_port()
        serving = asyncio.create_task(serve(port, interval=0.02))
        try:
            async with aiohttp.ClientSession() as session:
                for _ in range(100):  # Wait for the server, then for the head to leave center
                    await asyncio.sleep(0.05)
                    try:
                        async with session.ws_connect(f"ws://127.0.0.1:{port}/stream") as stream:
                            for _ in range(100):
                                state = cbor2.loads((await asyncio.wait_for(stream.receive(), timeout=2)).data)
                                if state["kind"] == "robot_state" and                                         state["motors"]["commands"].get("pan", {}).get("value") not in (None, 90.0):
                                    return state
                    except aiohttp.ClientConnectionError:
                        continue
                raise AssertionError("The scripted targets never moved the head")
        finally:
            serving.cancel()
            await asyncio.gather(serving, return_exceptions=True)

    state = asyncio.run(scenario())

    assert state["motors"]["simulated"] is True
    assert {"pan", "tilt", "waist", "mouth"} <= set(state["motors"]["commands"])
    assert {node["name"] for node in state["nodes"]} == {"OrientationNode", "MouthNode", "RuntimeServerNode"}
    assert all(node["state"] == "running" for node in state["nodes"])


def test_synthetic_people_put_the_followed_person_in_any_row():
    import numpy as np

    from ultraskelly.core.bot.sensory.pose_common import CocoKeypoint, locate_target
    from ultraskelly.replay import synthetic_people

    for order in ([0, 1, 2], [2, 0, 1], [1, 2, 0]):
        keypoints, scores = synthetic_people(3, 1.5, np.array(order))
        row, (x, y, _) = locate_target(
            keypoints=keypoints, scores=scores, target_keypoint=CocoKeypoint.NOSE, keypoint_threshold=0.5)
        assert row == order.index(1)  # The middle person, wherever their row is
        assert abs(x - 320) <= 40 and (x, y) == tuple(int(value) for value in keypoints[row, 0, :2])


def test_serve_with_people_streams_several_people_with_the_followed_one_as_target():
    import numpy as np

    async def scenario():
        port = free_port()
        serving = asyncio.create_task(serve(port, interval=0.02, people=3))
        frames = []
        try:
            async with aiohttp.ClientSession() as session:
                for _ in range(100):  # Wait for the server
                    await asyncio.sleep(0.05)
                    try:
                        async with session.ws_connect(f"ws://127.0.0.1:{port}/stream") as stream:
                            while len(frames) < 18:
                                message = cbor2.loads((await asyncio.wait_for(stream.receive(), timeout=2)).data)
                                if message["kind"] == "frame" and message["trackers"]:
                                    frames.append(message)
                            return frames
                    except aiohttp.ClientConnectionError:
                        continue
                raise AssertionError("No frames with people arrived")
        finally:
            serving.cancel()
            await asyncio.gather(serving, return_exceptions=True)

    frames = asyncio.run(scenario())

    positions, measured, hands = set(), 0, 0
    for frame in frames:
        faces = [tracker for tracker in frame["trackers"] if tracker["tracker_id"].startswith("face-")]
        assert [face["tracker_id"] for face in faces] == ["face-0", "face-1", "face-2"]  # Each person's face, best first
        measured += any(channel["kind"] == "ultraskelly.head_pose" for channel in faces[0]["channels"])
        hands += any(tracker["tracker_id"].startswith("hand-") for tracker in frame["trackers"])
        ids = [tracker["tracker_id"] for tracker in frame["trackers"] if tracker["tracker_id"].split("-")[0] not in ("face", "hand")]
        assert len(ids) == 3 and ids.count("target") == 1
        target = frame["trackers"][ids.index("target")]
        nose = np.frombuffer(target["channels"][0]["data"], "<f4")[:2]
        assert abs(nose[0] - 320) <= 41  # The middle person, wherever their row is
        assert [channel["kind"] for channel in target["channels"]] == ["OVERLAY_2D", "ultraskelly.target"]
        positions.add(ids.index("target"))
        assert frame["cameras"][0]["id"] == "synthetic" and frame["image"][24 + 56:24 + 58] == b"\xff\xd8"
    assert len(positions) > 1  # The rows really were reordered
    assert 0 < measured < len(frames)  # Head pose in some frames, as the OAK measures every second one
    assert 0 < hands < len(frames) / 2  # Hands in every sixth
