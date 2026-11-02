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
