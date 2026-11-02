import asyncio

from aiohttp.test_utils import TestClient, TestServer

from ultraskelly.core.bot.motor.motor_test import MotorTester, MotorTestLimits
from ultraskelly.core.bot.motor.simulated_actuators import CommandRecorder
from ultraskelly.replay import create_controllers
from ultraskelly.ui.runtime_server import RuntimeServerNode, RuntimeServerParams


def make_server(limits=MotorTestLimits(settle_seconds=0.0)):
    recorder = CommandRecorder()
    bus, orientation, mouth = create_controllers(recorder)
    server = RuntimeServerNode.create(
        pubsub=bus, params=RuntimeServerParams(), cameras={}, recorder=recorder, motors_simulated=True,
        node_status=lambda: [], motor_tests=MotorTester(orientation=orientation, mouth=mouth, limits=limits),
    )
    return server, recorder, bus


def serve(scenario, **options):
    async def wrapper():
        server, recorder, bus = make_server(**options)
        async with TestClient(TestServer(server.build_app())) as client:
            await scenario(client, server, recorder)
        bus.close()
    asyncio.run(wrapper())


def test_a_test_is_answered_with_how_it_ended():
    async def scenario(client, server, recorder):
        state = await (await client.get("/api/motor-test")).json()
        assert state["enabled"] and state["limits"]["servo_range"] == 30.0 and state["running"] is None
        reply = await client.post("/api/motor-test", json={"actuator": "pan", "angle": 100})
        body = await reply.json()
        assert reply.status == 200 and body["ok"] and body["test"]["outcome"] == "done" and body["test"]["sent"] == 100
        assert recorder.latest["pan"].value == 100
        assert server.robot_state()["motor_test"]["last"]["description"] == "pan to 100 degrees"
    serve(scenario)


def test_bad_or_disallowed_tests_are_refused_with_the_reason():
    async def scenario(client, server, recorder):
        before = len(recorder.history)
        for body, status, reason in [
            ({"actuator": "pan", "angle": 150}, 400, "within 30 degrees"),
            ({"actuator": "waist", "throttle": 0.5}, 400, "a throttle and seconds"),
            ({"actuator": "arm", "angle": 90}, 400, "actuator"),
            ({"actuator": "pan", "angle": 90, "speed": 2}, 400, "speed"),
        ]:
            reply = await client.post("/api/motor-test", json=body)
            assert reply.status == status and reason in (await reply.json())["error"]
        reply = await client.post("/api/motor-test", data="actuator=pan&angle=90")  # A plain form, as any page could send
        assert reply.status == 415
        assert len(recorder.history) == before  # Nothing moved
    serve(scenario)


def test_one_test_at_a_time_and_stop_cuts_it_short():
    async def scenario(client, server, recorder):
        first = asyncio.create_task(client.post("/api/motor-test", json={"actuator": "waist", "throttle": 0.5, "seconds": 0.5}))
        while server.motor_tests.current is None:
            await asyncio.sleep(0.005)
        busy = await client.post("/api/motor-test", json={"actuator": "tilt", "angle": 80})
        assert busy.status == 409 and "Busy" in (await busy.json())["error"]
        stopped = await client.post("/api/motor-test/stop", json={})
        assert (await stopped.json())["stopped"]
        reply = await first
        assert (await reply.json())["test"]["outcome"] == "stopped"
        assert recorder.latest["waist"].value == 0.0
    serve(scenario)


def test_tests_can_be_turned_off():
    async def scenario(client, server, recorder):
        reply = await client.post("/api/motor-test", json={"actuator": "pan", "angle": 90})
        assert reply.status == 403
    serve(scenario, limits=MotorTestLimits(enabled=False))


def test_the_dashboard_panel_is_served():
    async def scenario(client, server, recorder):
        reply = await client.get("/motor-test.js")
        assert reply.status == 200 and reply.content_type == "text/javascript"
        assert "export class MotorTestPanel" in await reply.text()
        page = await (await client.get("/")).text()
        assert 'id="motor-test"' in page
    serve(scenario)
