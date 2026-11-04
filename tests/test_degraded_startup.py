"""The bot starts with whatever hardware works and says what is wrong and how to fix it: the real
launcher, controllers and server, on fake parts that are present, missing or failing."""
import asyncio
import socket
import struct
import threading
import time
import types

import numpy as np
import pytest

from ultraskelly import bot_launcher
from ultraskelly.__main__bot import launch_config
from ultraskelly.bot_launcher import BotLauncher, Hardware
from ultraskelly.core.bot.base_abcs import Node, NodeParams
from ultraskelly.core.bot.diagnostics import Sources
from ultraskelly.core.bot.health import Health, Probes, Status, usb_devices
from ultraskelly.core.bot.motor.motor_test import MotorTestRefused, MotorTestRequest
from ultraskelly.core.bot.motor.simulated_actuators import SimulatedServoKit
from ultraskelly.core.bot.stall_monitor_node import StallMonitorNode, StallMonitorParams, StreamStalledError
from ultraskelly.core.pubsub.bot_topics import FrameMessage, FrameTopic

OAK_USB, ZED_VIDEO_USB, ZED_HID_USB = "03e7:2485", "2b03:f682", "2b03:f681"
MOTORKIT_CHANNELS = {1: (9, 10), 2: (11, 12), 3: (3, 4), 4: (5, 6)}  # adafruit_motorkit's bridge inputs


class Device:
    """The Motor HAT's I2C device: keeps each transfer."""

    def __init__(self):
        self.transfers = []

    def __enter__(self):
        return self

    def __exit__(self, *exception):
        return False

    def write(self, buffer, *, start=0, end=None):
        self.transfers.append(bytes(buffer[start:end]))


class MotorKit:
    def __init__(self, pwm_frequency=1600.0):
        self.pwm_frequency = pwm_frequency
        self._pca = types.SimpleNamespace(i2c_device=Device())
        for number, (positive, negative) in MOTORKIT_CHANNELS.items():
            setattr(self, f"motor{number}", types.SimpleNamespace(
                _positive=types.SimpleNamespace(_index=positive), _negative=types.SimpleNamespace(_index=negative)))


class ServoKit:
    def __init__(self):
        self.servo = [types.SimpleNamespace(angle=None) for _ in range(16)]


class FakeOak(Node):
    """Publishes a frame every 10 ms; crashes as if unplugged after `crash_after`, or goes quiet after `quiet_after`."""

    params: NodeParams = NodeParams()
    assembler: object = None
    models_error: str | None = None
    crash_after: float | None = None
    quiet_after: float | None = None
    frames: int = 0

    def results_for(self, frame_number):
        return None

    def status(self) -> dict:
        return {"rates": {"frames": 30.0 if self.frames else 0.0}}

    async def run(self) -> None:
        started = asyncio.get_running_loop().time()
        while not self.stop_event.is_set():
            elapsed = asyncio.get_running_loop().time() - started
            if self.crash_after is not None and elapsed > self.crash_after:
                raise RuntimeError("OAK pipeline stopped unexpectedly (camera unplugged?)")
            if self.quiet_after is None or elapsed < self.quiet_after:
                await self.pubsub.topics[FrameTopic].publish(FrameMessage(frame=np.zeros((2, 2, 3), np.uint8)))
                self.frames += 1
            await asyncio.sleep(0.01)


class Idle(Node):
    """Stands in for the Hailo detector and depth: runs until stopped."""

    params: NodeParams = NodeParams()

    async def run(self) -> None:
        await self.stop_event.wait()


class FakeZed(Idle):
    """Like the ZED node: `source` is None until the camera opens, then frames count up."""

    source: object = None
    frames_published: int = 0
    opens_after: float | None = None

    async def run(self) -> None:
        started = asyncio.get_running_loop().time()
        while not self.stop_event.is_set():
            if self.source is None and self.opens_after is not None:
                if asyncio.get_running_loop().time() - started > self.opens_after:
                    self.source = "opened"
            if self.source is not None:
                self.frames_published += 1
            await asyncio.sleep(0.01)


class FakeImu(Idle):
    samples: int = 0


class Robot:
    """Which parts are plugged in, as factories and probes for the launcher."""

    def __init__(self, *, oak=True, hailo=True, zed=True, imu=True, servo_board=True, motor_board=True,
                 usb=None, oak_fails=False, oak_options=None, zed_opens_after=None, hailo_fails=0):
        self.usb = set(usb) if usb is not None else (
            ({OAK_USB} if oak or oak_fails else set()) | ({ZED_VIDEO_USB, ZED_HID_USB} if zed else set()))
        self.oak, self.hailo, self.zed, self.imu = oak, hailo, zed, imu
        self.servo_board, self.motor_board, self.oak_fails = servo_board, motor_board, oak_fails
        self.oak_options, self.zed_opens_after = oak_options or {}, zed_opens_after
        self.servo_kit, self.motor_kit = ServoKit(), MotorKit()
        self.opened = []  # The OAK and Hailo-8 factories called, in order
        self.oak_threads = []  # Where each OAK was made: the main thread, or a worker
        self.hailo_fails = hailo_fails  # Opens of a Hailo-8 that is there that fail first, as when another program holds it

    def hardware(self) -> Hardware:
        def oak_camera(*, pubsub, params):
            self.opened.append("oak_camera")
            self.oak_threads.append("main" if threading.current_thread() is threading.main_thread() else "worker")
            if self.oak_fails:
                raise RuntimeError("X_LINK_DEVICE_NOT_FOUND")
            if not self.oak:
                raise RuntimeError("No available devices")
            return FakeOak(pubsub=pubsub, **self.oak_options)

        def hailo_pose(*, pubsub, params):
            self.opened.append("hailo")
            if not self.hailo or self.hailo_fails > 0:
                self.hailo_fails -= 1
                raise RuntimeError("HAILO_OUT_OF_PHYSICAL_DEVICES")
            return Idle(pubsub=pubsub)

        def stereo_depth(*, pubsub, params, eye_size):
            if ZED_VIDEO_USB not in self.usb:  # Its calibration is found by the camera's serial number
                raise RuntimeError("The ZED reports no serial number, so its calibration file is unknown")
            return Idle(pubsub=pubsub)

        def zed_camera(*, pubsub, params, wait=False):
            if not self.zed and not wait:
                raise RuntimeError("No ZED camera among the video devices.")
            return FakeZed(pubsub=pubsub, source="opened" if self.zed else None, opens_after=self.zed_opens_after)

        def zed_imu(*, pubsub, params, wait=False):
            if not self.imu and not wait:
                raise OSError("No ZED IMU")
            return FakeImu(pubsub=pubsub)

        def servo_board():
            if not self.servo_board:
                raise ValueError("No I2C device at address: 0x40")
            return self.servo_kit

        def motor_board(pwm_frequency):
            if not self.motor_board:
                raise ValueError("No I2C device at address: 0x60")
            self.motor_kit.pwm_frequency = pwm_frequency
            return self.motor_kit

        return Hardware(
            oak_camera=oak_camera, hailo_pose=hailo_pose, zed_camera=zed_camera, zed_imu=zed_imu,
            stereo_depth=stereo_depth,
            servo_board=servo_board, motor_board=motor_board,
            probes=Probes(usb=lambda: self.usb, hailo_device=lambda: self.hailo, i2c_bus=lambda: True),
            # The live checks find nothing to look at: these tests never touch a real bus or command
            sources=Sources(open_bus=lambda: None, usb=lambda: self.usb, hailo_device=lambda: self.hailo,
                            hailo_pci=lambda: self.hailo, imu_device=lambda: False, throttled=lambda: None,
                            pmic=lambda: None, kernel_log=lambda: None),
        )


def free_port() -> int:
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def start(robot: Robot, tmp_path, **options) -> BotLauncher:
    config = launch_config(zed=True, depth=options.pop("depth", "sgbm"), port=free_port(), **options)
    config.runtime_server.host = "127.0.0.1"
    if config.oak_camera.models is not None:
        config.oak_camera.models.model_cache = tmp_path / "oak-zoo"  # Empty: no models downloaded
    return BotLauncher.from_config(config, hardware=robot.hardware())


def statuses(launcher: BotLauncher) -> dict[str, str]:
    return {component["id"]: component["status"] for component in launcher.health.snapshot()}


def test_with_nothing_plugged_in_the_bot_starts_and_says_what_is_missing(tmp_path):
    nothing = Robot(oak=False, hailo=False, zed=False, imu=False, servo_board=False, motor_board=False)
    launcher = start(nothing, tmp_path)

    assert statuses(launcher) == {
        "oak_camera": "missing", "oak_models": "missing", "hailo": "missing", "zed_camera": "missing",
        "zed_imu": "missing", "depth": "degraded", "servo_board": "missing", "motor_board": "missing",
    }
    depth = launcher.health.get("depth")  # Its calibration comes from the camera: not "no serial number"
    assert (depth.message, depth.because) == ("Starts once the ZED Mini streams", "zed_camera")
    oak, hailo = launcher.health.get("oak_camera"), launcher.health.get("hailo")
    assert oak.message == "No OAK on USB (no 03e7 device)" and "USB 3 port" in oak.hint
    assert "AI HAT+" in hailo.hint and "nothing is tracked" in hailo.hint
    assert "i2cdetect -y 1 should then show 40" in launcher.health.get("servo_board").hint
    names = [type(node).__name__ for node in launcher.nodes]
    assert "FakeOak" not in names and names.count("Idle") == 0  # No detector, and depth waits for the ZED
    assert "FakeZed" in names and "FakeImu" in names  # They wait for their device and pick it up
    assert {"OrientationNode", "MouthNode", "RuntimeServerNode", "StallMonitorNode"} <= set(names)


def test_a_missing_servo_board_leaves_the_waist_and_jaw_driven(tmp_path):
    robot = Robot(servo_board=False)
    launcher = start(robot, tmp_path)
    orientation = next(node for node in launcher.nodes if type(node).__name__ == "OrientationNode")

    orientation.waist_motor.throttle = 0.0  # Reaches the Motor HAT: motor 1, channels 9 and 10, in one transfer
    assert robot.motor_kit._pca.i2c_device.transfers[-1][:1] == bytes([0x06 + 4 * 9])
    assert isinstance(orientation.servo_kit, SimulatedServoKit)  # Recorded, never sent
    assert launcher.command_recorder.latest["pan"].value == 90.0
    with pytest.raises(MotorTestRefused, match="does not answer at 0x40, so nothing would move"):
        launcher.motor_tester.check(MotorTestRequest(actuator="pan", angle=95.0))
    launcher.motor_tester.check(MotorTestRequest(actuator="waist", throttle=0.3, seconds=0.2))  # Allowed


def test_a_missing_motor_hat_leaves_the_head_driven(tmp_path):
    robot = Robot(motor_board=False)
    launcher = start(robot, tmp_path)

    pan, tilt = launcher.config.orientation.pan_channel, launcher.config.orientation.tilt_channel
    assert robot.servo_kit.servo[pan].angle == robot.servo_kit.servo[tilt].angle == 90.0  # Sent to the servo board
    assert robot.motor_kit._pca.i2c_device.transfers == []
    assert statuses(launcher)["motor_board"] == "missing"
    with pytest.raises(MotorTestRefused, match="Motor HAT does not answer at 0x60"):
        launcher.motor_tester.check(MotorTestRequest(actuator="mouth", throttle=0.5, seconds=0.1))


def test_a_zed_showing_only_its_imu_says_to_flip_its_plug(tmp_path):
    launcher = start(Robot(zed=False, usb={OAK_USB, ZED_HID_USB}), tmp_path)

    zed = launcher.health.get("zed_camera")
    assert zed.status == Status.MISSING and "Flip the ZED's USB-C plug at the camera end" in zed.hint


def test_an_oak_on_usb_that_fails_is_an_error_not_missing(tmp_path):
    robot = Robot(oak=False, oak_fails=True)
    launcher = start(robot, tmp_path)

    oak = launcher.health.get("oak_camera")
    assert oak.status == Status.ERROR and "X_LINK_DEVICE_NOT_FOUND" in oak.message
    assert oak.detail == {"error": "RuntimeError: X_LINK_DEVICE_NOT_FOUND"}
    assert robot.opened == ["oak_camera", "hailo"]


def test_an_oak_or_hailo_that_is_not_there_is_not_searched_for(tmp_path):
    robot = Robot(oak=False, hailo=False)  # depthai would search USB for 11 s before giving up
    launcher = start(robot, tmp_path)

    assert robot.opened == []
    assert launcher.health.get("oak_camera").message == "No OAK on USB (no 03e7 device)"
    assert launcher.health.get("hailo").message == "No Hailo-8 (no /dev/hailo0)"


def test_parts_that_wait_for_another_name_it(tmp_path, quick, monkeypatch):
    monkeypatch.setattr(bot_launcher, "missing_models", lambda cache: {})  # Every OAK model downloaded
    launcher = start(Robot(oak=False, zed=False, imu=False), tmp_path)  # The Hailo-8 in, the OAK and ZED out

    async def scenario():
        task = await run_for(launcher, 0.4)
        found = {component["id"]: component for component in launcher.health.snapshot()}
        await stop(launcher, task)
        return found

    found = asyncio.run(scenario())
    because = {name: (part["status"], part["because"]) for name, part in found.items()}
    assert because == {
        "oak_camera": ("missing", None), "oak_models": ("degraded", "oak_camera"), "hailo": ("degraded", "oak_camera"),
        "zed_camera": ("missing", None), "zed_imu": ("missing", "zed_camera"), "depth": ("degraded", "zed_camera"),
        "servo_board": ("ok", None), "motor_board": ("ok", None),
    }
    assert found["zed_imu"]["hint"] is None and found["depth"]["hint"] is None  # The ZED's own hint applies
    assert found["hailo"]["message"] == "Waiting for the OAK camera's frames"


def test_stereonet_without_the_hailo_reports_depth_missing(tmp_path):
    robot = Robot(hailo=False)
    robot_hardware = robot.hardware()

    def stereonet(*, pubsub, params, eye_size):
        raise RuntimeError("HAILO_OUT_OF_PHYSICAL_DEVICES")

    config = launch_config(zed=True, depth="stereonet", port=free_port())
    config.oak_camera.models.model_cache = tmp_path
    robot_hardware.stereo_depth = stereonet
    launcher = BotLauncher.from_config(config, hardware=robot_hardware)

    depth = launcher.health.get("depth")
    assert depth.status == Status.MISSING and "--depth, which computes depth on the CPU" in depth.hint
    assert depth.because == "hailo"


def test_simulated_motors_are_reported_off_and_not_opened(tmp_path):
    robot = Robot(servo_board=False, motor_board=False)
    launcher = start(robot, tmp_path, simulate_motors=True)

    assert statuses(launcher)["servo_board"] == statuses(launcher)["motor_board"] == "off"
    assert "--simulate-motors" in launcher.health.get("motor_board").message
    launcher.motor_tester.check(MotorTestRequest(actuator="pan", angle=95.0))  # Simulated tests still run


def test_the_oak_models_report_what_is_downloaded_while_the_oak_is_out(tmp_path):
    launcher = start(Robot(oak=False), tmp_path)
    assert launcher.health.get("oak_models").message.startswith("Not downloaded: face_detection, ")
    assert "scripts.fetch_oak_models" in launcher.health.get("oak_models").hint

    off = start(Robot(), tmp_path, oak_models=False)
    assert off.health.get("oak_models").status == Status.OFF


def test_oak_models_fetched_while_the_bot_runs_are_seen(tmp_path, monkeypatch):
    launcher = start(Robot(oak=False), tmp_path)  # The test's model cache is empty
    monkeypatch.setattr(bot_launcher, "missing_models", lambda cache: {})  # Fetched meanwhile
    launcher._report_oak_models()
    assert launcher.health.get("oak_models").status == Status.MISSING  # The cache is read again only now and then

    monkeypatch.setattr(bot_launcher, "MODELS_RECHECK", 0.0)
    launcher._report_oak_models()
    models = launcher.health.get("oak_models")
    assert (models.status, models.message, models.because) == (
        Status.DEGRADED, "Downloaded; they run once the OAK streams", "oak_camera")


@pytest.fixture
def quick(monkeypatch):
    """Check health every 20 ms, a camera counts as streaming within 0.2 s of its last data."""
    monkeypatch.setattr(bot_launcher, "HEALTH_INTERVAL", 0.02)
    monkeypatch.setattr(bot_launcher, "STREAMING_WINDOW", 0.2)
    monkeypatch.setattr(bot_launcher, "RECOVERY_INTERVAL", 0.05)


def fast_stalls(launcher: BotLauncher) -> StallMonitorNode:
    monitor = next(node for node in launcher.nodes if isinstance(node, StallMonitorNode))
    monitor.params = StallMonitorParams(frame_timeout_seconds=0.2, pose_timeout_seconds=0.2, startup_seconds=0.5,
                                        check_interval_seconds=0.02)
    return monitor


async def run_for(launcher: BotLauncher, seconds: float) -> asyncio.Task:
    """Run the bot for `seconds` and return its task, still running unless it ended."""
    task = asyncio.create_task(launcher.run())
    await asyncio.sleep(seconds)
    return task


async def stop(launcher: BotLauncher, task: asyncio.Task) -> None:
    next(node for node in launcher.nodes if isinstance(node, StallMonitorNode)).stop()  # A core node: ends the bot
    await asyncio.wait_for(task, timeout=5)


def test_an_oak_unplugged_while_running_is_reported_and_the_bot_runs_on(tmp_path, quick):
    robot = Robot(oak_options={"crash_after": 0.3})
    launcher = start(robot, tmp_path)
    fast_stalls(launcher)

    async def scenario():
        task = await run_for(launcher, 0.2)
        robot.usb.discard(OAK_USB)  # The cable comes out, and the OAK node stops with depthai's error
        await asyncio.sleep(1.0)  # Five frame timeouts: not a freeze, since the OAK counts as gone
        assert not task.done()
        oak = launcher.health.get("oak_camera")
        await stop(launcher, task)
        return oak

    oak = asyncio.run(scenario())
    assert oak.status == Status.MISSING and oak.message == "No OAK on USB (no 03e7 device)"
    assert "camera unplugged?" in oak.detail["error"]


def test_a_zed_plugged_in_later_is_picked_up(tmp_path, quick):
    robot = Robot(zed=False, zed_opens_after=0.5)
    launcher = start(robot, tmp_path)

    async def scenario():
        task = await run_for(launcher, 0.3)
        before = launcher.health.get("zed_camera").status
        robot.usb |= {ZED_VIDEO_USB, ZED_HID_USB}
        await asyncio.sleep(0.6)
        after = launcher.health.get("zed_camera"), launcher.health.get("depth")
        await stop(launcher, task)
        return before, after

    before, (zed, depth) = asyncio.run(scenario())
    assert before == Status.MISSING
    assert zed.status == Status.OK and zed.message == "Streaming both eyes"
    assert (depth.status, depth.message) == (Status.OK, "Depth from the ZED's eyes (sgbm)")  # Started once it streamed


def test_an_oak_plugged_in_later_is_brought_back(tmp_path, quick):
    robot = Robot(oak=False, oak_options={"assembler": "the OAK's models"})
    launcher = start(robot, tmp_path)

    async def scenario():
        task = await run_for(launcher, 0.2)
        before = launcher.health.get("oak_camera").status, launcher._oak_status()
        robot.oak = True
        robot.usb.add(OAK_USB)
        await asyncio.sleep(0.5)
        after = {name: launcher.health.get(name) for name in ("oak_camera", "hailo")}
        seen = launcher._oak_status(), launcher.server.results_camera, [type(node).__name__ for node in launcher.nodes]
        await stop(launcher, task)
        return before, after, seen

    (oak_before, status_before), after, (status, results_camera, names) = asyncio.run(scenario())
    assert oak_before == Status.MISSING and status_before is None
    assert (after["oak_camera"].status, after["oak_camera"].message) == (Status.OK, "Streaming")
    assert after["hailo"].status == Status.OK and after["hailo"].because is None  # Frames to look at again
    assert status == {"rates": {"frames": 30.0}} and results_camera == "oak"  # The dashboard follows the new OAK
    assert names.count("FakeOak") == 1
    assert robot.oak_threads == ["worker"]  # Made off the loop: the motors' control loop ran on meanwhile


def test_an_oak_unplugged_and_plugged_back_comes_back_in_its_place(tmp_path, quick):
    robot = Robot(oak_options={"crash_after": 0.3})
    launcher = start(robot, tmp_path)

    async def scenario():
        task = await run_for(launcher, 0.2)
        robot.usb.discard(OAK_USB)  # The cable comes out, and the OAK node stops with depthai's error
        await asyncio.sleep(0.4)
        gone = launcher.health.get("oak_camera").status
        robot.oak_options = {}  # The next one runs until stopped
        robot.usb.add(OAK_USB)
        await asyncio.sleep(0.4)
        back = launcher.health.get("oak_camera").status
        states = [entry["state"] for entry in launcher.node_status() if entry["name"] == "FakeOak"]
        await stop(launcher, task)
        return gone, back, states

    gone, back, states = asyncio.run(scenario())
    assert gone == Status.MISSING and back == Status.OK
    assert states == ["running"] and robot.opened.count("oak_camera") == 2  # Replaced, not added beside the old one


def test_a_hailo_held_by_another_program_is_picked_up_once_free(tmp_path, quick):
    robot = Robot(hailo_fails=2)
    launcher = start(robot, tmp_path)
    assert launcher.health.get("hailo").status == Status.ERROR  # /dev/hailo0 is there, so not missing

    async def scenario():
        task = await run_for(launcher, 0.8)
        hailo = launcher.health.get("hailo")
        await stop(launcher, task)
        return hailo

    hailo = asyncio.run(scenario())
    assert (hailo.status, hailo.message) == (Status.OK, "Finds people in the OAK's frames")
    assert robot.opened.count("hailo") == 3


def test_an_oak_that_keeps_stopping_is_tried_less_and_less_often(tmp_path, quick):
    robot = Robot(oak_options={"crash_after": 0.0})  # It opens, then stops at once, every time (a bad cable)
    launcher = start(robot, tmp_path)

    async def scenario():
        task = await run_for(launcher, 1.0)
        await stop(launcher, task)

    asyncio.run(scenario())
    # Without the waits: about 20 opens, one every 0.05 s. With them, 0.05 s doubling: 1 at startup and about 4 more
    assert 3 <= robot.opened.count("oak_camera") <= 6
    assert launcher.health.get("oak_camera").status == Status.ERROR  # On USB, but failing


def test_a_part_that_fails_to_start_waits_longer_each_time(tmp_path, monkeypatch):
    monkeypatch.setattr(bot_launcher, "RECOVERY_INTERVAL", 5.0)
    monkeypatch.setattr(bot_launcher, "RETRY_LIMIT", 30.0)
    robot = Robot(hailo_fails=100)
    launcher = start(robot, tmp_path)
    revival = next(revival for revival in launcher.revivals if revival.component == "hailo")

    async def tries() -> list[int]:
        waits = []
        for _ in range(5):
            before = time.monotonic()
            await launcher._revive(revival)
            waits.append(round(revival.next_try - before))
            revival.next_try = 0.0  # Due again at once
        return waits

    assert asyncio.run(tries()) == [5, 10, 20, 30, 30]
    assert launcher.health.get("hailo").status == Status.ERROR
    robot.hailo = False  # Gone: once it is back, it is tried at once
    asyncio.run(launcher._revive(revival))
    assert (revival.failures, revival.next_try) == (0, 0.0)


def test_a_stream_that_freezes_while_its_camera_counts_as_working_still_restarts_the_bot(tmp_path, quick):
    launcher = start(Robot(oak_options={"quiet_after": 0.3}), tmp_path)
    fast_stalls(launcher)

    async def scenario():
        await asyncio.wait_for(launcher.run(), timeout=5)

    with pytest.raises(StreamStalledError, match="No camera frames for"):
        asyncio.run(scenario())
    assert not launcher.healthy()


def test_health_goes_to_every_viewer(tmp_path):
    import cbor2
    from aiohttp.test_utils import TestClient, TestServer

    launcher = start(Robot(oak=False, hailo=False), tmp_path)
    server = next(node for node in launcher.nodes if type(node).__name__ == "RuntimeServerNode")

    async def scenario():
        pump = asyncio.create_task(server.pump())
        async with TestClient(TestServer(server.build_app())) as client:
            socket = await client.ws_connect("/stream")
            await socket.send_str('{"frameNumber": 1, "displayImageSizes": {}, "kinds": []}')  # A hidden page
            while True:
                message = cbor2.loads((await asyncio.wait_for(socket.receive(), timeout=5)).data)
                if message["kind"] == "health":
                    break
            await socket.close()
        server.stop()
        await pump
        return message

    message = asyncio.run(scenario())
    components = {component["id"]: component for component in message["components"]}
    assert components["oak_camera"]["status"] == "missing" and components["hailo"]["status"] == "missing"
    assert set(components["oak_camera"]) == {"id", "label", "group", "status", "message", "hint", "because",
                                             "since", "restart", "detail"}
    assert message["time"] >= components["oak_camera"]["since"]


def test_since_moves_only_when_the_status_changes():
    now = [10.0]
    health = Health(clock=lambda: now[0])
    health.report("hailo", Status.MISSING, "No Hailo-8")
    now[0] = 15.0
    version = health.version
    health.report("hailo", Status.MISSING, "No Hailo-8 (no /dev/hailo0)")  # New words, same status
    assert health.get("hailo").since == 10.0 and health.version == version + 1
    health.report("hailo", Status.MISSING, "No Hailo-8 (no /dev/hailo0)")  # Nothing new
    assert health.version == version + 1
    health.report("hailo", Status.OK, "Finds people")
    assert health.get("hailo").since == 15.0


def test_usb_devices_reads_linuxs_device_list(tmp_path):
    for name, ids in {"2-1": ("03e7", "2485"), "4-1": ("2b03", "f681"), "4-1.0": None}.items():  # An interface
        (tmp_path / name).mkdir()
        if ids:
            (tmp_path / name / "idVendor").write_text(ids[0] + "\n")
            (tmp_path / name / "idProduct").write_text(ids[1] + "\n")

    assert usb_devices(tmp_path) == {"03e7:2485", "2b03:f681"}
    assert usb_devices(tmp_path / "absent") == set()
