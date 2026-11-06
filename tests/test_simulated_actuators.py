import asyncio
import errno
import subprocess
import sys

import pytest

from ultraskelly.core.bot.motor.mouth_node import MouthNode, MouthNodeParams
from ultraskelly.core.bot.motor.orientation_node import OrientationNode, OrientationNodeParams
from ultraskelly.core.bot.motor.simulated_actuators import (
    CommandRecorder,
    RecordingMotor,
    RecordingServoKit,
    SimulatedMotor,
    SimulatedServoKit,
    create_simulated_motor_nodes,
    record_hardware_commands,
    stop_board_motors,
)
from ultraskelly.core.pubsub.bot_topics import ServoStateTopic, TargetLocationMessage, TargetLocationTopic
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager


async def follow_corner(bus, head, mouth, seconds=0.5):
    """Step both controllers on a fake clock, a target at the frame's top-right corner every 30 ms."""
    now = [0.0]
    head.clock = mouth.clock = lambda: now[0]
    for tick in range(round(seconds * 100)):
        if tick % 3 == 0:
            await bus.publish(TargetLocationTopic, TargetLocationMessage(x=640, y=0, captured=now[0]))
        await head.step()
        await mouth.step()
        now[0] += 0.01


class BoardServo:
    """Hardware servo double that fails if anything reads it back over I2C."""

    def __init__(self):
        self.written = []

    @property
    def angle(self):
        raise AssertionError("angle was read back from the board")

    @angle.setter
    def angle(self, value):
        self.written.append(value)


class BoardServoKit:
    def __init__(self):
        self.servo = [BoardServo() for _ in range(16)]


class BoardMotor:
    """Hardware DC motor double that fails if anything reads it back over I2C."""

    def __init__(self):
        self.written = []

    @property
    def throttle(self):
        raise AssertionError("throttle was read back from the board")

    @throttle.setter
    def throttle(self, value):
        self.written.append(value)


def test_recorder_keeps_only_changes_in_a_bounded_history():
    now = [0.0]
    recorder = CommandRecorder(clock=lambda: now[0], history_size=3)
    for value in [0.0, 0.0, 0.5, 0.5, -0.5, 0.0, 0.7]:
        now[0] += 1
        recorder.record("waist", value)

    assert recorder.change_count == 5
    assert [(command.value, command.at) for command in recorder.history] == [(-0.5, 5.0), (0.0, 6.0), (0.7, 7.0)]
    assert recorder.latest["waist"].value == 0.7


def test_simulated_actuators_hold_and_record_commands():
    recorder = CommandRecorder(clock=lambda: 0.0)
    kit = SimulatedServoKit(recorder=recorder, names={11: "pan"})
    motor = SimulatedMotor(name="waist", recorder=recorder)

    kit.servo[11].angle = 120
    kit.servo[3].angle = 45
    motor.throttle = -0.8

    assert (kit.servo[11].angle, kit.servo[3].angle, motor.throttle) == (120, 45, -0.8)
    assert {name: command.value for name, command in recorder.latest.items()} == {
        "pan": 120,
        "servo3": 45,
        "waist": -0.8,
    }


def test_simulated_motor_nodes_record_controller_commands():
    async def scenario():
        bus = PubSubTopicManager.create()
        recorder = CommandRecorder()
        head, mouth = create_simulated_motor_nodes(
            pubsub=bus,
            orientation_params=OrientationNodeParams(),
            mouth_params=MouthNodeParams(),
            recorder=recorder,
        )
        assert recorder.latest["pan"].value == recorder.latest["tilt"].value == 90.0

        await follow_corner(bus, head, mouth)

        assert recorder.latest["pan"].value == head.pan_angle > 90
        assert recorder.latest["tilt"].value == head.tilt_angle < 90
        assert recorder.latest["mouth"].value == 0
        assert "waist" in recorder.latest
        bus.close()

    asyncio.run(scenario())


def test_recording_wrappers_forward_every_command_and_never_read_the_board():
    recorder = CommandRecorder(clock=lambda: 0.0)
    board_kit, board_motor = BoardServoKit(), BoardMotor()
    kit = RecordingServoKit(kit=board_kit, recorder=recorder, names={11: "pan"})
    motor = RecordingMotor(motor=board_motor, name="waist", recorder=recorder)

    kit.servo[11].angle = 100
    kit.servo[11].angle = 100
    motor.throttle = 0.75

    assert board_kit.servo[11].written == [100, 100]
    assert board_motor.written == [0.75]
    assert (kit.servo[11].angle, motor.throttle) == (100, 0.75)
    assert {name: command.value for name, command in recorder.latest.items()} == {"pan": 100, "waist": 0.75}


def test_record_hardware_commands_wraps_both_controllers():
    async def scenario():
        bus = PubSubTopicManager.create()
        board_kit, waist_board, mouth_board = BoardServoKit(), BoardMotor(), BoardMotor()
        head = OrientationNode(
            pubsub=bus,
            params=OrientationNodeParams(),
            servo_kit=board_kit,
            waist_motor=waist_board,
            target_subscription=bus.get_subscription(TargetLocationTopic),
        )
        mouth = MouthNode(
            pubsub=bus,
            params=MouthNodeParams(),
            mouth_motor=mouth_board,
            head_servo_subscription=bus.get_subscription(ServoStateTopic),
        )
        recorder = CommandRecorder()
        record_hardware_commands(orientation=head, mouth=mouth, recorder=recorder)
        assert {name: command.value for name, command in recorder.latest.items()} == {
            "pan": 90.0,
            "tilt": 90.0,
            "roll": 90.0,
            "mouth": 0.0,
        }

        await follow_corner(bus, head, mouth)

        assert board_kit.servo[head.params.pan_channel].written[-1] == recorder.latest["pan"].value == head.pan_angle > 90
        assert board_kit.servo[head.params.tilt_channel].written[-1] == recorder.latest["tilt"].value == head.tilt_angle < 90
        assert waist_board.written[-1] == recorder.latest["waist"].value
        assert mouth_board.written == [0]
        bus.close()

    asyncio.run(scenario())


def test_simulated_motor_nodes_load_no_motor_board_libraries():
    code = """
import sys
from ultraskelly.core.bot.motor.mouth_node import MouthNodeParams
from ultraskelly.core.bot.motor.orientation_node import OrientationNodeParams
from ultraskelly.core.bot.motor.simulated_actuators import CommandRecorder, create_simulated_motor_nodes
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager
create_simulated_motor_nodes(
    pubsub=PubSubTopicManager.create(), orientation_params=OrientationNodeParams(),
    mouth_params=MouthNodeParams(), recorder=CommandRecorder(),
)
assert not any(name.startswith(('adafruit_', 'board', 'busio')) for name in sys.modules)
"""
    subprocess.run([sys.executable, "-c", code], check=True, timeout=10)


class FlakyBoardMotor(BoardMotor):
    """A motor channel whose first `failures` writes fail like a disturbed I2C bus."""

    def __init__(self, failures):
        super().__init__()
        self.failures = failures

    @BoardMotor.throttle.setter
    def throttle(self, value):
        if self.failures:
            self.failures -= 1
            raise OSError(errno.EAGAIN, "Resource temporarily unavailable")
        self.written.append(value)


def test_motor_writes_are_retried_and_stops_hardest(monkeypatch):
    monkeypatch.setattr("ultraskelly.core.bot.motor.simulated_actuators.time.sleep", lambda seconds: None)
    recorder = CommandRecorder()
    board = FlakyBoardMotor(failures=2)
    motor = RecordingMotor(motor=board, name="waist", recorder=recorder)
    motor.throttle = 0.5
    assert board.written == [0.5] and motor.failed_writes == 2

    board.failures = 5  # More than a drive command retries, fewer than a stop does
    with pytest.raises(OSError):
        motor.throttle = 0.6
    assert motor.throttle == recorder.latest["waist"].value == 0.5  # A failed command is not recorded
    motor.throttle = 0.0
    assert board.written == [0.5, 0.0] and recorder.latest["waist"].value == 0.0


def test_unchanged_throttle_is_only_resent_to_refresh_the_board():
    now = [0.0]
    board = BoardMotor()
    motor = RecordingMotor(motor=board, name="waist", recorder=CommandRecorder(), clock=lambda: now[0])
    for tick in range(100):  # One second of a 100 Hz controller holding one throttle
        now[0] = tick * 0.01
        motor.throttle = 0.7
    motor.throttle = 0.65
    assert board.written == [0.7, 0.7, 0.65]


def test_stopping_all_motors_tries_each_before_raising(monkeypatch):
    monkeypatch.setattr("ultraskelly.core.bot.motor.simulated_actuators.time.sleep", lambda seconds: None)
    dead, healthy = FlakyBoardMotor(failures=100), BoardMotor()
    with pytest.raises(OSError):
        stop_board_motors({"motor1": dead, "motor2": healthy})
    assert healthy.written == [0.0]
