import asyncio
import subprocess
import sys

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
)
from ultraskelly.core.pubsub.bot_topics import ServoStateTopic, TargetLocationMessage, TargetLocationTopic
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager


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
        head.clock = mouth.clock = lambda: 0.0
        assert recorder.latest["pan"].value == recorder.latest["tilt"].value == 90.0

        await bus.publish(TargetLocationTopic, TargetLocationMessage(x=640, y=0))
        await head.step()
        await mouth.step()

        assert recorder.latest["pan"].value == head.pan_angle == 106
        assert recorder.latest["tilt"].value == head.tilt_angle == 78
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
        head.clock = mouth.clock = lambda: 0.0
        assert {name: command.value for name, command in recorder.latest.items()} == {
            "pan": 90.0,
            "tilt": 90.0,
            "mouth": 0.0,
        }

        await bus.publish(TargetLocationTopic, TargetLocationMessage(x=640, y=0))
        await head.step()
        await mouth.step()

        assert board_kit.servo[11].written[-1] == recorder.latest["pan"].value == 106
        assert board_kit.servo[3].written[-1] == recorder.latest["tilt"].value == 78
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
