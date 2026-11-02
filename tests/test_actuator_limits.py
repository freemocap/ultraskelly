import pytest

from ultraskelly.core.bot.motor.actuator_limits import ActuatorLimits
from ultraskelly.core.bot.motor.mouth_node import MouthNodeParams
from ultraskelly.core.bot.motor.orientation_node import OrientationNodeParams
from ultraskelly.core.bot.motor.simulated_actuators import (
    CommandRecorder,
    RecordingMotor,
    RecordingServoKit,
    create_simulated_motor_nodes,
)
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

LIMITS = ActuatorLimits(
    pan_min_degrees=30, pan_max_degrees=150, tilt_min_degrees=60, tilt_max_degrees=120,
    waist_max_throttle=0.5, mouth_max_throttle=0.6,
)


def build(*, limits=LIMITS, **mouth_options):
    recorder, bus = CommandRecorder(), PubSubTopicManager.create()
    orientation, mouth = create_simulated_motor_nodes(
        pubsub=bus, orientation_params=OrientationNodeParams(), mouth_params=MouthNodeParams(**mouth_options),
        recorder=recorder, limits=limits,
    )
    return recorder, orientation, mouth, bus


class Board:
    """A motor board channel that keeps whatever it was sent."""
    angle = None
    throttle = None


class BoardKit:
    def __init__(self):
        self.servo = [Board() for _ in range(16)]


def test_simulated_commands_are_clamped_to_the_limits():
    recorder, orientation, mouth, bus = build()
    orientation.servo_kit.servo[11].angle = 200.0
    orientation.servo_kit.servo[3].angle = -5.0
    orientation.waist_motor.throttle = 0.9
    mouth.mouth_motor.throttle = -1.0
    bus.close()

    latest = {name: command.value for name, command in recorder.latest.items()}
    assert (latest["pan"], latest["tilt"], latest["waist"], latest["mouth"]) == (150.0, 60.0, 0.5, -0.6)


def test_hardware_commands_are_clamped_before_they_reach_the_board():
    recorder, board_kit, board_motor = CommandRecorder(), BoardKit(), Board()
    kit = RecordingServoKit(kit=board_kit, recorder=recorder, names={11: "pan"}, limits=LIMITS)
    motor = RecordingMotor(motor=board_motor, name="waist", recorder=recorder, max_throttle=LIMITS.waist_max_throttle)

    kit.servo[11].angle = 10.0
    motor.throttle = -1.0
    assert (board_kit.servo[11].angle, board_motor.throttle) == (30.0, -0.5)

    kit.servo[11].angle = None  # Releasing a servo and coasting a motor stay possible
    motor.throttle = None
    assert (board_kit.servo[11].angle, board_motor.throttle) == (None, None)


def test_startup_drives_no_dc_motor_unless_the_jaw_test_is_enabled(monkeypatch):
    monkeypatch.setattr("ultraskelly.core.bot.motor.mouth_node.time.sleep", lambda seconds: None)

    quiet, *_, quiet_bus = build()
    tested, *_, tested_bus = build(startup_test=True)
    quiet_bus.close()
    tested_bus.close()

    assert [command for command in quiet.history if command.actuator in ("waist", "mouth")] == []
    assert [command.value for command in tested.history if command.actuator == "mouth"] == [0.6, -0.6, 0.0]


def test_default_limits_match_the_controllers_existing_travel():
    limits = ActuatorLimits()

    assert limits.servo_range("pan") == limits.servo_range("tilt") == (0.0, 180.0)
    assert limits.waist_max_throttle == OrientationNodeParams().waist_max_throttle
    assert limits.mouth_max_throttle == 1.0
    assert limits.servo_range("servo5") is None and limits.max_throttle("motor3") is None


def test_inverted_ranges_are_rejected():
    with pytest.raises(ValueError, match="minimum angle"):
        ActuatorLimits(pan_min_degrees=120, pan_max_degrees=60)
