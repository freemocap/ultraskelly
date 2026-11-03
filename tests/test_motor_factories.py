import sys
import types

import pytest

from ultraskelly.core.bot.motor.mouth_node import MouthNode, MouthNodeParams
from ultraskelly.core.bot.motor.orientation_node import OrientationNode, OrientationNodeParams
from ultraskelly.core.bot.motor.waist_drive import WaistDriveParams
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

FAST_DECAY, SLOW_DECAY = 0, 1


class Motor:
    def __init__(self):
        self.throttle, self.decay_mode = None, FAST_DECAY  # adafruit_motor's default


class Servo:
    angle = None


@pytest.fixture
def boards(monkeypatch):
    """Stand-ins for the board libraries the factories import; returns the MotorKits they made."""
    kits = []

    class MotorKit:
        def __init__(self, address, pwm_frequency=1600):
            self.address, self.pwm_frequency = address, pwm_frequency
            self.motor1, self.motor2, self.motor3, self.motor4 = Motor(), Motor(), Motor(), Motor()
            kits.append(self)

    class ServoKit:
        def __init__(self, channels):
            self.servo = [Servo() for _ in range(channels)]

    motor_module = types.SimpleNamespace(FAST_DECAY=FAST_DECAY, SLOW_DECAY=SLOW_DECAY, DCMotor=Motor)
    monkeypatch.setitem(sys.modules, "adafruit_motor", types.SimpleNamespace(motor=motor_module))
    monkeypatch.setitem(sys.modules, "adafruit_motor.motor", motor_module)
    monkeypatch.setitem(sys.modules, "adafruit_motorkit", types.SimpleNamespace(MotorKit=MotorKit))
    monkeypatch.setitem(sys.modules, "adafruit_servokit", types.SimpleNamespace(ServoKit=ServoKit))
    return kits


def test_both_motor_factories_set_the_same_board_frequency_and_the_waist_brakes_between_pulses(boards):
    bus = PubSubTopicManager.create()
    orientation = OrientationNode.create(pubsub=bus, params=OrientationNodeParams(), pwm_frequency=100.0)
    mouth = MouthNode.create(pubsub=bus, params=MouthNodeParams(), pwm_frequency=100.0)
    assert [kit.pwm_frequency for kit in boards] == [100.0, 100.0]
    assert orientation.waist_motor.decay_mode == SLOW_DECAY
    assert mouth.mouth_motor.decay_mode == FAST_DECAY  # The jaw runs at full drive or none: no PWM to shape
    assert orientation.servo_kit.servo[11].angle == orientation.servo_kit.servo[3].angle == 90.0
    bus.close()


def test_the_waist_can_go_back_to_fast_decay(boards):
    bus = PubSubTopicManager.create()
    params = OrientationNodeParams(waist=WaistDriveParams(slow_decay=False))
    orientation = OrientationNode.create(pubsub=bus, params=params)
    assert orientation.waist_motor.decay_mode == FAST_DECAY and boards[0].pwm_frequency == 1600.0
    bus.close()
