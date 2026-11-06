import struct
import sys
import types

import pytest

from ultraskelly.core.bot.motor.bridge_motor import FAST_DECAY, SLOW_DECAY, BridgeMotor
from ultraskelly.core.bot.motor.mouth_node import MouthNode, MouthNodeParams
from ultraskelly.core.bot.motor.orientation_node import OrientationNode, OrientationNodeParams
from ultraskelly.core.bot.motor.waist_drive import WaistDriveParams
from ultraskelly.core.pubsub.pubsub_manager import PubSubTopicManager

MOTORKIT_CHANNELS = {1: (9, 10), 2: (11, 12), 3: (3, 4), 4: (5, 6)}  # adafruit_motorkit's bridge inputs


class Device:
    def __init__(self):
        self.transfers = []

    def __enter__(self):
        return self

    def __exit__(self, *exception):
        return False

    def write(self, buffer, *, start=0, end=None):
        self.transfers.append(bytes(buffer[start:end]))


class Servo:
    angle = None


@pytest.fixture
def boards(monkeypatch):
    """Stand-ins for the board libraries the factories import; returns the MotorKits they made."""
    kits = []

    class MotorKit:
        def __init__(self, address, pwm_frequency=1600):
            self.address, self.pwm_frequency = address, pwm_frequency
            self._pca = types.SimpleNamespace(i2c_device=Device())
            for number, (positive, negative) in MOTORKIT_CHANNELS.items():
                setattr(self, f"motor{number}", types.SimpleNamespace(
                    _positive=types.SimpleNamespace(_index=positive), _negative=types.SimpleNamespace(_index=negative),
                ))
            kits.append(self)

    class ServoKit:
        def __init__(self, channels):
            self.servo = [Servo() for _ in range(channels)]

    monkeypatch.setitem(sys.modules, "adafruit_motorkit", types.SimpleNamespace(MotorKit=MotorKit))
    monkeypatch.setitem(sys.modules, "adafruit_servokit", types.SimpleNamespace(ServoKit=ServoKit))
    return kits


def test_both_motor_factories_set_the_same_board_frequency_and_the_waist_brakes_between_pulses(boards):
    bus = PubSubTopicManager.create()
    orientation = OrientationNode.create(pubsub=bus, params=OrientationNodeParams(), pwm_frequency=100.0)
    mouth = MouthNode.create(pubsub=bus, params=MouthNodeParams(), pwm_frequency=100.0)
    assert [kit.pwm_frequency for kit in boards] == [100.0, 100.0]
    assert isinstance(orientation.waist_motor, BridgeMotor) and isinstance(mouth.mouth_motor, BridgeMotor)
    assert orientation.waist_motor.decay_mode == SLOW_DECAY
    assert mouth.mouth_motor.decay_mode == FAST_DECAY  # The jaw runs at full drive or none: no PWM to shape
    assert [orientation.servo_kit.servo[channel].angle for channel in orientation.params.servo_names] == [90.0] * 3
    mouth.mouth_motor.throttle = 0.0  # Motor 2: channels 11 and 12, braked in one transfer
    assert boards[1]._pca.i2c_device.transfers == [bytes([0x06 + 4 * 11]) + struct.pack("<HHHH", 0x1000, 0, 0x1000, 0)]
    bus.close()


def test_the_waist_can_go_back_to_fast_decay(boards):
    bus = PubSubTopicManager.create()
    params = OrientationNodeParams(waist=WaistDriveParams(slow_decay=False))
    orientation = OrientationNode.create(pubsub=bus, params=params)
    assert orientation.waist_motor.decay_mode == FAST_DECAY and boards[0].pwm_frequency == 1600.0
    assert orientation.waist_motor.register == 0x06 + 4 * 9  # Motor 1
    bus.close()
