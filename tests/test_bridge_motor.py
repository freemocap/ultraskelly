import struct
import types

import pytest

from ultraskelly.core.bot.motor.bridge_motor import (
    FAST_DECAY,
    SLOW_DECAY,
    BridgeMotor,
    bridge_duty_cycles,
    channel_registers,
)

THROTTLES = (None, 0.0, 0.003, 0.3, -0.3, 0.75, -0.75, 1.0, -1.0)


class Device:
    """An I2CDevice stand-in that keeps every transfer, or fails the next `failures` of them."""

    def __init__(self, failures: int = 0):
        self.transfers, self.failures = [], failures

    def __enter__(self):
        return self

    def __exit__(self, *exception):
        return False

    def write(self, buffer, *, start=0, end=None):
        if self.failures:
            self.failures -= 1
            raise OSError(11, "Resource temporarily unavailable")
        self.transfers.append(bytes(buffer[start:end]))


def test_channel_registers_follow_adafruit_pca9685():
    assert channel_registers(0xFFFF) == struct.pack("<HH", 0x1000, 0)  # Fully on
    assert channel_registers(0) == channel_registers(0x000F) == struct.pack("<HH", 0, 0x1000)  # Fully off
    assert channel_registers(0x8000) == struct.pack("<HH", 0, 0x800)
    with pytest.raises(ValueError):
        channel_registers(0x10000)


def test_bridge_duty_cycles_follow_adafruit_motor():
    assert bridge_duty_cycles(None, FAST_DECAY) == bridge_duty_cycles(None, SLOW_DECAY) == (0, 0)  # Coast
    assert bridge_duty_cycles(0.0, FAST_DECAY) == bridge_duty_cycles(0.0, SLOW_DECAY) == (0xFFFF, 0xFFFF)  # Brake
    duty = int(0xFFFF * 0.5)
    assert bridge_duty_cycles(0.5, FAST_DECAY) == (duty, 0) and bridge_duty_cycles(-0.5, FAST_DECAY) == (0, duty)
    assert bridge_duty_cycles(0.5, SLOW_DECAY) == (0xFFFF, 0xFFFF - duty)
    assert bridge_duty_cycles(-0.5, SLOW_DECAY) == (0xFFFF - duty, 0xFFFF)


def test_both_inputs_go_in_one_transfer():
    device = Device()
    motor = BridgeMotor(device, positive_channel=9)  # Motor 1 of the DC & Stepper Motor HAT
    motor.throttle = 0.0
    assert device.transfers == [bytes([0x06 + 4 * 9]) + struct.pack("<HHHH", 0x1000, 0, 0x1000, 0)]
    motor.decay_mode = SLOW_DECAY
    motor.throttle = -0.5
    assert len(device.transfers) == 2 and len(device.transfers[1]) == 9 and motor.throttle == -0.5


def test_a_failed_transfer_keeps_the_last_throttle_and_bad_values_are_refused():
    device = Device()
    motor = BridgeMotor(device, positive_channel=11)
    motor.throttle = 0.5
    device.failures = 1
    with pytest.raises(OSError):
        motor.throttle = 0.0
    assert motor.throttle == 0.5
    for bad in (1.5, -1.01):
        with pytest.raises(ValueError):
            motor.throttle = bad
    with pytest.raises(ValueError):
        motor.decay_mode = 2


def test_from_motorkit_uses_the_kits_channels_and_board():
    channel = lambda index: types.SimpleNamespace(_index=index)
    device = Device()
    kit = types.SimpleNamespace(
        motor2=types.SimpleNamespace(_positive=channel(11), _negative=channel(12)),
        motor9=types.SimpleNamespace(_positive=channel(4), _negative=channel(3)),
        _pca=types.SimpleNamespace(i2c_device=device),
    )
    motor = BridgeMotor.from_motorkit(kit, 2)
    assert motor.register == 0x06 + 4 * 11 and motor.i2c_device is device
    with pytest.raises(ValueError, match="neighbours"):
        BridgeMotor.from_motorkit(kit, 9)


class Bus:
    """A busio.I2C stand-in for one PCA9685: a register file with auto-increment, counting transfers."""

    def __init__(self):
        self.registers, self.transfers = bytearray(256), 0

    def try_lock(self):
        return True

    def unlock(self):
        pass

    def writeto(self, address, buffer, *, start=0, end=None):
        data = bytes(buffer[start:end])
        if data:
            self.transfers += 1
            self.registers[data[0]:data[0] + len(data) - 1] = data[1:]

    def readfrom_into(self, address, buffer, *, start=0, end=None):
        pass

    def writeto_then_readfrom(self, address, out_buffer, in_buffer, *, out_start=0, out_end=None, in_start=0, in_end=None):
        register = out_buffer[out_start]
        end = len(in_buffer) if in_end is None else in_end
        in_buffer[in_start:end] = self.registers[register:register + end - in_start]


@pytest.mark.parametrize("decay", [FAST_DECAY, SLOW_DECAY])
def test_registers_match_adafruit_motor_on_a_real_pca9685_driver(decay):
    pca9685 = pytest.importorskip("adafruit_pca9685")  # The hardware extra, installed on the Pi
    motor_module = pytest.importorskip("adafruit_motor.motor")
    theirs_bus, ours_bus = Bus(), Bus()
    theirs_board = pca9685.PCA9685(theirs_bus, address=0x60)
    ours_board = pca9685.PCA9685(ours_bus, address=0x60)
    theirs = motor_module.DCMotor(theirs_board.channels[9], theirs_board.channels[10])
    ours = BridgeMotor(ours_board.i2c_device, 9)
    theirs.decay_mode = ours.decay_mode = decay
    block = slice(0x06 + 4 * 9, 0x06 + 4 * 11)
    for throttle in THROTTLES:
        before = theirs_bus.transfers, ours_bus.transfers
        theirs.throttle = ours.throttle = throttle
        assert ours_bus.registers[block] == theirs_bus.registers[block], throttle
        assert (theirs_bus.transfers - before[0], ours_bus.transfers - before[1]) == (2, 1)
