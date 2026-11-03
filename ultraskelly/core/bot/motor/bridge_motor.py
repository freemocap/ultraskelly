"""Drive a motor board's DC motor with both of its bridge inputs set in one I2C transfer.

adafruit_motor's DCMotor sets a motor's two bridge inputs, two PCA9685 channels, in two
transfers. Between them the bridge is in a mixed state, and a brake (both inputs fully on)
first turns one input fully on while the other is still low, which is full drive one way;
a bus that fails just then leaves the motor there. The PCA9685 changes its outputs when a
transfer ends, however many PWM registers it set (MODE2's OCH bit, 0 by default; NXP
PCA9685 data sheet Rev. 4, Table 6), and the Adafruit motor boards wire each motor's two
inputs to neighbouring channels. So one transfer can carry both channels' registers, and
both outputs switch together. BridgeMotor writes exactly the register values that
adafruit_motor and adafruit_pca9685 would write, in that one transfer.
"""
import struct

FAST_DECAY, SLOW_DECAY = 0, 1  # Same values as adafruit_motor.motor's
LED0_ON_L = 0x06  # First PWM register; each channel has four: ON and OFF counts, 16-bit little-endian
FULL = 0x1000  # Bit 12 of an ON or OFF count: the channel is fully on, or fully off


def channel_registers(duty_cycle: int) -> bytes:
    """A channel's four PWM register bytes for a 16-bit duty cycle, as adafruit_pca9685 writes them."""
    if not 0 <= duty_cycle <= 0xFFFF:
        raise ValueError(f"Duty cycle {duty_cycle} is outside 0..65535")
    if duty_cycle == 0xFFFF:
        on, off = FULL, 0
    elif duty_cycle < 0x0010:
        on, off = 0, FULL
    else:
        on, off = 0, duty_cycle >> 4
    return struct.pack("<HH", on, off)


def bridge_duty_cycles(throttle: float | None, decay_mode: int) -> tuple[int, int]:
    """The (positive, negative) input duty cycles for a throttle, as adafruit_motor's DCMotor sets them."""
    if throttle is None:
        return 0, 0  # Both inputs off: the motor coasts
    if throttle == 0:
        return 0xFFFF, 0xFFFF  # Both inputs on: the motor brakes
    duty = int(0xFFFF * abs(throttle))
    if decay_mode == SLOW_DECAY:  # Brake between pulses
        return (0xFFFF - duty, 0xFFFF) if throttle < 0 else (0xFFFF, 0xFFFF - duty)
    return (0, duty) if throttle < 0 else (duty, 0)  # Coast between pulses


class BridgeMotor:
    """One DC motor of a PCA9685 motor board; `throttle` and `decay_mode` work like adafruit_motor's DCMotor."""

    def __init__(self, i2c_device: object, positive_channel: int) -> None:
        """`i2c_device` is the board's adafruit_bus_device I2CDevice; the negative input is the next channel."""
        if not 0 <= positive_channel <= 14:
            raise ValueError(f"No channel pair starts at {positive_channel}")
        self.i2c_device = i2c_device
        self.register = LED0_ON_L + 4 * positive_channel
        self._decay_mode = FAST_DECAY
        self._throttle: float | None = None

    @classmethod
    def from_motorkit(cls, kit: object, number: int) -> "BridgeMotor":
        """Motor `number` (1 to 4) of an adafruit_motorkit MotorKit, whose own setup turns the bridge's enable input on."""
        motor = getattr(kit, f"motor{number}")
        positive, negative = motor._positive._index, motor._negative._index
        if negative != positive + 1:
            raise ValueError(f"Motor {number}'s inputs are channels {positive} and {negative}, not neighbours")
        return cls(kit._pca.i2c_device, positive)

    @property
    def decay_mode(self) -> int:
        return self._decay_mode

    @decay_mode.setter
    def decay_mode(self, mode: int) -> None:
        if mode not in (FAST_DECAY, SLOW_DECAY):
            raise ValueError("Decay mode must be FAST_DECAY or SLOW_DECAY")
        self._decay_mode = mode

    @property
    def throttle(self) -> float | None:
        """The last throttle the board accepted, without reading the board."""
        return self._throttle

    @throttle.setter
    def throttle(self, value: float | None) -> None:
        if value is not None and not -1.0 <= value <= 1.0:
            raise ValueError("Throttle must be None or between -1.0 and +1.0")
        positive, negative = bridge_duty_cycles(value, self._decay_mode)
        transfer = bytes([self.register]) + channel_registers(positive) + channel_registers(negative)
        with self.i2c_device as i2c:  # One transfer: both outputs change at its STOP
            i2c.write(transfer)
        self._throttle = value
