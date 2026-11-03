import struct
import sys
import types

from scripts import stop_motors


class Device:
    """The board's I2C device: keeps each transfer."""

    def __init__(self):
        self.transfers = []

    def __enter__(self):
        return self

    def __exit__(self, *exception):
        return False

    def write(self, buffer, *, start=0, end=None):
        self.transfers.append(bytes(buffer[start:end]))


BRAKE = struct.pack("<HHHH", 0x1000, 0, 0x1000, 0)  # Both bridge inputs fully on


def fake_board(monkeypatch, failures: int) -> list:
    """Install a MotorKit stand-in whose first `failures` constructions fail; return the kits it made."""
    kits, remaining = [], [failures]

    class MotorKit:
        def __init__(self, address):
            if remaining[0]:
                remaining[0] -= 1
                raise TypeError("'NoneType' object is not subscriptable")  # How blinka reports a lost read
            self._pca = types.SimpleNamespace(i2c_device=Device())
            for number, positive in ((1, 9), (2, 11), (3, 3), (4, 5)):  # adafruit_motorkit's bridge inputs
                setattr(self, f"motor{number}", types.SimpleNamespace(
                    _positive=types.SimpleNamespace(_index=positive), _negative=types.SimpleNamespace(_index=positive + 1),
                ))
            kits.append(self)

    monkeypatch.setitem(sys.modules, "adafruit_motorkit", types.SimpleNamespace(MotorKit=MotorKit))
    monkeypatch.setattr(stop_motors.time, "sleep", lambda seconds: None)
    return kits


def test_every_motor_is_braked_once_the_board_answers(monkeypatch):
    kits = fake_board(monkeypatch, failures=3)
    assert stop_motors.main() == 0
    transfers = kits[0]._pca.i2c_device.transfers  # One transfer per motor, both inputs at once
    assert transfers == [bytes([0x06 + 4 * channel]) + BRAKE for channel in (9, 11, 3, 5)]


def test_an_unreachable_board_is_reported(monkeypatch):
    fake_board(monkeypatch, failures=1000)
    assert stop_motors.main() == 1
