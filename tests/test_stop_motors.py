import sys
import types

from scripts import stop_motors


class Motor:
    def __init__(self):
        self.written = []

    @property
    def throttle(self):
        raise AssertionError("throttle was read back from the board")

    @throttle.setter
    def throttle(self, value):
        self.written.append(value)


def fake_board(monkeypatch, failures: int) -> list:
    """Install a MotorKit stand-in whose first `failures` constructions fail; return the kits it made."""
    kits, remaining = [], [failures]

    class MotorKit:
        def __init__(self, address):
            if remaining[0]:
                remaining[0] -= 1
                raise TypeError("'NoneType' object is not subscriptable")  # How blinka reports a lost read
            self.motor1, self.motor2, self.motor3, self.motor4 = Motor(), Motor(), Motor(), Motor()
            kits.append(self)

    monkeypatch.setitem(sys.modules, "adafruit_motorkit", types.SimpleNamespace(MotorKit=MotorKit))
    monkeypatch.setattr(stop_motors.time, "sleep", lambda seconds: None)
    return kits


def test_every_motor_is_braked_once_the_board_answers(monkeypatch):
    kits = fake_board(monkeypatch, failures=3)
    assert stop_motors.main() == 0
    assert [kits[0].motor1.written, kits[0].motor4.written] == [[0.0], [0.0]]


def test_an_unreachable_board_is_reported(monkeypatch):
    fake_board(monkeypatch, failures=1000)
    assert stop_motors.main() == 1
