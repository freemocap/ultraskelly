"""The live hardware checks, on a fake I2C bus and the Pi's command output as recorded."""
import asyncio

import pytest

from ultraskelly.core.bot.diagnostics import (
    NOT_MOVING,
    Diagnostics,
    Sources,
    count_lost_arbitration,
    decode_throttled,
    judge_servo_board,
    parse_pmic,
    pulse_us,
    pwm_frequency,
    servo_pulse_us,
)
from ultraskelly.core.bot.health import Status

# From the robot's Pi on 2026-11-04, `vcgencmd pmic_read_adc`, shortened
PMIC = """ 3V7_WL_SW_A current(0)=0.05855580A
   3V3_SYS_V volt(9)=3.30724900V
   1V8_SYS_V volt(10)=1.79877700V
   VDD_CORE_V volt(15)=0.87570120V
     EXT5V_V volt(24)=5.11344000V
"""
PAN, TILT = 11, 3
HALF_MS = 307  # 12-bit OFF count of a 1.5 ms pulse at 50 Hz: 90 degrees


class FakeBus:
    """PCA9685 registers by address; an address not in `boards` does not answer."""

    def __init__(self, boards):
        self.boards = boards

    def write_quick(self, address):
        if address not in self.boards:
            raise OSError(121, "Remote I/O error")

    def read_byte_data(self, address, register):
        self.write_quick(address)
        return self.boards[address].get(register, 0)

    def read_i2c_block_data(self, address, register, length):
        self.write_quick(address)
        return [self.boards[address].get(register + offset, 0) for offset in range(length)]

    def close(self):
        pass


def servo_board(*, mode1=0x20, prescale=121, pulses=None):
    """A PCA9685's registers: MODE1, PRESCALE, and each channel's OFF count (ON at 0)."""
    registers = {0x00: mode1, 0xFE: prescale}
    for channel, off in (pulses or {PAN: HALF_MS, TILT: HALF_MS}).items():
        base = 0x06 + 4 * channel
        registers.update({base: 0, base + 1: 0, base + 2: off & 0xFF, base + 3: off >> 8})
    return registers


def sources(boards, **overrides):
    values = dict(open_bus=lambda: FakeBus(boards), usb=lambda: {"1915:1025"}, hailo_device=lambda: False,
                  hailo_pci=lambda: False, imu_device=lambda: False, throttled=lambda: "throttled=0x0\n",
                  pmic=lambda: PMIC, kernel_log=lambda: "i2c_designware 1f00074000.i2c: lost arbitration\nboot\n")
    values.update(overrides)
    return Sources(**values)


def test_servo_pulses_follow_servokits_defaults():
    assert servo_pulse_us(0) == 750 and servo_pulse_us(90) == 1500 and servo_pulse_us(180) == 2250
    assert pwm_frequency(121) == pytest.approx(50.03, abs=0.01)  # What ServoKit sets for 50 Hz
    assert pulse_us(0, HALF_MS, pwm_frequency(121)) == pytest.approx(1498, abs=1)
    assert pulse_us(0, 0x1000, 50.0) == 0.0 and pulse_us(0x1000, 0, 50.0) == 20000.0  # Full off, full on


def test_the_pis_command_output_is_read():
    assert decode_throttled("throttled=0x50005\n") == {
        "throttled": "0x50005",
        "flags": ["under-voltage now", "throttled now", "under-voltage since boot", "throttled since boot"],
    }
    assert decode_throttled("throttled=0x0")["flags"] == []
    rails = parse_pmic(PMIC)
    assert rails["EXT5V"] == 5.113 and rails["3V3_SYS"] == 3.307 and "3V7_WL_SW" not in rails  # Currents are skipped
    assert count_lost_arbitration("a\ni2c_designware: lost arbitration\nlost arbitration\n") == 2


def test_todays_robot_seen_by_the_checks():
    diagnostics = Diagnostics(sources({0x40: servo_board(), 0x60: servo_board(prescale=3)}))

    result = diagnostics.collect(head_channels={"pan": PAN, "tilt": TILT}, commanded={"pan": 90.0, "tilt": 90.0})

    assert result["i2c"] == {"available": True, "addresses": {"0x40": True, "0x60": True, "0x70": False},
                             "lost_arbitration": 1}
    board = result["pca9685"]
    assert (board["awake"], board["mode1"], board["frequency_hz"]) == (True, "0x20", 50.0)
    assert board["channels"]["pan"] == {"channel": PAN, "commanded": 90.0, "pulse_us": 1498, "expected_us": 1500,
                                        "matches": True}
    assert result["usb"] == {"oak": False, "zed_video": False, "zed_hid": False, "zed_imu_device": False}
    assert result["hailo"] == {"device": False, "pci": False}
    assert result["power"] == {"available": True, "throttled": "0x0", "flags": [],
                               "rails": {"EXT5V": 5.113, "3V3_SYS": 3.307, "1V8_SYS": 1.799}}


def test_without_a_pi_every_check_says_it_cannot_look():
    nothing = sources({}, open_bus=lambda: None, throttled=lambda: None, pmic=lambda: None, kernel_log=lambda: None)

    result = Diagnostics(nothing).collect(head_channels={"pan": PAN}, commanded={"pan": 90.0})

    assert result["i2c"] == {"available": False} and result["pca9685"] is None
    assert result["power"] == {"available": False}


def test_a_missing_servo_board_has_no_readback():
    result = Diagnostics(sources({0x60: servo_board()})).collect(head_channels={"pan": PAN}, commanded={})
    assert result["pca9685"] is None and result["i2c"]["addresses"]["0x40"] is False


def readback(**board):
    return Diagnostics(sources({0x40: servo_board(**board)})).servo_board(
        {"pan": PAN, "tilt": TILT}, {"pan": 95.0, "tilt": 90.0})


def test_the_servo_boards_readback_is_judged():
    asleep = judge_servo_board(readback(mode1=0x30))
    assert asleep[0] == Status.DEGRADED and "asleep (MODE1 0x30)" in asleep[1]
    wrong_rate = judge_servo_board(readback(prescale=3))
    assert wrong_rate[0] == Status.DEGRADED and "not the 50 Hz" in wrong_rate[1]
    stale = judge_servo_board(readback())  # Pan still at 1.5 ms although 95 degrees was commanded
    assert stale[0] == Status.DEGRADED and stale[1] == "pan puts out 1498 µs where 95° needs 1542 µs"
    sent = judge_servo_board(readback(pulses={PAN: 316, TILT: HALF_MS}))  # 1543 µs for 95 degrees
    assert sent == (Status.OK, "Answers; awake at 50 Hz, and the pulses match the commanded angles", NOT_MOVING)
    assert "OE wire" in NOT_MOVING and "V+" in NOT_MOVING and "orientation" in NOT_MOVING
    assert judge_servo_board(None) is None


def test_the_bot_reports_that_commands_reach_the_servo_chip(tmp_path, monkeypatch):
    from ultraskelly import bot_launcher
    from tests.test_degraded_startup import Robot, start, stop

    monkeypatch.setattr(bot_launcher, "DIAGNOSTICS_INTERVAL", 0.05)
    robot = Robot(oak=False, hailo=False, zed=False, imu=False)
    hardware = robot.hardware()
    hardware.sources = sources({0x40: servo_board(), 0x60: servo_board(prescale=3)})
    robot.hardware = lambda: hardware
    launcher = start(robot, tmp_path)

    async def scenario():
        task = asyncio.create_task(launcher.run())
        await asyncio.sleep(0.3)
        board, diagnostics = launcher.health.get("servo_board"), launcher.health.diagnostics
        await stop(launcher, task)
        return board, diagnostics

    board, diagnostics = asyncio.run(scenario())
    assert board.status == Status.OK and board.message.startswith("Answers; awake at 50 Hz") and board.hint == NOT_MOVING
    assert diagnostics["i2c"]["addresses"] == {"0x40": True, "0x60": True, "0x70": False}
    assert diagnostics["motor_watchdog"] == {"trips": 0, "tripped_at": None}
    assert diagnostics["retried_writes"] == {"waist": 0, "mouth": 0}
