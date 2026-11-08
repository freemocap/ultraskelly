"""The live hardware checks, on a fake I2C bus and the Pi's command output as recorded."""
import asyncio

import pytest

from ultraskelly.core.bot.diagnostics import (
    HEAT,
    NOT_MOVING,
    UNDER_VOLTAGE,
    Diagnostics,
    Sources,
    count_lost_arbitration,
    decode_throttled,
    judge_power,
    judge_servo_board,
    parse_pmic,
    pulse_us,
    pwm_frequency,
    servo_pulse_us,
)
from ultraskelly.core.bot.health import Finding, Status

# From the robot's Pi on 2026-11-04, `vcgencmd pmic_read_adc`, shortened
PMIC = """ 3V7_WL_SW_A current(0)=0.05855580A
   3V3_SYS_V volt(9)=3.30724900V
   1V8_SYS_V volt(10)=1.79877700V
   VDD_CORE_V volt(15)=0.87570120V
     EXT5V_V volt(24)=5.11344000V
"""
PAN, TILT, ROLL = 3, 11, 7  # OrientationNodeParams' channels
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
    for channel, off in (pulses or {PAN: HALF_MS, TILT: HALF_MS, ROLL: HALF_MS}).items():
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
    assert sent == Finding(Status.OK, "Answers; awake at 50 Hz, and the pulses match the commanded angles", NOT_MOVING)
    assert "OE wire" in NOT_MOVING and "V+" in NOT_MOVING and "orientation" in NOT_MOVING
    assert judge_servo_board(None) is None


def test_the_live_checks_say_when_they_ran():
    from ultraskelly.core.bot.health import Health

    now = [10.0]
    health = Health(clock=lambda: now[0])
    assert health.message(11.0)["diagnostics_time"] is None  # None yet
    health.set_diagnostics({"i2c": {"available": False}})
    version = health.version
    now[0] = 15.0
    health.set_diagnostics({"i2c": {"available": False}})  # The same readings, checked again
    assert health.message(16.0)["diagnostics_time"] == 15.0 and health.version == version


def test_a_check_that_fails_leaves_the_others():
    garbled = sources({0x40: servo_board()}, throttled=lambda: "throttled=garbled\n")  # A firmware with new output

    diagnostics = Diagnostics(garbled)
    result = diagnostics.collect(head_channels={"pan": PAN}, commanded={"pan": 90.0})

    assert result["power"] is None and set(diagnostics.errors) == {"power"}
    assert diagnostics.errors["power"].startswith("ValueError: invalid literal for int()")
    assert result["pca9685"]["channels"]["pan"]["matches"] is True  # The servo readback still comes
    assert judge_power(result["power"]) is None


def test_failing_checks_say_why_and_since_when():
    from ultraskelly.core.bot.health import Health

    now = [10.0]
    health = Health(clock=lambda: now[0])
    health.diagnostics_failed(RuntimeError("bus gone"))  # From the very first run: not the same as not run yet
    first = health.message(11.0)
    assert first["diagnostics_time"] is None
    assert first["diagnostics_error"] == {"error": "RuntimeError: bus gone", "since": 10.0}
    now[0] = 15.0
    health.set_diagnostics({"i2c": {"available": False}}, {"power": "ValueError: garbled"})  # One check still fails
    assert health.message(16.0)["diagnostics_error"] == {"error": "power: ValueError: garbled", "since": 10.0}
    health.set_diagnostics({"i2c": {"available": False}})
    assert health.message(17.0)["diagnostics_error"] is None and health.diagnostics_time == 15.0


def test_the_bot_sends_why_its_checks_fail(tmp_path, monkeypatch):
    from tests.test_degraded_startup import stop

    def broken(**checks):
        raise RuntimeError("the checks broke")

    launcher = launch_on({0x40: servo_board(), 0x60: servo_board(prescale=3)}, tmp_path, monkeypatch)
    launcher.diagnostics.collect = broken  # From the very first run

    async def scenario():
        task = asyncio.create_task(launcher.run())
        await asyncio.sleep(0.2)
        message = launcher.health.message(0.0)
        await stop(launcher, task)
        return message

    message = asyncio.run(scenario())
    assert message["diagnostics_time"] is None and message["diagnostics"] == {}
    assert message["diagnostics_error"]["error"] == "RuntimeError: the checks broke"


def power(throttled: str, supply: float = 5.113) -> dict:
    return {"available": True, **decode_throttled(f"throttled={throttled}"), "rails": {"EXT5V": supply}}


def test_the_pis_supply_is_judged():
    assert judge_power(power("0x50005", 4.79)) == Finding(Status.ERROR, "Under-voltage now (EXT5V 4.79 V)", UNDER_VOLTAGE)
    assert judge_power(power("0x6")) == Finding(
        Status.DEGRADED, "CPU frequency capped, throttled now (EXT5V 5.11 V)", HEAT)
    assert judge_power(power("0x80008")).message == "Temperature limit now (EXT5V 5.11 V)"
    assert judge_power(power("0x10000")) == Finding(
        Status.OK, "Fine now (EXT5V 5.11 V); since boot: under-voltage", UNDER_VOLTAGE)
    assert judge_power(power("0x0")) == Finding(Status.OK, "No under-voltage or throttling (EXT5V 5.11 V)")
    assert judge_power({"available": False}) is None  # Not a Pi: no power line at all


def launch_on(boards: dict, tmp_path, monkeypatch, **robot):
    """The bot on a fake I2C bus holding `boards`, which the test may change while it runs."""
    from ultraskelly import bot_launcher
    from tests.test_degraded_startup import Robot, start

    monkeypatch.setattr(bot_launcher, "DIAGNOSTICS_INTERVAL", 0.05)
    robot = Robot(oak=False, hailo=False, zed=False, imu=False, **robot)
    hardware = robot.hardware()
    hardware.sources = sources(boards)
    robot.hardware = lambda: hardware
    return start(robot, tmp_path)


def test_a_motor_board_that_answers_after_the_start_asks_for_a_restart(tmp_path, monkeypatch):
    from tests.test_degraded_startup import stop
    from ultraskelly.core.bot.motor.motor_test import MotorTestRefused, MotorTestRequest

    boards = {0x60: servo_board(prescale=3)}  # The servo board is off the header when the bot starts
    launcher = launch_on(boards, tmp_path, monkeypatch, servo_board=False)

    async def scenario():
        task = asyncio.create_task(launcher.run())
        await asyncio.sleep(0.2)
        before = launcher.health.get("servo_board")
        boards[0x40] = servo_board()  # Pressed back on
        await asyncio.sleep(0.2)
        back = launcher.health.get("servo_board")
        with pytest.raises(MotorTestRefused) as refused:
            launcher.motor_tester.check(MotorTestRequest(actuator="pan", angle=95.0))
        del boards[0x40]  # Loose again
        await asyncio.sleep(0.2)
        gone = launcher.health.get("servo_board")
        await stop(launcher, task)
        return before, back, str(refused.value), gone

    before, back, refusal, gone = asyncio.run(scenario())
    assert before.status == Status.MISSING
    assert (back.status, back.message) == (Status.DEGRADED, "Answers at 0x40 now, but the bot started without it")
    assert back.restart == "Restart the bot to drive the head's pan and tilt: it sets up its motor boards only when it starts."
    assert refusal.endswith(back.restart)  # The motor test says why nothing would move
    assert gone.status == Status.MISSING and gone.restart is None


def test_a_driven_board_that_stops_answering_is_an_error_until_it_answers(tmp_path, monkeypatch):
    from tests.test_degraded_startup import stop

    boards = {0x40: servo_board(), 0x60: servo_board(prescale=3)}
    launcher = launch_on(boards, tmp_path, monkeypatch)

    async def scenario():
        task = asyncio.create_task(launcher.run())
        await asyncio.sleep(0.2)
        seen = [launcher.health.get("servo_board")]
        del boards[0x40]  # Off the header: the bot finds out before its next command does
        await asyncio.sleep(0.3)
        seen.append(launcher.health.get("servo_board"))
        boards[0x40] = servo_board()
        for _ in range(20):  # Up to 1 s: the readback confirms the pulses at a diagnostics pass after the board is back
            await asyncio.sleep(0.05)
            if launcher.health.get("servo_board").message.startswith("Answers;"):
                break
        seen.append(launcher.health.get("servo_board"))
        await stop(launcher, task)
        return seen

    working, lost, back = asyncio.run(scenario())
    assert working.status == Status.OK and lost.status == Status.ERROR
    assert lost.message == "Stopped answering at 0x40" and "stops at its next command" in lost.hint
    assert (back.status, back.message) == (Status.OK, "Answers; awake at 50 Hz, and the pulses match the commanded angles")


def test_the_zed_only_bot_leaves_the_motor_boards_bus_alone(tmp_path):
    from tests.test_degraded_startup import Robot, free_port
    from ultraskelly.__main__bot import launch_config
    from ultraskelly.bot_launcher import BotLauncher

    opened = []
    hardware = Robot().hardware()
    hardware.sources = sources({0x40: servo_board()}, open_bus=lambda: opened.append("bus") or FakeBus({}))
    launcher = BotLauncher.from_config(launch_config(zed_only=True, port=free_port()), hardware=hardware)

    result = launcher.diagnostics.collect(head_channels={}, commanded={})
    assert opened == [] and result["i2c"] == {"available": False}  # It holds no robot lock


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
        board, diagnostics, supply = (launcher.health.get("servo_board"), launcher.health.diagnostics,
                                      launcher.health.get("power"))
        await stop(launcher, task)
        return board, diagnostics, supply

    board, diagnostics, supply = asyncio.run(scenario())
    assert (supply.status, supply.message) == (Status.OK, "No under-voltage or throttling (EXT5V 5.11 V)")
    assert board.status == Status.OK and board.message.startswith("Answers; awake at 50 Hz") and board.hint == NOT_MOVING
    assert diagnostics["i2c"]["addresses"] == {"0x40": True, "0x60": True, "0x70": False}
    assert diagnostics["motor_watchdog"] == {"trips": 0, "tripped_at": None}
    assert diagnostics["retried_writes"] == {"waist": 0, "mouth": 0}
