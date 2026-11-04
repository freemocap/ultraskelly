"""Live checks of the robot's hardware for the dashboard: the checks done by hand when it misbehaves.

Every few seconds, off the event loop, the bot looks at:
- I2C: which of the servo board (0x40), the Motor HAT (0x60) and their all-call address
  (0x70) answer, each probed with a zero-length write, as i2cdetect does. The all-call
  answers only while the boards' ALLCALL bit is on, which their drivers clear when the bot
  sets them up, so 0x70 is information only.
- The PCA9685 servo board's own registers: asleep or awake, its PWM frequency, and the pulse
  each head servo channel is putting out against the pulse its commanded angle needs, the
  proof that commands reach the chip. A servo that still does not move is then outside the
  chip: the OE wire, V+ servo power, or a reversed plug.
- USB: the OAK, the ZED's video and its IMU (the ZED with only its IMU needs its plug flipped).
- The Hailo-8 on PCIe and its /dev/hailo0.
- Power: the Pi's under-voltage and throttling flags and the PMIC's 5 V input and 3.3 V rails,
  which also make the `power` component (judge_power).
- I2C bus errors (lost arbitration) the kernel logged since boot, and the motor watchdog's brakes.

The bot holds the robot lock, so probing its boards needs no other coordination: the kernel
runs each I2C transfer whole, and reads and quick writes change nothing on a PCA9685.
Elsewhere than a Pi each check reports that it cannot look.
"""
import logging
import re
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from ultraskelly.core.bot.health import HAILO_DEVICE, ZED_HID, ZED_VIDEO, Finding, Status, oak_on_usb, usb_devices

logger = logging.getLogger(__name__)

I2C_BUS_NUMBER = 1
SERVO_BOARD, MOTOR_BOARD, ALL_CALL = 0x40, 0x60, 0x70
MODE1, PRESCALE, LED0_ON_L = 0x00, 0xFE, 0x06
SLEEP = 0x10  # MODE1 bit: oscillator off, every output stopped
OSCILLATOR_HZ = 25_000_000
FULL = 0x1000  # Bit 12 of an ON or OFF count: the output fully on or fully off
# adafruit_motor.servo's defaults, which ServoKit's channels use
MIN_PULSE_US, MAX_PULSE_US, ACTUATION_DEGREES = 750, 2250, 180
PULSE_TOLERANCE_US = 20
PCI_DEVICES = Path("/sys/bus/pci/devices")
HAILO_PCI_VENDOR = "0x1e60"
ZED_IMU_DEVICE = Path("/dev/zed-mini-imu")
NOT_MOVING = ("If a servo still does not move: unplug the board's OE wire (held high it switches every output "
              "off), check the servo power on V+ (5-6 V at the screw terminal and at a servo header's middle "
              "pin), and check each servo plug's orientation (its dark wire on GND).")
UNDER_VOLTAGE = ("Power the Pi from a 5.1 V, 5 A USB-C supply (the official 27 W one) on a short cable; under-voltage "
                 "can drop the USB cameras and the boards on the header.")
HEAT = "Give the Pi air (its fan running, the case's vents clear) and check its supply; it runs slower while throttled."
THROTTLED_FLAGS = {
    0: "under-voltage now", 1: "CPU frequency capped now", 2: "throttled now", 3: "temperature limit now",
    16: "under-voltage since boot", 17: "frequency capped since boot", 18: "throttled since boot",
    19: "temperature limit since boot",
}


def servo_pulse_us(angle: float) -> float:
    """The pulse a ServoKit channel sends for `angle` degrees."""
    return MIN_PULSE_US + (MAX_PULSE_US - MIN_PULSE_US) * angle / ACTUATION_DEGREES


def pwm_frequency(prescale: int) -> float:
    return OSCILLATOR_HZ / (4096 * (prescale + 1))


def pulse_us(on: int, off: int, frequency: float) -> float:
    """A channel's pulse width from its 12-bit ON and OFF counts and their full-on and full-off bits."""
    if off & FULL:
        return 0.0
    if on & FULL:
        return 1e6 / frequency
    return ((off - on) % 4096) / 4096 * 1e6 / frequency


def decode_throttled(text: str) -> dict:
    """`vcgencmd get_throttled` output, such as "throttled=0x50005", as its value and named flags."""
    value = int(text.strip().split("=")[-1], 16)
    return {"throttled": f"{value:#x}", "flags": [name for bit, name in THROTTLED_FLAGS.items() if value >> bit & 1]}


def parse_pmic(text: str) -> dict[str, float]:
    """The voltages in `vcgencmd pmic_read_adc` output, by rail: EXT5V, 3V3_SYS and the rest."""
    return {match[1]: round(float(match[2]), 3) for match in re.finditer(r"(\w+)_V volt\(\d+\)=([\d.]+)V", text)}


def count_lost_arbitration(kernel_log: str) -> int:
    return sum("lost arbitration" in line for line in kernel_log.splitlines())


def run(command: list[str], timeout: float = 5.0) -> str | None:
    """A command's output, or None if it is missing or fails."""
    try:
        return subprocess.run(command, capture_output=True, text=True, timeout=timeout, check=True).stdout
    except (OSError, subprocess.SubprocessError):
        return None


def hailo_on_pci(devices: Path = PCI_DEVICES) -> bool:
    """Whether a Hailo device (vendor 1e60) is on PCIe."""
    for device in devices.glob("*") if devices.is_dir() else ():
        try:
            if (device / "vendor").read_text().strip() == HAILO_PCI_VENDOR:
                return True
        except OSError:
            continue
    return False


def open_i2c_bus() -> object | None:
    """The Pi's I2C bus 1 through smbus2, or None where there is none."""
    try:
        from smbus2 import SMBus

        return SMBus(I2C_BUS_NUMBER)
    except (ImportError, OSError):
        return None


@dataclass
class Sources:
    """Where the checks look; tests pass their own."""

    open_bus: Callable[[], object | None] = open_i2c_bus
    usb: Callable[[], set[str]] = usb_devices
    hailo_device: Callable[[], bool] = HAILO_DEVICE.exists
    hailo_pci: Callable[[], bool] = hailo_on_pci
    imu_device: Callable[[], bool] = ZED_IMU_DEVICE.exists
    throttled: Callable[[], str | None] = lambda: run(["vcgencmd", "get_throttled"])
    pmic: Callable[[], str | None] = lambda: run(["vcgencmd", "pmic_read_adc"])
    kernel_log: Callable[[], str | None] = lambda: run(
        ["journalctl", "-k", "-b", "-o", "cat", "--no-pager"], timeout=10.0,
    )


class Diagnostics:
    """Collects the checks; `collect` blocks for a few milliseconds of I2C and two short commands,
    so run it in a worker thread."""

    def __init__(self, sources: Sources | None = None) -> None:
        self.sources = sources or Sources()
        self.bus = None
        self.lost_arbitration: int | None = None
        self.kernel_log_checks = 0
        self.errors: dict[str, str] = {}  # The checks that failed in the last collect, and how

    def collect(self, *, head_channels: dict[str, int], commanded: dict[str, float | None]) -> dict:
        """Every check; `head_channels` names the servo board's channels (pan, tilt) and
        `commanded` their angles, to compare with the pulses the chip puts out. A check that fails,
        such as on a command's unexpected output, is null and its error is in `errors`; the others
        still come."""
        checks = {
            "i2c": self.i2c,
            "pca9685": lambda: self.servo_board(head_channels, commanded),
            "usb": self.usb,
            "hailo": lambda: {"device": self.sources.hailo_device(), "pci": self.sources.hailo_pci()},
            "power": self.power,
        }
        result, self.errors = {}, {}
        for name, check in checks.items():
            try:
                result[name] = check()
            except Exception as error:
                result[name] = None
                self.errors[name] = f"{type(error).__name__}: {error}"
        return result

    def _bus(self) -> object | None:
        if self.bus is None:
            self.bus = self.sources.open_bus()
        return self.bus

    def i2c(self) -> dict:
        bus = self._bus()
        if bus is None:
            return {"available": False}
        addresses = {}
        for address in (SERVO_BOARD, MOTOR_BOARD, ALL_CALL):
            try:
                bus.write_quick(address)
                addresses[f"{address:#04x}"] = True
            except OSError:
                addresses[f"{address:#04x}"] = False
        if self.kernel_log_checks % 6 == 0:  # The kernel log every sixth check: it can be long
            log = self.sources.kernel_log()
            self.lost_arbitration = None if log is None else count_lost_arbitration(log)
        self.kernel_log_checks += 1
        return {"available": True, "addresses": addresses, "lost_arbitration": self.lost_arbitration}

    def servo_board(self, head_channels: dict[str, int], commanded: dict[str, float | None]) -> dict | None:
        """The servo board's mode, frequency and head channel pulses, or None if it does not answer."""
        bus = self._bus()
        if bus is None:
            return None
        try:
            mode1 = bus.read_byte_data(SERVO_BOARD, MODE1)
            frequency = pwm_frequency(bus.read_byte_data(SERVO_BOARD, PRESCALE))
            channels = {}
            for name, channel in head_channels.items():
                on_l, on_h, off_l, off_h = bus.read_i2c_block_data(SERVO_BOARD, LED0_ON_L + 4 * channel, 4)
                pulse = pulse_us(on_l | on_h << 8, off_l | off_h << 8, frequency)
                angle = commanded.get(name)
                expected = None if angle is None else servo_pulse_us(angle)
                channels[name] = {
                    "channel": channel, "commanded": angle, "pulse_us": round(pulse),
                    "expected_us": None if expected is None else round(expected),
                    "matches": None if expected is None else abs(pulse - expected) <= PULSE_TOLERANCE_US,
                }
        except OSError:
            return None
        return {"address": f"{SERVO_BOARD:#04x}", "mode1": f"{mode1:#04x}", "awake": not mode1 & SLEEP,
                "frequency_hz": round(frequency, 1), "channels": channels}

    def usb(self) -> dict:
        devices = self.sources.usb()
        return {
            "oak": oak_on_usb(devices),
            "zed_video": ZED_VIDEO in devices, "zed_hid": ZED_HID in devices,
            "zed_imu_device": self.sources.imu_device(),
        }

    def power(self) -> dict:
        throttled, pmic = self.sources.throttled(), self.sources.pmic()
        result = {"available": throttled is not None}
        if throttled is not None:
            result.update(decode_throttled(throttled))
        if pmic is not None:
            rails = parse_pmic(pmic)
            result["rails"] = {name: rails[name] for name in ("EXT5V", "3V3_SYS", "1V8_SYS") if name in rails}
        return result

    def close(self) -> None:
        if self.bus is not None:
            try:
                self.bus.close()
            except OSError:
                pass
            self.bus = None


def judge_servo_board(readback: dict | None) -> Finding | None:
    """What the servo board's readback says about a board the bot drives, as (status, message,
    hint); None when there is nothing to judge (no readback, or no angle commanded yet)."""
    if readback is None:
        return None
    if not readback["awake"]:
        return Finding(Status.DEGRADED, f"The PCA9685 is asleep (MODE1 {readback['mode1']}): every output is off",
                       "Restart the bot, which wakes the board when it sets it up.")
    if not 45 <= readback["frequency_hz"] <= 55:
        return Finding(Status.DEGRADED, f"The PCA9685 runs at {readback['frequency_hz']:g} Hz, not the 50 Hz servos need",
                       "Restart the bot, which sets the servo frequency when it sets up the board.")
    wrong = [f"{name} puts out {channel['pulse_us']} µs where {channel['commanded']:g}° needs "
             f"{channel['expected_us']} µs" for name, channel in readback["channels"].items() if channel["matches"] is False]
    if wrong:
        return Finding(Status.DEGRADED, "; ".join(wrong),
                       "Another program may be driving the board; check that only the bot holds the robot lock.")
    if any(channel["matches"] for channel in readback["channels"].values()):
        return Finding(Status.OK, "Answers; awake at 50 Hz, and the pulses match the commanded angles", NOT_MOVING)
    return None


def judge_power(power: dict) -> Finding | None:
    """The Pi's supply as a component: an error while under-voltage now, degraded while throttled,
    frequency-capped or at its temperature limit now, else ok, naming what happened since boot;
    None where the flags cannot be read."""
    if not power or not power.get("available"):
        return None
    supply = power.get("rails", {}).get("EXT5V")
    reading = "" if supply is None else f" (EXT5V {supply:.2f} V)"
    now = [flag.removesuffix(" now") for flag in power["flags"] if flag.endswith(" now")]
    before = [flag.removesuffix(" since boot") for flag in power["flags"] if flag.endswith(" since boot")]
    if "under-voltage" in now:
        return Finding(Status.ERROR, f"Under-voltage now{reading}", UNDER_VOLTAGE)
    if now:
        named = ", ".join(now)
        return Finding(Status.DEGRADED, f"{named[:1].upper()}{named[1:]} now{reading}", HEAT)
    if before:
        return Finding(Status.OK, f"Fine now{reading}; since boot: {', '.join(before)}",
                       UNDER_VOLTAGE if "under-voltage" in before else HEAT)
    return Finding(Status.OK, f"No under-voltage or throttling{reading}")
