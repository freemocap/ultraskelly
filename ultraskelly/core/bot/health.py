"""What works on the robot and what does not, in plain language, for the dashboard.

The launcher tries every part of the robot and keeps running with whatever works: motors
without cameras, cameras without motors, no Hailo-8 means no tracking. Each part is a
component with a status, a message saying what is wrong, a hint saying how to fix it, and
since when. The stream sends them as `health` messages (docs/stream.md, "health").

Statuses: ok; degraded (works partly, such as the OAK without its models); missing (not on
USB, PCIe or I2C); error (there, but it failed); off (not used on purpose, such as
--simulate-motors). Presence probes read Linux's device lists, so a part that fails to open
is told apart from one that is not there; elsewhere they find nothing.
"""
import threading
import time
from dataclasses import dataclass, field, replace
from enum import StrEnum
from pathlib import Path
from typing import Callable


class Status(StrEnum):
    OK = "ok"
    DEGRADED = "degraded"
    MISSING = "missing"
    ERROR = "error"
    OFF = "off"


# Every component the launcher may report, in the order the dashboard shows them: (label, group).
COMPONENTS: dict[str, tuple[str, str]] = {
    "oak_camera": ("OAK camera", "cameras"),
    "oak_models": ("OAK models", "ai"),
    "hailo": ("Hailo-8", "ai"),
    "zed_camera": ("ZED Mini", "cameras"),
    "zed_imu": ("ZED IMU", "sensors"),
    "depth": ("Depth", "ai"),
    "servo_board": ("Servo board (0x40)", "motors"),
    "motor_board": ("Motor HAT (0x60)", "motors"),
}


@dataclass(frozen=True)
class Component:
    id: str
    label: str
    group: str
    status: Status
    message: str
    hint: str | None = None
    since: float = 0.0  # When the status last changed, on time.monotonic()'s clock
    restart: str | None = None  # Back, but the bot must restart to use it, and why
    detail: dict = field(default_factory=dict)

    def to_message(self) -> dict:
        return {
            "id": self.id, "label": self.label, "group": self.group, "status": self.status.value,
            "message": self.message, "hint": self.hint, "since": self.since, "restart": self.restart,
            "detail": self.detail,
        }


class Health:
    """The components this launch uses and their state. Safe to update from any thread."""

    def __init__(self, clock: Callable[[], float] = time.monotonic) -> None:
        self.clock = clock
        self.components: dict[str, Component] = {}
        self.version = 0  # Counts changes, so a publisher sends only when something changed
        self._lock = threading.Lock()

    def report(
        self,
        id: str,
        status: Status,
        message: str,
        hint: str | None = None,
        *,
        restart: str | None = None,
        detail: dict | None = None,
    ) -> Component:
        """Set a component's state; `since` moves only when its status changes."""
        label, group = COMPONENTS[id]
        with self._lock:
            previous = self.components.get(id)
            since = previous.since if previous is not None and previous.status == status else self.clock()
            component = Component(id=id, label=label, group=group, status=status, message=message, hint=hint,
                                   since=since, restart=restart, detail=detail or {})
            if component != previous:
                self.components[id] = component
                self.version += 1
            return component

    def get(self, id: str) -> Component | None:
        with self._lock:
            return self.components.get(id)

    def ok(self, id: str) -> bool:
        component = self.get(id)
        return component is not None and component.status in (Status.OK, Status.DEGRADED)

    def snapshot(self) -> list[dict]:
        """Every reported component, in COMPONENTS order."""
        with self._lock:
            order = list(COMPONENTS)
            ordered = sorted(self.components.values(), key=lambda component: order.index(component.id))
            return [component.to_message() for component in ordered]

    def mark(self, id: str, **changes) -> None:
        """Change fields of a reported component (such as `restart`) without touching its status."""
        with self._lock:
            component = self.components.get(id)
            if component is not None and (changed := replace(component, **changes)) != component:
                self.components[id] = changed
                self.version += 1


# --- Presence probes: Linux's device lists, read without opening anything -----------------

USB_DEVICES = Path("/sys/bus/usb/devices")
OAK_VENDOR = "03e7"
ZED_VIDEO, ZED_HID = "2b03:f682", "2b03:f681"
HAILO_DEVICE = Path("/dev/hailo0")
I2C_BUS = Path("/dev/i2c-1")


def usb_devices(root: Path = USB_DEVICES) -> set[str]:
    """`vendor:product` of every USB device plugged in, as lsusb lists them."""
    found = set()
    for device in root.glob("*") if root.is_dir() else ():
        try:
            found.add(f"{(device / 'idVendor').read_text().strip()}:{(device / 'idProduct').read_text().strip()}")
        except OSError:
            continue  # Interfaces and root hubs without ids
    return found


@dataclass
class Probes:
    """Where the launcher looks to tell a missing part from a failing one; tests pass their own."""

    usb: Callable[[], set[str]] = usb_devices
    hailo_device: Callable[[], bool] = HAILO_DEVICE.exists
    i2c_bus: Callable[[], bool] = I2C_BUS.exists


# --- What each failure means, and what to do --------------------------------------------

def diagnose_oak(error: BaseException, probes: Probes) -> tuple[Status, str, str]:
    if not any(device.startswith(f"{OAK_VENDOR}:") for device in probes.usb()):
        return (Status.MISSING, "No OAK on USB (no 03e7 device)",
                "Plug the OAK's cable into a blue USB 3 port on the Pi; lsusb should then list 03e7. "
                "Without it the bot has no tracking camera.")
    return (Status.ERROR, f"The OAK is on USB but failed: {error}",
            "Unplug the OAK, wait 5 s and plug it back in; if another program holds it, stop that program.")


def diagnose_hailo(error: BaseException, probes: Probes) -> tuple[Status, str, str]:
    if not probes.hailo_device():
        return (Status.MISSING, "No Hailo-8 (no /dev/hailo0)",
                "Power the Pi off, seat the AI HAT+ on its PCIe ribbon and on the 40-pin header, and power on. "
                "Without it the bot finds no people, so nothing is tracked.")
    return (Status.ERROR, f"The Hailo-8 is there but failed: {error}",
            "Run scripts/setup-pi.sh --check (Hailo driver fix, held packages); reboot if it fixed something.")


def diagnose_zed(error: BaseException, probes: Probes) -> tuple[Status, str, str]:
    devices = probes.usb()
    if ZED_VIDEO not in devices:
        if ZED_HID in devices:
            return (Status.MISSING, "Only the ZED's IMU is on USB (2b03:f681), not its video",
                    "Flip the ZED's USB-C plug at the camera end; its video (2b03:f682) needs the other way round.")
        return (Status.MISSING, "No ZED Mini on USB (no 2b03:f682 device)",
                "Plug the ZED Mini into a blue USB 3 port; the bot picks it up within a few seconds.")
    return (Status.ERROR, f"The ZED Mini is on USB but does not stream: {error}",
            "Unplug the ZED, wait 5 s and plug it back in; the bot tries again every second.")


def diagnose_zed_imu(error: BaseException, probes: Probes) -> tuple[Status, str, str]:
    if ZED_HID not in probes.usb():
        return (Status.MISSING, "No ZED IMU on USB (no 2b03:f681 device)",
                "Plug the ZED Mini in; the bot reads its IMU as soon as it appears.")
    return (Status.ERROR, f"The ZED's IMU is on USB but cannot be read: {error}",
            "Run scripts/setup-pi.sh, which installs the udev rule that lets the bot read it.")


def diagnose_board(
    name: str, address: int, error: BaseException, probes: Probes, without: str,
) -> tuple[Status, str, str]:
    """`without`: what the robot cannot do until the board answers."""
    if not probes.i2c_bus():
        return (Status.ERROR, "I2C is off (no /dev/i2c-1)",
                f"Run scripts/setup-pi.sh, which turns I2C on, then reboot. {without}")
    if "No I2C device" in str(error):
        return (Status.MISSING, f"The {name} does not answer at {address:#04x}",
                "Press the HAT stack down onto the Pi's 40-pin header (every pin, no gap) and check the board's "
                f"power LED; sudo i2cdetect -y 1 should then show {address:x}. {without}")
    return (Status.ERROR, f"The {name} at {address:#04x} failed: {error}",
            f"Check the board's power and seating; brief I2C errors after reseating clear on a restart. {without}")
