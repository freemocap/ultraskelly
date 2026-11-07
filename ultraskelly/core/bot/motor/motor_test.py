"""Move one actuator at a time from the dashboard, within test limits, to check wiring and motion.

A test is a discrete command with its own delivery contract, unlike the bot's state topics,
which keep only the newest value. Each accepted test runs once, to completion or until it is
stopped, and its caller is told which. A test asked for while another runs is refused, never
queued or merged. A test outside the test limits is refused with the reason rather than
clamped, so what moves is what was asked for. While a test runs, tracking holds still: both
controllers stop commanding, the DC motor not under test is braked, and tracking resumes
afterwards from where the test left the head. Commands still pass through the actuator limits
and the command recorder, so they show on the dashboard and in recordings.
"""
import asyncio
import logging
import math
import time
from typing import Callable, Literal

from pydantic import BaseModel, ConfigDict, Field

from ultraskelly.core.bot.base_abcs import NodeParams
from ultraskelly.core.bot.motor.mouth_node import MouthNode
from ultraskelly.core.bot.motor.orientation_node import OrientationNode
from ultraskelly.core.bot.motor.waist_drive import WaistDrive

logger = logging.getLogger(__name__)

SERVO_TRAVEL = (0.0, 180.0)  # Degrees: every angle the servo board's scale has (750-2250 µs)
SWEEP_STEP = 0.02  # Seconds between the commands of a servo test's sweep


class MotorTestLimits(NodeParams):
    """What dashboard tests may do. A servo test may send any angle the servo takes, its whole 0-180 (the owner,
    2026-11-07: "if the motor can make it there then do it"); the actuator limits clamp every command as well."""

    enabled: bool = Field(default=True, description="Accept motor tests from the dashboard")
    waist_max_throttle: float = Field(default=0.7, ge=0.0, le=1.0)
    waist_max_seconds: float = Field(default=0.5, gt=0.0, le=2.0)
    waist_max_travel: float = Field(
        default=1.0, gt=0.0,
        description="Largest net drive of all waist tests one way, in throttle-seconds: the waist has no angle sensor",
    )
    mouth_max_throttle: float = Field(default=1.0, ge=0.0, le=1.0)
    mouth_max_seconds: float = Field(default=0.3, gt=0.0, le=1.0)
    settle_seconds: float = Field(default=0.3, ge=0.0, le=2.0, description="Pause after a test before tracking resumes")


class MotorTestRequest(BaseModel):
    """One test: a servo angle, or a motor pulse of a signed throttle for some seconds."""

    model_config = ConfigDict(extra="forbid")

    actuator: Literal["pan", "tilt", "waist", "mouth"]
    angle: float | None = Field(default=None, ge=SERVO_TRAVEL[0], le=SERVO_TRAVEL[1], description="Servo angle in degrees, 0-180")
    throttle: float | None = Field(default=None, ge=-1.0, le=1.0, description="Motor drive; negative jaw drive opens it")
    seconds: float | None = Field(default=None, gt=0.0, description="How long to drive the motor")

    def describe(self) -> str:
        if self.angle is not None:
            return f"{self.actuator} to {self.angle:.0f} degrees"
        return f"{self.actuator} {self.throttle:+.2f} for {self.seconds:.2f} s"


class MotorTestRefused(Exception):
    """A test that will not run, and why. `busy` means another test is running."""

    def __init__(self, reason: str, *, busy: bool = False) -> None:
        super().__init__(reason)
        self.busy = busy


class MotorTester:
    """Runs dashboard motor tests on the bot's own controllers' actuators, one at a time."""

    def __init__(
        self,
        *,
        orientation: OrientationNode,
        mouth: MouthNode,
        limits: MotorTestLimits,
        clock: Callable[[], float] = time.monotonic,
        unavailable: Callable[[str], str | None] | None = None,
        simulated: bool = False,
    ) -> None:
        self.orientation = orientation
        self.simulated = simulated  # Motors simulated (--simulate-motors): a test records commands and moves nothing
        self.mouth = mouth
        self.limits = limits
        self.clock = clock
        # Why an actuator's board cannot be driven now (missing or failed), or None; a test on it is refused
        self.unavailable = unavailable
        self.task: asyncio.Task | None = None
        self.stopping = asyncio.Event()
        self.current: dict | None = None  # The running test
        self.last: dict | None = None  # The latest finished test and how it ended
        self.waist_travel = 0.0  # Net throttle-seconds of all waist tests so far

    def state(self) -> dict:
        """For the dashboard: whether tests are allowed, their limits, and what is running or ran last."""
        return {
            "enabled": self.limits.enabled,
            "simulated": self.simulated,
            "limits": {
                **self.limits.model_dump(),
                "servo_windows": {name: list(SERVO_TRAVEL) for name in ("pan", "tilt")},
                # +1: a larger angle turns the view right (pan) or down (tilt), as the head's aim has it (head_aim.py)
                "directions": {"pan": self.orientation.params.aim.pan_direction, "tilt": self.orientation.params.aim.tilt_direction},
            },
            "running": self.current,
            "last": self.last,
            "waist_travel": round(self.waist_travel, 3),
            "following": self.orientation.following,  # Whether tracking turns the head and waist, between tests
        }

    def check(self, request: MotorTestRequest) -> None:
        """Refuse a test that is not allowed now, with the reason."""
        limits = self.limits
        if not limits.enabled:
            raise MotorTestRefused("Motor tests are turned off in the launch configuration")
        if self.unavailable is not None and (reason := self.unavailable(request.actuator)) is not None:
            raise MotorTestRefused(reason)
        if self.task is not None and not self.task.done():
            raise MotorTestRefused(f"Busy: {self.current['description']} is running", busy=True)
        if request.actuator in ("pan", "tilt"):
            if request.angle is None or request.throttle is not None or request.seconds is not None:
                raise MotorTestRefused("A servo test takes an angle and nothing else")
            return
        if request.throttle is None or request.seconds is None or request.angle is not None:
            raise MotorTestRefused("A motor test takes a throttle and seconds and nothing else")
        waist = request.actuator == "waist"
        largest = limits.waist_max_throttle if waist else limits.mouth_max_throttle
        longest = limits.waist_max_seconds if waist else limits.mouth_max_seconds
        if abs(request.throttle) > largest:
            raise MotorTestRefused(f"{request.actuator} tests drive at most {largest:.2f}")
        if request.seconds > longest:
            raise MotorTestRefused(f"{request.actuator} tests last at most {longest:.2f} s")
        if waist and abs(self.waist_travel + request.throttle * request.seconds) > limits.waist_max_travel:
            raise MotorTestRefused(
                f"That would take waist tests past {limits.waist_max_travel:.2f} throttle-seconds one way "
                f"(now {self.waist_travel:+.2f}); pulse the other way first"
            )

    async def run(self, request: MotorTestRequest) -> dict:
        """Run one test and return its record once it has finished or been stopped.

        Raises MotorTestRefused, without moving anything, if the test is not allowed now.
        A caller that goes away does not cut the test short: it runs on and is recorded.
        """
        self.check(request)
        self.stopping.clear()
        self.current = {"description": request.describe(), **request.model_dump(), "started": self.clock()}
        self.task = asyncio.create_task(self._execute(request))
        return await asyncio.shield(self.task)

    async def stop(self) -> bool:
        """Stop the running test, if any, and wait until its motors are braked; return whether one ran."""
        if self.task is None or self.task.done():
            return False
        self.stopping.set()
        await asyncio.shield(self.task)
        return True

    async def _interrupted(self, seconds: float) -> bool:
        """Wait up to `seconds`; True if a stop came first."""
        try:
            await asyncio.wait_for(self.stopping.wait(), timeout=seconds)
            return True
        except TimeoutError:
            return False

    async def _sweep(self, servo, actuator: str, target: float) -> bool:
        """Turn a head servo to `target` no faster than the head turns it when following (pan_servo_speed for pan,
        servo_speed for tilt), not at the servo's own full speed: a whole-range test then takes seconds, not a jerk.
        True if a stop came first; the servo holds where the sweep had got to."""
        aim = self.orientation.params.aim
        speed = min(aim.servo_speed, aim.pan_servo_speed) if actuator == "pan" else aim.servo_speed
        start = float(getattr(self.orientation, f"{actuator}_angle"))
        steps = max(1, math.ceil(abs(target - start) / (speed * SWEEP_STEP)))
        for step in range(1, steps + 1):
            servo.angle = start + (target - start) * step / steps
            setattr(self.orientation, f"{actuator}_angle", float(servo.angle))  # The dashboard shows the sweep
            if step < steps and await self._interrupted(SWEEP_STEP):
                return True
        return False

    def _brake_dc_motors(self) -> None:
        for motor in (self.orientation.waist_motor, self.mouth.mouth_motor):
            try:
                motor.throttle = 0.0
            except Exception:
                logger.exception("Could not brake a motor after a motor test")

    async def _execute(self, request: MotorTestRequest) -> dict:
        started, outcome, sent = self.clock(), "done", None
        self.orientation.manual = self.mouth.manual = True  # Tracking commands nothing now
        try:
            self._brake_dc_motors()  # Whatever tracking was doing stops: only the tested actuator moves
            if request.actuator in ("pan", "tilt"):
                channel = self.orientation.params.pan_channel if request.actuator == "pan" else self.orientation.params.tilt_channel
                servo = self.orientation.servo_kit.servo[channel]
                if await self._sweep(servo, request.actuator, request.angle):
                    outcome = "stopped"
                sent = servo.angle  # After the actuator limits
                setattr(self.orientation, f"{request.actuator}_angle", float(sent))  # Tracking resumes from here
            else:
                motor = self.orientation.waist_motor if request.actuator == "waist" else self.mouth.mouth_motor
                motor.throttle = request.throttle
                sent = motor.throttle
                if await self._interrupted(request.seconds):
                    outcome = "stopped"
        except Exception as error:
            logger.exception(f"Motor test failed: {request.describe()}")
            outcome = f"failed: {error!r}"
        finally:
            driven = self.clock() - started
            self._brake_dc_motors()
            if request.actuator == "waist" and sent:
                self.waist_travel += sent * min(driven, request.seconds)
            if outcome == "done" and await self._interrupted(self.limits.settle_seconds):
                outcome = "stopped"
            self.orientation.waist_drive = WaistDrive(self.orientation.params.waist)  # No stale move resumes
            self.orientation.manual = self.mouth.manual = False
            if self.simulated and outcome == "done":
                outcome = "simulated, nothing moved (the bot runs with --simulate-motors)"
            self.last = {**self.current, "outcome": outcome, "sent": sent, "took": round(self.clock() - started, 3)}
            self.current = None
        logger.info(f"Motor test {self.last['description']}: {outcome}")
        return self.last
