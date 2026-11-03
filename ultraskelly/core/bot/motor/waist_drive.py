"""Continuous, ramped waist drive that turns the torso toward where the head is looking.

The waist used to run in bursts: 0.7-0.8 drive for half a second, an electrical brake for
0.4 s, repeated while the head was turned, so every burst started and stopped hard. This
drive starts a move just below where the waist breaks loose and ramps up to a speed that
grows with how far the head is turned. It keeps turning until the head is nearly centered,
then ramps down, releases the motor to coast, and brakes it only once it has stopped. A
move that runs longer than its safety limit ramps down and rests before the next one.

The drive only decides commands. It does not know the waist's angle: no sensor measures
it, and the camera loop closes the loop by bringing the head back toward neutral.
"""
import logging
import math
from enum import Enum

from pydantic import Field, model_validator

from ultraskelly.core.bot.base_abcs import NodeParams

logger = logging.getLogger(__name__)

MAX_STEP = 0.1  # Seconds; a stalled control loop does not jump a ramp further than this


class WaistDriveParams(NodeParams):
    """How the waist follows the head. Throttles are magnitudes; angles are head pan from neutral."""

    start_angle: float = Field(default=15.0, ge=0.0, le=90.0, description="Head pan from neutral (degrees) that starts a move")
    stop_angle: float = Field(default=5.0, ge=0.0, le=90.0, description="A move continues until the head pan is this close to neutral")
    min_throttle: float = Field(
        default=0.7, ge=0.0, le=1.0,
        description="Drive at start_angle; enough to keep the waist moving (the old controller's floor until measured)",
    )
    max_throttle: float = Field(default=0.8, ge=0.0, le=1.0, description="Largest drive")
    gain: float = Field(default=0.008, gt=0.0, le=0.05, description="Extra drive per degree of head pan beyond start_angle")
    start_throttle: float = Field(default=0.45, gt=0.0, le=1.0, description="A move begins here, below where the waist breaks loose")
    release_throttle: float = Field(default=0.4, ge=0.0, le=1.0, description="A slowing move releases the motor at this drive")
    ramp_up: float = Field(default=1.5, gt=0.0, description="Largest drive increase per second")
    ramp_down: float = Field(default=2.0, gt=0.0, description="Largest drive decrease per second")
    coast_duration: float = Field(
        default=0.3, ge=0.0, description="Seconds a released motor coasts before it is braked; no move starts meanwhile"
    )
    max_move_duration: float = Field(
        default=6.0, gt=0.0, description="Safety limit: a move this long without the head centering ramps down and rests"
    )
    rest_duration: float = Field(default=2.0, ge=0.0, description="Seconds after a move reached the limit before the next may start")

    @model_validator(mode="after")
    def _ordered(self) -> "WaistDriveParams":
        if self.stop_angle > self.start_angle:
            raise ValueError("stop_angle must not exceed start_angle")
        if not self.release_throttle <= self.min_throttle <= self.max_throttle:
            raise ValueError("Throttles must satisfy release_throttle <= min_throttle <= max_throttle")
        if self.start_throttle > self.max_throttle:
            raise ValueError("start_throttle must not exceed max_throttle")
        return self


class WaistState(Enum):
    IDLE = "idle"  # Braked at rest
    TURNING = "turning"  # Driving the torso toward the head's gaze
    STOPPING = "stopping"  # Ramping down, then coasting
    RESTING = "resting"  # Braked; a move reached the time limit and the next must wait


def _approach(value: float, target: float, amount: float) -> float:
    return value + max(-amount, min(amount, target - value))


class WaistDrive:
    """Feed it every control tick; it returns that tick's waist command."""

    def __init__(self, params: WaistDriveParams) -> None:
        self.params = params
        self.throttle = 0.0  # Signed drive while turning or ramping down; 0 once released
        self.phase = WaistState.IDLE  # IDLE, TURNING or STOPPING; `state` adds RESTING
        self.move_started: float | None = None
        self.released_at: float | None = None
        self.rest_until: float | None = None
        self.last_update: float | None = None

    @property
    def state(self) -> WaistState:
        resting = self.rest_until is not None and self.last_update is not None and self.last_update < self.rest_until
        return WaistState.RESTING if resting and self.phase is WaistState.IDLE else self.phase

    def wanted(self, pan_offset: float | None) -> float:
        """Signed drive the head's pan asks for, before ramps and limits; 0 asks for none.

        Negative drive turns the torso toward positive pan, the existing motor polarity.
        """
        if pan_offset is None:
            return 0.0
        p = self.params
        direction = -1.0 if pan_offset > 0 else 1.0
        continuing = self.throttle != 0.0 and math.copysign(1.0, self.throttle) == direction
        magnitude = abs(pan_offset)
        if magnitude <= (p.stop_angle if continuing else p.start_angle):
            return 0.0
        return direction * min(p.max_throttle, p.min_throttle + p.gain * max(0.0, magnitude - p.start_angle))

    def update(self, *, now: float, pan_offset: float | None) -> float | None:
        """This tick's command: a signed throttle, 0.0 to brake, or None to let the motor coast.

        `pan_offset` is the head's pan angle minus neutral in degrees, or None without a valid target.
        """
        p = self.params
        dt = 0.0 if self.last_update is None else min(max(now - self.last_update, 0.0), MAX_STEP)
        self.last_update = now
        if self.phase is WaistState.TURNING and now - self.move_started >= p.max_move_duration:
            logger.warning(f"Waist drove {p.max_move_duration:.1f} s without the head centering; resting {p.rest_duration:.1f} s")
            self.rest_until = now + p.rest_duration
            self.phase = WaistState.STOPPING
        resting = self.rest_until is not None and now < self.rest_until
        wanted = 0.0 if resting else self.wanted(pan_offset)
        same_way = wanted != 0.0 and self.throttle != 0.0 and math.copysign(1.0, wanted) == math.copysign(1.0, self.throttle)

        if self.phase is WaistState.STOPPING and same_way:
            self.phase = WaistState.TURNING  # Wanted again before it stopped: carry on from here
        if self.phase is WaistState.TURNING:
            if same_way:
                rate = p.ramp_up if abs(wanted) > abs(self.throttle) else p.ramp_down
                self.throttle = _approach(self.throttle, wanted, rate * dt)
                return self.throttle
            self.phase = WaistState.STOPPING  # Includes reversing: stop first, then start the other way

        if self.phase is WaistState.STOPPING:
            if self.throttle:
                self.throttle = _approach(self.throttle, 0.0, p.ramp_down * dt)
                if abs(self.throttle) > p.release_throttle:
                    return self.throttle
                self.throttle = 0.0
                self.released_at = now
            if now - self.released_at < p.coast_duration:
                return None
            self.phase = WaistState.IDLE

        if wanted:
            self.rest_until = None
            self.throttle = math.copysign(min(p.start_throttle, abs(wanted)), wanted)
            self.move_started = now
            self.phase = WaistState.TURNING
            return self.throttle
        return 0.0
