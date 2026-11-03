"""Smooth, continuous waist drive that turns the torso toward where the head is looking.

The waist used to run in bursts: 0.7-0.8 drive for half a second, an electrical brake for
0.4 s, repeated while the head was turned, so every burst started and stopped hard. This
drive makes one continuous move instead, following published guidance for quiet brushed
DC motors (sources in docs/audit-2026.md):

- The motor board brakes between PWM pulses (slow decay, the TB6612's own PWM mode), set
  by `slow_decay` in the motor's factory. That lowers current ripple, and so the whine, and
  makes speed follow the drive closely enough to start a move gently.
- A move starts at `start_throttle`, below where the waist breaks loose, and follows
  S-curves: the drive's rate of change is limited, and so is the change of that rate, so
  the torque never jumps.
- It runs at a drive that grows with how far the head is turned. While the head shows no
  progress the waist has not broken loose, so the drive rises slowly toward
  `breakaway_throttle`; once the head comes back that extra decays again, so the waist
  does not lurch.
- It turns until the head is nearly centered, ramps down (with slow decay the motor slows
  with its drive), releases the motor to coast, and brakes it only once it has stopped.
- It starts past `start_angle` and stops within `stop_angle`; a stopping move never
  resumes, so the waist rests between the two angles instead of hunting.
- A move that lasts `max_move_duration` ramps down and rests before the next.

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
RESTART_AFTER = 0.5  # Seconds without an update: the motor watchdog has braked the waist, so start over


class WaistDriveParams(NodeParams):
    """How the waist follows the head. Throttles are magnitudes; angles are head pan from neutral."""

    start_angle: float = Field(default=15.0, ge=0.0, le=90.0, description="Head pan from neutral (degrees) that starts a move")
    stop_angle: float = Field(default=5.0, ge=0.0, le=90.0, description="A move stops once the head pan is this close to neutral")
    min_throttle: float = Field(default=0.5, ge=0.0, le=1.0, description="Running drive with the head at start_angle")
    max_throttle: float = Field(default=0.7, ge=0.0, le=1.0, description="Largest running drive")
    gain: float = Field(default=0.008, gt=0.0, le=0.05, description="Extra running drive per degree of head pan beyond start_angle")
    breakaway_throttle: float = Field(
        default=0.8, ge=0.0, le=1.0, description="Most the drive rises to while the head shows the waist is not moving",
    )
    start_throttle: float = Field(default=0.15, gt=0.0, le=1.0, description="A move begins here, well below where the waist breaks loose")
    release_throttle: float = Field(default=0.2, ge=0.0, le=1.0, description="A stopping move releases the motor at this drive")
    ramp_up: float = Field(default=1.5, gt=0.0, description="Largest drive increase per second")
    ramp_down: float = Field(default=2.0, gt=0.0, description="Largest drive decrease per second")
    jerk: float = Field(default=10.0, gt=0.0, description="Largest change of the ramp rate per second squared: the S-curves' corners")
    coast_duration: float = Field(
        default=0.3, ge=0.0, description="Seconds a released motor coasts before it is braked; no move starts meanwhile",
    )
    stuck_seconds: float = Field(
        default=0.5, gt=0.0, description="At its running drive with no progress for this long, the waist is taken to be stuck",
    )
    progress_degrees: float = Field(default=2.0, gt=0.0, description="How far the head must come back to count as progress")
    breakaway_rate: float = Field(
        default=0.3, gt=0.0, description="Drive added per second while stuck, and taken away per second once moving again",
    )
    max_move_duration: float = Field(
        default=6.0, gt=0.0, description="Safety limit: a move this long without the head centering ramps down and rests"
    )
    rest_duration: float = Field(default=2.0, ge=0.0, description="Seconds after a move reached the limit before the next may start")
    slow_decay: bool = Field(
        default=True, description="Brake between PWM pulses: less ripple and whine, and speed that follows the drive",
    )

    @model_validator(mode="after")
    def _ordered(self) -> "WaistDriveParams":
        if self.stop_angle > self.start_angle:
            raise ValueError("stop_angle must not exceed start_angle")
        if not self.release_throttle <= self.min_throttle <= self.max_throttle <= self.breakaway_throttle:
            raise ValueError("Throttles must satisfy release <= min <= max <= breakaway")
        if self.start_throttle > self.min_throttle:
            raise ValueError("start_throttle must not exceed min_throttle")
        return self


class WaistState(Enum):
    IDLE = "idle"  # Braked at rest
    TURNING = "turning"  # Driving the torso toward the head's gaze
    STOPPING = "stopping"  # Ramping down, then coasting
    RESTING = "resting"  # Braked; a move reached the time limit and the next must wait


def s_curve_step(level: float, rate: float, target: float, *, up: float, down: float, jerk: float, dt: float) -> tuple[float, float]:
    """One tick of a jerk-limited ramp: move `level` toward `target`, its rate of change at most `up`
    (rising) or `down` (falling) per second, that rate changing at most `jerk` per second squared.
    Returns the new level and rate."""
    error = target - level
    if error == 0.0 and rate == 0.0:
        return level, rate
    limit = up if error > 0 else down
    # The fastest rate that can still ease to zero, at `jerk` in steps of `dt`, by the time the
    # target is reached: it covers rate**2 / (2 * jerk) + rate * dt / 2 while easing off.
    half_step = jerk * dt / 2.0
    wanted = math.copysign(min(limit, math.sqrt(half_step**2 + 2.0 * jerk * abs(error)) - half_step), error)
    rate += max(-jerk * dt, min(jerk * dt, wanted - rate))
    moved = level + rate * dt
    if (target - moved) * error <= 0.0 and abs(rate) <= jerk * dt:  # Arrived slowly enough to stop: settle
        return target, 0.0
    return moved, rate  # Otherwise ease through and back, as a target that moved toward the ramp needs


class WaistDrive:
    """Feed it every control tick; it returns that tick's waist command."""

    def __init__(self, params: WaistDriveParams) -> None:
        self.params = params
        self.phase = WaistState.IDLE  # IDLE, TURNING or STOPPING; `state` adds RESTING
        self.direction = 0.0  # Sign of the drive during a move
        self.level = 0.0  # Drive magnitude during a move
        self.rate = 0.0  # Its rate of change, per second
        self.extra = 0.0  # Breakaway drive added while the waist seemed stuck
        self.coasting = False  # Released after a stop, until the coast ends
        self.move_started: float | None = None
        self.released_at: float | None = None
        self.rest_until: float | None = None
        self.last_update: float | None = None
        self.closest = math.inf  # Smallest head offset seen in this move
        self.stuck_for = 0.0  # Seconds at the running drive without progress

    @property
    def state(self) -> WaistState:
        resting = self.rest_until is not None and self.last_update is not None and self.last_update < self.rest_until
        return WaistState.RESTING if resting and self.phase is WaistState.IDLE else self.phase

    def running_throttle(self, magnitude: float) -> float:
        """Drive for a head this many degrees from neutral, before any breakaway extra."""
        p = self.params
        return min(p.max_throttle, p.min_throttle + p.gain * max(0.0, magnitude - p.start_angle))

    def _start(self, now: float, pan_offset: float) -> None:
        self.phase = WaistState.TURNING
        self.direction = -1.0 if pan_offset > 0 else 1.0  # Negative drive turns toward positive pan
        self.level, self.rate, self.extra = self.params.start_throttle, 0.0, 0.0
        self.move_started, self.rest_until = now, None
        self.closest, self.stuck_for = abs(pan_offset), 0.0

    def _turning_target(self, magnitude: float, dt: float) -> float:
        """The drive a turning move heads for, raising or easing off the breakaway extra."""
        p = self.params
        running = self.running_throttle(magnitude)
        if magnitude <= self.closest - p.progress_degrees:
            self.closest, self.stuck_for = magnitude, 0.0  # The head is coming back: the waist is moving
        elif self.level >= running + self.extra - 0.01:
            self.stuck_for += dt
        if self.stuck_for >= p.stuck_seconds:
            self.extra = min(self.extra + p.breakaway_rate * dt, p.breakaway_throttle - running)
        else:
            self.extra = max(0.0, self.extra - p.breakaway_rate * dt)
        return running + self.extra

    def update(self, *, now: float, pan_offset: float | None) -> float | None:
        """This tick's command: a signed throttle, 0.0 to brake, or None to let the motor coast.

        `pan_offset` is the head's pan angle minus neutral in degrees, or None without a valid target.
        """
        p = self.params
        if self.last_update is not None and now - self.last_update > RESTART_AFTER and self.phase is not WaistState.IDLE:
            logger.warning(f"No waist update for {now - self.last_update:.1f} s; the next move starts from rest")
            self.phase, self.level, self.rate, self.extra, self.coasting = WaistState.IDLE, 0.0, 0.0, 0.0, False
        dt = 0.0 if self.last_update is None else min(max(now - self.last_update, 0.0), MAX_STEP)
        self.last_update = now
        if self.phase is WaistState.TURNING and now - self.move_started >= p.max_move_duration:
            logger.warning(f"Waist drove {p.max_move_duration:.1f} s without the head centering; resting {p.rest_duration:.1f} s")
            self.rest_until = now + p.rest_duration
            self.phase = WaistState.STOPPING
        magnitude = None if pan_offset is None else abs(pan_offset)
        target = 0.0
        if self.phase is WaistState.TURNING:
            same_side = pan_offset is not None and (-1.0 if pan_offset > 0 else 1.0) == self.direction
            if magnitude is None or not same_side or magnitude <= p.stop_angle:
                self.phase = WaistState.STOPPING  # Also before turning the other way: stop first
            else:
                target = self._turning_target(magnitude, dt)

        if self.phase is WaistState.STOPPING and not self.coasting:
            self.level, self.rate = s_curve_step(
                self.level, self.rate, 0.0, up=p.ramp_up, down=p.ramp_down, jerk=p.jerk, dt=dt,
            )
            if self.level > p.release_throttle:
                return self.direction * self.level
            self.level = self.rate = 0.0
            self.coasting, self.released_at = True, now
        if self.coasting:
            if now - self.released_at < p.coast_duration:
                return None
            self.coasting, self.phase = False, WaistState.IDLE

        if self.phase is WaistState.IDLE:
            resting = self.rest_until is not None and now < self.rest_until
            if resting or magnitude is None or magnitude <= p.start_angle:
                return 0.0
            self._start(now, pan_offset)
            return self.direction * self.level

        self.level, self.rate = s_curve_step(
            self.level, self.rate, target, up=p.ramp_up, down=p.ramp_down, jerk=p.jerk, dt=dt,
        )
        if self.level > p.breakaway_throttle:  # Easing through a target never passes the ceiling
            self.level, self.rate = p.breakaway_throttle, 0.0
        return self.direction * self.level
