"""Calm head aiming: turn toward where the person is, allowing for the camera's delay and the head's gearing.

The head used to add `head_gain` times the pixel error to its servo angles on every camera
sample. Each sample is measured on a frame captured some 50-100 ms earlier, while the head
was still turning toward the previous ones, so those corrections piled up. And the head's
linkage turns the camera only a fraction of a degree per servo degree (`pan_ratio`,
`tilt_ratio`, measured on the robot), so the servos wound toward their end stops while the
waist, which followed the pan servo's angle, hunted back and forth (docs/audit-2026.md,
"Calm head tracking").

HeadAim works in degrees of the camera's view and converts to servo degrees only for the
commands. It keeps its goal relative to the robot's base: where the camera pointed when the
frame was captured (the head's commanded view then, less the servos' lag, plus how far the
body had turned) plus `reach` of the person's offset in that frame, converted to degrees
through the camera's pinhole geometry. Aiming short means neither a wrong scale (pixels per
degree, or the gear ratio) nor the parallax of a camera mounted ahead of the pan axis can
carry the head past the person; the frames that follow close the rest. The goal is smoothed,
and one far from the current goal (someone new, or a stray detection) must show up
`jump_samples` times in a row before the head believes it.

Each tick the head turns toward the goal less the body's turn, so it counter-rotates as soon
as the waist moves, without waiting for the camera. It turns no faster than `max_speed` (and
its servos no faster than `servo_speed`, pan's no faster than `pan_servo_speed`), speeding up
and slowing down at `max_acceleration`, keeps `end_margin` servo degrees clear of its end stops, and while the body is still it moves
only once its goal is more than `deadband` away, so jitter never moves it. Each servo is
commanded half its linkage's slack (`pan_slack`, `tilt_slack`) past the view in the way it
moves, so a reversal takes up the play at once. The goal's bearing from the body tells the
waist where the person is, even beyond the head's reach.
"""
import math
from collections import deque
from typing import Literal

from pydantic import Field

from ultraskelly.core.bot.base_abcs import NodeParams
from ultraskelly.core.bot.motor.waist_drive import MAX_STEP, s_curve_step

MIDDLE = 90.0  # Servo degrees at which the head faces the way the body does
HISTORY_SECONDS = 1.0  # How far back the head remembers where the camera pointed
SLACK_TURN = 0.2  # Degrees of view the head must turn back before its servo takes up the slack the other way
FRAME = (640, 480)  # The picture the targets are measured in, pixels


def zero_vibration(frequency: float, damping: float) -> tuple[float, float, float]:
    """A two-impulse zero-vibration shaper for a mode of natural `frequency` (rad/s) and `damping` ratio:
    (A1, A2, delay). A command split into A1 now and A2 half a ringing period later cancels the ringing
    the first part starts: K = exp(-damping pi / sqrt(1 - damping^2)), A1 = 1/(1+K), A2 = K/(1+K)."""
    ringing = frequency * math.sqrt(1.0 - damping ** 2)
    k = math.exp(-damping * math.pi / math.sqrt(1.0 - damping ** 2))
    return 1.0 / (1.0 + k), k / (1.0 + k), math.pi / ringing


SERVO_ENDS = (0.0, 180.0)  # The servo's own ends, where its angle scale stops


class HeadAimParams(NodeParams):
    """How the head turns toward its target. Angles are degrees of the camera's view unless they say servo degrees."""

    px_per_degree: float = Field(
        default=8.1, gt=0.0,
        description="Image pixels per degree at the frame's centre: the OAK-1's 640 px over 69 degrees, a pinhole",
    )
    pan_ratio: float = Field(
        default=0.26, gt=0.0, le=1.0,
        description="Degrees the view turns per pan servo degree: 0.261 fitted to the robot's recorded steps (2026-11-06)",
    )
    tilt_ratio: float = Field(
        default=0.08, gt=0.0, le=1.0, description="Degrees the view nods per tilt servo degree: 0.081 fitted (2026-11-06)",
    )
    pan_slack: float = Field(
        default=2.0, ge=0.0, le=20.0,
        description="Servo degrees of play in the pan linkage taken up at each reversal: 2.5 fitted; a little under, "
                    "as too much makes the head jump",
    )
    tilt_slack: float = Field(
        default=1.5, ge=0.0, le=20.0, description="Servo degrees of play in the tilt linkage: 1.8 fitted",
    )
    shaper: bool = Field(
        default=False,
        description="Split each command so the head's own ringing cancels (a zero-vibration input shaper). Off: the "
                    "acceleration limit already keeps the ringing under the deadband, and the shaper's half-period "
                    "delay (0.28 s on pan) slowed every move (simulator/tuning.py, 2026-11-06)",
    )
    pan_mode: tuple[float, float] = Field(
        default=(11.84, 0.34),
        description="The pan head's ringing mode the shaper cancels: natural frequency (rad/s) and damping ratio, "
                    "fitted to the robot (1.9 Hz; damping 0.41 turning up and 0.28 down)",
    )
    tilt_mode: tuple[float, float] = Field(
        default=(37.97, 0.5), description="The tilt head's mode: 6 Hz, damping 0.64 turning up and 0.37 down",
    )
    edge_margin: float = Field(
        default=40.0, ge=0.0, le=200.0,
        description="Pixels from the picture's left or right edge within which a target must show up jump_samples "
                    "times in a row before the head turns to it: there partial bodies give brief false targets. Not "
                    "the bottom, where a standing person's face sits, as the head cannot look down (116-217 px below "
                    "the middle in run 20261106-222520), nor the top",
    )
    pan_direction: Literal[1, -1] = Field(default=1, description="+1: a target right of the aim point needs a larger pan angle")
    tilt_direction: Literal[1, -1] = Field(default=1, description="+1: a target below the aim point needs a larger tilt angle")
    end_margin: float = Field(default=5.0, ge=0.0, le=45.0, description="Servo degrees the head keeps clear of each end stop")
    # Pan turns the view steadily from servo 10 to 170 but jammed after being held at 5; tilt's linkage goes
    # over centre past about 140, where more servo angle turns the head back (full sweeps,
    # scripts/check_servo_travel.py, 2026-11-06). Following uses all of that travel (the owner, 2026-11-07:
    # "head servos should move their FULL range of motion"); motor tests may send anything in 0-180.
    pan_range: tuple[float, float] = Field(
        default=(10.0, 170.0), description="Servo degrees the pan may use: all the travel where its linkage turns the head",
    )
    # Tilt follows all the way down to the servo's own end (the owner, 2026-11-07: "the face track doesnt go ALL
    # the way down for tilt"): the head was seen going lower at 180 than at 140, the end the 2026-11-06 sweep set.
    tilt_range: tuple[float, float] = Field(
        default=(10.0, 180.0), description="Servo degrees the tilt may use: down to the servo's own end",
    )
    # The aim below was tuned on the measured plant (simulator/tuning.py, 2026-11-06: cost 64.4 against 2209 for
    # the stopgap it replaces; docs/audit-2026.md, "A simulator fitted to the robot").
    smoothing: float = Field(default=0.69, gt=0.0, le=1.0, description="Share of each new estimate taken into the goal")
    deadband: float = Field(default=0.39, ge=0.0, le=10.0, description="A still head turns only when its goal is further than this")
    max_speed: float = Field(default=15.0, gt=0.0, le=600.0, description="Top turning speed of the view, degrees per second")
    servo_speed: float = Field(
        default=150.0, gt=0.0, description="Top servo speed, servo degrees per second, slow enough that the servos keep up",
    )
    pan_servo_speed: float = Field(
        default=150.0, gt=0.0,
        description="Pan's own top servo speed, servo degrees per second, under servo_speed; the robot launches with 30",
    )
    max_acceleration: float = Field(
        default=34.0, gt=0.0,
        description="Degrees per second squared, speeding up and slowing down: a change of acceleration this large "
                    "starts ringing of about max_acceleration / omega_n^2 in pan's 1.9 Hz mode, 0.24 degrees, under "
                    "the deadband (300 gave 2.1, and swung a head geared twice the fit)",
    )
    jump_degrees: float = Field(default=8.0, gt=0.0, description="A goal further than this from the current one is a jump")
    jump_samples: int = Field(
        default=5, ge=1,
        description="Samples in a row a jump must show up in before the head turns to it: more than the robot's false "
                    "detections, which came 1-3 frames at a time",
    )
    candidate_gap: float = Field(
        default=0.2, ge=0.0,
        description="Seconds without a sighting after which a jump's count starts again: false detections that come "
                    "a burst at a time never add up to one, while a real person's dropped frames do not reset it",
    )
    reach: float = Field(
        default=0.30, gt=0.0, le=1.0,
        description="Share of a target's offset each frame aims for: the goal approaches without passing the target "
                    "while 1 - reach * g >= 0, g the real gearing over pan_ratio, so up to g = 3.3; later frames close the rest",
    )
    servo_lag: float = Field(
        default=0.055, ge=0.0, le=0.5,
        description="How far the camera trails the commands: pan's fitted 0.044 s dead time and a little of its "
                    "rise. Looked up in the commands kept each 10 ms control tick, so 0.055 reads those of 60 ms "
                    "before (the tuner's choice); a value on a tick, such as 0.05, would rest on rounding",
    )
    assumed_latency: float = Field(
        default=0.06, ge=0.0, le=0.5, description="Camera-to-target delay assumed for targets that do not say when they were captured",
    )


class HeadAim:
    """Feed `observe` each fresh target and call `update` every control tick for the servo commands.

    `body` is how far the body has turned (degrees, in pan's sense) as the caller estimates
    it; only its changes over the last second matter.
    """

    def __init__(self, params: HeadAimParams, pan: float = MIDDLE, tilt: float = MIDDLE) -> None:
        self.params = params
        self.servos = [pan, tilt]  # The servo angles last commanded
        self.view = [self._view(0, pan), self._view(1, tilt)]  # The view they turn the camera to, from straight ahead
        self.rates = [0.0, 0.0]  # How fast the view turns, degrees per second
        self.turning = [False, False]
        self.pushing = [0, 0]  # Which way each servo last pushed its linkage: +1, -1, or 0 not yet
        self.extreme = list(self.view)  # The furthest view reached since the last reversal
        self.body = 0.0  # The body's turn at the last update
        self.goal: tuple[float, float] | None = None  # Pan relative to the base (body + pan), and tilt
        self.candidate: tuple[float, float] | None = None  # A jump waiting to be believed
        self.candidate_count = 0
        self.candidate_seen = 0.0  # When the jump was last seen
        self.history: deque[tuple[float, float, float, float]] = deque()  # (time, body, pan, tilt view) commanded
        self.planned: deque[tuple[float, float, float]] = deque()  # (time, pan, tilt view) before shaping
        self.shaped = list(self.view)  # The views the servos are commanded to, after shaping
        self.commanded = list(self.view)  # The shaped views last turned into servo commands
        self.last_update: float | None = None

    @property
    def pan(self) -> float:
        return self.servos[0]

    @property
    def tilt(self) -> float:
        return self.servos[1]

    def _ratio(self, axis: int) -> float:
        return (self.params.pan_ratio, self.params.tilt_ratio)[axis]

    def _view(self, axis: int, servo: float) -> float:
        return self._ratio(axis) * (servo - MIDDLE)

    def _range(self, axis: int) -> tuple[float, float]:
        return (self.params.pan_range, self.params.tilt_range)[axis]

    def _slack(self, axis: int) -> float:
        return (self.params.pan_slack, self.params.tilt_slack)[axis]

    def span(self, axis: int) -> tuple[float, float]:
        """The views an axis can be commanded to (0 pan, 1 tilt): its servo, pushing past the view by half
        the slack, stays end_margin clear of an end stop inside its travel. At the servo's own end (0 or 180)
        there is no stop to keep clear of: the servo goes all the way."""
        ratio, push = self._ratio(axis), self._slack(axis) / 2
        low, high = self._range(axis)
        low_margin = push + (self.params.end_margin if low > SERVO_ENDS[0] else 0.0)
        high_margin = push + (self.params.end_margin if high < SERVO_ENDS[1] else 0.0)
        return ratio * (low + low_margin - MIDDLE), ratio * (high - high_margin - MIDDLE)

    def place(self, pan: float, tilt: float) -> None:
        """Something else moved the head here (a motor test): hold it here and forget the old goal."""
        self.servos, self.view = [pan, tilt], [self._view(0, pan), self._view(1, tilt)]
        self.rates, self.turning = [0.0, 0.0], [False, False]
        self.pushing, self.extreme, self.shaped, self.commanded = [0, 0], list(self.view), list(self.view), list(self.view)
        self.goal, self.candidate, self.candidate_count = None, None, 0
        self.history.clear()
        self.planned.clear()

    def bearing(self) -> float | None:
        """How far the target is from the body's forward direction, in pan's sense, or None without a goal."""
        return None if self.goal is None else self.goal[0] - self.body

    def _recorded(self, when: float) -> tuple[float, float, float]:
        """The body's turn and the commanded pan and tilt views at time `when`, from the recent commands."""
        chosen = None
        for entry in self.history:
            if entry[0] > when:
                break
            chosen = entry
        if chosen is None:
            chosen = self.history[0] if self.history else (when, self.body, self.view[0], self.view[1])
        return chosen[1], chosen[2], chosen[3]

    def pointing_at(self, captured: float) -> tuple[float, float]:
        """Where the camera pointed when a frame was captured (body + pan, and tilt): as commanded a servo lag earlier.

        While the body turns, that reads the body's turn a servo lag early too, which puts the estimate
        behind the person by the body's turn over the lag; the head's counter-rotation reaches the
        camera that same lag late, ahead of the person, and the two cancel."""
        body, pan, tilt = self._recorded(captured - self.params.servo_lag)
        return body + pan, tilt

    def observe(self, *, now: float, error_x: float, error_y: float, captured: float | None = None) -> None:
        """One detection: the target's offset in pixels from the aim point, in a frame captured at `captured`."""
        p = self.params
        pan_then, tilt_then = self.pointing_at(captured if captured is not None else now - p.assumed_latency)
        # Aim short of the target: an offset measured from afar carries the scale's error and the
        # parallax of a camera mounted ahead of the pan axis, both in proportion to the offset.
        focal = p.px_per_degree * 180.0 / math.pi  # The pinhole's focal length in pixels
        low, high = self.span(1)
        estimate = (
            pan_then + p.reach * p.pan_direction * math.degrees(math.atan(error_x / focal)),
            min(max(tilt_then + p.reach * p.tilt_direction * math.degrees(math.atan(error_y / focal)), low), high),
        )
        at_edge = abs(error_x) > FRAME[0] / 2 - p.edge_margin  # Left or right only: see edge_margin
        if self.goal is not None and math.dist(estimate, self.goal) <= p.jump_degrees and not at_edge:
            self.candidate, self.candidate_count = None, 0
            self.goal = tuple(goal + p.smoothing * (new - goal) for goal, new in zip(self.goal, estimate))
            return
        # Someone new or somewhere new: believe it once it has held for jump_samples samples in a row, none
        # more than candidate_gap after the last. Counting sightings alone, bursts of false detections
        # seconds apart added up to a jump, as did two on either side once reach brought them together.
        seen = captured if captured is not None else now
        if (self.candidate is not None and math.dist(estimate, self.candidate) <= p.jump_degrees
                and seen - self.candidate_seen <= p.candidate_gap):
            self.candidate_count += 1
            self.candidate = tuple((old + new) / 2 for old, new in zip(self.candidate, estimate))
        else:
            self.candidate, self.candidate_count = estimate, 1
        self.candidate_seen = seen
        if self.candidate_count >= p.jump_samples:
            self.goal, self.candidate, self.candidate_count = self.candidate, None, 0

    def _shape(self, now: float) -> None:
        """The views to command now: each planned view split by the axis's zero-vibration shaper, A1 of it
        now and A2 of what was planned half a ringing period ago, so the ringing the first part starts,
        the second cancels."""
        p = self.params
        self.planned.append((now, self.view[0], self.view[1]))
        for axis, mode in enumerate((p.pan_mode, p.tilt_mode)):
            if not p.shaper:
                self.shaped[axis] = self.view[axis]
                continue
            first, second, delay = zero_vibration(*mode)
            earlier = self.planned[0][axis + 1]
            for time, *views in self.planned:
                if time > now - delay:
                    break
                earlier = views[axis]
            self.shaped[axis] = first * self.view[axis] + second * earlier
        while len(self.planned) > 1 and self.planned[1][0] <= now - HISTORY_SECONDS:
            self.planned.popleft()

    def _command(self, axis: int) -> None:
        """The servo angle that puts the linkage at this axis's shaped view: past it by half the slack, the way
        it moves.

        A linkage with play follows its servo only once the servo pushes it from one side. After a
        reversal the servo first crosses the play while the head stands still; pushing half the slack
        ahead of the view takes the play up at once. A turn back must exceed SLACK_TURN to count, so
        the servo never chatters across the play."""
        view = self.shaped[axis]
        moved = view - self.extreme[axis]
        if (self.pushing[axis] >= 0 and moved < -SLACK_TURN) or (self.pushing[axis] <= 0 and moved > SLACK_TURN):
            self.pushing[axis], self.extreme[axis] = (1 if moved > 0 else -1), view
        elif self.pushing[axis] * moved > 0:
            self.extreme[axis] = view  # Still going the same way
        servo = MIDDLE + view / self._ratio(axis) + self.pushing[axis] * self._slack(axis) / 2
        low, high = self._range(axis)
        self.servos[axis] = min(max(servo, low), high)

    def update(self, *, now: float, body: float = 0.0) -> tuple[float, float]:
        """This tick's pan and tilt servo commands, for a body turned `body` degrees."""
        p = self.params
        dt = 0.0 if self.last_update is None else min(max(now - self.last_update, 0.0), MAX_STEP)
        self.last_update = now
        body_turning = body != self.body
        self.body = body
        if self.goal is not None:
            for axis, aim in enumerate((self.goal[0] - body, self.goal[1])):
                low, high = self.span(axis)
                wanted = min(max(aim, low), high)
                if not self.turning[axis] and (abs(wanted - self.view[axis]) > p.deadband or (axis == 0 and body_turning)):
                    self.turning[axis] = True  # Counter-rotate against the body at once
                if self.turning[axis]:
                    servo_speed = min(p.servo_speed, p.pan_servo_speed) if axis == 0 else p.servo_speed
                    speed = min(p.max_speed, servo_speed * self._ratio(axis))
                    self.view[axis], self.rates[axis] = s_curve_step(
                        self.view[axis], self.rates[axis], wanted, up=speed, down=speed, jerk=p.max_acceleration, dt=dt,
                    )
                    if self.view[axis] == wanted and self.rates[axis] == 0.0:
                        self.turning[axis] = False
        self._shape(now)
        for axis in (0, 1):
            if self.shaped[axis] != self.commanded[axis]:  # Also after turning stops: the shaper's second part lands later
                self._command(axis)
                self.commanded[axis] = self.shaped[axis]
        self.history.append((now, body, self.shaped[0], self.shaped[1]))
        while self.history and self.history[0][0] < now - HISTORY_SECONDS:
            self.history.popleft()
        return self.pan, self.tilt
