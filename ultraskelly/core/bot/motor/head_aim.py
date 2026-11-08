"""Head aiming from an internal model of the head: where the camera pointed, where the person will be, how to turn.

The camera's picture is the head's only sensor of motion. Each frame says where the person was relative to
where the camera pointed when the frame was captured, and arrives some 60-100 ms later. The old aim took
"where the camera pointed" to be the commanded angle one servo lag earlier, which ignores the head's own
ringing, so it could take only 30 % of each offset without swinging, and smoothed and slowed that again
(docs/control-design.md).

**Where the person is.** HeadAim keeps an internal model of the head: an efference copy, a Smith predictor
in angle space. Per axis it runs the servo commands it sent through the chain fitted to the robot
(simulator/plant.py): dead time, the servo's slew, the linkage's slack and gearing, and the head's own mode,
damped differently each way. That model says where the camera pointed at each frame's capture time; adding
the person's angle in the picture gives where the person is in the base's frame, with the body's turn from
the waist's own model (waist_drive.py). That estimate stays put while the head turns, so the camera's delay
delays the response without feeding back; only the model's errors feed back.

**Where they will be.** An alpha-beta tracker follows each axis's direction in the base's frame and predicts
it forward across the head's own delay (`prediction_lead`). It is reset whenever someone new is believed, is
never fed detections older than `stale_after`, and learns the person's speed only within `follow_offset`
of the picture's middle and never during a planned move or while the waist breaks loose or stops (then the
estimate drifts with the models' errors). That speed counts only above `min_target_speed`, for at most `max_lead` degrees and `coast`
seconds after the last sighting, scaled by `prediction_gain` (a walker who stops is overshot by about the
lead).

**Who.** Someone new, or somewhere new (further than `jump_degrees` from the tracked direction), is believed
once seen in `confirm_frames` of the last `confirm_window` frames: more than the 1-3 frames the robot's false
detections came in. By the picture's left or right edge, where partial bodies give them, every sighting is
confirmed that way, even beside the person followed.

**How to turn: a planned move and a feedback.** Each axis's command is the sum of two parts:

- *A planned move* on every re-target (someone new believed): `reach` of the way to where they will be, a
  rest-to-rest move at the axis's `*_speed` and `*_acceleration`, split by a zero-vibration input shaper (A1
  of it now and A2 half a ringing period later) so the ringing it starts in pan's 1.9 Hz mode cancels. The
  shaper sits on this planned path only, never inside the feedback.
- *A feedback* that closes the rest, and follows people as they move: proportional on what the plan leaves
  of the predicted direction (`*_gain`, view degrees per second per degree), with the person's own speed fed
  forward, its rate changing at most `*_acceleration`. It ends each move on an exponential approach with no
  corner, so it does not ring the head, and with no integral it cannot hunt across the slack.

A settled axis holds still until its aim is more than `*_resume` away, and settles once within `*_settle`
with its move done: a hysteresis band around pan's 0.65 degrees of play, so detection jitter never moves it.
The servo is commanded half its linkage's slack past the view in the way it moves, so a reversal takes up the
play at once. The internal model is driven by the servo commands actually sent, so it includes the shaper
and the slack.

Angles are degrees of the camera's view from straight ahead unless they say servo degrees. Views are
commanded (the plan, the feedback, the shaped command) or modelled (the internal model's); none is measured.
"""
import math
from collections import deque
from typing import Callable, Literal

from pydantic import Field, model_validator

from ultraskelly.core.bot.base_abcs import NodeParams
from ultraskelly.core.bot.motor.waist_drive import MAX_STEP

MIDDLE = 90.0  # Servo degrees at which the head faces the way the body does
HISTORY_SECONDS = 1.0  # How far back the head remembers where its model says the camera pointed
SLACK_TURN = 0.2  # Degrees of view the head must turn back before its servo takes up the slack the other way
FRAME = (640, 480)  # The picture the targets are measured in, pixels
SERVO_ENDS = (0.0, 180.0)  # The servo's own ends, where its angle scale stops
STILL_RATE = 1.0  # Degrees of view per second below which a feedback within its settle band may stop at once
FRAME_PERIOD = 1 / 30  # Seconds between the camera's frames until they show it: the OAK runs at 30 a second
FRAMES_KEPT = 8  # Recent capture times the frame period is learned from
BODY_JUMP = 5.0  # Degrees the modelled body cannot turn in one tick: a jump is a new model, not a turn
OFFSET_BASELINE = 0.2  # Seconds over which the person's offset in the picture must hold for their speed to be learned


def zero_vibration(frequency: float, damping: float) -> tuple[float, float, float]:
    """A two-impulse zero-vibration shaper for a mode of natural `frequency` (rad/s) and `damping` ratio:
    (A1, A2, delay). A command split into A1 now and A2 half a ringing period later cancels the ringing
    the first part starts: K = exp(-damping pi / sqrt(1 - damping^2)), A1 = 1/(1+K), A2 = K/(1+K)."""
    ringing = frequency * math.sqrt(1.0 - damping ** 2)
    k = math.exp(-damping * math.pi / math.sqrt(1.0 - damping ** 2))
    return 1.0 / (1.0 + k), k / (1.0 + k), math.pi / ringing


def second_order(offset: float, velocity: float, frequency: float, damping: float, dt: float) -> tuple[float, float]:
    """Exactly where an underdamped mass-spring settling to 0 is `dt` later: (offset, velocity)."""
    if damping >= 1.0:  # Critically or over damped: small Euler steps are plenty
        steps = max(1, math.ceil(dt * frequency * 20))
        h = dt / steps
        for _ in range(steps):
            velocity += (-frequency ** 2 * offset - 2 * damping * frequency * velocity) * h
            offset += velocity * h
        return offset, velocity
    decay, ringing = damping * frequency, frequency * math.sqrt(1.0 - damping ** 2)
    fade, c, s = math.exp(-decay * dt), math.cos(ringing * dt), math.sin(ringing * dt)
    return (fade * (offset * c + (velocity + decay * offset) / ringing * s),
            fade * (velocity * c - (decay * velocity + frequency ** 2 * offset) / ringing * s))


def move_step(level: float, rate: float, target: float, *, speed: float, acceleration: float, dt: float) -> tuple[float, float]:
    """One tick of a rest-to-rest move toward `target`: its rate within `speed` either way, changing at most
    `acceleration` per second squared, the fastest that can still ease to rest on arrival. Returns the new
    level and rate."""
    error = target - level
    if error == 0.0 and rate == 0.0:
        return level, rate
    half_step = acceleration * dt / 2.0
    wanted = math.copysign(min(speed, math.sqrt(half_step ** 2 + 2.0 * acceleration * abs(error)) - half_step), error)
    rate += max(-acceleration * dt, min(acceleration * dt, wanted - rate))
    moved = level + rate * dt
    if (target - moved) * error <= 0.0 and abs(rate) <= acceleration * dt:  # Arrived slowly enough to stop: settle
        return target, 0.0
    return moved, rate


class HeadAimParams(NodeParams):
    """How the head turns toward its target. Angles are degrees of the camera's view unless they say servo degrees."""

    px_per_degree: float = Field(
        default=8.1, gt=0.0, le=50.0,
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
    edge_margin: float = Field(
        default=40.0, ge=0.0, le=200.0,
        description="Pixels from the picture's left or right edge within which every target must be confirmed as "
                    "someone new is, even beside the person followed: there partial bodies give brief false targets. "
                    "Not the bottom, where a standing person's face sits, as the head cannot look down (116-217 px "
                    "below the middle in run 20261106-222520), nor the top",
    )

    # The internal model of each axis (the efference copy), fitted to the robot's recorded steps, staircases and
    # ramps (simulator/plant.py, tests/test_plant_fidelity.py, 2026-11-06). The gearing and slack above are its too.
    pan_dead_time: float = Field(default=0.044, ge=0.0, le=0.3, description="Seconds from a pan command to the picture starting to move: 0.044 fitted")
    tilt_dead_time: float = Field(default=0.040, ge=0.0, le=0.3, description="Seconds from a tilt command to the picture moving: 0.040 fitted")
    pan_slew: float = Field(default=576.0, gt=0.0, le=3000.0, description="The pan servo's own top speed, servo degrees per second: 576 fitted")
    tilt_slew: float = Field(default=830.0, gt=0.0, le=3000.0, description="The tilt servo's top speed, servo degrees per second: 830 fitted")
    pan_frequency: float = Field(
        default=11.84, ge=1.0, le=200.0,
        description="Natural frequency of the head's pan mode, radians per second: 11.84 fitted (1.9 Hz). The shaper cancels it",
    )
    pan_damping: float = Field(
        default=0.41, gt=0.0, le=2.0, description="Damping ratio of pan's mode turning toward larger pan views: 0.41 fitted",
    )
    pan_damping_down: float = Field(
        default=0.28, gt=0.0, le=2.0, description="Damping ratio of pan's mode turning toward smaller pan views: 0.28 fitted",
    )
    tilt_frequency: float = Field(default=37.97, ge=1.0, le=300.0, description="Tilt's mode, radians per second: 37.97 fitted (6 Hz)")
    tilt_damping: float = Field(default=0.64, gt=0.0, le=2.0, description="Damping ratio of tilt's mode turning toward larger views: 0.64 fitted")
    tilt_damping_down: float = Field(default=0.37, gt=0.0, le=2.0, description="Damping ratio of tilt's mode the other way: 0.37 fitted")

    # How the head turns. The defaults are the robot's launched tuning (simulator/tuning.py; docs/control-design.md).
    pan_gain: float = Field(
        default=1.7, gt=0.0, le=10.0,
        description="Pan's feedback: view degrees per second of turn per degree the person is off where the plan "
                    "ends (SIMC on the fitted delay: 2-3)",
    )
    tilt_gain: float = Field(default=1.7, gt=0.0, le=10.0, description="Tilt's feedback, degrees per second per degree off")
    pan_speed: float = Field(
        default=48.0, gt=0.0, le=150.0,
        description="Top speed of pan's planned move and of its feedback, degrees of view per second: the servo's own "
                    "576 servo degrees a second is 150",
    )
    tilt_speed: float = Field(
        default=13.0, gt=0.0, le=67.0,
        description="Top speed of tilt's, degrees of view per second: 12 is 150 tilt servo degrees a second",
    )
    pan_acceleration: float = Field(
        default=25.0, gt=0.0, le=1000.0,
        description="Pan's acceleration, speeding up and slowing down, degrees of view per second squared, for its "
                    "planned move and the feedback's rate. Unshaped, a change of acceleration this large starts "
                    "ringing of about pan_acceleration / pan_frequency^2 in the 1.9 Hz mode (0.18 degrees at 25)",
    )
    tilt_acceleration: float = Field(
        default=22.0, gt=0.0, le=1000.0,
        description="Tilt's acceleration, degrees of view per second squared: its 6 Hz mode rings a / 1442 per change",
    )
    shaper: bool = Field(
        default=True,
        description="Split each planned move so the head's own ringing cancels (a zero-vibration input shaper on pan's "
                    "and tilt's modes): A1 of it now and A2 half a ringing period (0.28 s on pan) later",
    )
    reach: float = Field(
        default=0.26, ge=0.0, le=1.0,
        description="Share of the way to someone newly believed that the planned move turns; the feedback closes the "
                    "rest. The move stops short of them while reach x (real gearing x real pixels per degree, over "
                    "those assumed, x the parallax of a camera ahead of the pan axis, about 1.09) <= 1",
    )

    # Where the person will be
    tracking_alpha: float = Field(
        default=0.89, gt=0.0, le=1.0,
        description="Share of each new estimate's surprise taken into the tracked direction (an alpha-beta tracker)",
    )
    tracking_beta: float = Field(
        default=0.001, ge=0.0, le=1.0,
        description="Share of each new estimate's surprise, per frame interval, taken into the tracked speed",
    )
    prediction_gain: float = Field(
        default=0.47, ge=0.0, le=1.5,
        description="Share of the tracked speed the head leads the person by, and feeds forward: 0 aims where they "
                    "were. A walker who stops is overshot by about the lead",
    )
    prediction_lead: float = Field(
        default=0.043, ge=0.0, le=0.5,
        description="Seconds ahead of now the head aims and the waist's planned turn is taken: about its own "
                    "delay, pan's 0.044 s dead time plus 2 x damping / frequency of its mode, 0.058 s (tuned shorter)",
    )
    follow_offset: float = Field(
        default=10.0, ge=0.0, le=40.0,
        description="Degrees from the picture's middle within which a sighting teaches the tracker the person's speed "
                    "(never during a planned move, when the estimate drifts with the model's errors)",
    )
    steady_offset_rate: float = Field(
        default=4.0, ge=0.0, le=50.0,
        description="Degrees per second the person's angle in the picture may change over 0.2 s while the tracker "
                    "learns their speed: a person the head follows holds their place; one it still turns to closes in",
    )
    min_target_speed: float = Field(
        default=5.4, ge=0.0, le=30.0,
        description="Degrees per second a person must be tracked moving at before the head leads them: still people, "
                    "detection jitter and the estimate's drift as the head lands (up to 0.4 x its speed with the "
                    "gearing 40 % off) give no lead",
    )
    max_target_speed: float = Field(default=60.0, gt=0.0, le=300.0, description="Degrees per second the tracked speed is limited to")
    max_lead: float = Field(default=3.0, ge=0.0, le=20.0, description="Degrees the head may lead a moving person by at most")
    coast: float = Field(
        default=0.3, ge=0.0, le=2.0,
        description="Seconds after the last sighting for which the head still leads where the person was going; then it holds",
    )
    counter_rotation: float = Field(
        default=1.13, ge=0.0, le=1.5,
        description="Share of the body's planned turn over prediction_lead that the head counter-rotates against "
                    "ahead of time, as its own commands take that long to reach the camera; 0 counters only the "
                    "turn so far",
    )

    # The settled band: hysteresis around the linkages' play, so jitter never moves a settled head
    pan_settle: float = Field(default=0.3, ge=0.0, le=5.0, description="A turning pan settles once within this many degrees of its aim")
    pan_resume: float = Field(
        default=0.9, ge=0.0, le=5.0,
        description="A settled pan turns again once its aim is further than this, degrees: about pan's 0.65 degrees of play",
    )
    tilt_settle: float = Field(default=0.2, ge=0.0, le=5.0, description="A turning tilt settles within this many degrees of its aim")
    tilt_resume: float = Field(
        default=1.0, ge=0.0, le=5.0,
        description="A settled tilt turns again once its aim is further than this, degrees: tilt's play is only 0.15, "
                    "but its few degrees of reach make jitter the larger share",
    )

    # Believing someone new
    jump_degrees: float = Field(
        default=8.0, gt=0.0, le=90.0, description="Degrees from the tracked direction beyond which a sighting is someone new, or a false one",
    )
    confirm_frames: int = Field(
        default=4, ge=1, le=10,
        description="Frames, of the last confirm_window, a new target must be seen in before the head turns to it: more "
                    "than the robot's false detections, which came 1-3 frames at a time (run 20261106-194104)",
    )
    confirm_window: int = Field(default=5, ge=1, le=10, description="Frames over which confirm_frames are counted")
    stale_after: float = Field(
        default=0.2, gt=0.0, le=1.0,
        description="Seconds after its capture beyond which a detection is too old to steer by: the pose node's "
                    "delay is 50-70 ms",
    )
    assumed_latency: float = Field(
        default=0.06, ge=0.0, le=0.5, description="Camera-to-target delay assumed for targets that do not say when they were captured",
    )

    @model_validator(mode="after")
    def _consistent(self) -> "HeadAimParams":
        if self.confirm_frames > self.confirm_window:
            raise ValueError("confirm_frames must not exceed confirm_window")
        if self.pan_settle > self.pan_resume or self.tilt_settle > self.tilt_resume:
            raise ValueError("A settle band must not exceed its resume band")
        return self


# The head's two tunings, found by simulator/tuning.py on the plant fitted to the robot (2026-11-07). The defaults
# above are "calm", which the robot launches: its acceleration at most 25 degrees of view per second squared, robust
# on the 59 random plants and the gearing and pixel-scale corners. "Responsive" centres a 20-degree step within 1 s
# (the least acceleration that does, with the shaper on), but its planned move overshoots 2.4-3 degrees when the
# gearing is 15 % above the fit (or 30 %, as pan has measured), so it waits until the gearing is measured to +-10 %.
RESPONSIVE = {  # The responsive tuning (docs/control-design.md): HeadAimParams(**RESPONSIVE) runs it
    'pan_gain': 2.9,
    'tilt_gain': 1.7,
    'pan_speed': 133.0,
    'tilt_speed': 9.9,
    'pan_acceleration': 110.0,
    'tilt_acceleration': 63.0,
    'reach': 0.8,
    'tracking_alpha': 0.69,
    'tracking_beta': 0.009,
    'prediction_gain': 0.28,
    'prediction_lead': 0.12,
    'min_target_speed': 5.8,
    'pan_settle': 0.49,
    'pan_resume': 1.1,
    'tilt_settle': 0.11,
    'tilt_resume': 0.59,
}


def play(output: float, driver: float, slack: float) -> float:
    """Where a linkage with `slack` degrees of play sits: it moves only when the driver pushes it from one side."""
    return min(max(output, driver - slack / 2), driver + slack / 2)


class AxisModel:
    """The head's internal model of one axis (the efference copy): where its view is, from the servo commands sent.
    It reads its numbers from the params each time, so a change of tuning takes effect at once."""

    def __init__(self, params: HeadAimParams, axis: int, servo: float) -> None:
        self.params, self.axis = params, axis
        self.place(servo)

    def _fitted(self) -> tuple[float, float, float, float, float, float, float]:
        p = self.params
        if self.axis == 0:
            return p.pan_ratio, p.pan_dead_time, p.pan_slew, p.pan_slack, p.pan_frequency, p.pan_damping, p.pan_damping_down
        return p.tilt_ratio, p.tilt_dead_time, p.tilt_slew, p.tilt_slack, p.tilt_frequency, p.tilt_damping, p.tilt_damping_down

    def place(self, servo: float) -> None:
        """At rest with its servo at `servo`, as after a motor test."""
        self.commands: deque[tuple[float, float]] = deque([(-math.inf, servo)])  # (time sent, servo degrees)
        self.servo = self.linkage = servo
        self.view, self.velocity = self._fitted()[0] * (servo - MIDDLE), 0.0

    def command(self, time: float, servo: float) -> None:
        if servo != self.commands[-1][1]:
            self.commands.append((time, servo))

    def advance(self, now: float, dt: float) -> float:
        """Move the model on by `dt` to time `now`; returns its view."""
        ratio, dead_time, slew, slack, frequency, damping, damping_down = self._fitted()
        due = now - dead_time
        while len(self.commands) > 1 and self.commands[1][0] <= due:
            self.commands.popleft()
        step = slew * dt
        self.servo += min(max(self.commands[0][1] - self.servo, -step), step)
        self.linkage = play(self.linkage, self.servo, slack)
        goal = ratio * (self.linkage - MIDDLE)
        offset, self.velocity = second_order(self.view - goal, self.velocity, frequency,
                                             damping if self.velocity >= 0.0 else damping_down, dt)
        self.view = goal + offset
        return self.view


class Track:
    """An alpha-beta tracker of one axis's direction to the person, in the base's frame (modelled views plus what
    the picture showed: an estimate, not a measurement)."""

    def __init__(self, position: float, time: float) -> None:
        self.position, self.velocity, self.time = position, 0.0, time

    def update(self, estimate: float, time: float, alpha: float, beta: float, top: float, learn: bool = True) -> None:
        """One estimate from a frame captured at `time`; `learn`: whether it may teach the tracker the speed."""
        dt = time - self.time
        if dt <= 0.0:  # The same frame again, or an older one: nothing new to learn its speed from
            return
        predicted = self.position + self.velocity * dt
        surprise = estimate - predicted
        self.position = predicted + alpha * surprise
        if learn:
            self.velocity = min(max(self.velocity + beta * surprise / dt, -top), top)
        self.time = time

    def lead(self, when: float, p: HeadAimParams) -> tuple[float, float]:
        """Where the person will be at `when`, and how fast they go: the head leads them only while they are
        tracked moving faster than min_target_speed, and for at most `coast` seconds after the last sighting."""
        if abs(self.velocity) < p.min_target_speed or when - self.time > p.coast + p.prediction_lead:
            return self.position, 0.0
        speed = p.prediction_gain * self.velocity
        return self.position + min(max(speed * (when - self.time), -p.max_lead), p.max_lead), speed


class HeadAim:
    """Feed `frame` each fresh frame and `observe` each fresh target in one, and call `update` every control tick
    for the servo commands.

    `body` is how far the body has turned (degrees, in pan's sense) by the waist's model; only its changes
    over the last second matter.
    """

    def __init__(self, params: HeadAimParams, pan: float = MIDDLE, tilt: float = MIDDLE) -> None:
        self.params = params
        self.models = [AxisModel(params, 0, pan), AxisModel(params, 1, tilt)]
        self.place(pan, tilt)

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
        """Something else moved the head here (a motor test): hold it here and forget the person."""
        self.servos = [pan, tilt]  # The servo angles last commanded
        views = [self._view(0, pan), self._view(1, tilt)]
        # The planned moves and the feedback's part, pan's in the base's frame: the views less the body's turn
        self.plan, self.plan_rate, self.plan_end = list(views), [0.0, 0.0], list(views)
        self.plan_target = list(views)  # Where each plan heads now: its end, or the span's end before it
        self.correction, self.correction_rate = [0.0, 0.0], [0.0, 0.0]
        self.shift = [0.0, 0.0]  # The body's turn, as the commands meet it, that pan's commands take off
        self.shifted, self.span_rate = [0.0, 0.0], [0.0, 0.0]  # The last shift, and how fast it moves
        self.view = list(views)  # Plan plus feedback less the shift, before shaping: where each axis is commanded to go
        self.shaped = list(views)  # The views the servos are commanded to, after shaping
        self.settled = [True, True]
        self.retarget = [False, False]  # Someone new was believed: plan a move at the next update
        self.replanning = False  # The waist starts a move: plan pan's rest of the way at the next update
        self.pushing = [0, 0]  # Which way each servo last pushed its linkage: +1, -1, or 0 not yet
        self.extreme = list(views)  # The furthest shaped view reached since the last reversal
        self.body = 0.0  # The body's modelled turn now
        self.body_at: Callable[[float], float] | None = None  # Where the waist's model and plan put it ahead
        self.shaped_shift = 0.0  # The body's turn the shaped counter-rotation takes off
        for model, servo in zip(self.models, self.servos):
            model.place(servo)
        self.tracks: list[Track] | None = None  # The person followed: pan (base frame) and tilt
        self.candidate: tuple[float, float] | None = None  # Someone new, not believed yet
        self.sightings: deque[float] = deque()  # The capture times the candidate was seen at
        # (time, modelled pan view, tilt view, body turn, whether the body turned steadily then), each control tick
        self.history: deque[tuple[float, float, float, float, bool]] = deque()
        self.planned: deque[tuple[float, float, float]] = deque()  # (time, pan, tilt planned move) before shaping
        self.last_update: float | None = None
        self.last_frame: float | None = None  # The capture time of the newest detection counted
        self.frames: deque[float] = deque(maxlen=FRAMES_KEPT)  # Recent frames' capture times, to learn the frame period
        self.offsets: deque[tuple[float, float, float]] = deque()  # (capture time, the person's angle across and down)

    @property
    def candidate_count(self) -> int:
        """How many times the candidate has been seen lately."""
        return len(self.sightings)

    def demand(self) -> float | None:
        """How far the person followed is from the body's forward direction, in pan's sense (the pan the head
        would need, even beyond its reach), or None without one."""
        return None if self.tracks is None else self.tracks[0].position - self.body

    def pan_offset(self) -> float:
        """The pan view the head is commanded toward (plan plus feedback less the body's turn, before shaping):
        its offset from the body's front. Commanded, not measured."""
        return self.view[0]

    def replan(self) -> None:
        """The waist starts a move: if pan holds at an end of its span with a still person beyond, plan the rest
        of the way to them in the base's frame now. The planned move then follows the span's end as the body
        turns, no faster than it can still stop on the person, so the camera lands on them while the waist
        turns, rather than finishing the approach after it stops."""
        self.replanning = True

    def planning(self, axis: int = 0) -> bool:
        """Whether an axis's planned move is still under way (pan's by default): not yet at its end, or at the
        span's end where its own lies beyond."""
        return self.plan_rate[axis] != 0.0 or self.plan[axis] != self.plan_target[axis]

    def _pointing(self, when: float) -> tuple[float, float, float, bool]:
        """Where the internal model says the camera pointed at `when` (pan and tilt views) and the body's turn
        then, interpolated between control ticks, and whether the body was turning steadily (or not at all)."""
        if not self.history:
            return self.models[0].view, self.models[1].view, self.body, True
        before = self.history[0]
        if when <= before[0]:
            return before[1:]
        for entry in self.history:
            if entry[0] >= when:
                share = (when - before[0]) / (entry[0] - before[0]) if entry[0] > before[0] else 1.0
                pan, tilt, body = (a + share * (b - a) for a, b in zip(before[1:4], entry[1:4]))
                return pan, tilt, body, before[4] and entry[4]
            before = entry
        return before[1:]

    def frame(self, *, now: float, captured: float | None = None) -> None:
        """A fresh frame from the camera, with or without anyone in it: the frame period is learned from these."""
        seen = captured if captured is not None else now - self.params.assumed_latency
        if not self.frames or seen > self.frames[-1]:
            self.frames.append(seen)

    def frame_period(self) -> float:
        """Seconds between the camera's frames: the median gap between recent frames, which late deliveries and
        dropped frames leave alone while under half of them, or FRAME_PERIOD until two have come."""
        gaps = sorted(b - a for a, b in zip(self.frames, list(self.frames)[1:]) if b > a)
        return min(max(gaps[len(gaps) // 2], 1 / 240), 0.5) if gaps else FRAME_PERIOD

    def _confirmed(self, seen: float) -> bool:
        """Whether the candidate, just seen in a frame captured at `seen`, has been seen in confirm_frames of the
        last confirm_window frames (as many frames as fit in the time, with half a frame's grace)."""
        p = self.params
        window = (p.confirm_window - 0.5) * self.frame_period()
        while self.sightings and self.sightings[0] <= seen - window:
            self.sightings.popleft()
        return len(self.sightings) >= p.confirm_frames

    def observe(self, *, now: float, error_x: float, error_y: float, captured: float | None = None) -> None:
        """One detection: the target's offset in pixels from the aim point, in a frame captured at `captured`."""
        p = self.params
        seen = captured if captured is not None else now - p.assumed_latency
        if now - seen > p.stale_after or (self.last_frame is not None and seen <= self.last_frame):
            return  # Too old to steer by, or a frame already counted
        self.last_frame = seen
        self.frame(now=now, captured=captured)
        pan_then, tilt_then, body_then, steady = self._pointing(seen)
        focal = p.px_per_degree * 180.0 / math.pi  # The pinhole's focal length in pixels
        seen_x, seen_y = math.degrees(math.atan(error_x / focal)), math.degrees(math.atan(error_y / focal))
        estimate = (body_then + pan_then + p.pan_direction * seen_x, tilt_then + p.tilt_direction * seen_y)
        # Only a person the camera follows shows how fast they go: one it follows holds their place in the
        # picture, while one it is still turning to closes in, and then the estimate drifts with the model's
        # errors (the gearing and pixel scale's, and the parallax) as the head turns; so it does while the waist
        # breaks loose or stops, with the waist model's.
        self.offsets.append((seen, seen_x, seen_y))
        while len(self.offsets) > 2 and self.offsets[1][0] <= seen - OFFSET_BASELINE:
            self.offsets.popleft()
        then, then_x, then_y = self.offsets[0]
        holding = seen - then >= OFFSET_BASELINE / 2 and math.hypot(seen_x - then_x, seen_y - then_y) <= p.steady_offset_rate * (seen - then)
        following = math.hypot(seen_x, seen_y) <= p.follow_offset and holding and steady and not self.planning(0)
        at_edge = abs(error_x) > FRAME[0] / 2 - p.edge_margin  # Left or right only: see edge_margin
        if self.tracks is not None and not at_edge:
            tracked = tuple(track.position + track.velocity * (seen - track.time) for track in self.tracks)
            if math.dist(estimate, tracked) <= p.jump_degrees:
                for track, value in zip(self.tracks, estimate):
                    track.update(value, seen, p.tracking_alpha, p.tracking_beta, p.max_target_speed, following)
                return
        # Someone new or somewhere new: believe it once seen in enough of the last frames
        if self.candidate is None or math.dist(estimate, self.candidate) > p.jump_degrees:
            self.sightings = deque()
        self.candidate = estimate
        self.sightings.append(seen)
        if self._confirmed(seen):
            self.tracks = [Track(estimate[0], seen), Track(estimate[1], seen)]
            self.candidate, self.sightings = None, deque()
            self.retarget = [True, True]

    def _rebase(self, jump: float) -> None:
        """Move everything held in the base's frame by `jump`, so the head holds still when the body's model
        starts again from another angle."""
        self.plan[0] += jump
        self.plan_end[0] += jump
        self.plan_target[0] += jump
        if self.tracks is not None:
            self.tracks[0].position += jump
        if self.candidate is not None:
            self.candidate = (self.candidate[0] + jump, self.candidate[1])
        self.history = deque((t, pan, tilt, body + jump, steady) for t, pan, tilt, body, steady in self.history)

    def shaper_lead(self) -> float:
        """Seconds the pan shaper delays a planned path on average (A2 x its delay), 0 without the shaper."""
        p = self.params
        if not p.shaper:
            return 0.0
        _, second, delay = zero_vibration(p.pan_frequency, (p.pan_damping + p.pan_damping_down) / 2)
        return second * delay

    def _shift(self, lead: float) -> float:
        """How far the body will have turned `lead` seconds from now, by the waist's model and plan (the share
        counter_rotation of the turn still to come): pan is planned in the base's frame and commanded less
        this, so it counter-rotates against the body's turn as its commands reach the camera."""
        ahead = self.body if self.body_at is None else self.body_at(self.last_update + lead)
        return self.body + self.params.counter_rotation * (ahead - self.body)

    def _turn(self, axis: int, now: float, dt: float) -> None:
        """Move this axis's plan and feedback on by one tick, in the base's frame for pan."""
        p = self.params
        gain, speed, acceleration, settle, resume = (
            (p.pan_gain, p.pan_speed, p.pan_acceleration, p.pan_settle, p.pan_resume) if axis == 0
            else (p.tilt_gain, p.tilt_speed, p.tilt_acceleration, p.tilt_settle, p.tilt_resume))
        # Where the person will be across the head's own delay, and how fast they go
        aim, wanted_rate = self.tracks[axis].lead(now + p.prediction_lead, p)
        low, high = (end + self.shift[axis] for end in self.span(axis))  # The span as the body will have it
        if dt:
            self.span_rate[axis] = (self.shift[axis] - self.shifted[axis]) / dt  # How fast the body moves the span
        self.shifted[axis] = self.shift[axis]
        if aim <= low or aim >= high:
            wanted_rate = 0.0
        wanted = min(max(aim, low), high)  # Beyond its reach an axis goes to the end of its span and holds there
        if self.retarget[axis]:  # Someone new: a planned move, reach of the way there, within the span
            self.retarget[axis] = False
            self.plan_end[axis] = min(max(self.plan[axis] + p.reach * (wanted - self.plan[axis] - self.correction[axis]), low), high)
            self.settled[axis] = False
        if self.replanning and axis == 0:  # The waist is about to bring a still person beyond reach within it
            self.replanning = False
            if wanted_rate == 0.0 and (aim > high or aim < low):
                self.plan_end[0] = self.plan[0] + p.reach * (aim - self.plan[0] - self.correction[0])
                self.settled[0] = False
        # The planned move heads for its end, or for the span's end while its own lies beyond: it stops there
        # without hitting it, and as the body turns the span, it follows no faster than it can still stop on
        # its own end, which the body's turn then brings within reach.
        target = self.plan_target[axis] = min(max(self.plan_end[axis], low - self.correction[axis]), high - self.correction[axis])
        blocked = 0 if target == self.plan_end[axis] else (1 if target < self.plan_end[axis] else -1)
        self.plan[axis], self.plan_rate[axis] = move_step(
            self.plan[axis], self.plan_rate[axis], target, speed=speed, acceleration=acceleration, dt=dt,
        )
        off = wanted - target - self.correction[axis]  # What the plan leaves the feedback, within reach
        if self.settled[axis] and (abs(off) > resume or wanted_rate != 0.0):
            self.settled[axis] = False  # Out of the band, or following someone moving
        if not self.settled[axis]:
            # Toward the person, but beyond the span only a settle band past its end: the feedback reaches the
            # end, slowly enough not to ring the head when the span stops it, rather than easing short of it.
            # Proportional, but never faster than it can still stop from at its acceleration.
            goal = min(max(aim, low - settle), high + settle)
            error = goal - target - self.correction[axis]
            if blocked and error * blocked > 0:
                error = 0.0  # The plan already presses on the span's end that way: the feedback leaves it to the plan
            closing = math.copysign(min(gain * abs(error), math.sqrt(2.0 * acceleration * abs(error))), error)
            rate = min(max(closing + wanted_rate, -speed), speed)
            self.correction_rate[axis] += max(-acceleration * dt, min(acceleration * dt, rate - self.correction_rate[axis]))
            self.correction[axis] += self.correction_rate[axis] * dt
            band = 0.0 if aim > high or aim < low else settle  # Beyond its reach it settles only at the end
            if abs(off) <= band and wanted_rate == 0.0 and abs(self.correction_rate[axis]) <= STILL_RATE and not self.planning(axis):
                self.settled[axis], self.correction_rate[axis] = True, 0.0
        # The span is the feedback's only limit, and its anti-windup: it stops at the end, and turns back at once.
        # Held there, it keeps the end's own speed as the body turns, so it leaves the end smoothly once it must
        # slow down to stop on the person.
        total = self.plan[axis] + self.correction[axis]
        if total > high or total < low:
            self.correction[axis] = min(max(total, low), high) - self.plan[axis]
            self.correction_rate[axis] = (self.span_rate[axis] if dt else 0.0) - self.plan_rate[axis]
        self.view[axis] = self.plan[axis] + self.correction[axis] - self.shift[axis]

    def _shape(self, now: float) -> None:
        """The views to command now: each axis's planned move, pan's less the body's planned turn, split by its
        zero-vibration shaper, A1 of it now and A2 of what was planned half a ringing period ago, so the ringing
        the first part starts, the second cancels; plus the feedback's part, unshaped."""
        p = self.params
        self.planned.append((now, self.plan[0] - self.shaped_shift, self.plan[1]))  # Pan's in the body's frame
        modes = ((p.pan_frequency, (p.pan_damping + p.pan_damping_down) / 2),
                 (p.tilt_frequency, (p.tilt_damping + p.tilt_damping_down) / 2))
        longest = 0.0
        for axis, mode in enumerate(modes):
            if not p.shaper:
                self.shaped[axis] = self.view[axis]
                continue
            first, second, delay = zero_vibration(*mode)
            longest = max(longest, delay)
            shaped_plan = first * self.planned[-1][axis + 1] + second * self._planned_at(now - delay, axis)
            self.shaped[axis] = shaped_plan + self.correction[axis]
        while len(self.planned) > 2 and self.planned[1][0] <= now - longest - MAX_STEP:
            self.planned.popleft()

    def _planned_at(self, when: float, axis: int) -> float:
        """The planned move's view at `when`, interpolated between ticks."""
        before = self.planned[0]
        if when <= before[0]:
            return before[axis + 1]
        for entry in self.planned:
            if entry[0] >= when:
                share = (when - before[0]) / (entry[0] - before[0]) if entry[0] > before[0] else 1.0
                return before[axis + 1] + share * (entry[axis + 1] - before[axis + 1])
            before = entry
        return before[axis + 1]

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

    def update(self, *, now: float, body: float = 0.0, body_at: Callable[[float], float] | None = None,
               body_steady: bool = True) -> tuple[float, float]:
        """This tick's pan and tilt servo commands, for a body turned `body` degrees now by the waist's model;
        `body_at(time)` says where the waist's model and plan put it a little ahead (None: still);
        `body_steady`: whether it turns steadily or not at all, rather than breaking loose or stopping, where
        its model is least sure."""
        dt = 0.0 if self.last_update is None else min(max(now - self.last_update, 0.0), MAX_STEP)
        self.last_update = now
        if abs(body - self.body) > BODY_JUMP:
            self._rebase(body - self.body)  # A new waist model (following switched, a motor test): not a turn
        self.body, self.body_at = body, body_at
        for model in self.models:  # Where the commands sent so far have taken the camera by now
            model.advance(now, dt)
        self.history.append((now, self.models[0].view, self.models[1].view, body, body_steady))
        while len(self.history) > 2 and self.history[1][0] < now - HISTORY_SECONDS:
            self.history.popleft()
        self.shift = [self._shift(self.params.prediction_lead), 0.0]
        # The shaper delays the counter-rotation on average by its lead, so it takes the body's turn that much later
        self.shaped_shift = self._shift(self.params.prediction_lead + self.shaper_lead())
        if self.tracks is not None:
            for axis in (0, 1):
                self._turn(axis, now, dt)
        else:
            self.view[0] = self.plan[0] + self.correction[0] - self.shift[0]
        self._shape(now)
        for axis, model in enumerate(self.models):
            self._command(axis)
            model.command(now, self.servos[axis])
        return self.pan, self.tilt
