"""The waist: a slow, sticky torso that brings the head back toward the middle of its reach in planned moves.

The torso turns on a brushed DC motor that needs more than 0.6 of full drive to break loose, keeps turning
down to about 0.45, and turns 50.4 degrees a second at full negative drive but only 21.5 at full positive
(simulator/plant.py, fitted 2026-11-06). No sensor measures its angle, and the camera reports where things
were 0.15-0.2 s ago. Steering it continuously by the image made it overshoot by the travel during that
delay and its coast, 10-20 degrees (docs/control-design.md).

**When it moves: mid-ranging.** The head closes the only fast loop on the person; the waist only takes up
the head's offset from the middle of its reach. It starts a move once the head's pan has settled more than
`start_offset` degrees of view off the middle and stayed there `start_hold` seconds, or at once when the
person is further off the body's front than `start_now` (about the head's reach) once the head's own
planned move has landed: the head turns first, then the torso. It leaves `leave`
degrees for the head rather than centring, makes no move under `min_move`, and rests `rest` seconds after
each move (`reverse_rest` before turning back the other way). There is no integral action: small offsets
stay with the head.

**How it moves: a planned move per direction.** A move kicks at `kick_throttle` for `kick_seconds` to break
loose, then runs at the drive the waist's model needs for the speed wanted: the person's own speed away
plus `closing_gain` per degree still to go, never under `run_throttle` (above the stall) nor over
`max_throttle`. That inverse model differs by direction (`speed_negative`, `speed_positive`). It brakes once
the model says a brake now would stop it with `leave` left, allowing for how far the person moves on: the
stop comes from the model, while the distance still to go comes from the picture (the person's direction
in the base's frame less the body's modelled turn, which the model's own errors cancel out of), so the
picture's delay no longer carries the waist past them. Near the end the wanted speed is the lowest, so it
brakes from low speed.

**Getting nowhere.** No sensor says the torso has reached the end of its travel, but the picture does: once a
move has run progress_settle, the person's offset from the body's front must shrink, over every progress_window,
by progress_share of the turn the model drove, less what the person walked away meanwhile (as seen while the
body stood still before the move). If it does not, the body is not turning: the waist brakes and starts no move
that way again until it has turned the other way (a move, or a motor test). Before this a move pushed into the
stop for its whole max_move_seconds, rested, and pushed again.

**The model: an efference copy.** `WaistModel` runs the drive actually sent (after the actuator limits)
through the fitted friction model: breakaway, stall, a speed per direction, how fast it reaches speed and
stops. The head counter-rotates against its turn (head_aim.py), with a speed for each direction and nothing
before breakaway, where the old dead reckoning used one speed and no friction. Each command is decided
`plan_ahead` before it is sent, so the head, whose own commands take that long to reach the camera, turns
against the body's planned turn, start and stop included, as it happens. The turn is modelled, not measured.
"""
import logging
import math
from collections import deque
from enum import Enum

from pydantic import Field, model_validator

from ultraskelly.core.bot.base_abcs import NodeParams

logger = logging.getLogger(__name__)

MAX_STEP = 0.1  # Seconds; a stalled control loop does not jump a ramp or the model further than this
RESTART_AFTER = 0.5  # Seconds without an update: the motor watchdog has braked the waist, so start over
STILL_SPEED = 0.05  # Degrees per second under which the modelled waist has stopped
STOP_TICKS = 60  # Control ticks the stopping prediction looks ahead at most (0.6 s)
TICK = 0.01  # The orientation controller's period, seconds, for the model's predictions
STEADY_SPEED = 3.0  # Degrees per second within which the modelled waist turns at the speed its drive keeps


class WaistDriveParams(NodeParams):
    """How the waist turns toward the person. Throttles are magnitudes; angles are degrees of turn."""

    enabled: bool = Field(default=True, description="Turn the torso toward the target; off, the waist stays braked except in motor tests")
    slow_decay: bool = Field(
        default=True, description="Brake between PWM pulses: less ripple and whine, and speed that follows the drive",
    )

    # The waist's model (the efference copy), fitted to six recorded pulses (simulator/plant.py, 2026-11-06)
    breakaway: float = Field(default=0.65, ge=0.0, le=1.0, description="Drive a still waist needs to start turning: 0.6 did not move it, 0.7 did")
    stall: float = Field(default=0.45, ge=0.0, le=1.0, description="A turning waist stops below this drive (estimated, not measured)")
    speed_negative: float = Field(
        default=50.4, gt=0.0, le=180.0,
        description="Degrees a second at full negative drive, which turns the torso toward positive pan: 50.4 fitted",
    )
    speed_positive: float = Field(
        default=21.5, gt=0.0, le=180.0, description="Degrees a second at full positive drive, the other way: 21.5 fitted",
    )
    time_constant: float = Field(default=0.16, gt=0.0, le=2.0, description="Seconds the waist takes to reach its speed: 0.16 fitted")
    brake_time: float = Field(default=0.03, gt=0.0, le=2.0, description="Seconds a braked waist takes to stop")
    coast_time: float = Field(default=0.3, gt=0.0, le=5.0, description="Seconds a released waist takes to stop")

    # When to move: mid-ranging on the head's settled pan offset
    start_offset: float = Field(
        default=15.0, ge=0.0, le=60.0,
        description="Degrees of view the head's settled pan must sit off its middle, held start_hold, to start a "
                    "move: about half the head's reach",
    )
    start_hold: float = Field(default=0.47, ge=0.0, le=5.0, description="Seconds the head must hold that offset first")
    start_now: float = Field(
        default=21.0, ge=0.0, le=90.0,
        description="Degrees off the body's front beyond which a move starts at once: about the head's reach, "
                    "19 degrees of view with pan's servo window",
    )
    leave: float = Field(
        default=0.0, ge=0.0, le=30.0, description="Degrees of the person's offset a move leaves for the head, rather than centring",
    )
    min_move: float = Field(default=6.4, ge=0.0, le=30.0, description="Degrees; a smaller move is left to the head")
    rest: float = Field(default=0.6, ge=0.0, le=10.0, description="Seconds after a move before the next may start")
    reverse_rest: float = Field(
        default=2.0, ge=0.0, le=10.0, description="Seconds after a move before one the other way may start, unless past start_now",
    )

    # How to move: a planned move through the model
    kick_throttle: float = Field(
        default=0.8, ge=0.0, le=1.0, description="Drive of the short kick that breaks the waist loose: above breakaway",
    )
    kick_seconds: float = Field(default=0.046, ge=0.0, le=0.5, description="Seconds the kick lasts: 30-80 ms suggested")
    run_throttle: float = Field(
        default=0.53, ge=0.0, le=1.0, description="Least running drive, safely above the stall: the move's slowest speed",
    )
    max_throttle: float = Field(default=0.8, ge=0.0, le=1.0, description="Largest running drive (the actuator limit is 0.8)")
    closing_gain: float = Field(
        default=0.84, gt=0.0, le=20.0,
        description="Degrees a second of turn wanted per degree still to go, on top of the person's own speed away",
    )
    plan_ahead: float = Field(
        default=0.17, ge=0.0, le=0.5,
        description="Seconds each command is decided before it is sent, so the head counter-rotates in time: its "
                    "commands take about 0.1 s to reach the camera (HeadAimParams.prediction_lead) and its shaper "
                    "delays them 0.07 s more on average",
    )
    lead_time: float = Field(
        default=0.5, ge=0.0, le=3.0,
        description="Seconds of a walker's speed a move stops ahead of them by, so it need not turn again so soon: "
                    "the waist cannot run slower than about 25 degrees a second toward positive pan, so it follows "
                    "slower walkers that way in moves",
    )
    max_move_seconds: float = Field(
        default=6.0, gt=0.0, le=60.0, description="Safety limit: a move this long brakes and rests, whatever the picture says",
    )

    # Getting nowhere: at the end of its travel, or held, the waist turns but the person comes no nearer
    progress_window: float = Field(
        default=0.5, gt=0.0, le=3.0,
        description="Seconds over which a turning waist must bring the person nearer, counted once the kick has "
                    "broken it loose and the camera has seen it (progress_settle)",
    )
    progress_settle: float = Field(
        default=0.3, ge=0.0, le=2.0,
        description="Seconds after the kick before a move's progress is counted: breaking loose, and the camera's delay",
    )
    progress_share: float = Field(
        default=0.33, ge=0.0, le=1.0,
        description="Share of the turn the model drove over progress_window by which the person's offset from the "
                    "body's front must shrink; less, and the waist brakes and turns no further that way until it "
                    "has turned the other way (0: never)",
    )
    progress_min_turn: float = Field(
        default=3.0, gt=0.0, le=30.0, description="Degrees the model must have driven over the window before it is judged",
    )

    @model_validator(mode="after")
    def _ordered(self) -> "WaistDriveParams":
        if self.stall > self.breakaway:
            raise ValueError("stall must not exceed breakaway")
        if not self.stall < self.run_throttle <= self.max_throttle:
            raise ValueError("Throttles must satisfy stall < run_throttle <= max_throttle")
        if self.kick_throttle < self.breakaway:
            raise ValueError("kick_throttle must reach breakaway")
        return self


class WaistState(Enum):
    IDLE = "idle"  # Braked at rest
    KICK = "kick"  # Breaking loose
    TURNING = "turning"  # Driving the torso toward the person
    STOPPING = "stopping"  # Braked, slowing to a stop
    RESTING = "resting"  # Braked after a move; the next must wait


class WaistModel:
    """The waist's model, the efference copy: how far the torso has turned and how fast, from the drive sent
    (degrees and degrees a second, positive the way negative drive turns it, toward positive pan). Modelled,
    not measured. It reads its numbers from the params each time, so a change of tuning takes effect at once."""

    def __init__(self, params: WaistDriveParams) -> None:
        self.params = params
        self.angle = self.speed = 0.0
        self.moving = False

    def copy(self) -> "WaistModel":
        other = WaistModel(self.params)
        other.angle, other.speed, other.moving = self.angle, self.speed, self.moving
        return other

    def advance(self, drive: float | None, dt: float) -> None:
        """`dt` seconds of `drive`: a signed throttle, 0.0 braked, or None released to coast."""
        p = self.params
        strength = abs(drive or 0.0)
        self.moving = strength > p.stall if self.moving else strength >= p.breakaway
        if self.moving and drive:
            wanted, settling = -drive * (p.speed_negative if drive < 0 else p.speed_positive), p.time_constant
        else:
            wanted, settling = 0.0, p.coast_time if drive is None else p.brake_time
        self.speed += (wanted - self.speed) * min(1.0, dt / settling)
        if not self.moving and abs(self.speed) < STILL_SPEED:
            self.speed = 0.0
        self.angle += self.speed * dt

    def steady(self, drive: float | None) -> bool:
        """Whether it is still, or turning at about the speed `drive` keeps it at: not breaking loose, speeding
        up or stopping."""
        p = self.params
        if not self.moving:
            return self.speed == 0.0
        wanted = -(drive or 0.0) * (p.speed_negative if (drive or 0.0) < 0 else p.speed_positive)
        return abs(wanted - self.speed) <= STEADY_SPEED

    def stopping_distance(self) -> float:
        """How much further it turns if braked now, degrees (signed)."""
        model, start = self.copy(), self.angle
        for _ in range(STOP_TICKS):
            if model.speed == 0.0:
                break
            model.advance(0.0, TICK)
        return model.angle - start


class WaistDrive:
    """Each control tick: `advance` the model to now, then `update` for that tick's command, then `sent` with
    the command that reached the motor.

    Each command is decided `plan_ahead` seconds before it is sent, and `ahead` says where the model puts the
    body then: the head's own commands take about that long to reach the camera, so it counter-rotates
    against the body's planned turn, start and stop included, rather than trailing it."""

    def __init__(self, params: WaistDriveParams) -> None:
        self.params = params
        self.model = WaistModel(params)
        self.phase = WaistState.IDLE  # IDLE, KICK, TURNING or STOPPING, as decided; `state` adds RESTING
        self.drive: float | None = 0.0  # The command last sent, which the model runs on until the next
        self.queue: deque[tuple[float, float | None]] = deque()  # (when due, command) decided but not sent yet
        self.side = 0  # The side of the person this move turns toward: +1 positive pan, -1 negative
        self.last_side = 0  # The side the last move turned toward
        self.kick_until = self.move_started = 0.0
        self.offset_since: float | None = None  # Since when the settled head has held a start offset
        self.rest_until = self.reverse_until = -math.inf
        self.last_update: float | None = None
        self.last_advance: float | None = None
        # (when, the person's offset from the body's front, the model's angle) each tick of the move, for its progress
        self.progress: deque[tuple[float, float, float]] = deque()
        self.blocked = 0  # A side a move found it could not turn toward (+1 positive pan, -1 negative), until it turns the other way
        # How fast the person was going further away as the move started, degrees a second: seen while the body
        # was still, so the waist's model had not yet had any say in it
        self.start_away = 0.0
        self.idle_offsets: deque[tuple[float, float]] = deque()  # (now, the person's offset) while the waist is idle

    @property
    def state(self) -> WaistState:
        resting = self.last_update is not None and self.last_update < self.rest_until
        return WaistState.RESTING if resting and self.phase is WaistState.IDLE else self.phase

    def steady(self) -> bool:
        """Whether the body's model turns steadily or not at all now: not breaking loose, speeding up or
        stopping, when it is least sure."""
        return self.model.steady(self.drive)

    def advance(self, now: float) -> None:
        """Run the model on to `now` with the command last sent."""
        dt = 0.0 if self.last_advance is None else min(max(now - self.last_advance, 0.0), MAX_STEP)
        self.last_advance = now
        if dt:
            self.model.advance(self.drive, dt)

    def sent(self, throttle: float | None) -> None:
        """The command that reached the motor this tick, after the actuator limits: the model runs on it."""
        self.drive = throttle

    def ahead(self, when: float) -> WaistModel:
        """The model as it will be at `when`, running on the commands decided but not yet sent."""
        model, time, drive = self.model.copy(), self.last_advance, self.drive
        if time is None:
            return model
        for due, command in [*self.queue, (math.inf, None)]:
            until = min(due, when)
            while time < until - 1e-9:
                step = min(TICK, until - time)
                model.advance(drive, step)
                time += step
            if due > when:
                return model
            drive = command
        return model

    def _running_throttle(self, remaining: float, away: float) -> float:
        """The drive for the speed wanted, through the model for this move's direction."""
        p = self.params
        speed = away + p.closing_gain * remaining
        full = p.speed_positive if self.side < 0 else p.speed_negative  # Toward negative pan takes positive drive
        return min(max(speed / full, p.run_throttle), p.max_throttle)

    def moved(self, side: int) -> None:
        """The body turned toward `side` (+1 positive pan, -1 negative) another way than a move here, such as a
        motor test: turned away from the side it found blocked, it may try that way again."""
        if self.blocked and side == -self.blocked:
            logger.info("Waist turned the other way: it may try toward the blocked side again")
            self.blocked = 0

    def _stalled(self, when: float, demand: float) -> bool:
        """Whether this move has stopped bringing the person nearer: over the last progress_window the model
        drove the body at least progress_min_turn degrees toward them, and their offset from its front shrank
        by less than progress_share of that. The offset is the picture's, in the base's frame less the body's
        modelled turn: when the body really turns it shrinks by the turn, and when it does not (at the end of
        its travel, or held) the picture keeps the person where they were and it stays put."""
        p = self.params
        now = when - p.plan_ahead  # The offset and the model's angle are now's; `when` is the command's, ahead
        if p.progress_share <= 0.0 or now - self.move_started < p.progress_settle:
            return False
        self.progress.append((now, abs(demand), self.model.angle))
        while len(self.progress) > 1 and self.progress[1][0] <= now - p.progress_window:
            self.progress.popleft()
        then, offset_then, angle_then = self.progress[0]
        if now - then < p.progress_window - 1e-6:
            return False
        # Once the waist turns, the tracker cannot tell a body that does not turn from a person walking away as fast
        # as the model says it turns: so the person's own walking away is taken as it was before the move started
        driven = (self.model.angle - angle_then) * self.side
        expected = driven - self.start_away * (now - then)
        nearer = offset_then - abs(demand)
        if expected < p.progress_min_turn or nearer >= p.progress_share * expected:
            return False
        logger.warning(
            f"Waist: the model drove {driven:.1f} degrees toward {'positive' if self.side > 0 else 'negative'} pan in "
            f"{now - then:.2f} s, which should have brought the person {expected:.1f} degrees nearer, but they came "
            f"{nearer:.1f}; braking, and turning no further that way until it has turned the other way"
        )
        return True

    def _start(self, when: float, demand: float, settled: bool, offset: float | None, planning: bool) -> bool:
        """Whether a move starts at `when` toward a person `demand` degrees off the body's front."""
        p = self.params
        side = 1 if demand > 0 else -1
        if side == self.blocked:  # It could not turn that way last time: not until it has turned the other way
            self.offset_since = None
            return False
        beyond = abs(demand) > p.start_now and not planning  # The head turns first, then the torso
        holding = settled and offset is not None and abs(offset) > p.start_offset and (offset > 0) == (side > 0)
        if not holding:
            self.offset_since = None
        elif self.offset_since is None:
            self.offset_since = when
        ready = beyond or (self.offset_since is not None and when - self.offset_since >= p.start_hold)
        resting = when < self.rest_until or (side != self.last_side and when < self.reverse_until and not beyond)
        if not ready or resting or abs(demand) - p.leave < p.min_move:
            return False
        self.phase, self.side = WaistState.KICK, side
        self.kick_until, self.move_started, self.offset_since = when + p.kick_seconds, when, None
        self.progress.clear()
        self.moved(side)  # Turning away from a side it found blocked frees that side
        return True

    def _decide(self, when: float, demand: float | None, settled: bool, offset: float | None, planning: bool,
                person_speed: float) -> float:
        """The command for time `when`, plan_ahead from now."""
        p = self.params
        if self.phase is WaistState.IDLE:
            if demand is None:
                self.offset_since = None
                self.idle_offsets.clear()
                return 0.0
            now = when - p.plan_ahead
            self.idle_offsets.append((now, demand))
            while len(self.idle_offsets) > 1 and self.idle_offsets[1][0] <= now - p.progress_window:
                self.idle_offsets.popleft()
            if not self._start(when, demand, settled, offset, planning):
                return 0.0
            # The person's own walking away, from their offset while the body stood still, or as the tracker had it
            then, offset_then = self.idle_offsets[0]
            seen = (demand - offset_then) / (now - then) * self.side if now - then >= p.progress_window / 2 else 0.0
            self.start_away = max(0.0, seen, person_speed * self.side)
            self.idle_offsets.clear()
        drive = -float(self.side)  # The sign that turns the torso toward the person
        if self.phase is WaistState.KICK:
            if demand is None or when - self.move_started > p.max_move_seconds:
                self.phase = WaistState.STOPPING
            elif when < self.kick_until:
                return drive * p.kick_throttle
            else:
                self.phase = WaistState.TURNING
        future = self.ahead(when)  # Where the body will be when this command is sent, by its model
        if self.phase is WaistState.TURNING:
            if demand is None or (demand > 0) != (self.side > 0) or when - self.move_started > p.max_move_seconds:
                if demand is not None and when - self.move_started > p.max_move_seconds:
                    logger.warning(f"Waist drove {p.max_move_seconds:.1f} s without the person coming ahead; resting")
                self.phase = WaistState.STOPPING
            elif self._stalled(when, demand):  # Turning, but the person comes no nearer: it cannot turn this way
                self.blocked, self.phase = self.side, WaistState.STOPPING
            else:
                # The person's offset as it will be: less the turn still to come before this command is sent
                away = max(0.0, person_speed * self.side)  # The person going further this way
                remaining = abs(demand - (future.angle - self.model.angle)) + away * (p.plan_ahead + p.lead_time) - p.leave
                if remaining <= abs(future.stopping_distance()) + away * p.brake_time:
                    self.phase = WaistState.STOPPING
                else:
                    return drive * self._running_throttle(remaining, away)
        # Stopping: braked until the model will have stopped, then rest
        if future.speed == 0.0:
            self.phase, self.last_side = WaistState.IDLE, self.side
            self.rest_until, self.reverse_until = when + p.rest, when + p.reverse_rest
        return 0.0

    def update(self, *, now: float, demand: float | None, offset: float | None = None, settled: bool = False,
               planning: bool = False, person_speed: float = 0.0) -> float | None:
        """This tick's command: a signed throttle, 0.0 to brake, or None to let the motor coast.

        `demand`: how far the person is off the body's front, degrees in pan's sense, even beyond the head's
        reach, or None without a person. `offset`: the head's commanded pan view off its middle, `settled`
        whether pan has settled there, and `planning` whether its planned move is still under way (the head
        turns first). `person_speed`: the person's tracked speed in pan's sense, degrees a second. Negative
        drive turns the torso toward positive pan.
        """
        p = self.params
        if self.last_update is not None and now - self.last_update > RESTART_AFTER and self.phase is not WaistState.IDLE:
            logger.warning(f"No waist update for {now - self.last_update:.1f} s; the next move starts from rest")
            self.phase, self.drive = WaistState.IDLE, 0.0
            self.queue.clear()
        self.last_update = now
        when = now + p.plan_ahead
        self.queue.append((when, self._decide(when, demand, settled, offset, planning, person_speed)))
        command = self.drive
        while self.queue and self.queue[0][0] <= now + 1e-9:
            command = self.queue.popleft()[1]
        return command
