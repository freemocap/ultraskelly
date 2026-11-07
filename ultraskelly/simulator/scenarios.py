"""Closed-loop scenarios that catch a head or waist that swings, before any motor runs.

Each scenario puts people in front of the simulated robot and runs the production
controllers through the simulator's camera and servos (see engine.py). Its verdict fails on:

- overshoot: the camera swinging past a person it turned to by more than OVERSHOOT_DEGREES;
- oscillation: the head's turn or nod (what the linkage does, in degrees of view) reversing
  more often than the scenario allows;
- runaway: a servo command reaching its end stop, or losing a person who stays in view;
- aim: still off a standing person by more than AIM_DEGREES at the end;
- waist hunting: the waist turning one way and then back while the person stands still;
- roll: the head's roll turning back and forth (in every scenario), and in those about a
  tilted face, turning the face more than ROLL_OVERSHOOT past level, ending neither within
  ROLL_AIM of level nor ROLL_TOWARD closer to it than the person's own tilt (the robot's roll
  reaches only a couple of degrees), or not standing upright again once the face has gone.

Bearings are degrees from the robot's base: positive yaw to the robot's left as the
simulator draws it, positive pitch up.
"""
import asyncio
import threading
from dataclasses import dataclass, field
from typing import Callable

from ultraskelly.simulator.engine import HEIGHT, Person, Settings, Simulation

OVERSHOOT_DEGREES = 2.0
AIM_DEGREES = 2.5
REVERSAL_DEGREES = 0.5  # A turn back counts once the command has come back this far
LIMIT_MARGIN = 2.0  # Degrees from a servo's 0 or 180 that count as reaching its end stop
ROLL_OVERSHOOT = 1.5  # Degrees the followed face may be turned past level in the picture
ROLL_AIM = 1.5  # Degrees off level the face may stay at the end...
ROLL_TOWARD = 1.0  # ...or at least this much closer to level than the person's own tilt
ROLL_UPRIGHT = 0.5  # Degrees the head's actual roll may be off upright once nobody is left
ROLL_LIMITS = (60.0, 120.0)  # The roll servo's hard limits (ActuatorLimits)
SETTLE_DEGREES = 1.5  # Within this of the person, and staying there, the camera has settled


@dataclass(frozen=True)
class Scenario:
    name: str
    description: str
    seconds: float
    people: Callable[[float], list[Person]]
    settings: dict = field(default_factory=dict)  # Changes to the simulator's Settings
    step: bool = True  # People stand still after appearing: overshoot and final aim apply
    max_reversals: int = 1  # Per servo
    max_waist_moves: int = 1
    roll: bool = False  # About a tilted face: the roll's overshoot and final level apply
    max_roll_reversals: int = 1
    upright_at_end: bool = False  # Nobody is left: the roll must stand the head upright again
    aim: bool = True  # A still person within the head's reach: final aim applies (with `step`)
    still_within: float | None = None  # Only false detections: the camera must stay within this of where it began


@dataclass
class Trace:
    time: list[float] = field(default_factory=list)
    pan_command: list[float] = field(default_factory=list)  # Servo degrees
    tilt_command: list[float] = field(default_factory=list)
    head_yaw: list[float] = field(default_factory=list)  # How far the head has actually turned from the body
    head_pitch: list[float] = field(default_factory=list)  # And nodded up, both in degrees of view
    camera_yaw: list[float] = field(default_factory=list)
    camera_pitch: list[float] = field(default_factory=list)
    person_yaw: list[float | None] = field(default_factory=list)  # The person the camera follows, if any
    person_pitch: list[float | None] = field(default_factory=list)
    waist: list[float] = field(default_factory=list)
    waist_drive: list[float | None] = field(default_factory=list)
    roll_command: list[float] = field(default_factory=list)  # Servo degrees
    picture_roll: list[float] = field(default_factory=list)  # How far the roll actually turned the picture, degrees
    person_roll: list[float | None] = field(default_factory=list)  # The followed face's own tilt
    face_tilt: list[float | None] = field(default_factory=list)  # And its tilt in the picture
    target_y: list[float | None] = field(default_factory=list)  # The last sampled target's pixels below the middle
    half_fov: float = 34.5  # Degrees from the image centre to its side edge


@dataclass
class Verdict:
    overshoot: float = 0.0
    pan_reversals: int = 0
    tilt_reversals: int = 0
    runaway: str | None = None
    final_error: float | None = None
    waist_moves: int = 0
    waist_reversals: int = 0
    roll_reversals: int = 0
    roll_overshoot: float = 0.0
    final_tilt: float | None = None
    settle: float | None = None  # Seconds from the person's appearing until the camera stays within SETTLE_DEGREES
    servo_travel: float = 0.0  # Servo degrees the pan and tilt commands moved in all
    wandered: float = 0.0  # Degrees the camera went from where it began
    failures: list[str] = field(default_factory=list)

    @property
    def passed(self) -> bool:
        return not self.failures


def still(yaw: float, pitch: float = 0.0, appear: float = 0.0) -> Callable[[float], list[Person]]:
    return lambda t: [Person(yaw, pitch)] if t >= appear else []


def walking(start: float, end: float, speed: float, pitch: float = 0.0, pause: float = 0.5) -> Callable[[float], list[Person]]:
    """Someone standing at `start` who walks to `end` at `speed` degrees a second after `pause`, then stops."""
    def people(t: float) -> list[Person]:
        travelled = max(0.0, t - pause) * speed
        yaw = start + min(travelled, abs(end - start)) * (1 if end > start else -1)
        return [Person(yaw, pitch)]
    return people


def two_people(a: float, b: float, swap_period: float = 0.1) -> Callable[[float], list[Person]]:
    """Two people at yaw a and b; who scores higher swaps every `swap_period` seconds."""
    def people(t: float) -> list[Person]:
        a_first = int(t / swap_period) % 2 == 0
        return [Person(a, 0.0, 0.92 if a_first else 0.85), Person(b, 0.0, 0.85 if a_first else 0.92)]
    return people


def tilted(roll: float, until: float | None = None, then: float | None = 0.0,
           distance: float = 1.2) -> Callable[[float], list[Person]]:
    """Someone straight ahead, `distance` metres away, whose face is tilted `roll` degrees clockwise; from
    `until` it is tilted `then`, or gone if `then` is None."""
    def people(t: float) -> list[Person]:
        if until is None or t < until:
            return [Person(0.0, 0.0, roll=roll, distance=distance)]
        return [] if then is None else [Person(0.0, 0.0, roll=then, distance=distance)]
    return people


def ghosts(base: Callable[[float], list[Person]], every: float = 0.7, frames: tuple[int, ...] = (1, 2, 3),
           yaw: float = 30.0, score: float = 0.95, sides: tuple[int, ...] = (1, -1)) -> Callable[[float], list[Person]]:
    """`base`'s people plus a false detection every `every` seconds, for 1-3 frames in turn, at `yaw` on
    each of `sides` in turn (30 degrees lands within 25 px of the picture's edge with the head ahead): on
    the robot 28 % of person detections had at most two confident keypoints, 93 % of them at the edge, and
    ten bursts of up to three frames gave targets (run 20261106-194104)."""
    frame = 1 / 30
    def people(t: float) -> list[Person]:
        burst = int(t / every)
        length = frames[burst % len(frames)] * frame
        found = list(base(t))
        if burst > 0 and t - burst * every < length:
            found.append(Person(yaw * sides[burst % len(sides)], 0.0, score))
        return found
    return people


def nobody(t: float) -> list[Person]:
    return []


def comes_and_goes(first: float, second: float, gone: tuple[float, float]) -> Callable[[float], list[Person]]:
    """Someone at `first`, gone during `gone`, then someone else at `second`."""
    return lambda t: [] if gone[0] <= t < gone[1] else [Person(first if t < gone[0] else second, 0.0)]


SCENARIOS = [
    Scenario("left", "A person steps in 20 degrees to the robot's left", 4.0, still(20.0)),
    Scenario("right", "A person steps in 20 degrees to the robot's right", 4.0, still(-20.0)),
    # The tilt reaches about 5 degrees up and 2.7 down inside its window (see HEAD_BATTERY).
    Scenario("up", "A person 4 degrees up", 4.0, still(0.0, 4.0)),
    Scenario("down", "A person 2 degrees down", 4.0, still(0.0, -2.0)),
    Scenario("far", "A person 27 degrees off: the head turns and the waist brings it back", 8.0, still(27.0)),
    # Turning to someone, following them back and counter-rotating while the waist recentres
    # are three purposeful phases: two reversals, not a swing.
    Scenario("walk", "A person walks across, 30 to -30 degrees at 12 deg/s", 8.0, walking(30.0, -30.0, 12.0),
             step=False, max_reversals=2, max_waist_moves=2),
    Scenario("walk-away", "A person walks from 10 to 100 degrees at 15 deg/s, out of the head's reach", 9.0,
             walking(10.0, 100.0, 15.0), step=False, max_reversals=2, max_waist_moves=2),
    Scenario("two-people", "Two people 24 degrees apart whose scores keep swapping", 6.0, two_people(10.0, -14.0),
             step=False, max_reversals=1),
    Scenario("noisy", "A person 10 degrees off with 4 px of detection jitter and a third of detections dropped", 6.0,
             still(10.0), settings={"noise_px": 4.0, "landmark_noise_px": 1.5, "dropout": 0.33}, max_reversals=1),
    Scenario("lost", "A person at 15 degrees leaves for 2 s; another appears at -15", 8.0,
             comes_and_goes(15.0, -15.0, (3.0, 5.0)), step=False, max_reversals=2, max_waist_moves=2),
    Scenario("tilt", "A person ahead tilts their head 7 degrees clockwise", 5.0, tilted(7.0), roll=True),
    Scenario("tilt-slight", "A person ahead tilts their head 3 degrees: within the roll's reach", 5.0, tilted(3.0),
             roll=True),
    Scenario("tilt-other-way", "A person ahead tilts their head 7 degrees anticlockwise", 5.0, tilted(-7.0), roll=True),
    Scenario("tilt-and-back", "A person tilts their head 7 degrees for 3 s, then straightens it", 7.0,
             tilted(7.0, until=3.0), roll=True),
    Scenario("tilt-gone", "A person with their head tilted 7 degrees leaves after 3 s", 6.0,
             tilted(7.0, until=3.0, then=None), step=False, upright_at_end=True),
    Scenario("tilt-noisy", "A tilted face (6 degrees) with 1.5 px of landmark jitter and a third of detections dropped",
             6.0, tilted(6.0), settings={"landmark_noise_px": 1.5, "dropout": 0.33}, roll=True),
    Scenario("tilt-near", "A face 0.7 m away tilted 7 degrees: big, sharp eyes", 5.0, tilted(7.0, distance=0.7), roll=True),
]
BY_NAME = {scenario.name: scenario for scenario in SCENARIOS}

# The robot as it runs now (2026-11-06): the head alone, waist and roll off (HEAD_ALONE). Its pan reaches
# about 17 degrees either way inside its servo window (20-160) and its tilt about 5 up and 2.7 down
# (20-130, from a head resting tilted up), so these people stand where the head can reach them, but for
# the one beyond its reach, where only calm is asked.
HEAD_BATTERY = [
    Scenario("step-left", "A person steps in 12 degrees to the left", 4.0, still(12.0)),
    Scenario("step-right", "A person steps in 12 degrees to the right", 4.0, still(-12.0)),
    Scenario("step-edge", "A person 16 degrees off, at the edge of the head's reach", 4.0, still(16.0)),
    Scenario("small-step", "A person 4 degrees off: within the jitter a head should ignore, nearly", 3.0, still(4.0)),
    Scenario("look-up", "A person 4 degrees up", 3.0, still(0.0, 4.0)),
    Scenario("look-down", "A person 2 degrees down, as far as the tilt reaches below rest", 3.0, still(0.0, -2.0)),
    Scenario("beyond-reach", "A person 28 degrees off, beyond the head without the waist: stay calm", 5.0,
             still(28.0), aim=False),
    Scenario("walk", "A person walks across, 15 to -15 degrees at 10 deg/s", 6.0, walking(15.0, -15.0, 10.0),
             step=False, max_reversals=2),
    Scenario("two-people", "Two people 16 degrees apart whose scores keep swapping", 5.0, two_people(8.0, -8.0),
             step=False),
    Scenario("noisy", "A person 8 degrees off with 4 px of jitter and a third of detections dropped", 5.0, still(8.0),
             settings={"noise_px": 4.0, "dropout": 0.33}),
    Scenario("lost", "A person at 10 degrees leaves for 2 s; another appears at -10", 7.0,
             comes_and_goes(10.0, -10.0, (2.5, 4.5)), step=False, max_reversals=2),
    Scenario("ghosts", "Nobody there: false detections at the picture's edge for 1-3 frames at a time", 5.0,
             ghosts(nobody), step=False, still_within=0.5),
    Scenario("ghosts-and-person", "A person at 6 degrees, and false detections at the edge for 1-3 frames", 5.0,
             ghosts(still(6.0)), max_reversals=1),
    Scenario("ghosts-middle", "Nobody there: false detections 10 degrees off for 1-3 frames at a time", 5.0,
             ghosts(nobody, yaw=10.0), step=False, still_within=0.5),
    # A partial body flickering at one spot: bursts that, counted alone, add up to a person there.
    Scenario("ghosts-one-spot", "Nobody there: false detections at one spot by the edge, 1-3 frames at a time", 5.0,
             ghosts(nobody, sides=(1,)), step=False, still_within=0.5),
    Scenario("ghosts-one-spot-middle", "Nobody there: false detections at one spot 10 degrees off, 1-3 frames at a time",
             5.0, ghosts(nobody, yaw=10.0, sides=(1,)), step=False, still_within=0.5),
    Scenario("edge-partial", "Someone half out of the picture at 31 degrees, seen 2-3 frames at a time", 5.0,
             ghosts(nobody, every=0.4, frames=(2, 3), yaw=31.0, score=0.6), step=False, still_within=0.5),
]
HEAD_BY_NAME = {scenario.name: scenario for scenario in HEAD_BATTERY}
# On the robot (run 20261106-222520) tilt reached its end turning down, and a standing person's face still sat
# 116-217 px below the middle of the picture: the head cannot look down to them. Pan must follow such a face as
# it follows one at eye level (tests/test_scenarios_tuning.py).
LOW_FACE = Scenario("low-face", "A person 12 degrees left whose face stays low in the picture, 238 px below the "
                    "middle and still 204 once tilt has turned down as far as it can", 4.0, still(12.0, -24.5),
                    aim=False)


def head_alone(sim: Simulation) -> None:
    """The robot's setup while its head is tuned (2026-11-06): waist and roll off."""
    sim.head.params.waist.enabled = sim.head.params.roll.enabled = False


def reversals(values: list[float], threshold: float = REVERSAL_DEGREES) -> int:
    """How often a series turns back by at least `threshold` from its last extreme."""
    if not values:
        return 0
    count, direction, extreme = 0, 0, values[0]  # direction: +1 rising, -1 falling, 0 not yet moved
    for value in values[1:]:
        if direction == 0:
            if abs(value - extreme) >= threshold:
                direction, extreme = (1 if value > extreme else -1), value
        elif direction * (value - extreme) > 0:
            extreme = value  # Still going the same way
        elif abs(value - extreme) >= threshold:
            count, direction, extreme = count + 1, -direction, value
    return count


def roll_judgement(scenario: Scenario, trace: Trace, verdict: Verdict) -> None:
    """The roll's part of a verdict: it must not turn back and forth, and about a tilted face it must level
    the face without turning it past level; with nobody left it must stand the head upright."""
    verdict.roll_reversals = reversals(trace.picture_roll)
    if verdict.roll_reversals > scenario.max_roll_reversals:
        verdict.failures.append(f"roll oscillation: reversed {verdict.roll_reversals} times (allowed {scenario.max_roll_reversals})")
    if min(trace.roll_command) <= ROLL_LIMITS[0] + LIMIT_MARGIN or max(trace.roll_command) >= ROLL_LIMITS[1] - LIMIT_MARGIN:
        verdict.failures.append("roll runaway: the roll reached its limit")
    if scenario.roll:
        # Overshoot while the face keeps its first tilt: past level on the other side from where it started.
        first = next((roll for roll in trace.person_roll if roll is not None), None)
        if first is not None and abs(first) >= ROLL_OVERSHOOT:
            side = 1 if first > 0 else -1
            held = [tilt for tilt, roll in zip(trace.face_tilt, trace.person_roll) if roll == first]
            verdict.roll_overshoot = max(0.0, max(-tilt * side for tilt in held))
            if verdict.roll_overshoot > ROLL_OVERSHOOT:
                verdict.failures.append(f"roll overshoot: the face turned {verdict.roll_overshoot:.1f} degrees past level")
        final = [abs(tilt) for tilt in trace.face_tilt[-10:] if tilt is not None]
        verdict.final_tilt = max(final) if final else None
        own = next((abs(roll) for roll in reversed(trace.person_roll) if roll is not None), 0.0)
        allowed = max(ROLL_AIM, own - ROLL_TOWARD)
        if verdict.final_tilt is None or verdict.final_tilt > allowed:
            verdict.failures.append(f"roll aim: the face {verdict.final_tilt} degrees off level at the end "
                                    f"(its own tilt {own:.1f}, allowed {allowed:.1f})")
    if scenario.upright_at_end and abs(trace.picture_roll[-1]) > ROLL_UPRIGHT:
        verdict.failures.append(f"roll: the head not upright again ({trace.picture_roll[-1]:.1f} degrees)")


def followed(sim: Simulation) -> Person | None:
    """The person nearest where the camera points: the one it follows."""
    people = sim.present()
    if not people:
        return None
    yaw, pitch = sim.camera_pose()
    return min(people, key=lambda person: (person.yaw - yaw) ** 2 + (person.pitch - pitch) ** 2)


async def record(scenario: Scenario, prepare: Callable[[Simulation], None] | None = None) -> Trace:
    """Run a scenario through the simulator; `prepare` may change the controllers before it starts."""
    sim = Simulation(Settings(scenario="manual", **scenario.settings), people=scenario.people)
    if prepare is not None:
        prepare(sim)
    trace = Trace(half_fov=sim.settings.camera_hfov / 2)
    try:
        for _ in range(int(round(scenario.seconds / sim.dt))):
            await sim.step()
            yaw, pitch = sim.camera_pose()
            head_yaw, head_pitch = sim.head_turn()
            person = followed(sim)
            trace.time.append(sim.now())
            trace.pan_command.append(sim.head.pan_angle)
            trace.tilt_command.append(sim.head.tilt_angle)
            trace.head_yaw.append(head_yaw)
            trace.head_pitch.append(head_pitch)
            trace.camera_yaw.append(yaw)
            trace.camera_pitch.append(pitch)
            trace.person_yaw.append(None if person is None else person.yaw)
            trace.person_pitch.append(None if person is None else person.pitch)
            trace.waist.append(sim.waist)
            trace.waist_drive.append(sim.waist_drive())
            trace.roll_command.append(sim.head.roll_angle)
            trace.picture_roll.append(sim.picture_roll())
            trace.person_roll.append(None if person is None else person.roll)
            trace.face_tilt.append(None if person is None else person.roll - sim.picture_roll())
            row = sim.last_camera["y"]
            trace.target_y.append(None if row is None else row - HEIGHT / 2)
    finally:
        sim.close()
    return trace


def judge(scenario: Scenario, trace: Trace) -> Verdict:
    verdict = Verdict()
    verdict.pan_reversals = reversals(trace.head_yaw)
    verdict.tilt_reversals = reversals(trace.head_pitch)
    for name, count in (("pan", verdict.pan_reversals), ("tilt", verdict.tilt_reversals)):
        if count > scenario.max_reversals:
            verdict.failures.append(f"oscillation: {name} reversed {count} times (allowed {scenario.max_reversals})")

    for name, series in (("pan", trace.pan_command), ("tilt", trace.tilt_command)):
        if min(series) <= LIMIT_MARGIN or max(series) >= 180 - LIMIT_MARGIN:
            verdict.runaway = f"{name} reached its end stop"
    # A person who stays put in the frame's view must stay in view.
    half_fov = trace.half_fov
    errors = [
        None if py is None else ((py - cy) ** 2 + (pp - cp) ** 2) ** 0.5
        for py, pp, cy, cp in zip(trace.person_yaw, trace.person_pitch, trace.camera_yaw, trace.camera_pitch)
    ]
    if scenario.step and any(error is not None and error > half_fov for error in errors[len(errors) // 4:]):
        verdict.runaway = verdict.runaway or "lost the person it had turned to"
    if verdict.runaway:
        verdict.failures.append(f"runaway: {verdict.runaway}")

    if scenario.step:
        # Overshoot: how far the camera went past the person, on each axis, after first reaching them.
        for camera, person in ((trace.camera_yaw, trace.person_yaw), (trace.camera_pitch, trace.person_pitch)):
            pairs = [(c, p) for c, p in zip(camera, person) if p is not None]
            if not pairs or abs(pairs[0][1] - pairs[0][0]) < OVERSHOOT_DEGREES:
                continue  # Already on target on this axis
            start_side = pairs[0][1] - pairs[0][0]
            past = [-(p - c) * (1 if start_side > 0 else -1) for c, p in pairs]
            verdict.overshoot = max(verdict.overshoot, max(0.0, max(past)))
        if verdict.overshoot > OVERSHOOT_DEGREES:
            verdict.failures.append(f"overshoot: {verdict.overshoot:.1f} degrees past the person")
        final = [error for error in errors[-10:] if error is not None]
        verdict.final_error = max(final) if final else None
        if scenario.aim and (verdict.final_error is None or verdict.final_error > AIM_DEGREES):
            verdict.failures.append(f"aim: {verdict.final_error} degrees off at the end")

    moves, signs = 0, []
    driving = False
    for drive in trace.waist_drive:
        now_driving = bool(drive)
        if now_driving and not driving:
            moves += 1
            signs.append(1 if drive > 0 else -1)
        driving = now_driving
    verdict.waist_moves = moves
    verdict.waist_reversals = sum(1 for a, b in zip(signs, signs[1:]) if a != b)
    if moves > scenario.max_waist_moves:
        verdict.failures.append(f"waist hunting: {moves} moves (allowed {scenario.max_waist_moves})")
    elif scenario.step and verdict.waist_reversals:
        verdict.failures.append(f"waist hunting: turned back {verdict.waist_reversals} times")
    roll_judgement(scenario, trace, verdict)
    motion_judgement(scenario, trace, verdict)
    return verdict


def motion_judgement(scenario: Scenario, trace: Trace, verdict: Verdict) -> None:
    """How long the camera took to settle on a still person, how far the servos travelled in all, and, with
    only false detections about, how far the camera wandered."""
    verdict.servo_travel = float(sum(abs(b - a) for a, b in zip(trace.pan_command, trace.pan_command[1:]))
                                 + sum(abs(b - a) for a, b in zip(trace.tilt_command, trace.tilt_command[1:])))
    verdict.wandered = max(max(abs(y - trace.camera_yaw[0]), abs(p - trace.camera_pitch[0]))
                           for y, p in zip(trace.camera_yaw, trace.camera_pitch))
    if scenario.still_within is not None and verdict.wandered > scenario.still_within:
        verdict.failures.append(f"chased false detections: the camera moved {verdict.wandered:.1f} degrees")
    if scenario.step:
        seen = [i for i, yaw in enumerate(trace.person_yaw) if yaw is not None]
        if seen:
            off = [((trace.person_yaw[i] - trace.camera_yaw[i]) ** 2 + (trace.person_pitch[i] - trace.camera_pitch[i]) ** 2) ** 0.5
                   for i in seen]
            last_out = max((n for n, error in enumerate(off) if error > SETTLE_DEGREES), default=-1)
            if last_out < len(off) - 1:
                verdict.settle = trace.time[seen[last_out + 1]] - trace.time[seen[0]]


def run(scenario: Scenario, prepare: Callable[[Simulation], None] | None = None) -> tuple[Trace, Verdict]:
    trace = _on_this_threads_loop(record(scenario, prepare))
    return trace, judge(scenario, trace)


_threads = threading.local()


def _on_this_threads_loop(coroutine):
    """Run `coroutine` on one event loop kept for every run on this thread. A new loop per run (asyncio.run)
    opens a loopback socket pair on Windows whose port then waits about two minutes to close: one per run,
    105 per tuner candidate (measured 2026-11-06). The tuner's eight workers make about 200 a second,
    which uses up Windows' 16,384 ports within that time, and its worker pool stalled."""
    runner = getattr(_threads, "runner", None)
    if runner is None:
        runner = _threads.runner = asyncio.Runner()
    try:
        return runner.run(coroutine)
    finally:
        loop = runner.get_loop()
        leftover = asyncio.all_tasks(loop)  # A run leaves none today; one that did must not run into the next
        for task in leftover:
            task.cancel()
        if leftover:
            loop.run_until_complete(asyncio.gather(*leftover, return_exceptions=True))
