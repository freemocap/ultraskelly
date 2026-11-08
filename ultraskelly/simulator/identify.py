"""Fit the simulated robot's motors (simulator/plant.py) to an open-loop session's measured views.

Each head servo's chain (dead time, slew, slack, gain, the head's mode) is fitted as an output-error
model: driven by the commands sent, the plant should give the view the camera measured
(scripts/analyze_motor_response.py). The fit is SciPy's least_squares with a soft-L1 loss, so frames
where registration slipped count less, over short shots: every step starts from the view measured
just before it, at rest (multiple shooting), so one step's error does not carry into the next.

`simulate` steps many shots at once, by the same rules as plant.Axis (`step` seconds a step, the
head's mode in 1 ms steps), with one difference that makes the cost smooth in the dead time: a command
arrives part way through a step, and the servo moves for only the part of that step after it.
`axis_fit` reports fit % (100 (1 - |error| / |measured - its mean|), as system identification counts
it) and RMS degrees, on the shots it fitted and on held-out ones.
"""
import math
from dataclasses import dataclass, replace

import numpy as np

from ultraskelly.simulator.plant import MIDDLE, AxisModel

STEP = 0.002  # Seconds per simulation step, as the fidelity replay steps the plant
MODE_STEP = 0.001  # The head's mode, in steps of at most this (plant.with_friction)
BEFORE = 0.1  # Seconds of rest a shot starts before its command
REST = (0.25, 0.04)  # Seconds before the shot's command whose measured views give its starting view
LONGEST = 2.0  # Seconds a shot runs after its command at most: past the head's settling, and keeps every shot short


@dataclass
class Shots:
    """Steps of one servo to simulate side by side: each from rest, with its commands and its measured frames."""
    start: np.ndarray  # (S,) seconds: each shot's start
    servo: np.ndarray  # (S,) servo degrees at the start: the command before
    view: np.ndarray  # (S,) measured view at the start, degrees
    command_times: np.ndarray  # (S, C) seconds after the start each command was sent; inf pads
    command_values: np.ndarray  # (S, C)
    frame_times: np.ndarray  # (S, F) seconds after the start of each measured frame; nan pads
    measured: np.ndarray  # (S, F) degrees; nan pads
    names: list[str]  # Each shot's part

    def __len__(self) -> int:
        return len(self.start)

    def pick(self, keep: np.ndarray) -> "Shots":
        keep = np.asarray(keep)
        return Shots(self.start[keep], self.servo[keep], self.view[keep], self.command_times[keep],
                     self.command_values[keep], self.frame_times[keep], self.measured[keep],
                     [name for name, k in zip(self.names, keep) if k] if keep.dtype == bool else [self.names[i] for i in keep])


def shots_for(servo: str, times: np.ndarray, view: np.ndarray, ok: np.ndarray, commands, segments, *,
              parts: tuple[str, ...]) -> Shots:
    """One shot per command of `servo` in the segments named `<servo> <part>`, from BEFORE seconds before
    it to the next command (which cannot reach the view earlier, a dead time later)."""
    sent = sorted((t, v) for t, name, v in commands if name == servo and v is not None)
    stamps = np.array([t for t, _ in sent])
    rows = []
    for segment in segments:
        if not any(segment["name"] == f"{servo} {part}" for part in parts):
            continue
        inside = [k for k, t in enumerate(stamps) if segment["start"] <= t <= segment["end"]]
        for k in inside:
            begin = stamps[k] - BEFORE
            end = min(stamps[k + 1] if k + 1 < len(stamps) else math.inf, segment["end"] + 1.0, stamps[k] + LONGEST)
            rest = (times >= stamps[k] - REST[0]) & (times <= stamps[k] - REST[1]) & ok
            frames = (times > stamps[k]) & (times <= end) & ok
            if rest.sum() < 2 or frames.sum() < 3 or k == 0:
                continue
            rows.append((begin, sent[k - 1][1], float(np.median(view[rest])), [stamps[k] - begin], [sent[k][1]],
                         times[frames] - begin, view[frames], segment["name"]))
    return _pack(rows)


def part_shots(servo: str, times: np.ndarray, view: np.ndarray, ok: np.ndarray, commands, segments, *,
               parts: tuple[str, ...]) -> Shots:
    """One shot per whole part (a validation staircase, say), run free from the rest before it."""
    sent = sorted((t, v) for t, name, v in commands if name == servo and v is not None)
    rows = []
    for segment in segments:
        if not any(segment["name"] == f"{servo} {part}" for part in parts):
            continue
        inside = [(t, v) for t, v in sent if segment["start"] <= t <= segment["end"]]
        before = [(t, v) for t, v in sent if t < segment["start"]]
        if not inside or not before:
            continue
        begin = inside[0][0] - BEFORE
        rest = (times >= inside[0][0] - REST[0]) & (times <= inside[0][0] - REST[1]) & ok
        frames = (times > inside[0][0]) & (times <= segment["end"]) & ok
        rows.append((begin, before[-1][1], float(np.median(view[rest])), [t - begin for t, _ in inside],
                     [v for _, v in inside], times[frames] - begin, view[frames], segment["name"]))
    return _pack(rows)


def _pack(rows) -> Shots:
    count = len(rows)
    width_c = max((len(r[3]) for r in rows), default=1)
    width_f = max((len(r[5]) for r in rows), default=1)
    command_times, command_values = np.full((count, width_c), np.inf), np.zeros((count, width_c))
    frame_times, measured = np.full((count, width_f), np.nan), np.full((count, width_f), np.nan)
    for i, (_, _, _, ct, cv, ft, mv, _) in enumerate(rows):
        command_times[i, :len(ct)], command_values[i, :len(cv)] = ct, cv
        frame_times[i, :len(ft)], measured[i, :len(mv)] = ft, mv
    return Shots(np.array([r[0] for r in rows]), np.array([r[1] for r in rows]), np.array([r[2] for r in rows]),
                 command_times, command_values, frame_times, measured, [r[7] for r in rows])


def simulate(model: AxisModel, shots: Shots, offset: float = 0.0, step: float = STEP) -> np.ndarray:
    """The plant's view at each shot's frame times (S, F; nan where the shot has no frame), each shot started
    at rest: its servo at the command before, its linkage where the measured view says within the slack,
    its view as measured. `offset` is the view (degrees) the reference saw with the linkage at MIDDLE."""
    m = model
    count = len(shots)
    servo = shots.servo.astype(float).copy()
    half = m.slack / 2.0
    linkage = np.clip(MIDDLE + _inverse_view(m, shots.view - offset), servo - half, servo + half)
    view, velocity = shots.view.astype(float).copy(), np.zeros(count)
    end = np.nanmax(shots.frame_times) if np.isfinite(shots.frame_times).any() else 0.0
    steps = int(math.ceil(end / step)) + 1
    trace = np.empty((steps + 1, count))
    trace[0] = view
    rows = np.arange(count)
    sent = shots.command_times
    previous = servo.copy()  # The command acting before the latest arrival
    sub = max(1, math.ceil(step / MODE_STEP - 1e-9))
    h = step / sub
    spring = m.frequency ** 2
    up = 2.0 * m.damping * m.frequency
    down = 2.0 * (m.damping if m.damping_down is None else m.damping_down) * m.frequency
    for k in range(steps):
        now = (k + 1) * step  # The end of this step, as Axis.advance(now, dt) takes it
        due = now - m.dead_time
        arrived = (sent <= due).sum(axis=1) - 1  # Latest command sent a dead time ago, per shot
        target = np.where(arrived >= 0, shots.command_values[rows, np.maximum(arrived, 0)], previous)
        # Of this step, the part after its target's command arrived (all of it if earlier)
        arrival = np.where(arrived >= 0, sent[rows, np.maximum(arrived, 0)] + m.dead_time, -np.inf)
        moving = np.clip((now - arrival) / step, 0.0, 1.0)
        travel = m.slew * step * moving
        servo = servo + np.clip(target - servo, -travel, travel)
        linkage = np.clip(linkage, servo - half, servo + half)
        goal = m.gain * (linkage - MIDDLE) + m.curvature * (linkage - MIDDLE) ** 2 + offset
        if m.frequency <= 0.0:
            view, velocity = goal.copy(), np.zeros(count)
        else:
            offset_mode = view - goal
            for _ in range(sub):
                pull = -spring * offset_mode - np.where(velocity >= 0.0, up, down) * velocity
                still = velocity == 0.0
                stick = m.friction if m.stick is None else m.stick
                stick_down = stick if m.stick_down is None else m.stick_down
                stuck = still & (pull <= stick) & (pull >= -stick_down)
                pull = pull - np.where(still, np.sign(pull), np.sign(velocity)) * m.friction
                moved = velocity + pull * h
                moved = np.where(~still & (moved * velocity < 0.0), 0.0, moved)  # Friction stops it, never reverses it
                velocity = np.where(stuck, velocity, moved)
                offset_mode = np.where(stuck, offset_mode, offset_mode + velocity * h)
            view = offset_mode + goal
        trace[k + 1] = view
    grid = np.arange(steps + 1) * step
    out = np.full(shots.frame_times.shape, np.nan)
    for i in range(count):
        have = np.isfinite(shots.frame_times[i])
        out[i, have] = np.interp(shots.frame_times[i, have], grid, trace[:, i])
    return out


def _inverse_view(model: AxisModel, view: np.ndarray) -> np.ndarray:
    """Servo degrees from MIDDLE at which the linkage holds `view` at rest: model.view_of's inverse near MIDDLE."""
    if model.curvature == 0.0:
        return view / model.gain
    a, b = model.curvature, model.gain
    root = np.sqrt(np.maximum(b * b + 4.0 * a * view, 0.0))
    return (-b + np.sign(b) * root) / (2.0 * a)


def fit_percent(predicted: np.ndarray, measured: np.ndarray) -> float:
    """100 (1 - |error| / |measured - its mean|) over the frames both have, each shot (row) about its own
    mean: how much of each step's motion the model explains. 100 is perfect, 0 no better than standing still."""
    predicted, measured = np.atleast_2d(predicted), np.atleast_2d(measured)
    have = np.isfinite(predicted) & np.isfinite(measured)
    means = np.array([np.mean(row[keep]) if keep.any() else 0.0 for row, keep in zip(measured, have)])
    error = np.linalg.norm((predicted - measured)[have])
    spread = np.linalg.norm((measured - means[:, None])[have])
    return float(100.0 * (1.0 - error / spread)) if spread > 0 else float("nan")


def rms(predicted: np.ndarray, measured: np.ndarray) -> float:
    have = np.isfinite(predicted) & np.isfinite(measured)
    return float(np.sqrt(np.mean((predicted[have] - measured[have]) ** 2)))


# The parameters fitted, their bounds, and how each sits in AxisModel (None: the view offset)
FIELDS = ("gain", "dead_time", "slew", "slack", "frequency", "damping", "damping_down", "friction", "curvature", "stick",
          "stick_down")
BOUNDS = {"gain": (0.01, 1.0), "dead_time": (0.0, 0.15), "slew": (50.0, 2000.0), "slack": (0.0, 30.0),
          "frequency": (2.0, 200.0), "damping": (0.05, 2.0), "damping_down": (0.05, 2.0), "friction": (0.0, 500.0),
          "curvature": (-0.01, 0.01), "stick": (0.0, 5000.0), "stick_down": (0.0, 5000.0), "offset": (-5.0, 5.0)}


def axis_fit(start: AxisModel, shots: Shots, *, held_out: Shots | None = None, fixed: tuple[str, ...] = (),
             noise: float = 0.02, max_nfev: int = 200) -> dict:
    """Least squares (soft L1, scaled to `noise` degrees) of the model's fields and the view offset to
    the shots; `fixed` fields keep `start`'s values (a mode faster than the frames can see, say).
    Returns the model, offset, the fit's numbers on the fitted and held-out shots, and the
    parameters' standard errors from the Jacobian."""
    from scipy.optimize import least_squares

    free = [name for name in FIELDS if name not in fixed] + ["offset"]
    base = replace(start, damping_down=start.damping if start.damping_down is None else start.damping_down,
                   stick=start.stick if "stick" in fixed or start.stick is not None else start.friction,  # Fixed: as given
                   stick_down=start.stick_down if "stick_down" in fixed or start.stick_down is not None
                   else (start.friction if start.stick is None else start.stick))
    have = np.isfinite(shots.measured)

    def unpack(x):
        values = dict(zip(free, x))
        offset = values.pop("offset")
        return replace(base, **values), offset

    def residuals(x):
        model, offset = unpack(x)
        return (simulate(model, shots, offset) - shots.measured)[have]

    x0 = np.array([getattr(base, name) if name != "offset" else 0.0 for name in free])
    lower, upper = np.array([BOUNDS[n][0] for n in free]), np.array([BOUNDS[n][1] for n in free])
    x0 = np.clip(x0, lower + 1e-9, upper - 1e-9)
    found = least_squares(residuals, x0, bounds=(lower, upper), loss="soft_l1", f_scale=noise, x_scale="jac",
                          diff_step=1e-3, max_nfev=max_nfev)
    model, offset = unpack(found.x)
    spread = np.sqrt(np.abs(np.diag(np.linalg.pinv(found.jac.T @ found.jac))) * np.mean(found.fun ** 2))
    report = {"model": model, "offset": offset, "free": free,
              "standard_errors": dict(zip(free, spread.tolist())), "evaluations": int(found.nfev),
              "fitted": _scores(model, offset, shots)}
    if held_out is not None and len(held_out):
        report["held_out"] = _scores(model, offset, held_out)
    return report


def _scores(model: AxisModel, offset: float, shots: Shots) -> dict:
    predicted = simulate(model, shots, offset)
    return {"shots": len(shots), "fit_percent": fit_percent(predicted, shots.measured),
            "rms_degrees": rms(predicted, shots.measured)}
