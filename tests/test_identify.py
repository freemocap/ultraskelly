"""The identification fits (simulator/identify.py): their simulation is the plant's, and they recover a known plant."""
from dataclasses import replace

import numpy as np
import pytest

from ultraskelly.simulator import plant
from ultraskelly.simulator.identify import STEP, Shots, axis_fit, fit_percent, simulate


def shots(model: plant.AxisModel, steps: list[tuple[float, float]], *, frames: float = 1.0 / 30) -> Shots:
    """Shots of step commands (from, to), each sent so it arrives just after a simulation step ends, at rest."""
    count = len(steps)
    sent = 0.1 - (0.1 + model.dead_time) % STEP + 1e-9  # Arrives just after a step ends
    frame_times = np.tile(np.arange(sent + 0.01, sent + 1.0, frames), (count, 1))
    return Shots(start=np.zeros(count), servo=np.array([a for a, _ in steps], float),
                 view=np.array([model.view_of(a) for a, _ in steps]),
                 command_times=np.full((count, 1), sent), command_values=np.array([[b] for _, b in steps], float),
                 frame_times=frame_times, measured=np.zeros_like(frame_times), names=["step"] * count)


def axis_replay(model: plant.AxisModel, servo: float, value: float, sent: float, frame_times: np.ndarray) -> np.ndarray:
    """plant.Axis itself, at rest at `servo` with its linkage centred, sent `value` at `sent`."""
    axis = plant.Axis(model, start=servo)
    axis.command(sent, value)
    grid = np.arange(1, int(np.ceil(frame_times[-1] / STEP)) + 2) * STEP
    views = [axis.advance(now, STEP) for now in grid]
    return np.interp(frame_times, np.r_[0.0, grid], np.r_[axis.model.view_of(servo), views])


@pytest.mark.parametrize("model", [plant.PAN, plant.TILT, plant.ROLL, replace(plant.TILT, curvature=0.0004),
                                   replace(plant.PAN, friction=60.0, stick=700.0)],
                         ids=["pan", "tilt", "roll", "curved", "sticking"])
def test_the_fits_simulate_the_plant_exactly(model):
    steps = [(90.0, 120.0), (120.0, 60.0), (70.0, 72.0), (100.0, 95.0)]
    batch = shots(model, steps)
    found = simulate(model, batch)
    for i, (servo, value) in enumerate(steps):
        expected = axis_replay(model, servo, value, batch.command_times[i, 0], batch.frame_times[i])
        assert found[i] == pytest.approx(expected, abs=1e-6)


def test_a_known_plant_is_recovered_from_noisy_steps():
    pytest.importorskip("scipy")
    truth = replace(plant.PAN, gain=0.23, dead_time=0.051, slack=3.1, frequency=10.5, damping=0.35, damping_down=0.25)
    rng = np.random.default_rng(2)
    sizes = [2, 5, 10, 20, 40]
    steps, at = [], 90.0
    for size in sizes * 6:
        way = 1 if rng.random() < 0.5 else -1
        if not 40 <= at + way * size <= 140:
            way = -way
        steps.append((at, at + way * size))
        at += way * size
    batch = shots(truth, steps)
    batch.command_times += rng.uniform(0.0, 0.04, (len(steps), 1))  # Jittered, as the session sends them
    batch.measured = simulate(truth, batch) + rng.normal(0.0, 0.02, batch.measured.shape)
    held = rng.random(len(steps)) < 0.2  # Steps of every size kept back
    report = axis_fit(plant.PAN, batch.pick(~held), held_out=batch.pick(held), noise=0.02)
    model = report["model"]
    assert model.gain == pytest.approx(truth.gain, rel=0.01)
    assert model.dead_time == pytest.approx(truth.dead_time, abs=0.002)
    assert model.slack == pytest.approx(truth.slack, abs=0.2)
    assert model.frequency == pytest.approx(truth.frequency, rel=0.05)
    assert model.damping == pytest.approx(truth.damping, abs=0.05)
    assert report["held_out"]["rms_degrees"] < 0.03 and report["held_out"]["fit_percent"] > 95


def test_fit_percent_counts_each_shot_about_its_own_mean():
    measured = np.array([[0.0, 1.0, 2.0], [10.0, 11.0, 12.0]])
    assert fit_percent(measured, measured) == 100.0
    assert fit_percent(np.array([[1.0, 1.0, 1.0], [11.0, 11.0, 11.0]]), measured) == pytest.approx(0.0)
