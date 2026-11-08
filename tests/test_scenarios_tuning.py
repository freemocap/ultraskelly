"""The robot's head as launched, on the simulated robot as recorded on 2026-11-06.

The plant (simulator/plant.py) replays the recorded motor-response run within the fidelity bounds of
tests/test_plant_fidelity.py; the battery (scenarios.HEAD_BATTERY) is the head alone, waist braked. The
launched aim is one simulator/tuning.py found on its stated cost: here it must pass the battery on the
measured plant, and stay calm on plants that differ from the fit: the corners the tuner weighed (gearing
and pixels per degree 15 % either way), pan's gearing anywhere from 0.85 to 1.4 times the fit (it measured
0.26-0.36 between runs), a head that rings at 45 % instead of 22-32 %, 1.5 times the dead time, and others
it did not weigh. The release check on 59 random plants is `python -m ultraskelly.simulator.tuning check`;
a few of those plants run here.
"""
import dataclasses

import pytest

from ultraskelly.__main__bot import launch_config
from ultraskelly.simulator import plant, robustness
from ultraskelly.simulator import scenarios as S

CALM = ("overshoot", "oscillation", "runaway", "chased")  # Failures that mean the head swings or chases

# Plants the tuner did not weigh: the camera's delay either side of the 50-70 ms measured, the OAK's field
# of view (69 degrees on its datasheet, unmeasured) 5 degrees either way, and a tilt that barely turns or is
# loose: on 2026-11-06 it went from 9-10 px per 12 servo degrees to 0-4.5 px per 25, so its travel is
# uncertain until its linkage is checked. The head may then miss someone above or below, but not swing.
UNWEIGHED = {
    "camera delay 0.04 s": {"camera_latency": 0.04},
    "camera delay 0.10 s": {"camera_latency": 0.10},
    "field of view 64": {"camera_hfov": 64.0},
    "field of view 74": {"camera_hfov": 74.0},
    "tilt barely turns": {"tilt_ratio": 0.02},
    "tilt loose": {"tilt_backlash": 20.0},
}
STRESSED = {
    **robustness.CORNERS,
    **{f"pan gearing x{ratio}": {"pan_ratio": plant.PAN.gain * ratio} for ratio in (0.85, 1.15, 1.3, 1.4)},
    "less damped (45 % overshoot)": {"pan_damping": 0.25, "pan_damping_down": 0.18},
    "slower (1.5 x dead time)": {"delay_scale": 1.5},
    **UNWEIGHED,
}


def launched(sim):
    S.head_alone(sim)
    for key, value in launch_config().orientation.aim.model_dump().items():  # In place: the head holds this object
        setattr(sim.head.params.aim, key, value)


@pytest.mark.parametrize("scenario", S.HEAD_BATTERY, ids=lambda scenario: scenario.name)
def test_the_launched_head_passes_its_battery_on_the_measured_plant(scenario):
    trace, verdict = S.run(scenario, launched)
    assert verdict.passed, verdict.failures
    assert verdict.overshoot <= 0.5


@pytest.mark.parametrize("plant", STRESSED)
def test_the_launched_head_stays_calm_on_stressed_plants(plant):
    swinging = []
    for scenario in S.HEAD_BATTERY:
        trace, verdict = S.run(dataclasses.replace(scenario, settings={**scenario.settings, **STRESSED[plant]}), launched)
        swinging += [f"{scenario.name}: {failure}" for failure in verdict.failures if failure.startswith(CALM)]
    assert not swinging


@pytest.mark.parametrize("index", range(4))
def test_the_launched_head_passes_its_battery_on_random_plants(index):
    plants = robustness.plants(4, seed=7)  # A screen; the release check runs 59 (simulator/tuning.py)
    failing = []
    for scenario in S.HEAD_BATTERY:
        trace, verdict = S.run(dataclasses.replace(scenario, settings={**scenario.settings, **plants[index]}), launched)
        failing += [f"{scenario.name}: {failure}" for failure in verdict.failures]
    assert not failing


def yaw_settle(trace: S.Trace) -> float | None:
    """Seconds until the camera's yaw stays within SETTLE_DEGREES of the followed person's."""
    off = [abs(person - camera) for camera, person in zip(trace.camera_yaw, trace.person_yaw)]
    last_out = max((n for n, error in enumerate(off) if error > S.SETTLE_DEGREES), default=-1)
    return None if last_out == len(off) - 1 else trace.time[last_out + 1]


def test_a_face_low_in_the_picture_pans_as_one_at_eye_level():
    def tilt_as_in_that_run(sim):  # Run 20261106-222520's tilt window, 20-130, which kept this face in the bottom band.
        launched(sim)  # Since 2026-11-07 tilt follows to 140 and lifts it about 8 px out of the band.
        sim.head.params.aim.tilt_range = (20.0, 130.0)

    low, _ = S.run(S.LOW_FACE, tilt_as_in_that_run)
    level, _ = S.run(S.HEAD_BY_NAME["step-left"], launched)
    assert min(row for row in low.target_y if row is not None) > 200  # Past where the edge gate once began at
    # the bottom (240 - edge_margin) from first to last
    assert yaw_settle(low) <= yaw_settle(level) + 0.1
    assert abs(low.camera_yaw[-1] - low.person_yaw[-1]) <= 0.5


def test_the_battery_catches_a_head_that_swings():
    def aggressive(sim):  # A planned move all the way, a high gain, no shaping, hard acceleration, no band
        S.head_alone(sim)
        aim = sim.head.params.aim
        aim.reach, aim.pan_gain, aim.pan_speed, aim.pan_acceleration, aim.shaper = 1.0, 10.0, 150.0, 1000.0, False
        aim.pan_settle = aim.pan_resume = 0.0
        aim.pan_dead_time = 0.0  # And a model that ignores the servo's delay

    trace, verdict = S.run(S.HEAD_BY_NAME["step-left"], aggressive)
    assert verdict.overshoot > S.OVERSHOOT_DEGREES and verdict.pan_reversals > 2 and not verdict.passed


def test_the_battery_catches_a_head_that_believes_single_false_frames():
    def credulous(sim):  # Someone new believed on one frame: the robot's 1-3 frame false detections take it
        launched(sim)
        sim.head.params.aim.confirm_frames = sim.head.params.aim.confirm_window = 1

    trace, verdict = S.run(S.HEAD_BY_NAME["ghosts-one-spot"], credulous)
    assert verdict.wandered > 10.0 and not verdict.passed
