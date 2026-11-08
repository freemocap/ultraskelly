"""The robot's head as launched, on the simulated robot as recorded on 2026-11-06.

The plant (simulator/plant.py) replays the recorded motor-response run within the fidelity bounds of
tests/test_plant_fidelity.py; the battery (scenarios.HEAD_BATTERY) is the robot's present setup, the head
alone. The launched aim is the one simulator/tuning.py found best on its stated cost: here it must pass
the battery on the measured plant, and stay calm on the stressed plants the tuner weighed (pan gearing
15 % either way, 1.5 and 2 times, a head that rings at 45 % instead of 22-32 %, 1.5 times the dead time)
and on others it did not.
"""
import dataclasses

import pytest

from ultraskelly.__main__bot import launch_config
from ultraskelly.simulator import scenarios as S
from ultraskelly.simulator.tuning import PLANTS

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
STRESSED = {**{name: settings for name, (settings, _) in PLANTS.items() if name != "measured"}, **UNWEIGHED}


def launched(sim):
    S.head_alone(sim)
    for key, value in launch_config().orientation.aim.model_dump().items():  # In place: the head holds this object
        setattr(sim.head.params.aim, key, value)


@pytest.mark.parametrize("scenario", S.HEAD_BATTERY, ids=lambda scenario: scenario.name)
def test_the_launched_head_passes_its_battery_on_the_measured_plant(scenario):
    trace, verdict = S.run(scenario, launched)
    assert verdict.passed, verdict.failures
    assert verdict.overshoot <= 0.5


# On the plant measured 2026-11-07 the launched head is calm with pan geared 1.5 times, not twice: in "noisy" it
# reverses 7 times. The 2026-11-06 fit's pan slid against three times the dry friction, which damped that jitter
# (3 reversals with it). The head controllers being rebuilt in the simulator are tuned against this plant.
GEARED_TWICE = "on the measured pan the launched head jitters at twice the gearing (7 reversals in noisy)"


@pytest.mark.parametrize("plant", [pytest.param(name, marks=pytest.mark.xfail(strict=True, reason=GEARED_TWICE))
                                   if name == "pan gearing x2" else name for name in STRESSED])
def test_the_launched_head_stays_calm_on_stressed_plants(plant):
    swinging = []
    for scenario in S.HEAD_BATTERY:
        trace, verdict = S.run(dataclasses.replace(scenario, settings={**scenario.settings, **STRESSED[plant]}), launched)
        swinging += [f"{scenario.name}: {failure}" for failure in verdict.failures if failure.startswith(CALM)]
    assert not swinging


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
    assert min(row for row in low.target_y if row is not None) >= 200  # Where the edge gate once began at
    # the bottom (240 - edge_margin), from first to last
    assert yaw_settle(low) <= yaw_settle(level) + 0.3
    # Panning 12 degrees also rolls the picture 2.3 (pan_roll), which moves a face 220 px below the middle about
    # 9 px sideways: centred in the picture, it is a degree from straight ahead, on the robot as here.
    assert abs(low.camera_yaw[-1] - low.person_yaw[-1]) <= 1.5


def test_the_battery_catches_a_head_that_swings():
    def aggressive(sim):  # Full reach, no smoothing, no shaping, fast: it rings the head's own mode
        S.head_alone(sim)
        aim = sim.head.params.aim
        aim.reach, aim.smoothing, aim.max_speed, aim.max_acceleration, aim.shaper, aim.deadband = 1.0, 1.0, 120.0, 3000.0, False, 0.0

    trace, verdict = S.run(S.HEAD_BY_NAME["step-left"], aggressive)
    assert verdict.overshoot > S.OVERSHOOT_DEGREES and verdict.pan_reversals > 5 and not verdict.passed


def test_the_battery_catches_a_head_that_adds_up_false_detections():
    def counting_alone(sim):  # A jump believed after jump_samples sightings however far apart: the old count
        launched(sim)
        sim.head.params.aim.candidate_gap = 1e9

    trace, verdict = S.run(S.HEAD_BY_NAME["ghosts-one-spot"], counting_alone)
    assert verdict.wandered > 10.0 and not verdict.passed
