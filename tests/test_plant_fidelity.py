"""The simulated head against the robot: every recorded command replayed through the plant (simulator/plant.py).

The recording (tests/data/motor-response-20261106) stepped, staircased and ramped each servo with the
camera watching; scripts/analyze_motor_response.py turned the frames into the view per frame. The
plant's models were fitted to it by least squares on each segment's view plus the big steps' shape.
These are the gates every controller tuning on the simulator relies on.
"""
import numpy as np
import pytest

from scripts.analyze_motor_response import analyze_servo
from ultraskelly.simulator import plant
from ultraskelly.simulator.fidelity import load, replay, rms, segment_errors

MODELS = {"pan": plant.PAN, "tilt": plant.TILT, "roll": plant.ROLL}
# Degrees of RMS error over every segment of the servo, each aligned where it began.
LARGEST_RMS = {"pan": 1.5, "tilt": 1.0, "roll": 1.0}
OVERSHOOT_POINTS = 8.0  # Each big step's overshoot, percentage points either way
# Each big step's 5 % settling time, seconds either way. Pan's model rings 0.24 s longer than the robot
# after its 90 -> 140 step (0.76 s against 0.52): longer, the safe side for tuning a controller on it.
SETTLE_SECONDS = {"pan": 0.25, "tilt": 0.2, "roll": 0.2}
# Each big step's gain, as a share either way. The roll's gain was uneven on the robot (0.050-0.083 over
# its three big steps), so its one fitted gain (0.063) sits within a third of each.
GAIN_SHARE = {"pan": 0.15, "tilt": 0.15, "roll": 0.35}


@pytest.fixture(scope="module")
def recording():
    return load()


@pytest.fixture(scope="module")
def replays(recording):
    return {servo: replay(model, servo, recording) for servo, model in MODELS.items()}


@pytest.mark.parametrize("servo", list(MODELS))
def test_the_simulated_view_follows_the_recorded_one(servo, recording, replays):
    errors = segment_errors(replays[servo], recording, servo)
    assert rms(errors) < LARGEST_RMS[servo], {segment["name"]: round(float(np.sqrt(np.mean(e ** 2))), 2) for segment, e in errors}


@pytest.mark.parametrize("servo", list(MODELS))
def test_big_steps_overshoot_and_settle_as_the_robot_did(servo, recording, replays):
    measured = analyze_servo(servo, recording.views[servo], recording.times, recording.commands, recording.segments)
    modelled = analyze_servo(servo, replays[servo], recording.times, recording.commands, recording.segments)
    steps = list(zip(measured["big_steps"], modelled["big_steps"]))
    assert len(steps) == 3
    for real, simulated in steps:
        label = f"{servo} {real['from']:.0f}->{real['to']:.0f}"
        assert abs(simulated["overshoot_percent"] - real["overshoot_percent"]) <= OVERSHOOT_POINTS, label
        assert abs(simulated["settle_5_percent"] - real["settle_5_percent"]) <= SETTLE_SECONDS[servo], label
        assert simulated["gain"] == pytest.approx(real["gain"], rel=GAIN_SHARE[servo]), label


def test_the_waist_turns_as_its_pulses_did():
    for drive, seconds, measured in ((-0.6, 0.5, 0.0), (0.6, 0.5, 0.0), (-0.7, 0.5, 13.26), (0.7, 0.5, -5.66)):
        waist = plant.Waist(plant.WAIST)
        for tick in range(200):
            waist.advance(drive if tick * 0.01 < seconds else 0.0, 0.01)
        assert waist.angle == pytest.approx(measured, abs=0.5)
