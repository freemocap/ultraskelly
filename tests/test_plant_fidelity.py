"""The simulated head against the robot: its identification session of 2026-11-07 (tests/data/motor-response-20261107).

scripts/record_motor_response.py drove each servo by itself, nothing in the loop, with the camera's exposure
locked; scripts/analyze_motor_response.py measured the camera's turn in every frame against a fixed reference
(noise floor 0.001 degrees; camera.json). The plant (simulator/plant.py) was fitted by simulator/identify.py to
the staircases, ladders and one-way steps, each step from the view measured before it. The random staircase
that ends each servo's parts was held out of the fit: here it runs free through the plant and must follow the
robot. These are the gates every controller tuned on the simulator relies on.
"""
import numpy as np
import pytest

from ultraskelly.simulator import plant
from ultraskelly.simulator.fidelity import load, predict
from ultraskelly.simulator.identify import fit_percent, rms

MODELS = {"pan": plant.PAN, "tilt": plant.TILT, "roll": plant.ROLL}
# (least fit %, largest RMS degrees) on the held-out staircase, run free; and the RMS over the fitted steps.
# Fit % counts each part about its own mean: 100 follows it exactly, 0 no better than standing still.
HELD_OUT = {"tilt": (90.0, 0.2), "roll": (70.0, 0.3), "pan": (80.0, 1.0)}
FITTED_RMS = {"tilt": 0.12, "roll": 0.25, "pan": 1.0}
# On 2026-11-07 pan stuck and slipped (15-40 servo degrees still, then a jump), unlike on 2026-11-06 and in the
# afternoon's following runs; the plant keeps the 2026-11-06 pan until the owner has looked at the neck.
PAN_STUCK = pytest.mark.xfail(strict=True, reason="pan stuck and slipped on 2026-11-07; the plant keeps 2026-11-06's pan")
SERVOS = [pytest.param("pan", marks=PAN_STUCK), "tilt", "roll"]


@pytest.fixture(scope="module")
def recording():
    return load()


@pytest.mark.parametrize("servo", SERVOS)
def test_the_held_out_staircase_runs_free_as_the_robot_did(servo, recording):
    predicted, measured = predict(MODELS[servo], servo, recording, held_out=True)
    least_fit, largest_rms = HELD_OUT[servo]
    assert fit_percent(predicted, measured) >= least_fit
    assert rms(predicted, measured) <= largest_rms


@pytest.mark.parametrize("servo", SERVOS)
def test_each_fitted_step_follows_the_robot(servo, recording):
    predicted, measured = predict(MODELS[servo], servo, recording, held_out=False)
    assert rms(predicted, measured) <= FITTED_RMS[servo]


# The evening before (2026-11-06), every part run free: another day, battery and temperature, half-size frames with
# auto-exposure. Tilt and roll as fitted on 2026-11-07, pan as it slid that evening (fit % least, RMS degrees most).
CROSS_DAY = {"pan": (85.0, 1.0), "tilt": (70.0, 0.6), "roll": (45.0, 0.7)}


@pytest.mark.parametrize("servo", list(MODELS))
def test_the_evening_before_runs_free_through_the_plant(servo):
    from ultraskelly.simulator.fidelity import CROSS_DAY as FOLDER, every_part

    before = load(FOLDER)
    predicted, measured = predict(MODELS[servo], servo, before, held_out=True, parts=every_part(servo, before))
    least_fit, largest_rms = CROSS_DAY[servo]
    assert fit_percent(predicted, measured) >= least_fit
    assert rms(predicted, measured) <= largest_rms


def test_the_measurement_is_far_finer_than_the_gates(recording):
    import json

    from ultraskelly.simulator.fidelity import RECORDING

    noise = json.loads((RECORDING / "camera.json").read_text())["noise_floor_degrees"]
    assert max(noise.values()) < 0.005  # Against gates of 0.12 degrees and more
    assert recording.ok.mean() > 0.98


def test_the_waist_turns_as_its_pulses_did():
    # 2026-11-06: 0.6 drive did not move it in 0.5 s; 0.7 for 0.5 s turned it 13.3 degrees with negative drive
    # and 5.7 with positive (views then chained frame to frame, at the 8.1 px/degree once assumed).
    for drive, seconds, measured in ((-0.6, 0.5, 0.0), (0.6, 0.5, 0.0), (-0.7, 0.5, 13.26), (0.7, 0.5, -5.66)):
        waist = plant.Waist(plant.WAIST)
        for tick in range(200):
            waist.advance(drive if tick * 0.01 < seconds else 0.0, 0.01)
        assert waist.angle == pytest.approx(measured, abs=0.5)
