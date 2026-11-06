from dataclasses import replace

import pytest

from ultraskelly.simulator.scenarios import SCENARIOS, Scenario, reversals, run, still


@pytest.mark.parametrize("scenario", SCENARIOS, ids=lambda scenario: scenario.name)
def test_the_head_and_waist_follow_people_without_swinging(scenario):
    trace, verdict = run(scenario)
    assert verdict.passed, f"{scenario.description}: {verdict.failures}"


# The robot may differ from what the controller assumes. The camera's delay was measured at
# 50-70 ms; the servos' lag and the OAK's field of view (69 degrees on its datasheet, so
# 8.1 px per degree) are unmeasured. On 2026-11-06 12 servo degrees moved the picture 11-35 px
# in pan (0.11-0.36 degrees of view per servo degree; the head assumes 0.24) and 9-10 px in tilt
# (0.1); the waist was timed at 18-23 deg/s at 0.75 drive (the head assumes 27 at full drive).
# Every scenario must pass across that range; a pan ratio under 0.20, a waist much faster than
# assumed or more than about 6 servo degrees of slack in the linkage are not covered yet.
PLANTS = [
    {"camera_latency": latency, "servo_lag": lag, "camera_hfov": hfov}
    for latency in (0.04, 0.10) for lag in (0.02, 0.08) for hfov in (64.0, 74.0)
] + [
    {"pan_ratio": 0.20}, {"pan_ratio": 0.36}, {"tilt_ratio": 0.07}, {"tilt_ratio": 0.14},
    {"waist_speed": 17.0}, {"waist_speed": 32.0}, {"head_backlash": 4.0},
    {"pan_ratio": 0.20, "waist_speed": 17.0}, {"pan_ratio": 0.30, "waist_speed": 32.0},
    # The robot took about 0.15 s from a servo command to the camera seeing it (2026-11-06).
    {"camera_latency": 0.10, "servo_lag": 0.10}, {"camera_latency": 0.055, "servo_lag": 0.12},
]


@pytest.mark.parametrize("plant", PLANTS, ids=lambda plant: ",".join(f"{key}={value}" for key, value in plant.items()))
def test_the_head_stays_calm_on_a_robot_that_differs_from_its_estimates(plant):
    failing = []
    for scenario in SCENARIOS:
        trace, verdict = run(replace(scenario, settings={**scenario.settings, **plant}))
        if not verdict.passed:
            failing.append(f"{scenario.name}: {verdict.failures}")
    assert not failing


def test_a_reversed_pan_servo_is_caught_before_any_motor_runs():
    reversed_pan = Scenario("reversed-pan", "The pan servo turns the camera the wrong way", 4.0, still(20.0),
                            settings={"pan_sign": -1})
    trace, verdict = run(reversed_pan)
    assert verdict.runaway and not verdict.passed


def test_reversals_count_turns_back_but_not_wobble():
    assert reversals([0, 1, 2, 3, 2.4, 2.6]) == 1
    assert reversals([0, 3, 0, 3, 0]) == 3
    assert reversals([0, 0.3, 0.1, 0.4, 0.2]) == 0
