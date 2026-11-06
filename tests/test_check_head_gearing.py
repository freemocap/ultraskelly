import pytest

from scripts.check_head_gearing import PX_PER_DEGREE, gearing, sweep, waist_speeds
from ultraskelly.simulator.engine import play


def test_a_sweep_goes_up_down_past_the_middle_and_back():
    assert sweep(10.0, 30.0) == [100, 110, 120, 110, 100, 90, 80, 70, 60, 70, 80, 90]


def picture_shifts(angles, view_per_step):
    """Pixel shifts for a sweep, from how far the view turns on each move."""
    return [view_per_step(start, end) * PX_PER_DEGREE for start, end in zip(angles, angles[1:])]


def test_slack_in_the_linkage_shows_as_short_steps_after_each_reversal():
    angles = [90.0] + sweep(10.0, 30.0)
    linkage = [90.0]
    for angle in angles[1:]:
        linkage.append(play(linkage[-1], angle, 8.0))  # 8 servo degrees of slack, as the simulator models it
    shifts = [0.36 * (end - start) * PX_PER_DEGREE for start, end in zip(linkage, linkage[1:])]
    assert gearing(angles, shifts) == {"ratio_up": 0.36, "ratio_down": 0.36, "slack_servo_degrees": 8.0}


def test_a_ratio_that_differs_by_direction_shows_without_slack():
    angles = [90.0] + sweep(10.0, 30.0)
    shifts = picture_shifts(angles, lambda start, end: (0.11 if end > start else 0.36) * abs(end - start))
    assert gearing(angles, shifts) == {"ratio_up": 0.11, "ratio_down": 0.36, "slack_servo_degrees": 0.0}


def test_moves_with_no_measurement_are_left_out():
    angles = [90.0] + sweep(10.0, 30.0)
    shifts = picture_shifts(angles, lambda start, end: 0.24 * abs(end - start))
    shifts[2] = None
    assert gearing(angles, shifts)["ratio_up"] == 0.24


def test_the_waist_speed_comes_from_the_longer_move_s_extra_travel():
    moves = {-0.6: {0.25: -30.0, 0.5: -80.0}, 0.6: {0.25: 20.0, 0.5: 45.0}}
    speeds = waist_speeds(moves)
    assert speeds["negative_drive"]["deg_per_s"] == pytest.approx(24.7, abs=0.05)
    assert speeds["negative_drive"]["at_full_drive"] == pytest.approx(41.2, abs=0.05)
    assert speeds["positive_drive"]["deg_per_s"] == pytest.approx(12.3, abs=0.05)
