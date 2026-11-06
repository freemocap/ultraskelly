import numpy as np
import pytest

from scripts.check_motor_directions import SERVO_STEP, judge, rotation

cv2 = pytest.importorskip("cv2")  # OpenCV is in the hardware extra; this runs where it is installed


def scene() -> np.ndarray:
    """A detailed still scene, as RGB: blurred noise with blocks, so features match between pictures."""
    rng = np.random.default_rng(0)
    grey = cv2.GaussianBlur((rng.random((480, 640)) * 255).astype(np.uint8), (0, 0), 2.0)
    for _ in range(40):
        x, y = int(rng.integers(0, 600)), int(rng.integers(0, 440))
        cv2.rectangle(grey, (x, y), (x + 30, y + 30), int(rng.integers(0, 255)), -1)
    return cv2.cvtColor(grey, cv2.COLOR_GRAY2RGB)


def turn(image: np.ndarray, clockwise: float) -> np.ndarray:
    """The picture's content turned `clockwise` degrees about its middle, as seen on screen."""
    turning = cv2.getRotationMatrix2D((320, 240), -clockwise, 1.0)  # OpenCV's positive angle is anticlockwise on screen
    return cv2.warpAffine(image, turning, (640, 480), borderMode=cv2.BORDER_REFLECT)


@pytest.mark.parametrize("clockwise", [3.0, -2.0])
def test_a_turned_picture_is_measured_by_its_sign_and_degrees(clockwise):
    picture = scene()
    measured = rotation(cv2, picture, turn(picture, clockwise))
    assert measured["degrees"] == pytest.approx(clockwise, abs=0.2)
    assert measured["inliers"] >= 12


def test_a_still_picture_has_not_turned():
    picture = scene()
    assert rotation(cv2, picture, picture)["degrees"] == pytest.approx(0.0, abs=0.05)


def test_the_roll_verdict_says_which_way_and_how_far_and_whether_the_controller_agrees():
    moves = {f"{name} {sign}{SERVO_STEP:g} deg": {"dx": 0.0, "dy": 0.0, "response": 0.0}
             for name in ("pan", "tilt") for sign in "+-"}
    moves["waist -0.7 for 0.5 s"] = {"dx": 0.0, "dy": 0.0, "response": 0.0}
    moves[f"roll +{SERVO_STEP:g} deg"] = {"degrees": -3.6, "dx": 0.0, "dy": 0.0, "matches": 90, "inliers": 80}
    moves[f"roll -{SERVO_STEP:g} deg"] = {"degrees": 3.6, "dx": 0.0, "dy": 0.0, "matches": 90, "inliers": 80}
    assert judge(moves, roll_direction=1)["roll"] == {
        "more_degrees_turns_content": "anticlockwise", "picture_degrees_per_servo_degree": 0.3, "matches_controller": True,
    }
    assert judge(moves, roll_direction=-1)["roll"]["matches_controller"] is False
