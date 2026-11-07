"""The camera-rotation measurement (simulator/camera_rotation.py) on pictures rendered at known turns.

The world is a textured cylinder around the camera, so any yaw stays in view. Frames are rendered
through the robot's lens (its 69-degree pinhole with barrel distortion), with sensor noise, and with a
"person": a patch that changes every frame, masked as the logged boxes are.
"""
import math

import numpy as np
import pytest

from ultraskelly.simulator.camera_rotation import (
    Camera,
    RotationTracker,
    angles,
    box_mask,
    exposure_middle,
    orientation,
    rotation_ransac,
)

cv2 = pytest.importorskip("cv2")

SIZE = (640, 480)
LENS = Camera(Camera.from_field_of_view(SIZE, 69.0).matrix, SIZE, np.array([-0.12, 0.03, 0.0, 0.0, 0.0]))
WORLD_FOCAL = 600.0  # Pixels per radian of the panorama, finer than the camera's 464 per radian


def world(seed: int = 1) -> np.ndarray:
    """A 360-degree panorama of texture at several scales, as a room's walls might be."""
    rng = np.random.default_rng(seed)
    width, height = int(2 * math.pi * WORLD_FOCAL), 1400
    picture = np.zeros((height, width), np.float32)
    for scale, weight in ((3, 0.35), (12, 0.35), (48, 0.3)):
        coarse = rng.random((height // scale + 2, width // scale + 2)).astype(np.float32)
        picture += weight * cv2.resize(coarse, (width + 2 * scale, height + 2 * scale),
                                       interpolation=cv2.INTER_CUBIC)[scale:scale + height, scale:scale + width]
    picture = (picture - picture.min()) / (picture.max() - picture.min())
    return (picture * 220 + 15).astype(np.uint8)


def render(panorama: np.ndarray, turn: tuple[float, float, float], noise: float = 0.0, seed: int = 0) -> np.ndarray:
    """The robot's view of the panorama with the camera turned by (yaw, pitch, roll)."""
    width, height = SIZE
    grid = np.stack(np.meshgrid(np.arange(width), np.arange(height)), axis=2).reshape(-1, 2).astype(np.float32)
    rays = LENS.rays(grid) @ orientation(*turn).T  # Each pixel's ray, in the world's axes
    azimuth = np.arctan2(rays[:, 0], rays[:, 2])
    rise = rays[:, 1] / np.hypot(rays[:, 0], rays[:, 2])
    map_x = (azimuth * WORLD_FOCAL + panorama.shape[1] / 2).reshape(height, width).astype(np.float32)
    map_y = (rise * WORLD_FOCAL + panorama.shape[0] / 2).reshape(height, width).astype(np.float32)
    frame = cv2.remap(panorama, map_x, map_y, cv2.INTER_CUBIC, borderMode=cv2.BORDER_WRAP).astype(np.float32)
    frame += np.random.default_rng(seed).normal(0.0, noise, frame.shape)
    return np.clip(frame, 0, 255).astype(np.uint8)


@pytest.fixture(scope="module")
def panorama():
    return world()


def test_angles_undo_orientation():
    for turn in [(0.0, 0.0, 0.0), (17.0, -4.0, 2.5), (-120.0, 30.0, -10.0)]:
        assert angles(orientation(*turn)) == pytest.approx(turn, abs=1e-9)
    # Turning right brings what was ahead to the left of the picture, and down brings it up.
    ahead = np.array([0.0, 0.0, 1.0])
    assert (orientation(10.0, 0.0, 0.0).T @ ahead)[0] < 0
    assert (orientation(0.0, 10.0, 0.0).T @ ahead)[1] < 0


def test_rotation_ransac_ignores_outliers():
    rng = np.random.default_rng(3)
    a = rng.normal(size=(300, 3)) * [0.4, 0.3, 0.0] + [0.0, 0.0, 1.0]
    a /= np.linalg.norm(a, axis=1, keepdims=True)
    truth = orientation(5.0, -2.0, 1.0).T
    b = a @ truth.T
    b[:120] = rng.normal(size=(120, 3)) * 0.3 + [0, 0, 1]  # 40 % of matches wrong
    b /= np.linalg.norm(b, axis=1, keepdims=True)
    found, inliers = rotation_ransac(a, b, 0.004, rng)
    assert np.allclose(found, truth, atol=1e-9)
    assert inliers[120:].all() and inliers[:120].sum() < 5


@pytest.mark.parametrize("turn", [(0.05, 0.0, 0.0), (0.0, 0.03, 0.0), (0.0, 0.0, 0.3), (17.0, -5.0, 2.0),
                                  (-16.0, 4.0, -1.5), (3.0, 1.0, 0.5)])
def test_each_frame_is_measured_against_the_reference_to_a_hundredth_of_a_degree(panorama, turn):
    tracker = RotationTracker(LENS, render(panorama, (0.0, 0.0, 0.0), noise=2.0, seed=1))
    measured = tracker.measure(render(panorama, turn, noise=2.0, seed=2))
    assert measured.ok and measured.inliers > 100 and measured.correlation > 0.9
    assert (measured.yaw, measured.pitch, measured.roll) == pytest.approx(turn, abs=0.01)


def test_a_masked_person_does_not_move_the_measurement(panorama):
    tracker = RotationTracker(LENS, render(panorama, (0.0, 0.0, 0.0), noise=2.0, seed=1))
    rng = np.random.default_rng(5)
    for k, turn in enumerate([(2.0, 0.0, 0.0), (6.0, 1.0, 0.0), (10.0, -1.0, 0.5)]):
        frame = render(panorama, turn, noise=2.0, seed=10 + k)
        box = (180 + 40 * k, 100, 360 + 40 * k, 470)  # A person walking across, a third of the picture
        person = rng.integers(0, 255, (box[3] - box[1], box[2] - box[0]), dtype=np.uint8)
        frame[box[1]:box[3], box[0]:box[2]] = cv2.GaussianBlur(person, (7, 7), 2.0)
        measured = tracker.measure(frame, box_mask(SIZE, [box]))
        assert (measured.yaw, measured.pitch, measured.roll) == pytest.approx(turn, abs=0.01)


def test_keyframes_carry_a_turn_beyond_the_reference_and_back_without_drift(panorama):
    tracker = RotationTracker(LENS, render(panorama, (0.0, 0.0, 0.0)), new_keyframe_beyond=12.0)
    path = [(yaw, 0.5 * math.sin(yaw / 15.0), 0.0) for yaw in list(np.arange(0.0, 90.0, 3.0)) + list(np.arange(90.0, -1.0, -3.0))]
    errors = []
    for k, turn in enumerate(path):
        measured = tracker.measure(render(panorama, turn, noise=2.0, seed=k))
        assert measured.ok, turn
        errors.append(max(abs(measured.yaw - turn[0]), abs(measured.pitch - turn[1]), abs(measured.roll)))
    assert len(tracker.keyframes) > 5  # A quarter turn needs several
    assert max(errors) < 0.05  # Errors add up only from keyframe to keyframe
    assert errors[-1] < 0.01  # Back at the start it registers against the first reference again


def test_a_frame_is_timed_at_its_middle_rows_mid_exposure():
    # DepthAI stamps at the first row's end of exposure; the middle row is read half a readout later.
    assert exposure_middle(10.0, 0.004, 0.0165) == pytest.approx(10.0 - 0.002 + 0.00825)
