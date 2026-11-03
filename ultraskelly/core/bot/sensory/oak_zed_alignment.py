"""Where the OAK sits relative to the ZED: what carries a pixel of the OAK image into ZED depth.

The OAK-1 in the nose hole and the ZED Mini in the eye sockets are fixed together in the
skull, a few centimetres apart and facing the same way. To find how far away something the
pose detector saw is, the face locator follows that pixel's ray from the OAK into the ZED's
depth, so it needs the OAK's intrinsics and the OAK's pose in the ZED's rectified left
camera frame (x image right, y down, z forward, metres).

This OAK-1 carries no factory calibration, so its intrinsics start from its datasheet field
of view, 69 by 55 degrees, centred. The pose comes from features both cameras see: each
gives a 3D point from ZED depth and a pixel in the OAK image, and OpenCV's solvePnPRansac
finds the rotation and translation that reproject the points onto the pixels. The result
belongs to one mounting of the two cameras: redo it when either is re-seated.
"""
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ultraskelly.core.bot.sensory.stereo_rectification import CALIBRATION_FOLDER, StereoRectification

ALIGNMENT_FILE = CALIBRATION_FOLDER.parent / "oak-zed-alignment.json"
OAK_FIELD_OF_VIEW = (69.0, 55.0)  # Degrees, horizontal and vertical (OAK-1 datasheet)
MIN_POINTS = 12  # Fewer matched points with depth than this cannot pin down a pose


def intrinsics_from_field_of_view(size: tuple[int, int], field_of_view: tuple[float, float]) -> np.ndarray:
    """A centred pinhole camera matrix for an image of `size` (width, height) seeing `field_of_view` degrees."""
    width, height = size
    horizontal, vertical = np.radians(field_of_view)
    return np.array([
        [width / 2 / np.tan(horizontal / 2), 0.0, (width - 1) / 2],
        [0.0, height / 2 / np.tan(vertical / 2), (height - 1) / 2],
        [0.0, 0.0, 1.0],
    ])


@dataclass(frozen=True)
class OakZedAlignment:
    """The OAK camera's intrinsics and its pose in the ZED's rectified left camera frame:
    a point at X in the OAK camera's frame is at rotation @ X + translation in the ZED's."""

    oak_camera_matrix: np.ndarray  # 3x3, pixels of the OAK image as the bot sees it (after any flip)
    oak_size: tuple[int, int]  # (width, height)
    rotation: np.ndarray  # 3x3
    translation: np.ndarray  # (3,) metres: the OAK camera's centre in the ZED frame
    error_px: float = float("nan")  # Median reprojection error of the points the pose was fitted to
    points: int = 0  # How many matched points with depth agreed with the pose

    def ray(self, pixel: tuple[float, float]) -> tuple[np.ndarray, np.ndarray]:
        """The ray through an OAK pixel, in the ZED frame: its origin and unit direction."""
        direction = np.linalg.solve(self.oak_camera_matrix, np.array([pixel[0], pixel[1], 1.0]))
        direction = self.rotation @ direction
        return self.translation.copy(), direction / np.linalg.norm(direction)

    def to_json(self) -> dict:
        return {
            "oak_camera_matrix": self.oak_camera_matrix.tolist(),
            "oak_size": list(self.oak_size),
            "rotation": self.rotation.tolist(),
            "translation_m": self.translation.tolist(),
            "error_px": self.error_px,
            "points": self.points,
            "frames": "OAK camera (x image right, y down, z forward) to the ZED's rectified left camera, metres",
        }

    @classmethod
    def from_json(cls, data: dict) -> "OakZedAlignment":
        return cls(
            oak_camera_matrix=np.array(data["oak_camera_matrix"], dtype=float),
            oak_size=tuple(data["oak_size"]),
            rotation=np.array(data["rotation"], dtype=float),
            translation=np.array(data["translation_m"], dtype=float),
            error_px=float(data.get("error_px", float("nan"))),
            points=int(data.get("points", 0)),
        )

    def save(self, path: Path = ALIGNMENT_FILE) -> None:
        path.write_text(json.dumps(self.to_json(), indent=2) + "\n", encoding="utf-8")

    @classmethod
    def load(cls, path: Path = ALIGNMENT_FILE) -> "OakZedAlignment":
        return cls.from_json(json.loads(path.read_text(encoding="utf-8")))


def points_from_depth(pixels: np.ndarray, depth: np.ndarray, rectification: StereoRectification) -> tuple[np.ndarray, np.ndarray]:
    """3D points (metres, ZED frame) under (N, 2) pixels of a depth image, and which pixels had depth."""
    columns = np.clip(np.rint(pixels[:, 0]).astype(int), 0, depth.shape[1] - 1)
    rows = np.clip(np.rint(pixels[:, 1]).astype(int), 0, depth.shape[0] - 1)
    z = depth[rows, columns].astype(float) / 1000.0
    known = z > 0
    x = (pixels[:, 0] - rectification.cx) * z / rectification.focal_length
    y = (pixels[:, 1] - rectification.cy) * z / rectification.focal_length
    return np.stack([x, y, z], axis=1)[known], known


def fit_alignment(
    points: np.ndarray, oak_pixels: np.ndarray, oak_camera_matrix: np.ndarray, oak_size: tuple[int, int],
    reprojection_px: float = 4.0,
) -> OakZedAlignment:
    """The OAK's pose from 3D points in the ZED frame and the OAK pixels they appear at."""
    import cv2

    if len(points) < MIN_POINTS:
        raise ValueError(f"{len(points)} matched points with depth; at least {MIN_POINTS} are needed")
    found, rotation_vector, translation, inliers = cv2.solvePnPRansac(
        points.astype(np.float64), oak_pixels.astype(np.float64), oak_camera_matrix, None,
        reprojectionError=reprojection_px, iterationsCount=500, confidence=0.999, flags=cv2.SOLVEPNP_EPNP,
    )
    if not found or inliers is None or len(inliers) < MIN_POINTS:
        raise ValueError("No OAK pose agrees with enough of the matched points")
    inliers = inliers.ravel()
    found, rotation_vector, translation = cv2.solvePnP(  # Refine on the agreeing points
        points[inliers].astype(np.float64), oak_pixels[inliers].astype(np.float64), oak_camera_matrix, None,
        rotation_vector, translation, useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE,
    )
    projected, _ = cv2.projectPoints(points[inliers], rotation_vector, translation, oak_camera_matrix, None)
    error = float(np.median(np.linalg.norm(projected.reshape(-1, 2) - oak_pixels[inliers], axis=1)))
    zed_to_oak, _ = cv2.Rodrigues(rotation_vector)  # X_oak = R X_zed + t: invert it
    rotation = zed_to_oak.T
    return OakZedAlignment(
        oak_camera_matrix=oak_camera_matrix, oak_size=oak_size, rotation=rotation,
        translation=-(rotation @ translation.ravel()), error_px=error, points=len(inliers),
    )


def matched_points(oak_grey: np.ndarray, zed_grey: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Pixels of SIFT features matched between the OAK image and the ZED's rectified left image
    (Lowe's ratio test): (N, 2) in each."""
    import cv2

    sift = cv2.SIFT_create(nfeatures=3000)
    (oak_keys, oak_descriptors), (zed_keys, zed_descriptors) = (sift.detectAndCompute(image, None) for image in (oak_grey, zed_grey))
    if oak_descriptors is None or zed_descriptors is None:
        return np.zeros((0, 2)), np.zeros((0, 2))
    pairs = cv2.BFMatcher(cv2.NORM_L2).knnMatch(oak_descriptors, zed_descriptors, k=2)
    good = [best for best, second in (pair for pair in pairs if len(pair) == 2) if best.distance < 0.75 * second.distance]
    return (np.array([oak_keys[m.queryIdx].pt for m in good]).reshape(-1, 2),
            np.array([zed_keys[m.trainIdx].pt for m in good]).reshape(-1, 2))


def estimate_alignment(
    views: list[tuple[np.ndarray, np.ndarray, np.ndarray]], rectification: StereoRectification,
    field_of_view: tuple[float, float] = OAK_FIELD_OF_VIEW,
) -> OakZedAlignment:
    """The alignment from views taken at the same moments: (OAK RGB image, the ZED's rectified
    left RGB image, its uint16 millimetre depth), the last two the size `rectification` describes."""
    import cv2

    grey = lambda image: cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    oak_height, oak_width = views[0][0].shape[:2]
    all_points, all_pixels, matches = [], [], 0
    for oak, zed, depth in views:
        oak_pixels, zed_pixels = matched_points(grey(oak), grey(zed))
        matches += len(zed_pixels)
        if len(zed_pixels) == 0:
            continue
        points, known = points_from_depth(zed_pixels, depth, rectification)
        all_points.append(points)
        all_pixels.append(oak_pixels[known])
    points = np.concatenate(all_points) if all_points else np.zeros((0, 3))
    pixels = np.concatenate(all_pixels) if all_pixels else np.zeros((0, 2))
    if len(points) < MIN_POINTS:
        raise ValueError(
            f"{matches} features matched between the cameras over {len(views)} views, {len(points)} of them "
            f"with ZED depth; at least {MIN_POINTS} are needed. Point both cameras at a detailed scene "
            f"0.5 to 3 m away that they both see."
        )
    camera_matrix = intrinsics_from_field_of_view((oak_width, oak_height), field_of_view)
    return fit_alignment(points, pixels, camera_matrix, (oak_width, oak_height))
