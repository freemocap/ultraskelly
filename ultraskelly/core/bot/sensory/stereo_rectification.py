"""A ZED's factory calibration, and the maps that rectify its image pairs.

Stereolabs publishes each camera's calibration by serial number at
https://calib.stereolabs.com/?SN=<serial>; copies live in `ultraskelly/calibration/zed/`.
Per resolution it gives each eye's pinhole intrinsics and distortion (k1, k2, p1, p2, k3,
OpenCV's order). For the pair it gives the baseline and two small offsets in millimetres
(TY, TZ), and a rotation in radians as a Rodrigues vector: RX (tilt), CV (convergence)
and RZ (roll). Both go to `cv2.stereoRectify` as written, as in Stereolabs' MIT-licensed
OpenCV sample (zed-open-capture, examples/include/calibration.hpp).

Rectified images share one pinhole camera and matching points lie on the same row. A
point's disparity d, its column in the left image minus its column in the right image,
gives its depth Z = focal_length * baseline / d.
"""
from configparser import ConfigParser
from dataclasses import dataclass
from pathlib import Path

import numpy as np

CALIBRATION_FOLDER = Path(__file__).parents[3] / "calibration" / "zed"
USB_DEVICES = Path("/sys/bus/usb/devices")
STEREOLABS_VENDOR = "2b03"
# The calibration file's section suffix for each side-by-side mode's eye width.
SECTION_SUFFIX = {2208: "2K", 1920: "FHD", 1280: "HD", 672: "VGA"}


@dataclass(frozen=True)
class EyeCalibration:
    """One eye's pinhole intrinsics and distortion, in pixels of its image."""

    camera_matrix: np.ndarray  # 3x3
    distortion: np.ndarray  # k1, k2, p1, p2, k3


@dataclass(frozen=True)
class StereoCalibration:
    """Both eyes of one camera at one resolution, and where the right eye sits."""

    left: EyeCalibration
    right: EyeCalibration
    rotation: np.ndarray  # Rodrigues vector (RX, CV, RZ), radians
    translation: np.ndarray  # (baseline, TY, TZ), millimetres
    image_size: tuple[int, int]  # One eye's (width, height)


@dataclass(frozen=True)
class StereoRectification:
    """The pinhole camera both rectified eyes share. Coordinates follow OpenCV's camera
    convention: x right, y down, z forward; the right eye sits `baseline` metres along +x.
    `left_rotation` turns directions in the left eye's own camera frame into the rectified
    frame (and its transpose turns them back)."""

    width: int
    height: int
    focal_length: float  # Pixels, the same horizontally and vertically
    cx: float
    cy: float
    baseline: float  # Metres
    left_rotation: tuple[tuple[float, ...], ...]  # 3x3, row-major

    def depth(self, disparity: np.ndarray) -> np.ndarray:
        """Depth in metres for disparities in pixels; zero or negative disparity gives inf."""
        disparity = np.asarray(disparity, dtype=np.float64)
        with np.errstate(divide="ignore"):
            return np.where(disparity > 0, self.focal_length * self.baseline / disparity, np.inf)


def read_zed_calibration(text: str, image_size: tuple[int, int]) -> StereoCalibration:
    """Parse a Stereolabs calibration file for eyes of `image_size` (width, height)."""
    width, height = image_size
    if width not in SECTION_SUFFIX:
        raise ValueError(f"No ZED resolution has {width}-pixel-wide eyes")
    suffix = SECTION_SUFFIX[width]
    parser = ConfigParser()
    parser.read_string(text)

    def eye(side: str) -> EyeCalibration:
        section = parser[f"{side}_CAM_{suffix}"]
        value = section.getfloat
        return EyeCalibration(
            camera_matrix=np.array([[value("fx"), 0, value("cx")], [0, value("fy"), value("cy")], [0, 0, 1]]),
            distortion=np.array([value(name, 0.0) for name in ("k1", "k2", "p1", "p2", "k3")]),
        )

    stereo = parser["STEREO"]
    return StereoCalibration(
        left=eye("LEFT"),
        right=eye("RIGHT"),
        rotation=np.array([stereo.getfloat(f"{axis}_{suffix}", 0.0) for axis in ("RX", "CV", "RZ")]),
        translation=np.array([stereo.getfloat("Baseline"), stereo.getfloat("TY", 0.0), stereo.getfloat("TZ", 0.0)]),
        image_size=(width, height),
    )


def find_zed_serial(devices: Path = USB_DEVICES) -> str | None:
    """The serial number a connected ZED reports over USB: its HID interface carries it."""
    for device in sorted(devices.iterdir()) if devices.is_dir() else ():
        try:
            if (device / "idVendor").read_text().strip() == STEREOLABS_VENDOR:
                serial = (device / "serial").read_text().strip()
                if serial:
                    return serial
        except OSError:
            continue
    return None


def calibration_file(serial: str, folder: Path = CALIBRATION_FOLDER) -> Path:
    """Where the factory calibration of the ZED with this serial number is kept."""
    path = folder / f"SN{serial}.conf"
    if not path.is_file():
        raise FileNotFoundError(
            f"No factory calibration for ZED {serial}: save https://calib.stereolabs.com/?SN={serial} as {path}"
        )
    return path


def zed_rectifier(eye_size: tuple[int, int], calibration: Path | None = None, scale: float = 1.0) -> "StereoRectifier":
    """Rectification for the connected ZED's eyes of `eye_size` (width, height): from the
    `calibration` file, or by default the factory file for the serial number it reports.
    With `scale` below 1 the rectified images come out that much smaller."""
    if calibration is None:
        serial = find_zed_serial()
        if serial is None:
            raise RuntimeError("The ZED reports no serial number, so its calibration file is unknown")
        calibration = calibration_file(serial)
    return StereoRectifier(read_zed_calibration(calibration.read_text(), eye_size), scale)


def scaled_projection(projection: np.ndarray, scale: float) -> np.ndarray:
    """A projection matrix for images `scale` times the size, pixel centres kept: a pixel of the
    smaller image covers 1 / scale pixels of the larger one, and its centre is at their middle."""
    scaled = projection.copy()
    scaled[:2] *= scale
    scaled[:2, 2] += 0.5 * (scale - 1)
    return scaled


class StereoRectifier:
    """Rectifies image pairs with precomputed OpenCV remap tables. With `scale` below 1 it makes
    smaller rectified images directly, from the full-size camera images, in one remap."""

    def __init__(self, calibration: StereoCalibration, scale: float = 1.0) -> None:
        import cv2

        left, right = calibration.left, calibration.right
        rotation, _ = cv2.Rodrigues(calibration.rotation)
        left_rotation, right_rotation, left_projection, right_projection, *_ = cv2.stereoRectify(
            left.camera_matrix, left.distortion, right.camera_matrix, right.distortion,
            calibration.image_size, rotation, calibration.translation.reshape(3, 1),
            flags=cv2.CALIB_ZERO_DISPARITY, alpha=0, newImageSize=calibration.image_size,
        )
        left_projection = scaled_projection(left_projection, scale)
        right_projection = scaled_projection(right_projection, scale)
        width, height = calibration.image_size
        size = (round(width * scale), round(height * scale))
        # Per eye, two forms of the same tables: OpenCV 5.0's remap on the Pi 5 is fastest with
        # float tables for colour images (2.4 against 4.3 ms at 672x376, one thread) and with
        # fixed-point ones for grey images (1.3 against 1.8 ms).
        self.maps = [
            cv2.initUndistortRectifyMap(
                eye.camera_matrix, eye.distortion, eye_rotation, projection, size, cv2.CV_32FC1
            )
            for eye, eye_rotation, projection in (
                (left, left_rotation, left_projection), (right, right_rotation, right_projection),
            )
        ]
        self.fixed_maps = [cv2.convertMaps(map_x, map_y, cv2.CV_16SC2) for map_x, map_y in self.maps]
        # Per eye: camera matrix, distortion, rectifying rotation, projection, for rectify_points.
        self.eyes = [
            (left.camera_matrix, left.distortion, left_rotation, left_projection),
            (right.camera_matrix, right.distortion, right_rotation, right_projection),
        ]
        self.rectification = StereoRectification(
            width=size[0],
            height=size[1],
            focal_length=float(left_projection[0, 0]),
            cx=float(left_projection[0, 2]),
            cy=float(left_projection[1, 2]),
            baseline=float(np.linalg.norm(calibration.translation)) / 1000.0,
            left_rotation=tuple(tuple(float(value) for value in row) for row in left_rotation),
        )
        self._remap = lambda image, maps: cv2.remap(image, maps[0], maps[1], cv2.INTER_LINEAR)
        self._undistort_points = cv2.undistortPoints

    def rectify(self, left: np.ndarray, right: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Both eyes' images rectified: a scene point appears on the same row in each."""
        return self.rectify_eye(0, left), self.rectify_eye(1, right)

    def rectify_eye(self, eye: int, image: np.ndarray) -> np.ndarray:
        """One image, colour or grey, from eye 0 (left) or 1 (right), rectified."""
        return self._remap(image, self.maps[eye] if image.ndim == 3 else self.fixed_maps[eye])

    def rectify_points(self, eye: int, points: np.ndarray) -> np.ndarray:
        """Where (N, 2) pixel positions in eye 0 (left) or 1 (right) of the camera's
        own images land in the rectified images."""
        camera_matrix, distortion, rotation, projection = self.eyes[eye]
        pixels = np.asarray(points, dtype=np.float64).reshape(-1, 1, 2)
        return self._undistort_points(pixels, camera_matrix, distortion, R=rotation, P=projection).reshape(-1, 2)
