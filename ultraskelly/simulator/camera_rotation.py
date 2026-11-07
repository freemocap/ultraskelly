"""The camera's rotation in each frame, measured from its own picture against a fixed reference.

A camera that only turns maps a reference picture onto a later one by the homography K R K^-1
whatever the scene's depth (K: the camera's intrinsic matrix, R: its turn). Every frame is registered
against a reference, never against the frame before, so its error stays its own instead of adding up
along the run. (Summed frame-to-frame shifts drifted by about 0.08 view degrees per servo degree of
tilt on 2026-11-06.) Per frame:

1. ORB features outside the masked boxes (people and faces move on their own), matched to the
   reference's by Hamming distance with a ratio test;
2. RANSAC over the matched rays, undistorted with the calibration, for the rotation alone (three
   degrees of freedom, two matches a sample), then refitted over its inliers;
3. ECC on the undistorted, masked pictures, started from that rotation's K R K^-1 and refined as a
   homography, then projected back onto the nearest rotation.

When the camera turns too far from its reference to overlap it (the waist turning the body), the frame
becomes a new keyframe, placed by its registration to the nearest keyframe, and later frames register
to whichever keyframe their predicted view is nearest: error grows per keyframe, not per frame.

Angles are degrees, as the recordings and simulator/plant.py use them: yaw (the view turning right is
positive), pitch (down is positive) and roll (the picture's content turning clockwise is positive). The
camera's orientation is R_y(yaw) R_x(-pitch) R_z(-roll) in the reference camera's axes (x right, y down,
z forward), so a ray from the reference reaches the frame as orientation^T times the ray.

Each frame is timed at its middle row's mid-exposure (`exposure_middle`): DepthAI stamps a frame as its
first row finishes exposing, and the rolling shutter reads the rows out over the sensor mode's readout.
OpenCV is imported when a camera or tracker is built, so importing this module needs only NumPy.
"""
import math
from dataclasses import dataclass, field

import numpy as np

RATIO = 0.85  # A match is kept when its distance is under this share of the second best's
INLIER_PIXELS = 2.0  # A match agrees with a rotation when it lands within this many pixels of where it says


def exposure_middle(stamp: float, exposure: float, readout: float) -> float:
    """When the frame's middle row was exposed, in seconds, from DepthAI's stamp (taken as the first row
    finished exposing), the exposure time and the sensor's readout time."""
    return stamp - exposure / 2.0 + readout / 2.0


def _turn(axis: int, degrees: float) -> np.ndarray:
    c, s = math.cos(math.radians(degrees)), math.sin(math.radians(degrees))
    i, j = [(1, 2), (2, 0), (0, 1)][axis]
    turn = np.eye(3)
    turn[i, i] = turn[j, j] = c
    turn[i, j], turn[j, i] = -s, s
    return turn


def orientation(yaw: float, pitch: float, roll: float) -> np.ndarray:
    """The camera's orientation for these view angles (degrees), in its reference's axes."""
    return _turn(1, yaw) @ _turn(0, -pitch) @ _turn(2, -roll)


def angles(rotation: np.ndarray) -> tuple[float, float, float]:
    """(yaw, pitch, roll) in degrees of a camera orientation: `orientation`'s inverse."""
    yaw = math.degrees(math.atan2(rotation[0, 2], rotation[2, 2]))
    pitch = math.degrees(math.asin(min(1.0, max(-1.0, rotation[1, 2]))))
    roll = -math.degrees(math.atan2(rotation[1, 0], rotation[1, 1]))
    return yaw, pitch, roll


def turned_by(rotation: np.ndarray) -> float:
    """Degrees a rotation turns, about whatever axis."""
    return math.degrees(math.acos(min(1.0, max(-1.0, (np.trace(rotation) - 1.0) / 2.0))))


def nearest_rotation(matrix: np.ndarray) -> np.ndarray:
    """The rotation closest to a 3x3 matrix (its polar factor)."""
    u, _, vt = np.linalg.svd(matrix)
    return u @ np.diag([1.0, 1.0, np.sign(np.linalg.det(u @ vt))]) @ vt


def fit_rotation(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """The rotation M that best takes rays `a` onto rays `b` (both (N, 3), unit), by least squares."""
    return nearest_rotation(b.T @ a)


def rotation_ransac(a: np.ndarray, b: np.ndarray, threshold: float, rng: np.random.Generator,
                    samples: int = 400) -> tuple[np.ndarray, np.ndarray]:
    """The rotation taking most of rays `a` onto `b` within `threshold` (radians), and which agree.

    Each sample is two matches, whose rotation follows by the TRIAD construction; the best sample's
    rotation is refitted over its inliers until they stop changing."""
    n = len(a)
    i, j = rng.integers(0, n, samples), rng.integers(0, n, samples)

    def frames(rays_i, rays_j):
        second = np.cross(rays_i, rays_j)
        length = np.linalg.norm(second, axis=1, keepdims=True)
        second = second / np.maximum(length, 1e-12)
        return np.stack([rays_i, second, np.cross(rays_i, second)], axis=2), length[:, 0]

    source, spread_a = frames(a[i], a[j])
    target, spread_b = frames(b[i], b[j])
    usable = (spread_a > 0.02) & (spread_b > 0.02)  # Two matches a degree or more apart pin the rotation
    if not usable.any():
        return np.eye(3), np.zeros(n, dtype=bool)
    turns = target[usable] @ source[usable].transpose(0, 2, 1)
    errors = np.linalg.norm(np.einsum("kij,nj->kni", turns, a) - b[None], axis=2)
    best = int(np.argmax((errors < threshold).sum(axis=1)))
    inliers = errors[best] < threshold
    turn = turns[best]
    for _ in range(5):
        if inliers.sum() < 3:
            break
        turn = fit_rotation(a[inliers], b[inliers])
        agreeing = np.linalg.norm(a @ turn.T - b, axis=1) < threshold
        if np.array_equal(agreeing, inliers):
            break
        inliers = agreeing
    return turn, inliers


def box_mask(size: tuple[int, int], boxes, margin: float = 0.15) -> np.ndarray:
    """A mask of the frame (width, height): 255 where features may be used, 0 inside each box (x1, y1, x2,
    y2 in pixels) widened by `margin` of its size on every side."""
    width, height = size
    mask = np.full((height, width), 255, dtype=np.uint8)
    for x1, y1, x2, y2 in boxes:
        grow_x, grow_y = (x2 - x1) * margin, (y2 - y1) * margin
        left, right = max(0, int(x1 - grow_x)), min(width, int(math.ceil(x2 + grow_x)))
        top, bottom = max(0, int(y1 - grow_y)), min(height, int(math.ceil(y2 + grow_y)))
        mask[top:bottom, left:right] = 0
    return mask


def self_mask(frames: np.ndarray, *, still_spread: float = 6.0, edge: float = 40.0, grow: int = 9) -> np.ndarray:
    """Where the robot itself is in the picture (the OAK sees the rim of the head in a corner): pixels that
    stay the same while the camera turns, on an edge, since only edges pull a registration. `frames` (N,
    height, width) must come from many different turns. Returns a mask, 0 there and 255 elsewhere."""
    import cv2

    stack = np.asarray(frames, dtype=np.float32)
    mean, spread = stack.mean(axis=0), stack.std(axis=0)
    strength = np.hypot(cv2.Sobel(mean, cv2.CV_32F, 1, 0), cv2.Sobel(mean, cv2.CV_32F, 0, 1))
    fixed = ((spread < still_spread) & (strength > edge)).astype(np.uint8)
    return np.where(cv2.dilate(fixed, np.ones((grow, grow), np.uint8)) > 0, 0, 255).astype(np.uint8)


@dataclass(frozen=True)
class Camera:
    """The camera's pinhole for its frames' size, and its lens distortion (OpenCV's coefficients)."""
    matrix: np.ndarray
    size: tuple[int, int]  # (width, height) in pixels
    distortion: np.ndarray = field(default_factory=lambda: np.zeros(5))

    @classmethod
    def from_field_of_view(cls, size: tuple[int, int], horizontal_degrees: float) -> "Camera":
        """A distortion-free pinhole seeing `horizontal_degrees` across, centred."""
        width, height = size
        focal = width / 2.0 / math.tan(math.radians(horizontal_degrees) / 2.0)
        return cls(np.array([[focal, 0.0, width / 2.0], [0.0, focal, height / 2.0], [0.0, 0.0, 1.0]]), size)

    def scaled(self, size: tuple[int, int]) -> "Camera":
        """The same camera for frames resized to `size`."""
        sx, sy = size[0] / self.size[0], size[1] / self.size[1]
        matrix = np.diag([sx, sy, 1.0]) @ self.matrix
        return Camera(matrix, size, self.distortion)

    def rays(self, points: np.ndarray) -> np.ndarray:
        """Unit rays (N, 3) through pixel `points` (N, 2) of the camera's own, distorted picture."""
        import cv2

        normal = cv2.undistortPoints(points.reshape(-1, 1, 2).astype(np.float64), self.matrix, self.distortion)
        rays = np.concatenate([normal.reshape(-1, 2), np.ones((len(points), 1))], axis=1)
        return rays / np.linalg.norm(rays, axis=1, keepdims=True)


class _Undistorter:
    """Remaps the camera's pictures to an undistorted pinhole with the same matrix."""

    def __init__(self, cv2, camera: Camera) -> None:
        self.cv2 = cv2
        self.maps = cv2.initUndistortRectifyMap(camera.matrix, camera.distortion, None, camera.matrix,
                                                camera.size, cv2.CV_32FC1)
        # Pixels the undistorted picture really has, a little inside its edge
        whole = self.image(np.full((camera.size[1], camera.size[0]), 255, dtype=np.uint8), nearest=True)
        self.valid = cv2.erode(whole, np.ones((5, 5), np.uint8))

    def image(self, picture: np.ndarray, nearest: bool = False) -> np.ndarray:
        cv2 = self.cv2
        return cv2.remap(picture, *self.maps, cv2.INTER_NEAREST if nearest else cv2.INTER_LINEAR,
                         borderMode=cv2.BORDER_CONSTANT, borderValue=0)

    def mask(self, mask: np.ndarray | None) -> np.ndarray:
        return self.valid if mask is None else self.cv2.bitwise_and(self.valid, self.image(mask, nearest=True))


@dataclass
class Keyframe:
    image: np.ndarray  # Undistorted
    mask: np.ndarray  # Undistorted: where ECC may compare
    rays: np.ndarray  # Of its features
    descriptors: np.ndarray | None
    orientation: np.ndarray  # In the first reference's axes


@dataclass
class Measurement:
    yaw: float  # Degrees; nan when the frame could not be registered
    pitch: float
    roll: float
    orientation: np.ndarray  # 3x3, in the first reference's axes
    matches: int  # Feature matches with the keyframe
    inliers: int  # Of them, agreeing with the rotation found
    correlation: float  # ECC's correlation coefficient after refinement; nan when not refined
    refinement: float  # Degrees ECC moved the features' rotation
    keyframe: int  # Which keyframe the frame was registered against
    ok: bool


class RotationTracker:
    """Measures each frame's camera rotation against keyframes, the first of them `reference`.

    `new_keyframe_beyond` (degrees of turn from the nearest keyframe) makes a well-registered frame a new
    keyframe; infinity keeps the one reference, for runs where the camera never turns out of it."""

    def __init__(self, camera: Camera, reference: np.ndarray, reference_mask: np.ndarray | None = None, *,
                 self_mask: np.ndarray | None = None, refine: bool = True, new_keyframe_beyond: float = math.inf,
                 min_inliers: int = 25, features: int = 2000, seed: int = 0) -> None:
        import cv2

        self.cv2, self.camera, self.refine = cv2, camera, refine
        self.self_mask = self_mask  # 0 where the robot itself is in the picture, in every frame (see `self_mask`)
        self.new_keyframe_beyond, self.min_inliers = new_keyframe_beyond, min_inliers
        self.orb = cv2.ORB_create(nfeatures=features)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.undistort = _Undistorter(cv2, camera)
        self.rng = np.random.default_rng(seed)
        self.threshold = INLIER_PIXELS / camera.matrix[0, 0]
        self.keyframes: list[Keyframe] = []
        self._add_keyframe(reference, reference_mask, np.eye(3))
        self.last = self.before = np.eye(3)

    def _features(self, image: np.ndarray, mask: np.ndarray | None) -> tuple[np.ndarray, np.ndarray | None]:
        points, descriptors = self.orb.detectAndCompute(image, mask)
        if descriptors is None:
            return np.zeros((0, 3)), None
        return self.camera.rays(np.float32([p.pt for p in points])), descriptors

    def _with_self(self, mask: np.ndarray | None) -> np.ndarray | None:
        if self.self_mask is None:
            return mask
        return self.self_mask if mask is None else np.minimum(mask, self.self_mask)

    def _add_keyframe(self, image: np.ndarray, mask: np.ndarray | None, rotation: np.ndarray) -> None:
        mask = self._with_self(mask)
        rays, descriptors = self._features(image, mask)
        self.keyframes.append(Keyframe(self.undistort.image(image), self.undistort.mask(mask), rays, descriptors, rotation))

    def _nearest(self, guess: np.ndarray) -> int:
        return min(range(len(self.keyframes)), key=lambda k: turned_by(self.keyframes[k].orientation.T @ guess))

    def measure(self, image: np.ndarray, mask: np.ndarray | None = None, guess: np.ndarray | None = None) -> Measurement:
        """The rotation of the camera that took `image` (grey, the camera's size); `mask` (0: ignore) keeps
        moving things out, and `guess` (an orientation) picks the keyframe, by default from the last two."""
        if guess is None:
            guess = self.last @ self.before.T @ self.last  # The last frame's turn, carried on
        index = self._nearest(guess)
        keyframe = self.keyframes[index]
        mask = self._with_self(mask)
        rays, descriptors = self._features(image, mask)
        failed = Measurement(math.nan, math.nan, math.nan, np.full((3, 3), math.nan), 0, 0, math.nan, math.nan, index, False)
        if descriptors is None or keyframe.descriptors is None or len(rays) < 3:
            return failed
        pairs = self.matcher.knnMatch(keyframe.descriptors, descriptors, k=2)
        good = [p[0] for p in pairs if len(p) == 2 and p[0].distance < RATIO * p[1].distance]
        if len(good) < self.min_inliers:
            failed.matches = len(good)
            return failed
        a = keyframe.rays[[m.queryIdx for m in good]]
        b = rays[[m.trainIdx for m in good]]
        turn, inliers = rotation_ransac(a, b, self.threshold, self.rng)  # Takes keyframe rays to this frame's
        count = int(inliers.sum())
        if count < self.min_inliers:
            failed.matches, failed.inliers = len(good), count
            return failed
        correlation, moved = math.nan, 0.0
        if self.refine:
            refined = self._ecc(keyframe, image, mask, turn)
            if refined is not None:
                correlation, better = refined
                moved = turned_by(better @ turn.T)
                turn = better
        rotation = keyframe.orientation @ turn.T
        self.before, self.last = self.last, rotation
        if turned_by(turn) > self.new_keyframe_beyond and count >= 3 * self.min_inliers:
            self._add_keyframe(image, mask, rotation)  # Already holds the self mask; adding it again changes nothing
        yaw, pitch, roll = angles(rotation)
        return Measurement(yaw, pitch, roll, rotation, len(good), count, correlation, moved, index, True)

    def _ecc(self, keyframe: Keyframe, image: np.ndarray, mask: np.ndarray | None,
             turn: np.ndarray) -> tuple[float, np.ndarray] | None:
        """ECC's refinement of `turn` (keyframe rays to this frame's) as a homography, back on a rotation."""
        cv2, matrix = self.cv2, self.camera.matrix
        warp = matrix @ turn @ np.linalg.inv(matrix)
        warp = (warp / warp[2, 2]).astype(np.float32)
        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 60, 1e-7)
        try:
            correlation, warp = cv2.findTransformECCWithMask(
                keyframe.image, self.undistort.image(image), keyframe.mask, self.undistort.mask(mask), warp,
                cv2.MOTION_HOMOGRAPHY, criteria, 5)
        except cv2.error:
            return None
        # Back onto a rotation by what the homography does to the picture, not by its matrix: its two
        # perspective terms are barely pinned by a 69-degree picture, yet carry yaw and pitch in K^-1 H K.
        grid = self._grid(keyframe)
        moved = np.c_[grid, np.ones(len(grid))] @ warp.astype(np.float64).T
        inverse = np.linalg.inv(matrix)
        a, b = np.c_[grid, np.ones(len(grid))] @ inverse.T, moved @ inverse.T
        a /= np.linalg.norm(a, axis=1, keepdims=True)
        b /= np.linalg.norm(b, axis=1, keepdims=True)
        return float(correlation), fit_rotation(a, b)

    @staticmethod
    def _grid(keyframe: Keyframe, step: int = 16) -> np.ndarray:
        """Pixels every `step` over where the keyframe was compared."""
        ys, xs = np.mgrid[step // 2:keyframe.mask.shape[0]:step, step // 2:keyframe.mask.shape[1]:step]
        keep = keyframe.mask[ys, xs] > 0
        return np.c_[xs[keep], ys[keep]].astype(np.float64)


def calibrate(camera: Camera, reference: np.ndarray, frames: list[np.ndarray], *, mask: np.ndarray | None = None,
              radial_terms: int = 2, features: int = 2000, seed: int = 0) -> tuple[Camera, dict]:
    """The pinhole (one focal length, the principal point) and radial distortion that best explain one
    place's features seen at many turns: rotation-only bundle adjustment over `frames`, still ones, each
    with its own rotation, starting from `camera`. Needs SciPy. Returns the camera and the fit's report.

    A turning camera fixes its own intrinsics (no pattern needed), the more so the more different ways it
    turns: pan's sweep pins the focal length and distortion together, tilt the principal point's height."""
    import cv2
    from scipy.optimize import least_squares
    from scipy.sparse import lil_matrix

    orb, matcher = cv2.ORB_create(nfeatures=features), cv2.BFMatcher(cv2.NORM_HAMMING)
    rng = np.random.default_rng(seed)
    points, descriptors = orb.detectAndCompute(reference, mask)
    origin = np.float32([p.pt for p in points])
    pairs, turns = [], []
    for frame in frames:
        found, described = orb.detectAndCompute(frame, mask)
        if described is None:
            continue
        matches = [p[0] for p in matcher.knnMatch(descriptors, described, k=2) if len(p) == 2 and p[0].distance < RATIO * p[1].distance]
        if len(matches) < 30:
            continue
        a, b = origin[[m.queryIdx for m in matches]], np.float32([found[m.trainIdx].pt for m in matches])
        turn, inliers = rotation_ransac(camera.rays(a), camera.rays(b), 8.0 / camera.matrix[0, 0], rng)
        if inliers.sum() >= 30:
            pairs.append((a[inliers].astype(np.float64), b[inliers].astype(np.float64)))
            turns.append(cv2.Rodrigues(turn)[0].ravel())
    shared = 3 + radial_terms

    def unpack(x):
        f, cx, cy = x[:3]
        distortion = np.zeros(5)
        distortion[[0, 1, 4][:radial_terms]] = x[3:shared]
        return np.array([[f, 0.0, cx], [0.0, f, cy], [0.0, 0.0, 1.0]]), distortion

    def residuals(x):
        matrix, distortion = unpack(x)
        out = []
        for k, (a, b) in enumerate(pairs):
            normal = cv2.undistortPoints(a.reshape(-1, 1, 2), matrix, distortion).reshape(-1, 2)
            rays = np.c_[normal, np.ones(len(normal))]
            seen, _ = cv2.projectPoints(rays, x[shared + 3 * k:shared + 3 * k + 3], np.zeros(3), matrix, distortion)
            out.append((seen.reshape(-1, 2) - b).ravel())
        return np.concatenate(out)

    start = np.r_[camera.matrix[0, 0], camera.matrix[0, 2], camera.matrix[1, 2], np.zeros(radial_terms), np.concatenate(turns)]
    sparsity = lil_matrix((sum(2 * len(a) for a, _ in pairs), len(start)), dtype=int)
    row = 0
    for k, (a, _) in enumerate(pairs):
        sparsity[row:row + 2 * len(a), :shared] = 1
        sparsity[row:row + 2 * len(a), shared + 3 * k:shared + 3 * k + 3] = 1
        row += 2 * len(a)
    fit = least_squares(residuals, start, jac_sparsity=sparsity, loss="soft_l1", f_scale=1.0, x_scale="jac")
    matrix, distortion = unpack(fit.x)
    error = fit.fun.reshape(-1, 2)
    report = {"frames": len(pairs), "matches": len(error), "focal": float(matrix[0, 0]),
              "principal_point": (float(matrix[0, 2]), float(matrix[1, 2])), "distortion": distortion.tolist(),
              "horizontal_degrees": math.degrees(2 * math.atan(camera.size[0] / 2 / matrix[0, 0])),
              "median_error_px": float(np.median(np.linalg.norm(error, axis=1))), "cost": float(fit.cost)}
    return Camera(matrix, camera.size, distortion), report


class NpyFrames:
    """Grey frames stacked in a .npy file (N, height, width), read without loading the rest."""

    def __init__(self, path) -> None:
        self.path = str(path)
        self.frames = np.load(self.path, mmap_mode="r")

    def __len__(self) -> int:
        return len(self.frames)

    def read(self, index: int) -> np.ndarray:
        return np.ascontiguousarray(self.frames[index])

    def __getstate__(self):
        return {"path": self.path}

    def __setstate__(self, state) -> None:
        self.__init__(state["path"])


class AviFrames:
    """Grey frames of a Motion JPEG AVI, decoded in order and sought when a read skips ahead or back."""

    def __init__(self, path) -> None:
        self.path, self.video, self.next = str(path), None, 0

    def read(self, index: int) -> np.ndarray:
        import cv2

        if self.video is None:
            self.video = cv2.VideoCapture(self.path)
        if index != self.next:
            self.video.set(cv2.CAP_PROP_POS_FRAMES, index)
        ok, picture = self.video.read()
        if not ok:
            raise IndexError(f"{self.path} has no frame {index}")
        self.next = index + 1
        return cv2.cvtColor(picture, cv2.COLOR_BGR2GRAY)

    def __getstate__(self):
        return {"path": self.path}

    def __setstate__(self, state) -> None:
        self.__init__(state["path"])


FIELDS = ("yaw", "pitch", "roll", "matches", "inliers", "correlation", "refinement", "keyframe", "ok")


def _measure_job(job) -> list[tuple]:
    """A worker's share: one tracker, fixed keyframes, its frames in order. Each frame's RANSAC is seeded
    by its index, so the result does not depend on how the frames were shared out."""
    source, camera, reference, self_mask, keyframes, indices, guesses, masks, refine = job
    tracker = RotationTracker(camera, reference, self_mask=self_mask, refine=refine)
    if keyframes is not None:
        tracker.keyframes = keyframes
    rows = []
    for k, index in enumerate(indices):
        tracker.rng = np.random.default_rng(index)
        found = tracker.measure(source.read(index), None if masks is None else masks(index),
                                None if guesses is None else guesses[k])
        rows.append((index,) + tuple(getattr(found, name) for name in FIELDS))
    return rows


def measure_frames(source, camera: Camera, reference: np.ndarray, indices, *, self_mask: np.ndarray | None = None,
                   keyframes: list[Keyframe] | None = None, guesses: list[np.ndarray] | None = None, masks=None,
                   refine: bool = True, workers: int = 1) -> dict[str, np.ndarray]:
    """Every frame in `indices` of `source` (anything with `read(index)` giving a grey frame) measured
    against `reference` (or fixed `keyframes`), shared over `workers` processes. `masks(index)` gives a
    frame's mask; `guesses` an orientation per frame to pick its keyframe. Returns each FIELDS column and
    "index", in the order of `indices`."""
    indices = list(indices)
    chunks = max(1, min(len(indices), workers * 4))
    bounds = np.linspace(0, len(indices), chunks + 1).astype(int)
    jobs = [(source, camera, reference, self_mask, keyframes, indices[a:b], None if guesses is None else guesses[a:b], masks,
             refine)
            for a, b in zip(bounds[:-1], bounds[1:]) if b > a]
    if workers <= 1:
        results = [_measure_job(job) for job in jobs]
    else:
        import multiprocessing

        with multiprocessing.get_context("spawn").Pool(workers) as pool:
            results = pool.map(_measure_job, jobs, chunksize=1)
    rows = [row for part in results for row in part]
    columns = list(zip(*rows)) if rows else [()] * (len(FIELDS) + 1)
    table = {"index": np.array(columns[0], dtype=int)}
    for name, column in zip(FIELDS, columns[1:]):
        table[name] = np.array(column, dtype=bool if name == "ok" else float)
    return table
