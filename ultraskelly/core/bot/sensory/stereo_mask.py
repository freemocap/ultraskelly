"""Where both eyes see the room: the view mask, in the rectified left eye's pixels.

The skull blocks part of each eye's view: the nose bridge on its inner side, and since
the rebuild the eye-socket rims. A left-eye pixel blocked that way has no match in the
right eye at any disparity, and the matcher reports nonsense there. The mask keeps depth
only where both eyes see the room, and the matcher skips the rest. It belongs to one
mounting of the camera: `python -m scripts.zed_view_mask` proposes it from live pairs,
with an image to check, and it is redone whenever the ZED is re-seated.

Proposal: a left-eye pixel whose 9x9 patch differs from every right-eye patch on its row,
over disparities 0 to 128, by more than 12 grey levels on average has no match. Blocked
regions are those that touch the image's sides, filled out to the side they touch.
"""
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ultraskelly.core.bot.sensory.stereo_rectification import CALIBRATION_FOLDER

SEARCH = 128  # Disparities tried: past any matcher's range, so only blocked pixels miss
BLOCK = 9
NO_MATCH = 12.0  # Mean absolute grey-level difference of the best patch above which a pixel has no match
MIN_REGION = 0.01  # Blocked regions smaller than this share of the image are noise
SIDE = 0.1  # A region this close to a side, as a share of the width, counts as touching it


def best_match_cost(left: np.ndarray, right: np.ndarray, search: int = SEARCH, block: int = BLOCK) -> np.ndarray:
    """For each left-eye pixel, the smallest mean absolute difference between its patch and a
    right-eye patch on its row, over disparities 0 to `search`. Rectified grey images in."""
    import cv2

    left = left.astype(np.float32)
    right = right.astype(np.float32)
    height, width = left.shape
    best = np.full((height, width), np.inf, np.float32)
    shifted = np.empty_like(right)
    for disparity in range(search + 1):
        shifted[:, disparity:] = right[:, :width - disparity]
        shifted[:, :disparity] = 1e4  # Beyond the right eye's image: never a match
        np.minimum(best, cv2.boxFilter(np.abs(left - shifted), -1, (block, block)), out=best)
    return best


def blocked_regions(no_match: np.ndarray) -> np.ndarray:
    """The view mask (True = both eyes see the room) from per-pixel 'no match' flags: regions
    reaching a side are filled out to that side, row by row; smaller or inner ones are noise."""
    import cv2

    height, width = no_match.shape
    cleaned = cv2.morphologyEx(no_match.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(cleaned, connectivity=8)
    blocked = np.zeros((height, width), bool)
    for region in range(1, count):
        left, _, region_width, _, area = stats[region]
        if area < MIN_REGION * height * width:
            continue
        rows, columns = np.nonzero(labels == region)
        if left + region_width >= width * (1 - SIDE):  # Reaches the right side: fill rightwards
            for row in np.unique(rows):
                blocked[row, columns[rows == row].min():] = True
        if left <= width * SIDE:  # Reaches the left side: fill leftwards
            for row in np.unique(rows):
                blocked[row, :columns[rows == row].max() + 1] = True
    return ~blocked


def propose_view_mask(pairs: list[tuple[np.ndarray, np.ndarray]]) -> np.ndarray:
    """A view mask from rectified grey pairs of the camera as mounted (more pairs, steadier)."""
    costs = np.median(np.stack([best_match_cost(left, right) for left, right in pairs]), axis=0)
    return blocked_regions(costs > NO_MATCH)


@dataclass(frozen=True)
class ViewMask:
    """Which rectified left-eye pixels can have depth, and the window around them a matcher needs."""

    valid: np.ndarray  # bool (height, width)

    @classmethod
    def load(cls, path: Path, size: tuple[int, int]) -> "ViewMask":
        """Read a mask image (white = valid), scaled to (width, height) if it was made at another size."""
        import cv2

        image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise FileNotFoundError(f"Cannot read the view mask {path}")
        if (image.shape[1], image.shape[0]) != size:
            image = cv2.resize(image, size, interpolation=cv2.INTER_NEAREST)
        return cls(valid=image > 127)

    def save(self, path: Path) -> None:
        import cv2

        path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(path), self.valid.astype(np.uint8) * 255):
            raise OSError(f"Could not write {path}")

    def window(self, search_range: int) -> tuple[slice, slice]:
        """Rows and columns of both rectified eyes a matcher needs to cover every valid pixel: the
        valid pixels' bounding box, widened on the left by the search range, where the right eye's
        matches lie."""
        rows = np.flatnonzero(self.valid.any(axis=1))
        columns = np.flatnonzero(self.valid.any(axis=0))
        if rows.size == 0:
            return slice(0, 0), slice(0, 0)
        return (slice(int(rows[0]), int(rows[-1]) + 1),
                slice(max(0, int(columns[0]) - int(search_range)), int(columns[-1]) + 1))


def view_mask_file(serial: str, folder: Path = CALIBRATION_FOLDER) -> Path:
    """Where the view mask of the ZED with this serial number is kept, beside its calibration."""
    return folder / f"SN{serial}-view-mask.png"
