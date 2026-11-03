"""The Luxonis Model Zoo models the OAK's own chip runs, and where the Pi keeps them.

Each is the zoo's build for RVC2, the OAK-1's Myriad X. They download once, with
`python -m scripts.fetch_oak_models`, into MODEL_CACHE; after that the bot loads them from
there without the internet. With a model missing, the OAK runs none of them and says why in
the log and in robot_state.oak.models_error, naming that command; the bot still tracks.
"""
import os
from pathlib import Path

MODEL_CACHE = Path.home() / "ultraskelly-models" / "oak-zoo"
PLATFORM = "RVC2"
MODELS = {
    "face_detection": "luxonis/yunet:320x240",
    "head_pose": "luxonis/head-pose-estimation:60x60",
    "gaze": "luxonis/gaze-estimation-adas:60x60",
    "face_landmarks": "luxonis/mediapipe-face-landmarker:192x192",
    "palm_detection": "luxonis/mediapipe-palm-detection:128x128",
    "hand_landmarks": "luxonis/mediapipe-hand-landmarker:224x224",
}


def without_telemetry() -> None:
    """Keep depthai from uploading usage events to Luxonis, as it does by default from 3.x
    (queued in ~/.cache/depthai/telemetry), unless DEPTHAI_TELEMETRY is already set. Call it
    before an OAK is opened: depthai reads the setting when it connects to a device."""
    os.environ.setdefault("DEPTHAI_TELEMETRY", "0")


def load_archive(name: str, cache: Path = MODEL_CACHE):
    """The NNArchive for model `name`, from the cache (or the zoo, when online and not cached)."""
    import depthai as dai

    try:
        path = dai.getModelFromZoo(
            dai.NNModelDescription(MODELS[name], platform=PLATFORM), useCached=True, cacheDirectory=str(cache)
        )
    except RuntimeError as error:
        raise RuntimeError(
            f"OAK model {MODELS[name]} is not in {cache}; fetch it with `python -m scripts.fetch_oak_models` ({error})"
        ) from error
    return dai.NNArchive(str(path))
