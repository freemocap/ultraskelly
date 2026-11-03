"""The Luxonis Model Zoo models the OAK's own chip runs, and where the Pi keeps them.

Each is the zoo's build for RVC2, the OAK-1's Myriad X. They download once, with
`python -m scripts.fetch_oak_models`, into MODEL_CACHE; after that the bot loads them from
there without the internet. A missing model fails loudly at startup, naming that command.
"""
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
