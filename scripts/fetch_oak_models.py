"""Download the Model Zoo models the OAK runs, once, so the bot can start without the internet.

    .venv/bin/python -m scripts.fetch_oak_models

Needs no OAK and no robot lock. Prints each model's archive and its SHA-256.
"""
import hashlib
from pathlib import Path

from ultraskelly.core.bot.sensory.oak_models import MODEL_CACHE, MODELS, PLATFORM


def main() -> None:
    import depthai as dai

    for name, slug in MODELS.items():
        path = Path(dai.getModelFromZoo(dai.NNModelDescription(slug, platform=PLATFORM), useCached=True,
                                        cacheDirectory=str(MODEL_CACHE)))
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        print(f"{name}: {slug} -> {path.relative_to(MODEL_CACHE)} sha256 {digest}")


if __name__ == "__main__":
    main()
