"""Download the Model Zoo models the OAK runs, once, so the bot can start without the internet.

    .venv/bin/python -m scripts.fetch_oak_models           download what is missing
    .venv/bin/python -m scripts.fetch_oak_models --check   list what is missing, offline

Needs no OAK and no robot lock. It prints each model's archive and its SHA-256; run again,
it finds them in the cache and downloads nothing. `--check` reads only the cache and exits
1 if a model is missing. `scripts/setup-pi.sh` runs both.
"""
import argparse
import hashlib
from pathlib import Path

from ultraskelly.core.bot.sensory.oak_models import MODEL_CACHE, MODELS, PLATFORM, missing_models, without_telemetry


def shown(path: Path) -> str:
    """`path`, with the home folder as ~."""
    try:
        return f"~/{path.relative_to(Path.home()).as_posix()}"
    except ValueError:
        return str(path)


def main() -> None:
    parser = argparse.ArgumentParser(description="Download the OAK's Model Zoo models once, or check that they are here.")
    parser.add_argument("--check", action="store_true",
                        help="list the models not downloaded yet, without the internet; exit 1 if there are any")
    if parser.parse_args().check:
        missing = missing_models()
        if missing:
            print(f"not in {shown(MODEL_CACHE)}: " + ", ".join(f"{name} ({slug})" for name, slug in missing.items()))
            raise SystemExit(1)
        print(f"all {len(MODELS)} in {shown(MODEL_CACHE)}: {', '.join(MODELS)}")
        return

    without_telemetry()
    import depthai as dai

    for name, slug in MODELS.items():
        path = Path(dai.getModelFromZoo(dai.NNModelDescription(slug, platform=PLATFORM), useCached=True,
                                        cacheDirectory=str(MODEL_CACHE)))
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        print(f"{name}: {slug} -> {path.relative_to(MODEL_CACHE)} sha256 {digest}")


if __name__ == "__main__":
    main()
