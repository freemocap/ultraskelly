"""The OAK model check reads depthai's model cache alone, offline and without depthai, as
`scripts/setup-pi.sh --check` runs it."""
import os
import subprocess
import sys
from pathlib import Path

from scripts.fetch_oak_models import missing_models
from ultraskelly.core.bot.sensory.oak_models import MODELS, PLATFORM

REPO = Path(__file__).parents[1]


def cache_model(cache, slug, *, platform=PLATFORM, archive=b"archive"):
    """One model as depthai caches it: a folder with model.yaml, metadata.yaml and the archive."""
    folder = cache / slug.replace("/", "-").replace(":", "-")
    folder.mkdir(parents=True)
    (folder / "model.yaml").write_text(f"model: {slug}\nplatform: {platform}")  # depthai writes no final newline
    name = slug.split("/")[1].split(":")[0] + ".rvc2.tar.xz"
    (folder / "metadata.yaml").write_text(f"hash: caab36\nmodel_id: aim_8TPr\ndownloaded_files:\n  - {name}\n")
    if archive is not None:
        (folder / name).write_bytes(archive)


def test_every_model_in_the_cache_counts_whatever_else_is_there(tmp_path):
    for slug in MODELS.values():
        cache_model(tmp_path, slug)
    cache_model(tmp_path, "luxonis/yolov8-nano-pose-estimation:coco-512x288")  # Benchmarks leave others
    (tmp_path / ".locks").mkdir()

    assert missing_models(tmp_path) == {}


def test_a_model_without_its_archive_or_for_another_chip_is_missing(tmp_path):
    names = list(MODELS)
    cache_model(tmp_path, MODELS[names[0]], archive=None)  # A download cut short
    cache_model(tmp_path, MODELS[names[1]], archive=b"")
    cache_model(tmp_path, MODELS[names[2]], platform="RVC4")  # The OAK 4's build
    for name in names[3:]:
        cache_model(tmp_path, MODELS[name])

    assert missing_models(tmp_path) == {name: MODELS[name] for name in names[:3]}


def test_without_a_cache_every_model_is_missing(tmp_path):
    assert missing_models(tmp_path / "never made") == MODELS


def check(home, blocker):
    """`python -m scripts.fetch_oak_models --check` for a user whose home is `home`, unable to import depthai."""
    environment = dict(os.environ, HOME=str(home), USERPROFILE=str(home),
                       PYTHONPATH=os.pathsep.join([str(blocker), str(REPO)]))
    return subprocess.run([sys.executable, "-m", "scripts.fetch_oak_models", "--check"], cwd=REPO,
                          env=environment, capture_output=True, text=True, timeout=60)


def test_the_check_runs_offline_without_depthai_and_names_what_is_missing(tmp_path):
    blocker = tmp_path / "blocker"
    blocker.mkdir()
    (blocker / "depthai.py").write_text('raise ImportError("the check must not need depthai")\n')
    home = tmp_path / "home"

    before = check(home, blocker)
    for slug in MODELS.values():
        cache_model(home / "ultraskelly-models" / "oak-zoo", slug)
    after = check(home, blocker)

    assert before.returncode == 1 and before.stderr == ""
    assert before.stdout.startswith("not in ~/ultraskelly-models/oak-zoo: face_detection (luxonis/yunet:320x240), ")
    assert after.returncode == 0 and after.stdout.startswith(f"all {len(MODELS)} in ~/ultraskelly-models/oak-zoo")


def test_setup_runs_the_check_and_the_fetch():
    script = (REPO / "scripts" / "setup-pi.sh").read_text(encoding="utf-8")

    assert 'setting "OAK models" check_oak_models fix_oak_models' in script
    assert "-m scripts.fetch_oak_models --check" in script and "-m scripts.fetch_oak_models)" in script
