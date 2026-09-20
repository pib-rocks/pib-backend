import os
from pathlib import Path
import types

import depthai as dai

from ros_packages.camera.oak_d_lite.imitation import _world_values
from ros_packages.camera.oak_d_lite.imitation_archive import (
    LANDMARK_OUTPUTS,
    LANDMARK_PARSERS,
    PALM_OUTPUTS,
    create_landmark_archive,
    create_palm_archive,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
PALM_BLOB = REPO_ROOT / "models/palm_detection_sh4/palm_detection_sh4.blob"
LANDMARK_BLOB = REPO_ROOT / "models/hand_landmark_full_sh4/hand_landmark_full_sh4.blob"


def _heads(archive):
    return list(archive.getConfig().model.heads)


def test_real_palm_blob_loads_as_128_archive_with_four_shaves(tmp_path):
    archive = create_palm_archive(PALM_BLOB, tmp_path)

    assert isinstance(archive, dai.NNArchive)
    assert (archive.getInputWidth(), archive.getInputHeight()) == (128, 128)
    assert dai.OpenVINO.Blob(PALM_BLOB).numShaves == 4
    heads = _heads(archive)
    assert [head.parser for head in heads] == ["MPPalmDetectionParser"]
    assert list(heads[0].outputs) == list(PALM_OUTPUTS)
    assert heads[0].metadata.confThreshold == 0.5
    assert heads[0].metadata.extraParams["scale"] == 128


def test_real_landmark_blob_has_stable_four_head_order(tmp_path):
    archive = create_landmark_archive(LANDMARK_BLOB, tmp_path)

    assert isinstance(archive, dai.NNArchive)
    assert (archive.getInputWidth(), archive.getInputHeight()) == (224, 224)
    assert dai.OpenVINO.Blob(LANDMARK_BLOB).numShaves == 4
    heads = _heads(archive)
    assert tuple(head.parser for head in heads) == LANDMARK_PARSERS
    assert tuple(head.outputs[0] for head in heads) == LANDMARK_OUTPUTS


def test_archive_cache_reuses_the_sha_keyed_file(tmp_path):
    first = create_palm_archive(PALM_BLOB, tmp_path)
    [cache_path] = tmp_path.glob("*.tar.xz")
    first_stat = cache_path.stat()

    second = create_palm_archive(PALM_BLOB, tmp_path)
    second_stat = cache_path.stat()

    assert first.getInputSize() == second.getInputSize()
    assert cache_path.name == (
        "4d45ecfddc68d8365fef3cb633d78b916612f25f7a48898f90f8a47d4b151aa5" ".tar.xz"
    )
    assert (first_stat.st_ino, first_stat.st_mtime_ns) == (
        second_stat.st_ino,
        second_stat.st_mtime_ns,
    )
    assert os.stat(cache_path).st_mode & 0o777 == 0o600


def test_world_regression_values_are_not_clipped(tmp_path):
    archive = create_landmark_archive(LANDMARK_BLOB, tmp_path)
    world_head = _heads(archive)[3]
    assert world_head.parser == "RegressionParser"

    raw = [-2.5, 0.25, 3.75] * 21
    message = types.SimpleNamespace(
        predictions=[types.SimpleNamespace(prediction=value) for value in raw]
    )

    assert _world_values(message) == raw
