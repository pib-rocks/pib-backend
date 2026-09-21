from pathlib import Path

import pytest

pytest.importorskip(
    "depthai", reason="depthai not installed (tests/requirements-camera.txt)"
)

import depthai as dai

from ros_packages.camera.oak_d_lite.task_archives import (
    YUNET_MODEL_ID,
    create_archive,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
YUNET_BLOB = (
    REPO_ROOT / "models/face_detection_yunet_160x120/face_detection_yunet_160x120.blob"
)


def test_real_yunet_blob_builds_archive_from_its_tensor_metadata(tmp_path):
    blob = dai.OpenVINO.Blob(YUNET_BLOB)
    archive = create_archive(YUNET_MODEL_ID, YUNET_BLOB, tmp_path)

    assert isinstance(archive, dai.NNArchive)
    assert (archive.getInputWidth(), archive.getInputHeight()) == (160, 120)
    [head] = list(archive.getConfig().model.heads)
    assert head.parser == "YuNetParser"
    assert set(head.outputs) == set(blob.networkOutputs)
    assert head.metadata.confThreshold == 0.8
    assert head.metadata.iouThreshold == 0.3


def test_model_without_registered_parser_keeps_the_plain_network_path():
    assert create_archive("facemesh_192x192", "/unused.blob") is None
