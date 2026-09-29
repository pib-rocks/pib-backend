from pathlib import Path

import pytest

pytest.importorskip(
    "depthai", reason="depthai not installed (tests/requirements-camera.txt)"
)

import depthai as dai

from ros_packages.camera.oak_d_lite.task_archives import (
    YOLOV6N_COCO_LABELS,
    YOLOV6N_MODEL_ID,
    YUNET_MODEL_ID,
    create_archive,
    labels_for_model,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
YUNET_BLOB = (
    REPO_ROOT / "models/face_detection_yunet_160x120/face_detection_yunet_160x120.blob"
)
YOLOV6N_BLOB = REPO_ROOT / "models/yolov6n_coco_640x640/yolov6n_coco_640x640.blob"


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


def test_real_yolov6n_blob_builds_archive_from_its_tensor_metadata(tmp_path):
    blob = dai.OpenVINO.Blob(YOLOV6N_BLOB)
    archive = create_archive(YOLOV6N_MODEL_ID, YOLOV6N_BLOB, tmp_path)

    assert isinstance(archive, dai.NNArchive)
    assert (archive.getInputWidth(), archive.getInputHeight()) == (640, 640)
    [model_input] = list(archive.getConfig().model.inputs)
    [blob_input_name] = tuple(blob.networkInputs)
    assert model_input.name == blob_input_name
    archive_outputs = {
        output.name: list(output.shape) for output in archive.getConfig().model.outputs
    }
    blob_outputs = {
        name: list(tensor.dims) for name, tensor in blob.networkOutputs.items()
    }
    assert archive_outputs == blob_outputs
    [head] = list(archive.getConfig().model.heads)
    assert head.parser == "YOLOExtendedParser"
    assert set(head.outputs) == set(blob.networkOutputs)
    assert head.metadata.nClasses == 80
    assert head.metadata.subtype == "yolov6"


def test_labels_for_model_returns_eighty_coco_names_in_model_order():
    labels = labels_for_model(YOLOV6N_MODEL_ID)
    assert labels is YOLOV6N_COCO_LABELS
    assert len(labels) == 80
    assert labels[0] == "person"
    assert labels[2] == "car"
    assert labels[-1] == "toothbrush"
