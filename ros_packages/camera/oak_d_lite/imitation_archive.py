"""Build DepthAI v1 archives around the vendored imitation blobs."""

import hashlib
import io
import json
import os
from pathlib import Path
import tarfile
import tempfile

import depthai as dai

PALM_INPUT = "input"
PALM_OUTPUTS = ("regressors", "classificators")
LANDMARK_INPUT = "input_1"
LANDMARK_OUTPUTS = (
    "Identity_dense/BiasAdd/Add",
    "Identity_1",
    "Identity_2",
    "Identity_3_dense/BiasAdd/Add",
)
LANDMARK_PARSERS = (
    "KeypointParser",
    "RegressionParser",
    "RegressionParser",
    "RegressionParser",
)
ARCHIVE_CACHE = Path("/tmp") / f"pib-imitation-archives-{os.getuid()}"


def _image_input(name, size):
    return {
        "name": name,
        "dtype": "uint8",
        "input_type": "image",
        "shape": [1, 3, size, size],
        "layout": "NCHW",
        "preprocessing": {
            "mean": [0.0, 0.0, 0.0],
            "scale": [1.0, 1.0, 1.0],
            "reverse_channels": False,
            "interleaved_to_planar": False,
            "dai_type": None,
        },
    }


def _output(name, shape):
    return {
        "name": name,
        "dtype": "float16",
        "shape": shape,
        "layout": "NC",
    }


def palm_archive_config():
    """Return the v1 parser configuration for the 128x128 palm detector."""
    return {
        "config_version": "1.0",
        "model": {
            "metadata": {
                "name": "pib-mediapipe-palm-detection-128x128",
                "path": "model.blob",
                "precision": "float16",
            },
            "inputs": [_image_input(PALM_INPUT, 128)],
            "outputs": [
                _output(PALM_OUTPUTS[0], [1, 896, 18]),
                _output(PALM_OUTPUTS[1], [1, 896, 1]),
            ],
            "heads": [
                {
                    "name": None,
                    "parser": "MPPalmDetectionParser",
                    "metadata": {
                        "postprocessor_path": None,
                        "classes": ["palm"],
                        "n_classes": 1,
                        "iou_threshold": 0.5,
                        "conf_threshold": 0.5,
                        "max_det": 100,
                        "anchors": None,
                        "scale": 128,
                    },
                    "outputs": list(PALM_OUTPUTS),
                }
            ],
        },
    }


def landmark_archive_config():
    """Return landmark heads in the order consumed by GatherData."""
    image_keypoints, score, handedness, world = LANDMARK_OUTPUTS
    return {
        "config_version": "1.0",
        "model": {
            "metadata": {
                "name": "pib-mediapipe-hand-landmark-full-224x224",
                "path": "model.blob",
                "precision": "float16",
            },
            "inputs": [_image_input(LANDMARK_INPUT, 224)],
            "outputs": [
                _output(image_keypoints, [1, 63]),
                _output(score, [1, 1]),
                _output(handedness, [1, 1]),
                _output(world, [1, 63]),
            ],
            "heads": [
                {
                    "name": None,
                    "parser": "KeypointParser",
                    "metadata": {
                        "postprocessor_path": None,
                        "n_keypoints": 21,
                        "scale_factor": 224,
                    },
                    "outputs": [image_keypoints],
                },
                {
                    "name": None,
                    "parser": "RegressionParser",
                    "metadata": {
                        "postprocessor_path": None,
                        "score_threshold": 0.5,
                    },
                    "outputs": [score],
                },
                {
                    "name": None,
                    "parser": "RegressionParser",
                    "metadata": {"postprocessor_path": None},
                    "outputs": [handedness],
                },
                {
                    # RegressionParser deliberately preserves signed world XYZ.
                    "name": None,
                    "parser": "RegressionParser",
                    "metadata": {"postprocessor_path": None},
                    "outputs": [world],
                },
            ],
        },
    }


def _sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as blob_file:
        for chunk in iter(lambda: blob_file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _validate_blob(blob_path, input_name, outputs, input_size):
    blob = dai.OpenVINO.Blob(blob_path)
    if blob.numShaves != 4:
        raise ValueError(f"{blob_path} uses {blob.numShaves} shaves, expected 4")
    if set(blob.networkInputs) != {input_name}:
        raise ValueError(f"{blob_path} has unexpected inputs")
    if set(blob.networkOutputs) != set(outputs):
        raise ValueError(f"{blob_path} has unexpected outputs")
    input_dims = list(blob.networkInputs[input_name].dims)
    if input_size not in input_dims or input_dims.count(input_size) < 2:
        raise ValueError(f"{blob_path} has unexpected input dimensions {input_dims}")


def _write_archive(blob_path, config, archive_path):
    archive_path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    config_bytes = json.dumps(config, sort_keys=True, separators=(",", ":")).encode(
        "utf-8"
    )
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{archive_path.name}.", dir=archive_path.parent
    )
    os.close(descriptor)
    temporary_path = Path(temporary_name)
    try:
        with tarfile.open(temporary_path, "w:xz") as archive:
            config_info = tarfile.TarInfo("config.json")
            config_info.size = len(config_bytes)
            config_info.mode = 0o644
            archive.addfile(config_info, io.BytesIO(config_bytes))
            archive.add(blob_path, arcname="model.blob", recursive=False)
        os.chmod(temporary_path, 0o600)
        os.replace(temporary_path, archive_path)
    finally:
        temporary_path.unlink(missing_ok=True)


def _archive(blob_path, config, input_name, outputs, input_size, cache_dir=None):
    blob_path = Path(blob_path)
    _validate_blob(blob_path, input_name, outputs, input_size)
    cache_dir = ARCHIVE_CACHE if cache_dir is None else Path(cache_dir)
    archive_path = cache_dir / f"{_sha256(blob_path)}.tar.xz"
    if not archive_path.is_file():
        _write_archive(blob_path, config, archive_path)
    archive = dai.NNArchive(archive_path)
    if archive.getInputWidth() != input_size or archive.getInputHeight() != input_size:
        raise ValueError(f"{archive_path} has unexpected archive input dimensions")
    return archive


def create_palm_archive(blob_path, cache_dir=None):
    return _archive(
        blob_path,
        palm_archive_config(),
        PALM_INPUT,
        PALM_OUTPUTS,
        128,
        cache_dir,
    )


def create_landmark_archive(blob_path, cache_dir=None):
    return _archive(
        blob_path,
        landmark_archive_config(),
        LANDMARK_INPUT,
        LANDMARK_OUTPUTS,
        224,
        cache_dir,
    )
