"""Build parser archives for selectable single-network tasks."""

from pathlib import Path

import depthai as dai

from .imitation_archive import ARCHIVE_CACHE, _sha256, _write_archive

YUNET_MODEL_ID = "face_detection_yunet_160x120"
YUNET_INPUT_SIZE = (160, 120)
YUNET_LABELS = ("Face",)

YOLOV6N_MODEL_ID = "yolov6n_coco_640x640"
YOLOV6N_INPUT_SIZE = (640, 640)
# 80 COCO names in this model's class-index order, taken from
# luxonis/oak-examples neural-networks/object-detection/yolo-p (same order as
# meituan/YOLOv6 data/coco.yaml and the depthai-model-zoo
# yolov6n_coco_640x640 model.yml). A shuffled list would mislabel every box.
YOLOV6N_COCO_LABELS = (
    "person",
    "bicycle",
    "car",
    "motorcycle",
    "airplane",
    "bus",
    "train",
    "truck",
    "boat",
    "traffic light",
    "fire hydrant",
    "stop sign",
    "parking meter",
    "bench",
    "bird",
    "cat",
    "dog",
    "horse",
    "sheep",
    "cow",
    "elephant",
    "bear",
    "zebra",
    "giraffe",
    "backpack",
    "umbrella",
    "handbag",
    "tie",
    "suitcase",
    "frisbee",
    "skis",
    "snowboard",
    "sports ball",
    "kite",
    "baseball bat",
    "baseball glove",
    "skateboard",
    "surfboard",
    "tennis racket",
    "bottle",
    "wine glass",
    "cup",
    "fork",
    "knife",
    "spoon",
    "bowl",
    "banana",
    "apple",
    "sandwich",
    "orange",
    "broccoli",
    "carrot",
    "hot dog",
    "pizza",
    "donut",
    "cake",
    "chair",
    "couch",
    "potted plant",
    "bed",
    "dining table",
    "toilet",
    "tv",
    "laptop",
    "mouse",
    "remote",
    "keyboard",
    "cell phone",
    "microwave",
    "oven",
    "toaster",
    "sink",
    "refrigerator",
    "book",
    "clock",
    "vase",
    "scissors",
    "teddy bear",
    "hair drier",
    "toothbrush",
)


def _image_input(name, width, height):
    return {
        "name": name,
        "dtype": "uint8",
        "input_type": "image",
        "shape": [1, 3, height, width],
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
        "shape": list(shape),
        "layout": "NC",
    }


def _yunet_outputs(blob):
    """Return the real blob heads in the order expected by YuNetParser."""
    names = tuple(blob.networkOutputs)
    ordered = []
    for token in ("loc", "conf", "iou"):
        matches = [name for name in names if token in name.lower()]
        if len(matches) != 1:
            raise ValueError(
                f"YuNet blob must have one {token} output, found {matches or 'none'}"
            )
        ordered.append(matches[0])
    if set(ordered) != set(names):
        raise ValueError(f"YuNet blob has unexpected outputs {names}")
    return tuple(ordered)


def yunet_archive_config(blob):
    """Return a v1 archive config using the tensor names the blob really carries.

    The shipped YuNet blob names its input "input" (checked with
    dai.OpenVINO.Blob), so the name is read here instead of assumed - guessing it
    is what made the first version of this module fail on the real blob.
    """
    input_names = tuple(blob.networkInputs)
    if len(input_names) != 1:
        raise ValueError(f"YuNet blob has unexpected inputs {input_names}")
    input_name = input_names[0]
    input_dims = list(blob.networkInputs[input_name].dims)
    if 160 not in input_dims or 120 not in input_dims:
        raise ValueError(f"YuNet blob has unexpected input dimensions {input_dims}")

    loc_name, conf_name, iou_name = _yunet_outputs(blob)
    outputs = (loc_name, conf_name, iou_name)
    return {
        "config_version": "1.0",
        "model": {
            "metadata": {
                "name": YUNET_MODEL_ID,
                "path": "model.blob",
                "precision": "float16",
            },
            "inputs": [_image_input(input_name, *YUNET_INPUT_SIZE)],
            "outputs": [
                _output(name, list(blob.networkOutputs[name].dims)) for name in outputs
            ],
            "heads": [
                {
                    "name": None,
                    "parser": "YuNetParser",
                    "metadata": {
                        "postprocessor_path": None,
                        "classes": list(YUNET_LABELS),
                        "n_classes": 1,
                        "iou_threshold": 0.3,
                        "conf_threshold": 0.8,
                        "max_det": 5000,
                        # The parser takes the layer names explicitly; the defaults
                        # are None and it would have to guess them.
                        "input_size": list(YUNET_INPUT_SIZE),
                        "loc_output_layer_name": loc_name,
                        "conf_output_layer_name": conf_name,
                        "iou_output_layer_name": iou_name,
                    },
                    "outputs": list(outputs),
                }
            ],
        },
    }


def create_yunet_archive(blob_path, cache_dir=None):
    blob_path = Path(blob_path)
    blob = dai.OpenVINO.Blob(blob_path)
    # The shave count lives in the manifest entry, not in the archive: the start
    # request carries it and a mismatch is rejected there. Do not duplicate it here.
    config = yunet_archive_config(blob)
    cache_dir = ARCHIVE_CACHE if cache_dir is None else Path(cache_dir)
    archive_path = cache_dir / f"{_sha256(blob_path)}.tar.xz"
    if not archive_path.is_file():
        _write_archive(blob_path, config, archive_path)
    archive = dai.NNArchive(archive_path)
    if (archive.getInputWidth(), archive.getInputHeight()) != YUNET_INPUT_SIZE:
        raise ValueError(f"{archive_path} has unexpected archive input dimensions")
    return archive


def _yolov6_outputs(blob):
    """Return the real blob heads in the order YOLOExtendedParser expects."""
    names = tuple(blob.networkOutputs)
    ordered = []
    for token in ("output1_yolov6", "output2_yolov6", "output3_yolov6"):
        matches = [name for name in names if name == token]
        if len(matches) != 1:
            raise ValueError(
                f"YOLOv6 blob must have one {token} output, found {matches or 'none'}"
            )
        ordered.append(matches[0])
    if set(ordered) != set(names):
        raise ValueError(f"YOLOv6 blob has unexpected outputs {names}")
    return tuple(ordered)


def yolov6n_archive_config(blob):
    """Return a v1 archive config using the tensor names the blob really carries.

    The shipped YOLOv6n blob names its input "images" (checked with
    dai.OpenVINO.Blob), so the name is read here instead of assumed - guessing it
    is what made the first YuNet archive fail on the real blob.
    """
    input_names = tuple(blob.networkInputs)
    if len(input_names) != 1:
        raise ValueError(f"YOLOv6 blob has unexpected inputs {input_names}")
    input_name = input_names[0]
    input_dims = list(blob.networkInputs[input_name].dims)
    if 640 not in input_dims or input_dims.count(640) < 2:
        raise ValueError(f"YOLOv6 blob has unexpected input dimensions {input_dims}")

    outputs = _yolov6_outputs(blob)
    labels = list(YOLOV6N_COCO_LABELS)
    return {
        "config_version": "1.0",
        "model": {
            "metadata": {
                "name": YOLOV6N_MODEL_ID,
                "path": "model.blob",
                "precision": "float16",
            },
            "inputs": [_image_input(input_name, *YOLOV6N_INPUT_SIZE)],
            "outputs": [
                _output(name, list(blob.networkOutputs[name].dims)) for name in outputs
            ],
            "heads": [
                {
                    "name": None,
                    "parser": "YOLOExtendedParser",
                    "metadata": {
                        "postprocessor_path": None,
                        "classes": labels,
                        # YOLOExtendedParser.build reads classes into label_names;
                        # keep the constructor name too so extraParams carries it.
                        "label_names": labels,
                        "n_classes": 80,
                        "iou_threshold": 0.5,
                        "conf_threshold": 0.5,
                        "subtype": "yolov6",
                    },
                    "outputs": list(outputs),
                }
            ],
        },
    }


def create_yolov6n_archive(blob_path, cache_dir=None):
    blob_path = Path(blob_path)
    blob = dai.OpenVINO.Blob(blob_path)
    config = yolov6n_archive_config(blob)
    cache_dir = ARCHIVE_CACHE if cache_dir is None else Path(cache_dir)
    archive_path = cache_dir / f"{_sha256(blob_path)}.tar.xz"
    if not archive_path.is_file():
        _write_archive(blob_path, config, archive_path)
    archive = dai.NNArchive(archive_path)
    if (archive.getInputWidth(), archive.getInputHeight()) != YOLOV6N_INPUT_SIZE:
        raise ValueError(f"{archive_path} has unexpected archive input dimensions")
    return archive


_ARCHIVE_BUILDERS = {
    YUNET_MODEL_ID: create_yunet_archive,
    YOLOV6N_MODEL_ID: create_yolov6n_archive,
}
_MODEL_LABELS = {
    YUNET_MODEL_ID: YUNET_LABELS,
    YOLOV6N_MODEL_ID: YOLOV6N_COCO_LABELS,
}


def create_archive(model_id, blob_path, cache_dir=None):
    """Return a parser archive for a registered task, or ``None``."""
    builder = _ARCHIVE_BUILDERS.get(model_id)
    return None if builder is None else builder(blob_path, cache_dir)


def labels_for_model(model_id):
    """Return the class list embedded in a task's parser archive."""
    return _MODEL_LABELS.get(model_id, ())
