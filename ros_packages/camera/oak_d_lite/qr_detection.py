"""Host-side decoding for the raw WeChat QR detector blob."""

from dataclasses import dataclass
from typing import Tuple

import cv2
import numpy as np

QR_MODEL_ID = "qr_code_detection_384x384"
QR_INPUT_NAME = "data"
QR_OUTPUT_NAME = "detection_output"
QR_INPUT_DIMS = (384, 384, 1, 1)
QR_OUTPUT_DIMS = (7, 100, 1, 1)
QR_CONFIDENCE_THRESHOLD = 0.5


@dataclass(frozen=True)
class QrDetection:
    confidence: float
    box: Tuple[int, int, int, int]
    corners: Tuple[Tuple[float, float], ...]
    text: str


def validate_qr_blob(blob):
    """Reject a blob whose named tensors do not match the inspected model."""
    input_names = tuple(blob.networkInputs)
    output_names = tuple(blob.networkOutputs)
    if input_names != (QR_INPUT_NAME,):
        raise ValueError(
            f"{QR_MODEL_ID} must have input {QR_INPUT_NAME!r}, found {input_names}"
        )
    input_dims = tuple(blob.networkInputs[QR_INPUT_NAME].dims)
    if input_dims != QR_INPUT_DIMS:
        raise ValueError(
            f"{QR_MODEL_ID} input {QR_INPUT_NAME!r} must have shape "
            f"{QR_INPUT_DIMS}, found {input_dims}"
        )
    if QR_OUTPUT_NAME not in output_names:
        raise ValueError(
            f"{QR_MODEL_ID} has no output named {QR_OUTPUT_NAME!r}; "
            f"found {output_names}"
        )
    output_dims = tuple(blob.networkOutputs[QR_OUTPUT_NAME].dims)
    if output_dims != QR_OUTPUT_DIMS:
        raise ValueError(
            f"{QR_MODEL_ID} output {QR_OUTPUT_NAME!r} must have shape "
            f"{QR_OUTPUT_DIMS}, found {output_dims}"
        )


def _pixel(value, extent):
    return min(extent, max(0, int(round(float(value) * extent))))


def _decode_text(detector, frame, corners, box):
    points = np.asarray([corners], dtype=np.float32)
    try:
        text, _ = detector.decode(frame, points)
    except cv2.error:
        text = ""
    if text:
        return str(text)

    x_min, y_min, x_max, y_max = box
    crop = frame[y_min:y_max, x_min:x_max]
    if crop.size == 0:
        return ""
    try:
        text, _, _ = detector.detectAndDecode(crop)
    except cv2.error:
        return ""
    return str(text or "")


def decode_qr_detections(
    packet,
    frame,
    confidence_threshold=QR_CONFIDENCE_THRESHOLD,
    detector=None,
):
    """Decode SSD tuples and resolve their QR payloads against a cached frame."""
    names_getter = getattr(packet, "getAllLayerNames", None)
    names = tuple(names_getter()) if callable(names_getter) else ()
    if QR_OUTPUT_NAME not in names:
        raise ValueError(
            f"{QR_MODEL_ID} packet has no layer {QR_OUTPUT_NAME!r}; found {names}"
        )
    values = np.asarray(packet.getLayerFp16(QR_OUTPUT_NAME), dtype=np.float32)
    if values.size != QR_OUTPUT_DIMS[0] * QR_OUTPUT_DIMS[1]:
        raise ValueError(
            f"{QR_MODEL_ID} output {QR_OUTPUT_NAME!r} must contain 700 values, "
            f"found {values.size}"
        )

    height, width = frame.shape[:2]
    detector = detector or cv2.QRCodeDetector()
    decoded = []
    for image_id, _label, confidence, x_min, y_min, x_max, y_max in values.reshape(
        -1, 7
    ):
        if image_id < 0:
            break
        if (
            not np.isfinite([confidence, x_min, y_min, x_max, y_max]).all()
            or confidence < confidence_threshold
        ):
            continue
        box = (
            _pixel(x_min, width),
            _pixel(y_min, height),
            _pixel(x_max, width),
            _pixel(y_max, height),
        )
        if box[2] <= box[0] or box[3] <= box[1]:
            continue
        corners = (
            (float(box[0]), float(box[1])),
            (float(box[2]), float(box[1])),
            (float(box[2]), float(box[3])),
            (float(box[0]), float(box[3])),
        )
        decoded.append(
            QrDetection(
                confidence=float(confidence),
                box=box,
                corners=corners,
                text=_decode_text(detector, frame, corners, box),
            )
        )
    return decoded
