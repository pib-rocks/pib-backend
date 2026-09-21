import types
from unittest.mock import MagicMock

import numpy as np
import pytest

from ros_packages.camera.oak_d_lite.qr_detection import (
    QR_INPUT_DIMS,
    QR_INPUT_NAME,
    QR_OUTPUT_DIMS,
    QR_OUTPUT_NAME,
    decode_qr_detections,
    validate_qr_blob,
)


def _blob(
    input_name=QR_INPUT_NAME, input_dims=QR_INPUT_DIMS, output_dims=QR_OUTPUT_DIMS
):
    return types.SimpleNamespace(
        networkInputs={input_name: types.SimpleNamespace(dims=input_dims)},
        networkOutputs={
            QR_OUTPUT_NAME: types.SimpleNamespace(dims=output_dims),
        },
    )


def _packet(rows, layer_name=QR_OUTPUT_NAME):
    values = np.zeros((100, 7), dtype=np.float32)
    values[:, 0] = -1
    if rows:
        values[: len(rows)] = rows
    return types.SimpleNamespace(
        getAllLayerNames=lambda: [layer_name],
        # Mirrors the real depthai v3 API. The previous fixture invented
        # getLayerFp16 (a v2 name that no longer exists), which is exactly
        # why the live device dropped every packet while the test passed.
        getTensor=lambda name: values,
    )


def test_validates_the_named_grayscale_input_and_ssd_output():
    validate_qr_blob(_blob())

    with pytest.raises(ValueError, match="input 'data' must have shape"):
        validate_qr_blob(_blob(input_dims=(384, 384, 3, 1)))
    with pytest.raises(ValueError, match="output 'detection_output' must have shape"):
        validate_qr_blob(_blob(output_dims=(4, 100, 1, 1)))


def test_rejects_a_packet_without_the_named_700_value_output():
    frame = np.zeros((100, 200, 3), dtype=np.uint8)
    packet = _packet([], layer_name="guessed_output")

    with pytest.raises(ValueError, match="no layer 'detection_output'"):
        decode_qr_detections(packet, frame)


def test_filters_ssd_candidates_scales_box_and_decodes_with_four_corners():
    frame = np.zeros((100, 200, 3), dtype=np.uint8)
    packet = _packet(
        [
            [0, 1, 0.9, 0.1, 0.2, 0.8, 0.9],
            [0, 1, 0.4, 0.0, 0.0, 1.0, 1.0],
            [-1, 0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]
    )
    detector = MagicMock()
    # cv2 findet hier nichts, also greift der Rueckfall ueber das Box-Viereck.
    detector.detectAndDecode.return_value = ("", None, None)
    detector.decode.return_value = ("https://pib.rocks", None)

    detections = decode_qr_detections(packet, frame, detector=detector)

    assert len(detections) == 1
    assert detections[0].box == (20, 20, 160, 90)
    assert detections[0].corners == (
        (20.0, 20.0),
        (160.0, 20.0),
        (160.0, 90.0),
        (20.0, 90.0),
    )
    assert detections[0].text == "https://pib.rocks"
    points = detector.decode.call_args.args[1]
    np.testing.assert_array_equal(points, np.asarray([detections[0].corners]))
    detector.detectAndDecode.assert_called_once()


def test_prefers_cv2_own_detection_on_the_whole_frame():
    frame = np.zeros((100, 200, 3), dtype=np.uint8)
    packet = _packet([[0, 1, 0.9, 0.1, 0.2, 0.8, 0.9]])
    detector = MagicMock()
    detector.decode.return_value = ("", None)
    detector.detectAndDecode.return_value = ("fallback", None, None)

    detections = decode_qr_detections(packet, frame, detector=detector)

    assert detections[0].text == "fallback"
    assert detector.detectAndDecode.call_args.args[0].shape == (100, 200, 3)
