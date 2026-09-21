"""Device-free contracts for face-crop classification."""

import sys
import types

import numpy as np
import pytest

from ros_packages.camera.oak_d_lite.face_crop import (
    EMOTION_LABELS,
    FACEMESH_LANDMARK_COUNT,
    face_crop_classifier_id,
    packet_timestamp,
    emotion_probabilities,
    softmax,
    translate_emotion,
    translate_facemesh,
)


def _face():
    box = types.SimpleNamespace(
        center=types.SimpleNamespace(x=0.5, y=0.4),
        size=types.SimpleNamespace(width=0.2, height=0.4),
        angle=17.0,
    )
    return types.SimpleNamespace(getBoundingBox=lambda: box)


def test_softmax_argmax_translation_keeps_face_rotated_rect(monkeypatch):
    class Detection:
        pass

    datatypes = types.ModuleType("datatypes")
    datatypes_msg = types.ModuleType("datatypes.msg")
    datatypes_msg.Detection = Detection
    datatypes.msg = datatypes_msg
    monkeypatch.setitem(sys.modules, "datatypes", datatypes)
    monkeypatch.setitem(sys.modules, "datatypes.msg", datatypes_msg)

    packet = types.SimpleNamespace(
        getTensor=lambda name: np.array([0.0, 3.0, 1.0, -1.0, 0.5])
    )
    detection = translate_emotion(packet, _face(), 1280, 720)

    expected = softmax([0.0, 3.0, 1.0, -1.0, 0.5])
    assert detection.label == "happy"
    assert detection.score == pytest.approx(expected[1])
    assert (detection.x_min, detection.y_min) == (512, 144)
    assert (detection.x_max, detection.y_max) == (768, 432)
    assert detection.keypoint_names == []
    assert detection.scalar_names == list(EMOTION_LABELS)
    assert detection.scalar_values == pytest.approx(expected)
    assert sum(detection.scalar_values) == pytest.approx(1.0)


def test_translation_uses_the_blobs_single_exported_output_name(monkeypatch):
    class Detection:
        pass

    datatypes = types.ModuleType("datatypes")
    datatypes_msg = types.ModuleType("datatypes.msg")
    datatypes_msg.Detection = Detection
    datatypes.msg = datatypes_msg
    monkeypatch.setitem(sys.modules, "datatypes", datatypes)
    monkeypatch.setitem(sys.modules, "datatypes.msg", datatypes_msg)

    def tensor(name):
        if name != "prob":
            raise RuntimeError("unknown layer")
        return np.array([0.0, 0.0, 0.0, 0.0, 2.0])

    packet = types.SimpleNamespace(
        getAllLayerNames=lambda: ["prob"],
        getTensor=tensor,
    )

    detection = translate_emotion(packet, _face(), 1280, 720)

    assert detection.label == "anger"


def test_softmax_rejects_wrong_size_and_nonfinite_values():
    with pytest.raises(ValueError, match="expected 5"):
        softmax([1.0, 2.0])
    with pytest.raises(ValueError, match="non-finite"):
        softmax([0.0, 1.0, 2.0, 3.0, float("nan")])


def test_facemesh_maps_pixel_xyz_through_the_padded_face_crop(monkeypatch):
    class Detection:
        pass

    datatypes = types.ModuleType("datatypes")
    datatypes_msg = types.ModuleType("datatypes.msg")
    datatypes_msg.Detection = Detection
    datatypes.msg = datatypes_msg
    monkeypatch.setitem(sys.modules, "datatypes", datatypes)
    monkeypatch.setitem(sys.modules, "datatypes.msg", datatypes_msg)

    values = np.tile([96.0, 96.0, 48.0], (FACEMESH_LANDMARK_COUNT, 1))
    values[0] = [0.0, 0.0, -96.0]
    values[-1] = [192.0, 192.0, 96.0]
    packet = types.SimpleNamespace(getTensor=lambda name: values.reshape(-1))

    detection = translate_facemesh(packet, _face(), 1280, 720)

    assert detection.label == "Face"
    assert detection.score == 1.0
    assert (detection.x_min, detection.y_min) == (512, 144)
    assert (detection.x_max, detection.y_max) == (768, 432)
    assert len(detection.keypoint_names) == FACEMESH_LANDMARK_COUNT
    assert detection.keypoint_names[:2] == ["landmark_0", "landmark_1"]
    assert detection.keypoint_names[-1] == "landmark_467"
    # Hand-computed: the crop is now a PIXEL square of side max(0.2*1280, 0.4*720)
    # * 1.2 = 345.6 px centred on (640, 288), so its corners are (467.2, 115.2) and
    # (812.8, 460.8) and its centre is (640, 288). Before the fix those were
    # 256.0 / 396.0 - the mesh sat far outside the face.
    assert detection.keypoint_x[0] == pytest.approx(467.2)
    assert detection.keypoint_y[0] == pytest.approx(115.2)
    assert detection.keypoint_x[-1] == pytest.approx(812.8)
    assert detection.keypoint_y[-1] == pytest.approx(460.8)
    assert detection.keypoint_x[1] == pytest.approx(640.0)
    assert detection.keypoint_y[1] == pytest.approx(288.0)
    # The mesh must now stay within roughly the face's own extent plus padding,
    # instead of spanning ~2.4x the face box.
    span_x = max(detection.keypoint_x) - min(detection.keypoint_x)
    span_y = max(detection.keypoint_y) - min(detection.keypoint_y)
    box_w = detection.x_max - detection.x_min
    box_h = detection.y_max - detection.y_min
    assert span_x < 2.0 * box_w and span_y < 2.0 * box_h


def test_facemesh_keeps_already_normalized_z(monkeypatch):
    class Detection:
        pass

    datatypes = types.ModuleType("datatypes")
    datatypes_msg = types.ModuleType("datatypes.msg")
    datatypes_msg.Detection = Detection
    datatypes.msg = datatypes_msg
    monkeypatch.setitem(sys.modules, "datatypes", datatypes)
    monkeypatch.setitem(sys.modules, "datatypes.msg", datatypes_msg)

    values = np.tile([0.25, 0.75, -0.5], (FACEMESH_LANDMARK_COUNT, 1))
    packet = types.SimpleNamespace(getTensor=lambda name: values.reshape(-1))

    detection = translate_facemesh(packet, _face(), 1280, 720)

    # Hand-computed for the pixel-square crop: side = max(0.2*1280, 0.4*720) * 1.2
    # = 345.6 px centred on (640, 288), so x = 467.2 + 0.25 * 345.6 = 553.6 and
    # y = 115.2 + 0.75 * 345.6 = 374.4.
    assert detection.keypoint_x[0] == pytest.approx(553.6)
    assert detection.keypoint_y[0] == pytest.approx(374.4)
    assert detection.keypoint_z[0] == pytest.approx(-0.5)


def test_face_crop_classifier_is_derived_and_unknown_models_fail_loudly():
    assert (
        face_crop_classifier_id(("face_detection_yunet_160x120", "facemesh_192x192"))
        == "facemesh_192x192"
    )
    with pytest.raises(ValueError, match="unsupported"):
        face_crop_classifier_id(("face_detection_yunet_160x120", "future_model"))


def test_packet_timestamp_accepts_depthai_and_ros_spellings():
    depthai_packet = types.SimpleNamespace(
        getTimestamp=lambda: types.SimpleNamespace(seconds=3, microseconds=4)
    )
    ros_packet = types.SimpleNamespace(
        getTimestamp=lambda: types.SimpleNamespace(sec=5, nanosec=6)
    )

    assert packet_timestamp(depthai_packet) == (3, 4000)
    assert packet_timestamp(ros_packet) == (5, 6)


def test_probability_output_is_not_softmaxed_again(monkeypatch):
    """The shipped blob ends in SoftMax, so its values must arrive unchanged.

    Measured on the robot before the fix: this distribution was published as
    (0.301, 0.176, 0.171, 0.168, 0.184), which reads as an unsure model.
    """

    class Detection:
        pass

    datatypes = types.ModuleType("datatypes")
    datatypes_msg = types.ModuleType("datatypes.msg")
    datatypes_msg.Detection = Detection
    datatypes.msg = datatypes_msg
    monkeypatch.setitem(sys.modules, "datatypes", datatypes)
    monkeypatch.setitem(sys.modules, "datatypes.msg", datatypes_msg)

    published = np.array([0.636, 0.096, 0.066, 0.056, 0.146])
    packet = types.SimpleNamespace(getTensor=lambda name: published.copy())

    detection = translate_emotion(packet, _face(), 1280, 720)

    assert np.allclose(
        [float(v) for v in detection.scalar_values], published, atol=1e-6
    )
    doubled = softmax(published)
    assert not np.allclose(
        [float(v) for v in detection.scalar_values], doubled, atol=1e-3
    )
    assert detection.label == "neutral"
    assert abs(float(detection.score) - 0.636) < 1e-6


def test_logit_output_still_falls_back_to_softmax():
    packet = types.SimpleNamespace(
        getTensor=lambda name: np.array([0.0, 3.0, 1.0, -1.0, 0.5])
    )
    probabilities = emotion_probabilities(packet)
    assert abs(float(np.sum(probabilities)) - 1.0) < 1e-9
    assert int(np.argmax(probabilities)) == 1


def test_emotion_probabilities_rejects_a_wrong_size():
    packet = types.SimpleNamespace(getTensor=lambda name: np.array([0.5, 0.5]))
    with pytest.raises(ValueError):
        emotion_probabilities(packet)


def test_pixel_square_crop_is_square_in_pixels_not_in_normalised_units():
    """The face must fill the network input; the crop must be a PIXEL square.

    Measured before the fix: a 149x159 px face on a 1280x720 frame produced a
    539x303 px crop, leaving the face a 53x57 px island in the 192x192 input and a
    mesh 2.4x too wide.
    """
    import types as _types

    from ros_packages.camera.oak_d_lite import imitation

    rect = _types.SimpleNamespace(
        center=_types.SimpleNamespace(x=0.5, y=0.4),
        size=_types.SimpleNamespace(width=149 / 1280, height=159 / 720),
        angle=0.0,
    )
    size_x, size_y = imitation.pixel_square_crop_size(rect, 0.1, (1152, 648))

    # In the BRANCH's pixels the face is 0.1164*1152 = 134.1 x 0.2208*648 = 143.1 px,
    # so the side is 143.1 * 1.2 = 171.7 px. (On the published 1280x720 frame the
    # same normalised numbers describe the same square, because both are 16:9.)
    face_w_px = (149 / 1280) * 1152
    face_h_px = (159 / 720) * 648
    side_px = max(face_w_px, face_h_px) * 1.2
    assert size_x == pytest.approx(side_px / 1152, rel=1e-9)
    assert size_y == pytest.approx(side_px / 648, rel=1e-9)
    # Square in pixels means a 16:9 rectangle in normalised units - the whole point.
    assert size_x * 1152 == pytest.approx(size_y * 648)
    assert size_x != pytest.approx(size_y, rel=1e-3)
    # The face now fills most of the crop instead of a quarter of it.
    assert face_h_px / (size_y * 648) > 0.75
    # And the y/x ratio the translator uses to de-stretch equals the frame aspect.
    assert (size_y / size_x) == pytest.approx(1152 / 648, rel=1e-9)


def test_detection_crop_config_stretches_a_pixel_square_crop():
    """The config must crop that rectangle and STRETCH it onto the square input."""
    import types as _types

    import depthai as _dai

    from ros_packages.camera.oak_d_lite import imitation

    rect = _types.SimpleNamespace(
        center=_types.SimpleNamespace(x=0.5, y=0.5),
        size=_types.SimpleNamespace(width=0.2, height=0.4),
        angle=0.0,
    )
    detection = _types.SimpleNamespace(getBoundingBox=lambda: rect)
    square = imitation.detection_crop_config(
        detection, 0.1, 192, 192, square_in_pixels=True, source_size=(1152, 648)
    )
    legacy = imitation.detection_crop_config(detection, 0.1, 192, 192)

    # depthai's ImageManipConfig exposes no getters, so the rendered form is the only
    # way to observe the resize mode from outside. Values are filled in below.
    stretch = int(_dai.ImageManipConfig.ResizeMode.STRETCH)
    letterbox = int(_dai.ImageManipConfig.ResizeMode.LETTERBOX)
    assert f"resizeMode: {stretch}" in repr(square)
    assert f"resizeMode: {letterbox}" in repr(legacy)
