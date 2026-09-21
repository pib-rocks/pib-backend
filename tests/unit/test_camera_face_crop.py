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
    assert detection.keypoint_x[0] == pytest.approx(256.0)
    assert detection.keypoint_y[0] == pytest.approx(72.0)
    assert detection.keypoint_x[-1] == pytest.approx(1024.0)
    assert detection.keypoint_y[-1] == pytest.approx(504.0)
    assert detection.keypoint_z[0] == pytest.approx(-0.5)
    assert detection.keypoint_z[-1] == pytest.approx(0.5)


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

    assert detection.keypoint_x[0] == pytest.approx(448.0)
    assert detection.keypoint_y[0] == pytest.approx(396.0)
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
