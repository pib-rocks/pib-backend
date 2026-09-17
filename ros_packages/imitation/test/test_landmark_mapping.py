"""Colcon-level smoke tests for the pure imitation mapping module."""

from imitation.landmark_mapping import HAND_LANDMARK_NAMES


def test_hand_contract_contains_all_mediapipe_landmarks():
    assert len(HAND_LANDMARK_NAMES) == 21
    assert HAND_LANDMARK_NAMES[0] == "wrist"
    assert HAND_LANDMARK_NAMES[-1] == "pinky_tip"
