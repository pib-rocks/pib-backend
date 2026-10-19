"""PC-102 duplex ROS wiring: fast VAD vs topic fallback."""

from __future__ import annotations

import logging

from voice_assistant.ros_duplex import build_duplex_vad_wiring


class _FakeTuning:
    def is_voice(self) -> bool:
        return True


def test_unavailable_register_port_falls_back_to_topic(caplog) -> None:
    caplog.set_level(logging.WARNING)
    wiring = build_duplex_vad_wiring(
        opener=lambda: None,
        log=logging.getLogger("test.ros_duplex"),
    )
    assert wiring.fast_sampler is None
    assert wiring.use_topic_fallback is True
    assert caplog.records
    assert "/voice_activity" in caplog.records[0].message


def test_tuning_port_enables_fast_sampler() -> None:
    wiring = build_duplex_vad_wiring(opener=lambda: _FakeTuning())
    assert wiring.fast_sampler is not None
    assert wiring.use_topic_fallback is False
