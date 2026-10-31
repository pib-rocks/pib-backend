"""Cause-naming for a turn that produced no answer (PR-2132)."""

from __future__ import annotations

import re

import pytest

from public_api_client import turn_failure as tf


@pytest.mark.parametrize(
    ("detail", "expected"),
    [
        # The measured failure on 192.168.178.88: the Hermes home was read-only.
        (
            "hermes: [Errno 30] Read-only file system: "
            "'/home/pib/.hermes/installs/ff92010f0bec7724/.install.lock'; "
            "run `hermes pm repair`",
            tf.REASON_HOME_READ_ONLY,
        ),
        (
            "sqlite3.OperationalError: attempt to write a readonly database",
            tf.REASON_HOME_READ_ONLY,
        ),
        ("OSError: [Errno 28] No space left on device", tf.REASON_DISK_FULL),
        ("bash: hermes: command not found", tf.REASON_AGENT_MISSING),
        ("429 insufficient_quota: budget exhausted", tf.REASON_PROVIDER_BUDGET),
        (
            "API key not valid. Please pass a valid API key.",
            tf.REASON_PROVIDER_KEY_REJECTED,
        ),
        ("the provider answered 401", tf.REASON_PROVIDER_KEY_REJECTED),
    ],
)
def test_the_classifier_names_the_cause_from_the_measured_text(detail, expected):
    assert tf.classify_turn_failure(detail=detail) == expected


def test_a_daemon_reason_is_kept_as_is():
    assert (
        tf.classify_turn_failure(reason=tf.REASON_HOME_READ_ONLY)
        == tf.REASON_HOME_READ_ONLY
    )


def test_a_transport_failure_is_not_a_storage_failure():
    assert (
        tf.classify_turn_failure(reason="ConnectionError", detail="remote end closed")
        == tf.REASON_AGENT_UNREACHABLE
    )


def test_a_real_timeout_keeps_the_timeout_sentence():
    sentence = tf.failure_sentence(tf.REASON_TURN_TIMEOUT)
    assert "zu lange gedauert" in sentence
    assert tf.classify_turn_failure(reason="turn_timeout") == tf.REASON_TURN_TIMEOUT


def test_every_sentence_names_the_cause_instead_of_the_shared_fallback():
    """The old text claimed a timeout for every failure; that is what changed."""
    for reason, sentence in tf.SENTENCES.items():
        assert sentence.strip(), reason
    assert (
        tf.SENTENCES[tf.REASON_HOME_READ_ONLY] != tf.SENTENCES[tf.REASON_TURN_TIMEOUT]
    )
    assert "schreibgeschützt" in tf.SENTENCES[tf.REASON_HOME_READ_ONLY]
    assert "Speicher" in tf.SENTENCES[tf.REASON_DISK_FULL]


def test_the_read_only_sentence_says_what_the_operator_can_do():
    sentence = tf.failure_sentence(tf.REASON_HOME_READ_ONLY)
    assert "schreibgeschützt" in sentence
    assert "prüfe" in sentence.lower()


def test_the_detail_line_is_single_line_and_bounded():
    detail = tf.failure_detail("x", "line one\nline two " + "y" * 500)
    assert "\n" not in detail
    assert len(detail) <= 200
    assert detail.endswith("…")


def test_the_log_line_carries_the_classified_reason_and_the_detail():
    line = tf.failure_log_line(
        "exit_1",
        chat_id="c-1",
        source="subprocess",
        detail="[Errno 30] Read-only file system: /home/pib/.hermes/installs/x/.install.lock",
    )
    assert line.startswith("[PERF_TRACE] HERMES_TURN_FAILED chat=c-1")
    assert f"reason={tf.REASON_HOME_READ_ONLY}" in line
    assert "source=subprocess" in line
    assert "Read-only file system" in line


def test_no_sentence_contains_a_path_or_a_secret():
    for sentence in tf.SENTENCES.values():
        assert "/home/pib" not in sentence
        assert "/var/lib" not in sentence
        assert not re.search(r"(?i)bearer|token=|api[_-]?key=", sentence)
