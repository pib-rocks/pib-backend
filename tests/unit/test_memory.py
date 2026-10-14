"""MEMORY.md consolidation: old entries become one stable section."""

from pib_hermes_config.memory import (
    ENTRY_DELIMITER,
    STABLE_HEADING,
    consolidate_due_profiles,
    consolidate_memory_file,
    consolidate_text,
    parse_entries,
)


def _entries(*parts: str) -> str:
    return ENTRY_DELIMITER.join(parts)


def test_old_entries_become_a_stable_section_and_recent_ones_stay_verbatim():
    original = _entries(
        "The garden is on the left.",
        "The kettle is blue.",
        "keep me, exactly.",
    )
    seen = {}

    def summarize(material: str) -> str:
        seen["material"] = material
        return "Garden left, kettle blue."

    result = consolidate_text(original, summarize, keep_recent=1)
    assert result.action == "consolidated"
    assert "The garden is on the left." in seen["material"]
    assert "The kettle is blue." in seen["material"]
    assert "keep me, exactly." not in seen["material"]

    entries = parse_entries(result.text)
    assert entries[0] == STABLE_HEADING + "\nGarden left, kettle blue."
    assert entries[1:] == ["keep me, exactly."]
    assert "The garden is on the left." not in result.text
    assert len(result.text) < len(original)


def test_a_previous_stable_section_is_folded_in_with_the_old_entries():
    original = _entries(
        STABLE_HEADING + "\nAlready known.",
        "A new old fact about the door.",
        "still recent",
    )
    seen = {}

    def summarize(material: str) -> str:
        seen["material"] = material
        return "Known, and the door."

    result = consolidate_text(original, summarize, keep_recent=1)
    assert "Already known." in seen["material"]
    assert "A new old fact about the door." in seen["material"]
    assert "still recent" not in seen["material"]
    assert parse_entries(result.text)[-1] == "still recent"


def test_recent_entries_alone_are_left_byte_for_byte():
    original = _entries("only recent", "also recent")
    called = {"n": 0}

    def summarize(_material: str) -> str:
        called["n"] += 1
        return "should not be used"

    result = consolidate_text(original, summarize, keep_recent=8)
    assert result.action == "unchanged"
    assert result.text == original
    assert called["n"] == 0


def test_a_summary_that_does_not_shrink_the_file_is_refused():
    original = _entries("alpha entry", "beta entry", "keep")

    result = consolidate_text(original, lambda _material: original, keep_recent=1)
    assert result.action == "unchanged"
    assert result.text == original


def test_a_missing_summary_leaves_the_file_alone(tmp_path):
    path = tmp_path / "pib_one" / "memories" / "MEMORY.md"
    path.parent.mkdir(parents=True)
    original = _entries("old enough to fold", "keep this")
    path.write_text(original, encoding="utf-8")

    status = consolidate_memory_file(
        str(path),
        lambda _material: None,
        now=1_000,
        interval=0,
        keep_recent=1,
    )
    assert status == "blocked"
    assert path.read_text(encoding="utf-8") == original
    assert not (path.parent / ".last_consolidation").exists()


def test_consolidation_waits_for_the_interval_then_runs_again(tmp_path):
    root = tmp_path / "profiles"
    path = root / "pib_one" / "memories" / "MEMORY.md"
    path.parent.mkdir(parents=True)
    path.write_text(
        _entries("old fact one", "old fact two", "recent fact"),
        encoding="utf-8",
    )
    calls = {"n": 0}

    def summarize(_material: str) -> str:
        calls["n"] += 1
        return "short"

    first = consolidate_due_profiles(
        summarize,
        now=10_000,
        profiles_root=str(root),
        interval=100,
        keep_recent=1,
    )
    assert list(first.values()) == ["consolidated"]
    assert calls["n"] == 1
    stored = path.read_text(encoding="utf-8")
    assert parse_entries(stored)[-1] == "recent fact"

    path.write_text(
        stored + ENTRY_DELIMITER + "another old fact" + ENTRY_DELIMITER + "newest",
        encoding="utf-8",
    )
    second = consolidate_due_profiles(
        summarize,
        now=10_050,
        profiles_root=str(root),
        interval=100,
        keep_recent=1,
    )
    assert list(second.values()) == ["skipped"]
    assert calls["n"] == 1
    assert "another old fact" in path.read_text(encoding="utf-8")

    third = consolidate_due_profiles(
        summarize,
        now=10_100,
        profiles_root=str(root),
        interval=100,
        keep_recent=1,
    )
    assert list(third.values()) == ["consolidated"]
    assert calls["n"] == 2
    entries = parse_entries(path.read_text(encoding="utf-8"))
    assert entries[-1] == "newest"
    assert "another old fact" not in path.read_text(encoding="utf-8")
