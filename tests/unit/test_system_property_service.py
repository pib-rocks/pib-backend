import logging
from pathlib import Path
import re

import pytest
from click.testing import CliRunner

import commands
from app.app import db
from commands import reconcile_system_properties
from model.system_property_model import SystemProperty
from service.system_property_service import (
    ALLOWED_HARDWARE_VARIANTS,
    HARDWARE_VARIANT_KEY,
    SOFTWARE_VERSION_KEY,
    get_variant,
    set_property,
)


def test_registry_rejects_unknown_key(app_ctx):
    with pytest.raises(ValueError, match="Unknown system property"):
        set_property("secret.token", "do-not-store-this", "default")


def test_registry_rejects_invalid_variant(app_ctx):
    with pytest.raises(ValueError, match="Invalid value"):
        set_property(HARDWARE_VARIANT_KEY, "pib9000", "default")


def test_registry_rejects_wrong_type(app_ctx):
    with pytest.raises(ValueError, match="requires str"):
        set_property(HARDWARE_VARIANT_KEY, 5, "default")


def test_get_variant_falls_back_and_warns(app_ctx, caplog):
    db.session.query(SystemProperty).filter_by(key=HARDWARE_VARIANT_KEY).delete()
    caplog.set_level(logging.WARNING)

    assert get_variant() == "pib5edu"
    assert "falling back to pib5edu" in caplog.text


def test_get_variant_falls_back_for_unknown_stored_value(app_ctx, caplog):
    system_property = db.session.get(SystemProperty, HARDWARE_VARIANT_KEY)
    system_property.value = "unknown-robot"
    db.session.flush()
    caplog.set_level(logging.WARNING)

    assert get_variant() == "pib5edu"
    assert "Unknown stored hardware variant" in caplog.text


def _run_reconciliation(app, monkeypatch, resolved_variant, resolved_source):
    monkeypatch.setattr(
        commands,
        "resolve_variant_and_source",
        lambda: (resolved_variant, resolved_source),
    )
    monkeypatch.setattr(commands, "read_app_version", lambda: "v-test")
    result = CliRunner().invoke(reconcile_system_properties, [])
    assert result.exit_code == 0, result.output


def test_reconciliation_writes_missing_variant(app, monkeypatch):
    with app.app_context():
        db.session.query(SystemProperty).filter_by(key=HARDWARE_VARIANT_KEY).delete()
        db.session.commit()

        _run_reconciliation(app, monkeypatch, "pib4edu", "file")

        variant = db.session.get(SystemProperty, HARDWARE_VARIANT_KEY)
        version = db.session.get(SystemProperty, SOFTWARE_VERSION_KEY)
        assert (variant.value, variant.source) == ("pib4edu", "file")
        assert version.value == "v-test"


def test_reconciliation_updates_different_default(app, monkeypatch, caplog):
    with app.app_context():
        set_property(HARDWARE_VARIANT_KEY, "pib5edu", "default")
        db.session.commit()
        caplog.set_level(logging.INFO)

        _run_reconciliation(app, monkeypatch, "pib4edu", "environment")

        variant = db.session.get(SystemProperty, HARDWARE_VARIANT_KEY)
        assert (variant.value, variant.source) == ("pib4edu", "environment")
        assert "Updating default hardware variant" in caplog.text


def test_reconciliation_warns_without_overwriting_other_source(
    app, monkeypatch, caplog
):
    with app.app_context():
        set_property(HARDWARE_VARIANT_KEY, "pib5edu", "file")
        db.session.commit()
        caplog.set_level(logging.WARNING)

        _run_reconciliation(app, monkeypatch, "pib4edu", "environment")

        variant = db.session.get(SystemProperty, HARDWARE_VARIANT_KEY)
        assert (variant.value, variant.source) == ("pib5edu", "file")
        assert "keeping the database value" in caplog.text


def test_installer_variant_whitelist_matches_registry():
    resolver = (
        Path(__file__).resolve().parents[2]
        / "setup"
        / "installation_scripts"
        / "resolve_hardware_variant.sh"
    ).read_text(encoding="utf-8")
    shell_variants = set(re.findall(r"\bpib(?:4|5)[a-z]+\b", resolver))

    assert shell_variants == set(ALLOWED_HARDWARE_VARIANTS)
