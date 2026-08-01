"""Unit coverage for Marimo notebook templates in marimo_service."""

from service.marimo_service import DEFAULT_DEMO_NOTEBOOK, EMPTY_NOTEBOOK


def test_default_demo_notebook_imports_pib_with_write_fallback():
    """Demo must accept both legacy Pib and current Write SDK exports."""
    assert "from pib_sdk import Pib" in DEFAULT_DEMO_NOTEBOOK
    assert "except ImportError:" in DEFAULT_DEMO_NOTEBOOK
    assert "from pib_sdk import Write as Pib" in DEFAULT_DEMO_NOTEBOOK
    # Outer handler must not be the only path for missing Pib (avoids noisy
    # "cannot import name Pib" when Write is available).
    pib_import_idx = DEFAULT_DEMO_NOTEBOOK.index("from pib_sdk import Pib")
    write_fallback_idx = DEFAULT_DEMO_NOTEBOOK.index(
        "from pib_sdk import Write as Pib"
    )
    assert pib_import_idx < write_fallback_idx


def test_empty_notebook_is_minimal_starter():
    assert "import marimo" in EMPTY_NOTEBOOK
    assert "from pib_sdk" not in EMPTY_NOTEBOOK
    assert 'def __():\n    return' in EMPTY_NOTEBOOK
