"""Unit coverage for Marimo notebook templates in marimo_service."""

from service.marimo_service import DEFAULT_DEMO_NOTEBOOK, EMPTY_NOTEBOOK


def test_default_demo_notebook_uses_clean_pib_sdk_import():
    """Demo must import Pib directly; system Python provides pib_sdk via setup."""
    assert "from pib_sdk import Pib" in DEFAULT_DEMO_NOTEBOOK
    assert "pib = Pib()" in DEFAULT_DEMO_NOTEBOOK
    assert "sys.path" not in DEFAULT_DEMO_NOTEBOOK
    assert "import glob" not in DEFAULT_DEMO_NOTEBOOK
    assert "/usr/local/lib/python3.*/dist-packages" not in DEFAULT_DEMO_NOTEBOOK
    assert "from pib_sdk import Write as Pib" not in DEFAULT_DEMO_NOTEBOOK

    import_idx = DEFAULT_DEMO_NOTEBOOK.index("from pib_sdk import Pib")
    instantiate_idx = DEFAULT_DEMO_NOTEBOOK.index("pib = Pib()")
    assert import_idx < instantiate_idx


def test_empty_notebook_is_minimal_starter():
    assert "import marimo" in EMPTY_NOTEBOOK
    assert "from pib_sdk" not in EMPTY_NOTEBOOK
    assert 'def __():\n    return' in EMPTY_NOTEBOOK
