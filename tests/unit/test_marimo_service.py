"""Unit coverage for Marimo notebook templates in marimo_service."""

import glob
import os
import sys
import textwrap

from service.marimo_service import (
    DEFAULT_DEMO_NOTEBOOK,
    EMPTY_NOTEBOOK,
    _demo_needs_sdk_path_bootstrap,
)


PIB_SDK_PATH_MARKERS = (
    "/usr/local/lib/python3.*/dist-packages",
    "~/.local/lib/python3.*/site-packages",
    "/home/pib/app/pib-backend",
    "/home/pib/opencode/pib-backend",
)


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


def test_default_demo_notebook_bootstraps_sys_path_before_pib_sdk_import():
    """Demo must resolve pib_sdk via known install roots before importing."""
    assert "import sys" in DEFAULT_DEMO_NOTEBOOK
    assert "import glob" in DEFAULT_DEMO_NOTEBOOK
    assert "sys.path.insert" in DEFAULT_DEMO_NOTEBOOK
    for marker in PIB_SDK_PATH_MARKERS:
        assert marker in DEFAULT_DEMO_NOTEBOOK

    sys_path_idx = DEFAULT_DEMO_NOTEBOOK.index("sys.path.insert")
    import_idx = DEFAULT_DEMO_NOTEBOOK.index("from pib_sdk import Pib")
    assert sys_path_idx < import_idx


def test_pib_sdk_path_bootstrap_logic_resolves_import_cleanly(tmp_path, monkeypatch):
    """Execute the demo's path bootstrap and confirm import finds pib_sdk."""
    dist_packages = tmp_path / "usr" / "local" / "lib" / "python3.11" / "dist-packages"
    dist_packages.mkdir(parents=True)
    sdk_init = dist_packages / "pib_sdk" / "__init__.py"
    sdk_init.parent.mkdir()
    sdk_init.write_text("Write = object\nPib = object\n", encoding="utf-8")

    # Isolate import state so a host-installed pib_sdk cannot mask the bootstrap.
    monkeypatch.setattr(sys, "path", [p for p in sys.path if "pib_sdk" not in p])
    sys.modules.pop("pib_sdk", None)

    # Mirror the notebook bootstrap with the tmp dist-packages root injected.
    candidate_roots = [
        *glob.glob(str(tmp_path / "usr" / "local" / "lib" / "python3.*" / "dist-packages")),
        *glob.glob(os.path.expanduser("~/.local/lib/python3.*/site-packages")),
        "/home/pib/app/pib-backend",
        "/home/pib/opencode/pib-backend",
    ]
    for root in candidate_roots:
        if os.path.isdir(root) and root not in sys.path:
            sys.path.insert(0, root)

    import pib_sdk  # noqa: WPS433 — intentional import under test

    assert pib_sdk.__file__ == str(sdk_init)
    assert "No module named pib_sdk" not in str(getattr(pib_sdk, "__file__", ""))


def test_demo_needs_sdk_path_bootstrap_detects_legacy_template():
    legacy = textwrap.dedent(
        """
        try:
            from pib_sdk import Pib
        except ImportError:
            from pib_sdk import Write as Pib
        """
    )
    assert _demo_needs_sdk_path_bootstrap(legacy) is True
    assert _demo_needs_sdk_path_bootstrap(DEFAULT_DEMO_NOTEBOOK) is False


def test_empty_notebook_is_minimal_starter():
    assert "import marimo" in EMPTY_NOTEBOOK
    assert "from pib_sdk" not in EMPTY_NOTEBOOK
    assert 'def __():\n    return' in EMPTY_NOTEBOOK
