"""
PR-1619 E2E: single build-injected system version under the Cerebra logo.

Asserts (against the live/deployed Pi):
  1. Exactly ONE version footer element `[data-test='TXT_AppVersion']` exists in
     the left nav shell, and its text is a well-formed version (v<major>.<minor>.<patch>).
  2. The backend `GET /api/version` returns a consistent, well-formed value.
  3. The single-value rule: footer and backend endpoint agree (same version).

RED/GREEN: this test asserts a real well-formed version. On a build without the
feature it fails (placeholder 'dev' or absent element) - i.e. it goes RED before
the fix is deployed and GREEN after.
"""

import os
import re
import requests
import pytest
from playwright.sync_api import sync_playwright, Page, expect

BASE_URL = os.getenv("PIB_ROBOT_URL", "http://192.168.1.28")
API_URL = os.getenv("PIB_API_URL", f"{BASE_URL}/api")
VERSION_RE = re.compile(r"^v?\d+\.\d+\.\d+")


@pytest.fixture(scope="function")
def page():
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        context = browser.new_context(viewport={"width": 1440, "height": 900})
        page = context.new_page()
        page.goto(f"{BASE_URL}/joint-control/head", wait_until="domcontentloaded")
        # Shell (nav) renders after auth; the version footer is inside the shell.
        page.wait_for_selector("#program-nav", timeout=15000)
        yield page
        context.close()
        browser.close()


class TestVersionFooterE2E:

    def test_exactly_one_well_formed_version_footer(self, page: Page) -> None:
        """The footer exists exactly once and is a well-formed version string."""
        loc = page.locator("[data-test='TXT_AppVersion']")
        n = loc.count()
        assert n == 1, f"expected exactly 1 version footer, found {n}"
        expect(loc).to_be_visible(timeout=15000)

        text = loc.inner_text().strip()
        assert VERSION_RE.match(text), f"footer version not well-formed: {text!r}"
        # not the local placeholder
        assert text.lower() != "dev", f"footer still shows dev placeholder: {text!r}"

    def test_backend_version_endpoint_consistent(self) -> None:
        """GET /api/version is reachable and returns a well-formed version."""
        resp = requests.get(f"{API_URL}/version", timeout=15)
        assert resp.ok, f"GET /api/version -> {resp.status_code}"
        data = resp.json()
        ver = data.get("version")
        assert isinstance(ver, str) and ver.strip(), f"empty version: {data}"
        assert VERSION_RE.match(
            ver.strip()
        ), f"backend version not well-formed: {ver!r}"

    def test_single_value_footer_matches_backend(self, page: Page) -> None:
        """The one footer value equals the one backend value (single source of truth)."""
        footer = page.locator("[data-test='TXT_AppVersion']").inner_text().strip()
        resp = requests.get(f"{API_URL}/version", timeout=15)
        assert resp.ok
        backend = resp.json().get("version", "").strip()

        # strip an optional leading 'v' on both sides before comparing
        norm = lambda s: s.lstrip("v") if s.startswith("v") else s  # noqa: E731
        assert norm(footer) == norm(
            backend
        ), f"footer {footer!r} != backend {backend!r}"
