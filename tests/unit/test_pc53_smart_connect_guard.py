"""PC-53: the live E2E suite must never overwrite the robot's real cloud token.

The defect this guards: a live run POSTed a placeholder to
``/system/smart-connect`` on the real robot, which stores it in the key store.
The operator's real pib.Cloud token was replaced, every cloud turn answered
``401`` and fell back to the local model (measured: 58.7 s per turn), and the
robot stayed that way until the token was re-entered by hand.

These tests drive the real helper module with a stubbed HTTP layer, so they
prove the four behaviours without a robot:

1. a stored token is never overwritten,
2. an empty store gets the placeholder and gets it removed again,
3. the removal happens even when the test body raises,
4. a stored token is never deleted by cleanup,

plus a source scan that the endpoint is no longer written to directly from a
test file.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
E2E_DIR = REPO_ROOT / "tests" / "e2e"
HELPER_PATH = E2E_DIR / "smart_connect_token.py"
MODULE_NAME = "pc53_smart_connect_token_under_test"


def _load_helper():
    loaded = sys.modules.get(MODULE_NAME)
    if loaded is not None:
        return loaded
    spec = importlib.util.spec_from_file_location(MODULE_NAME, HELPER_PATH)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[MODULE_NAME] = module
    spec.loader.exec_module(module)
    return module


helper = _load_helper()

API = "http://192.168.1.172/api"


class _Response:
    def __init__(self, payload, status_code=200):
        self._payload = payload
        self.status_code = status_code

    def json(self):
        return self._payload

    def raise_for_status(self):
        if self.status_code >= 400:
            raise helper.requests.HTTPError(f"status {self.status_code}")


class _FakeRequests:
    """Stands in for the module's ``requests`` and records what was asked."""

    HTTPError = Exception
    RequestException = Exception

    def __init__(self, providers):
        self.providers = providers
        self.calls: list[tuple[str, str]] = []
        self.post_bodies: list[dict] = []

    def get(self, url, timeout=None):
        self.calls.append(("GET", url))
        return _Response({"providers": list(self.providers)})

    def post(self, url, json=None, timeout=None):
        self.calls.append(("POST", url))
        self.post_bodies.append(json or {})
        return _Response({"successful": True, "credentialRef": "provider-4"})

    def delete(self, url, timeout=None):
        self.calls.append(("DELETE", url))
        return _Response("", status_code=204)

    def methods(self):
        return [method for method, _ in self.calls]


def _providers(cloud_ref):
    return [
        {"id": 1, "name": "Google", "credentialRef": "provider-1"},
        {"id": 4, "name": "pib.Cloud", "credentialRef": cloud_ref},
        {"id": 5, "name": "Local", "credentialRef": None},
    ]


@pytest.fixture
def fake(monkeypatch):
    def install(cloud_ref):
        stub = _FakeRequests(_providers(cloud_ref))
        monkeypatch.setattr(helper, "requests", stub)
        return stub

    return install


class _NoNetwork:
    """Fails loudly if a test forgets to install its own stub.

    Every test in this module must be hermetic: the CI runner cannot reach the
    robot, and a test that quietly calls the real API passes on the developer
    host and fails in CI with a connection timeout.
    """

    def _refuse(self, *args, **kwargs):
        raise AssertionError("this test must install an HTTP stub")

    get = post = delete = _refuse


@pytest.fixture(autouse=True)
def _forbid_real_network(monkeypatch):
    monkeypatch.setattr(helper, "requests", _NoNetwork())


def test_cloud_credential_ref_reads_the_provider_document(fake):
    fake("provider-4")

    assert helper.cloud_credential_ref(API) == "provider-4"
    assert helper.token_is_stored(API) is True


def test_an_empty_store_reports_no_token(fake):
    fake(None)

    assert helper.cloud_credential_ref(API) is None
    assert helper.token_is_stored(API) is False


def test_a_stored_token_is_never_overwritten(fake):
    stub = fake("provider-4")

    created = helper.ensure_smart_connect_token(API)

    assert created is False
    assert stub.methods() == ["GET"]


def test_an_empty_store_gets_the_placeholder(fake):
    stub = fake(None)

    created = helper.ensure_smart_connect_token(API)

    assert created is True
    assert stub.methods() == ["GET", "POST"]
    assert stub.post_bodies == [{"token": helper.PLACEHOLDER_TOKEN}]
    assert stub.calls[-1][1] == f"{API}/system/smart-connect"


def test_the_placeholder_is_removed_again(fake):
    stub = fake(None)

    with helper.smart_connect_available(API) as created:
        assert created is True

    assert stub.methods() == ["GET", "POST", "DELETE"]


def test_cleanup_runs_even_when_the_body_raises(fake):
    stub = fake(None)

    with pytest.raises(RuntimeError):
        with helper.smart_connect_available(API):
            raise RuntimeError("turn failed")

    assert stub.methods() == ["GET", "POST", "DELETE"]


def test_a_real_token_is_not_deleted_by_cleanup(fake):
    stub = fake("provider-4")

    with helper.smart_connect_available(API) as created:
        assert created is False

    assert stub.methods() == ["GET"]


def test_missing_or_malformed_provider_documents_are_not_a_token(monkeypatch):
    for providers in (
        [],
        [{"name": "pib.Cloud"}],
        [{"name": "pib.Cloud", "credentialRef": ""}],
    ):
        stub = _FakeRequests(providers)
        monkeypatch.setattr(helper, "requests", stub)
        assert helper.token_is_stored(API) is False
        assert stub.methods() == ["GET"]


def test_no_e2e_test_file_writes_the_endpoint_directly():
    """The endpoint may only be used from the shared guard module."""
    offenders = []
    for path in sorted(E2E_DIR.glob("test_*.py")):
        if "/system/smart-connect" in path.read_text(encoding="utf-8"):
            offenders.append(path.name)
    assert offenders == []
