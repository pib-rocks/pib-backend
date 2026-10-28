"""PR-2084: registry pull vs local compose build during backend update."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
HELPER_PATH = REPO_ROOT / "setup" / "update_backend_images.py"
WORKFLOW_PATH = REPO_ROOT / ".github" / "workflows" / "docker-images.yml"
COMPOSE_PATH = REPO_ROOT / "docker-compose.yaml"


def _load_helper():
    spec = importlib.util.spec_from_file_location("update_backend_images", HELPER_PATH)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


helper = _load_helper()


def test_registry_pull_enabled_by_default():
    assert helper.registry_pull_enabled({}) is True
    assert helper.registry_pull_enabled({"PIB_USE_REGISTRY_IMAGES": "1"}) is True


@pytest.mark.parametrize(
    "value",
    ["0", "false", "no", "off", "FALSE"],
)
def test_registry_pull_disabled(value: str):
    assert helper.registry_pull_enabled({"PIB_USE_REGISTRY_IMAGES": value}) is False


def test_deploy_strategy_table():
    assert (
        helper.deploy_strategy(registry_enabled=False, pull_succeeded=None) == "build"
    )
    assert helper.deploy_strategy(registry_enabled=True, pull_succeeded=True) == "pull"
    assert (
        helper.deploy_strategy(registry_enabled=True, pull_succeeded=False)
        == "build_after_pull_failed"
    )


def test_resolve_image_tag_prefers_explicit_env():
    assert helper.resolve_image_tag("v0.6.4", {"PIB_IMAGE_TAG": "custom"}) == "custom"


def test_resolve_image_tag_uses_app_version():
    assert helper.resolve_image_tag("v0.6.4", {}) == "v0.6.4"
    assert helper.resolve_image_tag("develop", {}) == "develop"


def test_log_summary_matches_strategy():
    assert helper.log_summary_for_strategy("pull") == helper.LOG_PULL_SUCCESS
    assert (
        helper.log_summary_for_strategy("build_after_pull_failed")
        == helper.LOG_BUILD_AFTER_PULL_FAILED
    )
    assert helper.log_summary_for_strategy("build") == helper.LOG_BUILD_NO_REGISTRY


def test_compose_image_names_use_prefix_and_tag():
    text = COMPOSE_PATH.read_text(encoding="utf-8")
    for service in helper.BACKEND_REGISTRY_SERVICES:
        needle = (
            f"${{PIB_IMAGE_PREFIX:-ghcr.io/pib-rocks/pib-backend}}-{service}:"
            "${PIB_IMAGE_TAG:-develop}"
        )
        assert needle in text, service


def test_workflow_matrix_matches_compose_build_specs():
    yaml = pytest.importorskip("yaml")
    workflow = yaml.safe_load(WORKFLOW_PATH.read_text(encoding="utf-8"))
    matrix = workflow["jobs"]["build-and-push"]["strategy"]["matrix"]["include"]
    workflow_services = {entry["service"]: entry for entry in matrix}
    assert set(workflow_services) == set(helper.BACKEND_REGISTRY_SERVICES)
    for service, context, dockerfile in helper.BACKEND_IMAGE_BUILD_SPECS:
        entry = workflow_services[service]
        assert entry["context"] == context
        assert entry["dockerfile"] == dockerfile


def test_workflow_yaml_parses():
    yaml = pytest.importorskip("yaml")
    yaml.safe_load(WORKFLOW_PATH.read_text(encoding="utf-8"))


def test_workflow_uses_arm64_runner_and_permissions():
    text = WORKFLOW_PATH.read_text(encoding="utf-8")
    assert "runs-on: ubuntu-24.04-arm" in text
    assert "packages: write" in text
    assert "fail-fast: true" in text


def test_compose_build_sections_match_specs():
    yaml = pytest.importorskip("yaml")
    compose = yaml.safe_load(COMPOSE_PATH.read_text(encoding="utf-8"))
    services = compose["services"]
    for service, context, dockerfile in helper.BACKEND_IMAGE_BUILD_SPECS:
        build = services[service]["build"]
        assert build["context"] == ("." if context == "." else f"./{context}")
        assert build["dockerfile"] == f"./{dockerfile}"
