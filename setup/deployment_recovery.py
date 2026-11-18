#!/usr/bin/env python3
"""Persist Compose deployment inputs and recover missing PIB containers."""

from __future__ import annotations

import argparse
import fcntl
import json
import os
import re
import subprocess
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

UPDATE_DIR = Path(os.environ.get("PIB_UPDATE_DIR", "/home/pib/app/.update"))
STATE_FILE = UPDATE_DIR / "deployments.json"
LOCK_FILE = UPDATE_DIR / "deployment.lock"
PROJECTS = {
    "pib-backend": Path(os.environ.get("PIB_BACKEND_DIR", "/home/pib/app/pib-backend")),
    "cerebra": Path(os.environ.get("PIB_CEREBRA_DIR", "/home/pib/app/cerebra")),
}
IMAGE_TAG_RE = re.compile(r"^[A-Za-z0-9_][A-Za-z0-9_.-]{0,127}$")


def _compose_base(name: str, profiles: list[str]) -> list[str]:
    directory = PROJECTS[name]
    command = [
        "docker",
        "compose",
        "--project-name",
        name,
        "--project-directory",
        str(directory),
        "--file",
        str(directory / "docker-compose.yaml"),
    ]
    for profile in profiles:
        command.extend(("--profile", profile))
    return command


def _read_compose_model(path: str, name: str) -> dict[str, Any]:
    try:
        model = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise RuntimeError(
            f"cannot read captured {name} Compose model: {error}"
        ) from error
    if not isinstance(model, dict) or not isinstance(model.get("services"), dict):
        raise RuntimeError(f"captured {name} Compose configuration has no services")
    if model.get("name") != name:
        raise RuntimeError(
            f"captured {name} Compose project name is {model.get('name')!r}, expected {name!r}"
        )
    return model


def _resolve_compose_model(
    name: str, profiles: list[str], overrides: dict[str, str]
) -> dict[str, Any]:
    environment = os.environ.copy()
    environment.update(overrides)
    result = subprocess.run(
        _compose_base(name, profiles) + ["config", "--format", "json"],
        cwd=PROJECTS[name],
        env=environment,
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode:
        detail = (result.stderr or result.stdout).strip()
        raise RuntimeError(
            f"cannot record resolved {name} Compose model: {detail or result.returncode}"
        )
    try:
        model = json.loads(result.stdout)
    except json.JSONDecodeError as error:
        raise RuntimeError(
            f"cannot parse resolved {name} Compose configuration: {error}"
        ) from error
    if not isinstance(model, dict) or not isinstance(model.get("services"), dict):
        raise RuntimeError(f"resolved {name} Compose configuration has no services")
    return model


def _model_has_tag(model: dict[str, Any], tag: str) -> bool:
    services = model.get("services")
    if not isinstance(services, dict):
        return False
    for service in services.values():
        image = service.get("image") if isinstance(service, dict) else None
        if isinstance(image, str) and image.rsplit(":", 1)[-1] == tag:
            return True
    return False


def _atomic_write(path: Path, document: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as output:
            json.dump(document, output, sort_keys=True, indent=2)
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
        os.chmod(temporary, 0o600)
        os.replace(temporary, path)
        directory_fd = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def record(args: argparse.Namespace) -> None:
    if not IMAGE_TAG_RE.fullmatch(args.backend_tag) or not IMAGE_TAG_RE.fullmatch(
        args.cerebra_tag
    ):
        raise ValueError(
            "backend and Cerebra image tags must be valid non-empty Docker tags"
        )
    if args.smart_chats not in {"0", "1"}:
        raise ValueError("smart-chats must be 0 or 1")

    backend_profiles = ["all"] + (["hermes"] if args.smart_chats == "1" else [])
    project_specs = []
    for name, tag, version, profiles, compose_config_path in (
        (
            "pib-backend",
            args.backend_tag,
            args.backend_version,
            backend_profiles,
            args.backend_compose_config,
        ),
        (
            "cerebra",
            args.cerebra_tag,
            args.cerebra_version,
            [],
            args.cerebra_compose_config,
        ),
    ):
        if compose_config_path:
            model = _read_compose_model(compose_config_path, name)
        else:
            # Older installer/updater versions call the persistent helper without
            # captured model files. Preserve that interface for rollback across
            # revisions; current callers pass models captured under their exact
            # Compose environment.
            overrides = {"PIB_IMAGE_TAG": tag, "APP_VERSION": version}
            if name == "pib-backend":
                overrides["PIB_SMART_CHATS"] = args.smart_chats
                overrides["PIB_HARDWARE_VARIANT"] = args.hardware_variant
                sdk_version_file = PROJECTS[name] / "pib_sdk_version.txt"
                if sdk_version_file.is_file():
                    overrides["PIB_SDK_VERSION"] = sdk_version_file.read_text(
                        encoding="utf-8"
                    ).strip()
            model = _resolve_compose_model(name, profiles, overrides)
            if model.get("name") != name:
                raise RuntimeError(
                    f"resolved {name} Compose project name is {model.get('name')!r}, expected {name!r}"
                )
        if not _model_has_tag(model, tag):
            raise RuntimeError(
                f"captured {name} Compose model does not reference recorded image tag {tag}"
            )
        service_environment = {
            service_name: service.get("environment", {})
            for service_name, service in model["services"].items()
            if isinstance(service, dict) and "environment" in service
        }
        project_specs.append(
            {
                "name": name,
                "projectName": name,
                "composeFile": str(PROJECTS[name] / "docker-compose.yaml"),
                "projectDirectory": str(PROJECTS[name]),
                "imageTag": tag,
                "profiles": profiles,
                "environment": service_environment,
                "composeConfig": model,
            }
        )

    _atomic_write(
        STATE_FILE,
        {
            "schemaVersion": 1,
            "updatedAt": datetime.now(timezone.utc).isoformat(),
            "projects": project_specs,
        },
    )
    print(
        f"Recorded deployment inputs for pib-backend ({args.backend_tag}) and cerebra ({args.cerebra_tag})"
    )


def _load_state() -> list[dict[str, Any]]:
    try:
        document = json.loads(STATE_FILE.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise RuntimeError(
            f"deployment state is missing: {STATE_FILE}; run the PIB installer to record it"
        ) from error
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise RuntimeError(
            f"deployment state cannot be read: {STATE_FILE}: {error}"
        ) from error
    if not isinstance(document, dict) or document.get("schemaVersion") != 1:
        raise RuntimeError(f"unsupported deployment state schema in {STATE_FILE}")
    projects = document.get("projects")
    if not isinstance(projects, list) or len(projects) != len(PROJECTS):
        raise RuntimeError(
            f"deployment state must contain exactly {', '.join(PROJECTS)}"
        )

    by_name: dict[str, dict[str, Any]] = {}
    for project in projects:
        if not isinstance(project, dict):
            raise RuntimeError("deployment state contains an invalid project entry")
        name = project.get("name")
        if not isinstance(name, str) or name not in PROJECTS or name in by_name:
            raise RuntimeError(
                f"deployment state contains an unknown or duplicate project: {name!r}"
            )
        tag = project.get("imageTag")
        profiles = project.get("profiles")
        environment = project.get("environment")
        compose_config = project.get("composeConfig")
        if not isinstance(tag, str) or not IMAGE_TAG_RE.fullmatch(tag):
            raise RuntimeError(f"deployment state has an invalid image tag for {name}")
        if not isinstance(profiles, list) or any(
            not isinstance(value, str) or not value for value in profiles
        ):
            raise RuntimeError(f"deployment state has invalid profiles for {name}")
        if not isinstance(environment, dict) or not isinstance(compose_config, dict):
            raise RuntimeError(f"deployment state has invalid environment for {name}")
        expected_environment = (
            {
                service_name: service.get("environment", {})
                for service_name, service in compose_config.get("services", {}).items()
                if isinstance(service, dict) and "environment" in service
            }
            if isinstance(compose_config.get("services"), dict)
            else None
        )
        if environment != expected_environment:
            raise RuntimeError(
                f"deployment state has incomplete environment for {name}"
            )
        expected_directory = PROJECTS[name].resolve()
        if project.get("projectName") != name or project.get("projectDirectory") != str(
            expected_directory
        ):
            raise RuntimeError(
                f"deployment state has an unexpected Compose project path/name for {name}"
            )
        if compose_config.get("name") != name or not isinstance(
            compose_config.get("services"), dict
        ):
            raise RuntimeError(
                f"deployment state has an invalid saved Compose model for {name}"
            )
        compose_file = Path(str(project.get("composeFile", "")))
        if (
            compose_file.resolve()
            != (expected_directory / "docker-compose.yaml").resolve()
            or not compose_file.is_file()
        ):
            raise RuntimeError(f"source Compose file for {name} is missing")
        if not _model_has_tag(compose_config, tag):
            raise RuntimeError(
                f"saved Compose model for {name} does not reference the recorded image tag {tag}"
            )
        by_name[name] = project
    if set(by_name) != set(PROJECTS):
        raise RuntimeError(
            f"deployment state is missing {', '.join(sorted(set(PROJECTS) - set(by_name)))}"
        )
    return [by_name[name] for name in PROJECTS]


def recover() -> None:
    UPDATE_DIR.mkdir(parents=True, exist_ok=True)
    with LOCK_FILE.open("a", encoding="utf-8") as lock:
        os.chmod(LOCK_FILE, 0o660)
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        # Read only after acquiring the lock: an updater may have replaced the
        # state while recovery was waiting for it to finish.
        projects = _load_state()
        for project in projects:
            name = project["name"]
            descriptor, compose_path = tempfile.mkstemp(
                prefix=f".pib-recovery-{name}.", suffix=".json", dir=UPDATE_DIR
            )
            try:
                with os.fdopen(descriptor, "w", encoding="utf-8") as compose_file:
                    json.dump(project["composeConfig"], compose_file, sort_keys=True)
                    compose_file.write("\n")
                os.chmod(compose_path, 0o600)
                command = [
                    "docker",
                    "compose",
                    "--project-name",
                    name,
                    "--project-directory",
                    project["projectDirectory"],
                    "--file",
                    compose_path,
                ]
                for profile in project["profiles"]:
                    command.extend(("--profile", profile))
                command.extend(
                    ("up", "-d", "--no-recreate", "--no-build", "--pull", "never")
                )
                print(
                    f"Reconciling {name} from local image tag {project['imageTag']}",
                    flush=True,
                )
                result = subprocess.run(
                    command,
                    cwd=PROJECTS[name],
                    check=False,
                )
            finally:
                try:
                    os.unlink(compose_path)
                except FileNotFoundError:
                    pass
            if result.returncode:
                raise RuntimeError(
                    f"failed to recover {name} (exit {result.returncode}); recovery did not pull or build images"
                )
    print("PIB Compose recovery completed")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="action", required=True)
    capture = subparsers.add_parser(
        "record", help="atomically record the installed Compose deployment inputs"
    )
    capture.add_argument("--backend-tag", required=True)
    capture.add_argument("--cerebra-tag", required=True)
    capture.add_argument("--backend-version", default="")
    capture.add_argument("--cerebra-version", default="")
    capture.add_argument("--hardware-variant", default="")
    capture.add_argument("--smart-chats", default="1")
    capture.add_argument("--backend-compose-config")
    capture.add_argument("--cerebra-compose-config")
    subparsers.add_parser(
        "recover",
        help="recreate missing or stopped PIB Compose services from local images",
    )
    args = parser.parse_args()
    try:
        if args.action == "record":
            record(args)
        else:
            recover()
    except (OSError, RuntimeError, ValueError) as error:
        print(f"PIB Compose recovery error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
