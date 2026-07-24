import io
import json
import logging
import os
import re
import zipfile
from datetime import datetime, timezone
from typing import Any, Callable, Dict, List, Optional, Tuple

from app.app import app
from schema.assistant_model_schema import assistant_models_schema
from schema.bricklet_schema import bricklets_schema
from schema.button_program_schema import button_programs_schema
from schema.camera_settings_schema import camera_settings_schema
from schema.motor_schema import motors_schema
from schema.personality_schema import personalities_schema
from schema.pose_schema import pose_schema, poses_schema
from schema.program_schema import programs_schema_without_code
from service import (
    assistant_model_service,
    bricklet_service,
    bricklet_status_service,
    button_program_service,
    camera_service,
    docker_service,
    motor_service,
    personality_service,
    pose_service,
    program_service,
    system_info_service,
)


logger = logging.getLogger(__name__)

SECRET_KEY_PATTERN = re.compile(
    r"(password|secret|token|api[_-]?key|authorization|private)",
    re.IGNORECASE,
)

# Host paths mounted into the flask container for diagnostics.
HOST_FILE_CANDIDATES: List[Tuple[str, str]] = [
    ("host-logs/setup-pib.log", "/host/pib/setup-pib.log"),
    ("host-logs/update-pib.log", "/host/pib/update-pib.log"),
    ("host-logs/nm-dispatcher.log", "/host/tmp/nm-dispatcher.log"),
    ("host-logs/nginx-error.log", "/host/var-log/nginx/error.log"),
    ("host-logs/nginx-access.log", "/host/var-log/nginx/access.log"),
    ("host-logs/syslog", "/host/var-log/syslog"),
    ("host-logs/messages", "/host/var-log/messages"),
    ("host-logs/daemon.log", "/host/var-log/daemon.log"),
    (
        "host-logs/motor_current.log",
        "/host/pib/ros_working_dir/src/motors/motor_current.log",
    ),
    ("host-logs/stereo.log", "/host/pib/ros_working_dir/src/camera/stereo.log"),
    (
        "host-logs/assistant.log",
        "/host/pib/ros_working_dir/src/voice_assistant/assistant.log",
    ),
    ("host-logs/program.log", "/host/pib/ros_working_dir/src/programs/program.log"),
    ("host-logs/display.log", "/host/pib/ros_working_dir/src/display/display.log"),
    ("config/os-release", "/host/os-release"),
    ("config/hostname", "/host/etc/hostname"),
    ("config/hosts", "/host/etc/hosts"),
    ("config/resolv.conf", "/host/etc/resolv.conf"),
    ("config/boot-firmware-config.txt", "/host/boot/firmware/config.txt"),
    ("config/boot-config.txt", "/host/boot/config.txt"),
    ("config/watchdog.conf", "/host/etc/watchdog.conf"),
    ("config/sysctl.conf", "/host/etc/sysctl.conf"),
    ("config/nginx.conf", "/host/etc/nginx/nginx.conf"),
    ("config/docker-compose.yaml", "/host/pib/app/pib-backend/docker-compose.yaml"),
    ("config/ros-audio.env", "/host/pib/app/pib-backend/ros-audio.env"),
    ("config/ros_config.sh", "/host/pib/ros_working_dir/ros_config.sh"),
    ("config/host_ip.txt", "/host/pib/app/pib-backend/pib_api/flask/host_ip.txt"),
    ("config/public_api_config.json", "/host/pib/public_api/config.json"),
    (
        "config/nm-dispatcher-99-update-ip.sh",
        "/host/etc/NetworkManager/dispatcher.d/99-update-ip.sh",
    ),
    (
        "config/docker_cleaner.service",
        "/host/etc/systemd/system/docker_cleaner.service",
    ),
]

MAX_FILE_BYTES = 2 * 1024 * 1024  # 2 MiB per host file
DOCKER_LOG_TAIL = 5000


def _json_bytes(payload: Any) -> bytes:
    return json.dumps(payload, indent=2, default=str, ensure_ascii=False).encode("utf-8")


def _safe_collect(name: str, collector: Callable[[], Any]) -> Tuple[str, Any]:
    try:
        return name, collector()
    except Exception as exc:
        logger.exception("Diagnostics collector failed: %s", name)
        return name, {"error": str(exc)}


def _redact_mapping(data: Dict[str, Any]) -> Dict[str, Any]:
    redacted: Dict[str, Any] = {}
    for key, value in data.items():
        if SECRET_KEY_PATTERN.search(str(key)):
            redacted[key] = "***REDACTED***"
        elif isinstance(value, dict):
            redacted[key] = _redact_mapping(value)
        else:
            redacted[key] = value
    return redacted


def _read_host_file(path: str, max_bytes: int = MAX_FILE_BYTES) -> Optional[bytes]:
    try:
        with open(path, "rb") as handle:
            data = handle.read(max_bytes + 1)
        if len(data) > max_bytes:
            truncated = data[:max_bytes]
            note = f"\n\n--- TRUNCATED after {max_bytes} bytes ---\n".encode("utf-8")
            return truncated + note
        return data
    except OSError:
        return None


def _write_json(archive: zipfile.ZipFile, path: str, payload: Any) -> None:
    archive.writestr(path, _json_bytes(payload))


def _write_text(archive: zipfile.ZipFile, path: str, content: str) -> None:
    archive.writestr(path, content.encode("utf-8", errors="replace"))


def _add_host_files(archive: zipfile.ZipFile) -> Dict[str, str]:
    summary: Dict[str, str] = {}
    for archive_path, host_path in HOST_FILE_CANDIDATES:
        data = _read_host_file(host_path)
        if data is None:
            summary[archive_path] = f"missing: {host_path}"
            continue
        archive.writestr(archive_path, data)
        summary[archive_path] = f"ok ({len(data)} bytes from {host_path})"
    return summary


def _add_systemd_units(archive: zipfile.ZipFile) -> List[str]:
    unit_dirs = [
        "/host/etc/systemd/system",
        "/host/lib/systemd/system",
    ]
    collected: List[str] = []
    patterns = ("pib", "ros_", "docker", "nginx", "watchdog", "cerebra", "blockly")
    for unit_dir in unit_dirs:
        if not os.path.isdir(unit_dir):
            continue
        try:
            names = sorted(os.listdir(unit_dir))
        except OSError:
            continue
        for name in names:
            lower = name.lower()
            if not any(token in lower for token in patterns):
                continue
            full = os.path.join(unit_dir, name)
            if not os.path.isfile(full):
                continue
            data = _read_host_file(full, max_bytes=200_000)
            if data is None:
                continue
            archive.writestr(f"systemd/{name}", data)
            collected.append(name)
    return collected


def _collect_git_info(archive: zipfile.ZipFile) -> Dict[str, str]:
    repos = {
        "backend": "/host/pib/app/pib-backend",
        "cerebra": "/host/pib/app/cerebra",
    }
    summary: Dict[str, str] = {}
    for name, path in repos.items():
        git_dir = os.path.join(path, ".git")
        if not os.path.isdir(git_dir) and not os.path.isfile(git_dir):
            summary[name] = f"missing: {path}"
            continue
        head = system_info_service._run_command(
            ["git", "-C", path, "rev-parse", "HEAD"]
        )
        describe = system_info_service._run_command(
            ["git", "-C", path, "describe", "--tags", "--always", "--dirty"]
        )
        status = system_info_service._run_command(
            ["git", "-C", path, "status", "-sb"]
        )
        remote = system_info_service._run_command(
            ["git", "-C", path, "remote", "-v"]
        )
        body = "\n".join(
            [
                f"## HEAD\n{(head.get('stdout') or head.get('error') or '').strip()}",
                f"## describe\n{(describe.get('stdout') or describe.get('error') or '').strip()}",
                f"## status\n{(status.get('stdout') or status.get('error') or '').strip()}",
                f"## remotes\n{(remote.get('stdout') or remote.get('error') or '').strip()}",
            ]
        )
        _write_text(archive, f"versions/{name}-git.txt", body + "\n")
        summary[name] = "ok"
    return summary


def _collect_database_exports() -> Dict[str, Any]:
    exports: Dict[str, Any] = {}

    bricklets = bricklet_service.get_all_bricklets()
    exports["bricklets"] = bricklets_schema.dump(bricklets)

    motors = motor_service.get_all_motors()
    exports["motors"] = motors_schema.dump(motors)

    try:
        camera = camera_service.get_camera_settings()
        exports["cameraSettings"] = camera_settings_schema.dump(camera)
    except Exception as exc:
        exports["cameraSettings"] = {"error": str(exc)}

    programs = program_service.get_all_programs()
    exports["programs"] = programs_schema_without_code.dump(programs)

    button_programs = button_program_service.get_all_button_programs()
    exports["buttonPrograms"] = button_programs_schema.dump(button_programs)

    poses = pose_service.get_all_poses()
    exports["poses"] = poses_schema.dump(poses)
    exports["posesDetailed"] = [
        pose_schema.dump(pose_service.get_pose(pose.pose_id)) for pose in poses
    ]

    personalities = personality_service.get_all_personalities()
    exports["personalities"] = personalities_schema.dump(personalities)

    models = assistant_model_service.get_all_assistant_models()
    exports["assistantModels"] = assistant_models_schema.dump(models)

    return exports


def _collect_docker_bundle(archive: zipfile.ZipFile) -> Dict[str, Any]:
    summary: Dict[str, Any] = {}

    _, containers = _safe_collect("containers", docker_service.list_containers)
    _write_json(archive, "docker/containers.json", {"containers": containers})
    summary["containers"] = (
        len(containers) if isinstance(containers, list) else containers
    )

    if isinstance(containers, list):
        for container in containers:
            name = container.get("name")
            if not name:
                continue
            try:
                logs = docker_service.get_container_logs(name, tail=DOCKER_LOG_TAIL)
                _write_text(archive, f"docker/logs/{name}.log", logs.get("logs", ""))
            except Exception as exc:
                _write_text(
                    archive,
                    f"docker/logs/{name}.log",
                    f"Unable to collect logs: {exc}\n",
                )
            try:
                inspect = docker_service.get_container_inspect(name)
                _write_json(archive, f"docker/inspect/{name}.json", inspect)
            except Exception as exc:
                _write_json(
                    archive,
                    f"docker/inspect/{name}.json",
                    {"error": str(exc)},
                )
            try:
                stats = docker_service.get_container_stats_snapshot(name)
                if stats is not None:
                    _write_json(archive, f"docker/stats/{name}.json", stats)
            except Exception as exc:
                _write_json(
                    archive,
                    f"docker/stats/{name}.json",
                    {"error": str(exc)},
                )

    for key, collector in (
        ("info", docker_service.get_docker_info),
        ("version", docker_service.get_docker_version),
        ("images", docker_service.list_images),
        ("networks", docker_service.list_networks),
        ("volumes", docker_service.list_volumes),
    ):
        name, payload = _safe_collect(key, collector)
        _write_json(archive, f"docker/{name}.json", payload)
        summary[name] = "ok" if "error" not in (payload or {}) else payload

    return summary


def _collect_flask_runtime() -> Dict[str, Any]:
    return {
        "flaskConfig": _redact_mapping(dict(app.config)),
        "processEnvironment": _redact_mapping(dict(os.environ)),
        "cwd": os.getcwd(),
        "pid": os.getpid(),
    }


def _build_readme() -> str:
    return """pib / Cerebra diagnostics archive
=================================

This archive was generated from Cerebra (System → Download diagnostics ZIP).
Send it to pib support when investigating robot issues.

Contents overview
-----------------
- meta.json / README.txt
- system/              Host & runtime information
- docker/              Containers, inspect, stats, images, networks, volumes, logs
- bricklets/           Live Tinkerforge status + DB configuration
- database/            Motors, camera, programs, poses, personalities (no chat messages)
- config/              Host configuration files (when mounted)
- host-logs/           Setup/update/nginx/ROS logs (when present)
- systemd/             Relevant unit files (when mounted)
- commands/            Output of diagnostic shell commands inside flask-app

Secrets (passwords, tokens, chat messages) are excluded or redacted.
"""


def build_diagnostics_zip() -> bytes:
    buffer = io.BytesIO()
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    collection_errors: List[str] = []

    with zipfile.ZipFile(buffer, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        _write_text(archive, "README.txt", _build_readme())

        # System information
        _, system_info = _safe_collect("systemInfo", system_info_service.get_system_info)
        _write_json(archive, "system/system-info.json", system_info)
        if isinstance(system_info, dict) and "error" in system_info:
            collection_errors.append(f"systemInfo: {system_info['error']}")

        _, commands = _safe_collect(
            "commands", system_info_service.collect_runtime_commands
        )
        _write_json(archive, "commands/runtime-commands.json", commands)
        if isinstance(commands, dict):
            for name, result in commands.items():
                stdout = (result or {}).get("stdout") or ""
                stderr = (result or {}).get("stderr") or ""
                error = (result or {}).get("error")
                body = stdout
                if stderr:
                    body += ("\n\n[stderr]\n" + stderr) if body else stderr
                if error:
                    body += ("\n\n[error]\n" + error) if body else error
                if body:
                    _write_text(archive, f"commands/{name}.txt", body)

        # Docker
        try:
            docker_summary = _collect_docker_bundle(archive)
        except Exception as exc:
            docker_summary = {"error": str(exc)}
            collection_errors.append(f"docker: {exc}")
            _write_json(archive, "docker/error.json", docker_summary)

        # Bricklets
        _, bricklets_live = _safe_collect(
            "brickletsLive", bricklet_status_service.get_bricklets_status
        )
        _write_json(archive, "bricklets/live-status.json", bricklets_live)

        # Database exports
        _, db_exports = _safe_collect("database", _collect_database_exports)
        if isinstance(db_exports, dict) and "error" not in db_exports:
            for key, value in db_exports.items():
                _write_json(archive, f"database/{key}.json", value)
        else:
            _write_json(archive, "database/error.json", db_exports)
            collection_errors.append(f"database: {db_exports}")

        # Flask runtime (redacted)
        _, runtime = _safe_collect("flaskRuntime", _collect_flask_runtime)
        _write_json(archive, "system/flask-runtime.json", runtime)

        # Host files & systemd units
        host_files = _add_host_files(archive)
        _write_json(archive, "config/host-files-summary.json", host_files)
        systemd_units = _add_systemd_units(archive)
        _write_json(archive, "systemd/collected-units.json", {"units": systemd_units})
        git_summary = _collect_git_info(archive)
        _write_json(archive, "versions/git-summary.json", git_summary)

        meta = {
            "generatedAt": timestamp,
            "source": "cerebra-system-diagnostics",
            "version": 2,
            "dockerSummary": docker_summary,
            "hostFiles": host_files,
            "systemdUnits": systemd_units,
            "git": git_summary,
            "collectionErrors": collection_errors,
            "notes": [
                "Chat messages are intentionally excluded.",
                "Secret-like environment and config values are redacted.",
                "Host files are included only when the corresponding volume mounts exist.",
            ],
        }
        _write_json(archive, "meta.json", meta)

    return buffer.getvalue()


def diagnostics_filename() -> str:
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S")
    return f"pib-diagnostics-{timestamp}.zip"
