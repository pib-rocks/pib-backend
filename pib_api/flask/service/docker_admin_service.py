import os
import re
import socket
import json
import logging
import fcntl
import stat
from urllib.parse import quote
from pathlib import Path
from typing import Dict, Any, List, Tuple, Optional

DOCKER_SOCK = "/var/run/docker.sock"

ALLOWED_CONTAINER_PREFIXES = ("multirepo-", "pib-backend-", "cerebra-", "pib_")
ALLOWED_CONTAINER_NAMES = frozenset(
    {
        "hermes-daemon",
        "pib-blockly-server",
        "flask-app",
        "rosbridge-ws",
        "ros-camera",
        "ros-motors",
        "ros-voice-assistant",
        "ros-programs",
        "ros-display",
        "ros-audio-io",
        "matter-server",
        "homeassistant",
        "angular-app",
    }
)
_CONTAINER_NAME_PATTERN = re.compile(r"^[a-zA-Z0-9][a-zA-Z0-9_.-]*$")


def is_allowed_container_name(name: str) -> bool:
    if not name or not _CONTAINER_NAME_PATTERN.fullmatch(name):
        return False
    if name in ALLOWED_CONTAINER_NAMES:
        return True
    return any(name.startswith(prefix) for prefix in ALLOWED_CONTAINER_PREFIXES)


def _container_not_managed_response(name: str) -> Tuple[int, Dict[str, Any]]:
    return 403, {
        "status": "error",
        "message": f"Container '{name}' is not managed by pib.",
    }


# Pruning images and the build cache on an SD card takes minutes, not seconds.
# The default 10 s socket timeout made a purge that was still running look like
# a failure on 192.168.1.172 while it freed 6.3 GB in the background (PR-2099).
PURGE_TIMEOUT_SECONDS = 600.0


def _docker_request(
    method: str, path: str, payload: Optional[dict] = None, timeout: float = 10.0
) -> Tuple[int, bytes]:
    if not os.path.exists(DOCKER_SOCK):
        raise FileNotFoundError(f"Docker socket not found at {DOCKER_SOCK}")

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.settimeout(timeout)
    try:
        sock.connect(DOCKER_SOCK)
        req_lines = [
            f"{method.upper()} {path} HTTP/1.1",
            "Host: localhost",
            "Connection: close",
        ]
        if payload is not None:
            body_bytes = json.dumps(payload).encode("utf-8")
            req_lines.append("Content-Type: application/json")
            req_lines.append(f"Content-Length: {len(body_bytes)}")
            header_str = "\r\n".join(req_lines) + "\r\n\r\n"
            sock.sendall(header_str.encode("utf-8") + body_bytes)
        else:
            header_str = "\r\n".join(req_lines) + "\r\n\r\n"
            sock.sendall(header_str.encode("utf-8"))

        response = b""
        while True:
            data = sock.recv(4096)
            if not data:
                break
            response += data

        parts = response.split(b"\r\n\r\n", 1)
        if len(parts) < 2:
            return 500, b"Invalid HTTP response from Docker socket"

        header, body = parts[0], parts[1]

        first_line = header.split(b"\r\n")[0].decode("utf-8", errors="ignore")
        status_code = 200
        first_line_parts = first_line.split(" ")
        if len(first_line_parts) >= 2 and first_line_parts[1].isdigit():
            status_code = int(first_line_parts[1])

        if (
            b"Transfer-Encoding: chunked" in header
            or b"transfer-encoding: chunked" in header
        ):
            chunks = []
            pos = 0
            while pos < len(body):
                line_end = body.find(b"\r\n", pos)
                if line_end == -1:
                    break
                size_str = body[pos:line_end].split(b";")[0]
                try:
                    chunk_size = int(size_str, 16)
                except ValueError:
                    break
                if chunk_size == 0:
                    break
                chunks.append(body[line_end + 2 : line_end + 2 + chunk_size])
                pos = line_end + 2 + chunk_size + 2
            body = b"".join(chunks)

        return status_code, body
    finally:
        sock.close()


def _clean_docker_logs(raw_bytes: bytes) -> str:
    if not raw_bytes:
        return ""

    logs = []
    pos = 0
    length = len(raw_bytes)
    demux_success = False

    while pos + 8 <= length:
        stream_type = raw_bytes[pos]
        zeros = raw_bytes[pos + 1 : pos + 4]
        if stream_type in (1, 2) and zeros == b"\x00\x00\x00":
            frame_size = int.from_bytes(raw_bytes[pos + 4 : pos + 8], byteorder="big")
            if pos + 8 + frame_size <= length:
                frame_data = raw_bytes[pos + 8 : pos + 8 + frame_size]
                logs.append(frame_data.decode("utf-8", errors="replace"))
                pos += 8 + frame_size
                demux_success = True
                continue
        break

    if demux_success and pos == length:
        return "".join(logs)

    return raw_bytes.decode("utf-8", errors="replace")


def get_containers() -> List[Dict[str, Any]]:
    try:
        status_code, body = _docker_request("GET", "/containers/json?all=true")
        if status_code != 200:
            return []
        containers_raw = json.loads(body.decode("utf-8"))
        result = []
        for c in containers_raw:
            names = c.get("Names", [])
            raw_name = names[0].lstrip("/") if names else "unknown"
            state = c.get("State", "unknown")
            status_str = c.get("Status", "")

            if "unhealthy" in status_str.lower():
                health = "unhealthy"
            elif "healthy" in status_str.lower():
                health = "healthy"
            elif state == "running":
                health = "healthy"
            else:
                health = "unhealthy"

            display_status = "stopped" if state == "exited" else state

            result.append(
                {
                    "id": c.get("Id", "")[:12],
                    "name": raw_name,
                    "image": c.get("Image", ""),
                    "status": display_status,
                    "statusText": status_str,
                    "health": health,
                    "created": c.get("Created", 0),
                }
            )
        return result
    except Exception as e:
        logging.error(f"Error fetching docker containers: {e}")
        return []


def start_container(name: str) -> Tuple[int, Dict[str, Any]]:
    if not is_allowed_container_name(name):
        return _container_not_managed_response(name)
    try:
        status_code, body = _docker_request(
            "POST", f"/containers/{name}/start", timeout=20.0
        )
        if status_code in (200, 204, 304):
            return 200, {
                "status": "success",
                "message": f"Container '{name}' started successfully.",
            }
        elif status_code == 404:
            return 404, {"status": "error", "message": f"Container '{name}' not found."}
        else:
            err_msg = body.decode("utf-8", errors="ignore")
            return status_code, {
                "status": "error",
                "message": err_msg or f"Failed to start container '{name}'.",
            }
    except Exception as e:
        return 500, {"status": "error", "message": str(e)}


def stop_container(name: str) -> Tuple[int, Dict[str, Any]]:
    if not is_allowed_container_name(name):
        return _container_not_managed_response(name)
    try:
        # Use t=5 second stop timeout and 30s HTTP socket timeout
        status_code, body = _docker_request(
            "POST", f"/containers/{name}/stop?t=5", timeout=30.0
        )
        if status_code in (200, 204, 304):
            return 200, {
                "status": "success",
                "message": f"Container '{name}' stopped successfully.",
            }
        elif status_code == 404:
            return 404, {"status": "error", "message": f"Container '{name}' not found."}
        else:
            err_msg = body.decode("utf-8", errors="ignore")
            return status_code, {
                "status": "error",
                "message": err_msg or f"Failed to stop container '{name}'.",
            }
    except Exception as e:
        return 500, {"status": "error", "message": str(e)}


def restart_container(name: str) -> Tuple[int, Dict[str, Any]]:
    if not is_allowed_container_name(name):
        return _container_not_managed_response(name)
    try:
        status_code, body = _docker_request(
            "POST", f"/containers/{name}/restart?t=5", timeout=30.0
        )
        if status_code in (200, 204, 304):
            return 200, {
                "status": "success",
                "message": f"Container '{name}' restarted successfully.",
            }
        elif status_code == 404:
            return 404, {"status": "error", "message": f"Container '{name}' not found."}
        else:
            err_msg = body.decode("utf-8", errors="ignore")
            return status_code, {
                "status": "error",
                "message": err_msg or f"Failed to restart container '{name}'.",
            }
    except Exception as e:
        return 500, {"status": "error", "message": str(e)}


def get_container_logs(name: str, tail: int = 500) -> Tuple[int, Dict[str, Any]]:
    if not is_allowed_container_name(name):
        return _container_not_managed_response(name)
    try:
        status_code, body = _docker_request(
            "GET", f"/containers/{name}/logs?stdout=1&stderr=1&tail={tail}"
        )
        if status_code == 200:
            logs = _clean_docker_logs(body)
            return 200, {"status": "success", "container": name, "logs": logs}
        elif status_code == 404:
            return 404, {"status": "error", "message": f"Container '{name}' not found."}
        else:
            err_msg = body.decode("utf-8", errors="ignore")
            return status_code, {
                "status": "error",
                "message": err_msg or f"Failed to get logs for container '{name}'.",
            }
    except Exception as e:
        return 500, {"status": "error", "message": str(e)}


def clear_container_logs(name: str) -> Tuple[int, Dict[str, Any]]:
    if not is_allowed_container_name(name):
        return _container_not_managed_response(name)
    try:
        status_code, body = _docker_request("GET", f"/containers/{name}/json")
        if status_code != 200:
            if status_code == 404:
                return 404, {
                    "status": "error",
                    "message": f"Container '{name}' not found.",
                }
            return status_code, {
                "status": "error",
                "message": f"Failed to inspect container '{name}'.",
            }

        info = json.loads(body.decode("utf-8"))
        log_path = info.get("LogPath", "")
        if log_path and os.path.exists(log_path):
            try:
                with open(log_path, "w") as f:
                    f.truncate(0)
            except Exception as e:
                logging.warning(f"Failed to truncate log file at {log_path}: {e}")

        return 200, {
            "status": "success",
            "message": f"Logs cleared for container '{name}'.",
        }
    except Exception as e:
        return 500, {"status": "error", "message": str(e)}


def _human_bytes(value: int) -> str:
    size = float(max(value, 0))
    for unit in ("B", "kB", "MB", "GB", "TB"):
        if size < 1000 or unit == "TB":
            return f"{size:.1f} {unit}"
        size /= 1000
    return f"{size:.1f} TB"


def _current_recovery_image_refs() -> set[str]:
    update_dir = Path(os.environ.get("PIB_UPDATE_DIR", "/app/.update"))
    state_path = update_dir / "deployments.json"
    try:
        document = json.loads(state_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise RuntimeError(f"cannot read deployment recovery state: {error}") from error
    if not isinstance(document, dict) or document.get("schemaVersion") != 1:
        raise RuntimeError("deployment recovery state has an unsupported schema")
    projects = document.get("projects")
    if not isinstance(projects, list) or not projects:
        raise RuntimeError("deployment recovery state has no Compose projects")

    references: set[str] = set()
    project_names: set[str] = set()
    for project in projects:
        name = project.get("name") if isinstance(project, dict) else None
        if not isinstance(name, str) or not name.strip():
            raise RuntimeError("deployment recovery state has a project without a name")
        project_names.add(name.strip())
        model = project.get("composeConfig") if isinstance(project, dict) else None
        services = model.get("services") if isinstance(model, dict) else None
        if not isinstance(services, dict) or not services:
            raise RuntimeError("deployment recovery state has an invalid Compose model")
        for service in services.values():
            image = service.get("image") if isinstance(service, dict) else None
            if not isinstance(image, str) or not image.strip():
                raise RuntimeError(
                    "deployment recovery state has a service without an image"
                )
            references.add(_normalize_image_reference(image))
    if (
        project_names != {"pib-backend", "cerebra"}
        or len(projects) != 2
        or not references
    ):
        raise RuntimeError(
            "deployment recovery state does not contain both PIB Compose projects"
        )
    return references


def _normalize_image_reference(reference: str) -> str:
    """Canonicalize Docker's implicit tag and Docker Hub defaults for matching."""
    reference = reference.strip()
    name, separator, digest = reference.partition("@")
    slash = name.rfind("/")
    colon = name.rfind(":")
    if not separator and colon <= slash:
        name += ":latest"
    slash = name.rfind("/")
    colon = name.rfind(":")
    repository, tag = (name[:colon], name[colon:]) if colon > slash else (name, "")
    if "/" not in repository:
        repository = f"docker.io/library/{repository}"
    elif repository.startswith("docker.io/"):
        if "/" not in repository[len("docker.io/") :]:
            repository = f"docker.io/library/{repository[len('docker.io/') :]}"
    elif (
        "." not in repository.split("/", 1)[0]
        and ":" not in repository.split("/", 1)[0]
    ):
        repository = f"docker.io/{repository}"
    suffix = f"@{digest}" if separator else ""
    return f"{repository}{tag}{suffix}"


def _prune_obsolete_images() -> Tuple[Dict[str, Any], List[str]]:
    """Delete obsolete tags while retaining saved and currently running images."""
    try:
        protected = _current_recovery_image_refs()
    except RuntimeError as error:
        return {"skipped": str(error)}, [f"images: {error}"]

    code, body = _docker_request(
        "GET", "/containers/json?all=false", timeout=PURGE_TIMEOUT_SECONDS
    )
    if code != 200:
        return {}, [f"images: running containers list failed with {code}"]
    try:
        containers = json.loads(body.decode("utf-8"))
        if not isinstance(containers, list) or any(
            not isinstance(container, dict)
            or not isinstance(container.get("ImageID"), str)
            or not container["ImageID"].strip()
            for container in containers
        ):
            raise ValueError(
                "response must contain a non-empty ImageID for each container"
            )
        running_image_ids = {container["ImageID"].strip() for container in containers}
    except (UnicodeError, json.JSONDecodeError, ValueError) as error:
        return {}, [f"images: running containers list is invalid: {error}"]

    code, body = _docker_request(
        "GET", "/images/json?all=true", timeout=PURGE_TIMEOUT_SECONDS
    )
    if code != 200:
        return {}, [f"images: list failed with {code}"]
    try:
        images = json.loads(body.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        return {}, [f"images: list response is invalid: {error}"]
    if not isinstance(images, list):
        return {}, ["images: list response is not an array"]

    removed: List[str] = []
    errors: List[str] = []
    protect_dangling_prune = False
    for image in images:
        if isinstance(image, dict) and image.get("Id") in running_image_ids:
            continue
        tags = image.get("RepoTags") if isinstance(image, dict) else None
        tags = tags if isinstance(tags, list) else []
        digests = image.get("RepoDigests") if isinstance(image, dict) else None
        digests = digests if isinstance(digests, list) else []
        protected_tags = [
            reference
            for reference in tags
            if isinstance(reference, str)
            and reference != "<none>:<none>"
            and _normalize_image_reference(reference) in protected
        ]
        protected_digests = [
            reference
            for reference in digests
            if isinstance(reference, str)
            and _normalize_image_reference(reference) in protected
        ]
        if protected_digests and not protected_tags:
            protect_dangling_prune = True
            continue
        for reference in tags:
            if (
                not isinstance(reference, str)
                or reference == "<none>:<none>"
                or _normalize_image_reference(reference) in protected
            ):
                continue
            delete_path = f"/images/{quote(reference, safe='')}?force=0"
            delete_code, _ = _docker_request(
                "DELETE", delete_path, timeout=PURGE_TIMEOUT_SECONDS
            )
            if delete_code in (200, 204, 404):
                removed.append(reference)
            else:
                errors.append(f"images: could not remove {reference} ({delete_code})")

    reclaimed = 0
    if protect_dangling_prune:
        dangling_prune = "skipped to preserve a current digest-only recovery image"
    else:
        dangling_filter = quote(json.dumps({"dangling": ["true"]}))
        prune_code, prune_body = _docker_request(
            "POST",
            f"/images/prune?filters={dangling_filter}",
            timeout=PURGE_TIMEOUT_SECONDS,
        )
        dangling_prune = "completed"
        if prune_code in (200, 204):
            try:
                prune_result = json.loads(prune_body.decode("utf-8"))
                reclaimed = int(prune_result.get("SpaceReclaimed", 0))
            except (
                AttributeError,
                TypeError,
                ValueError,
                UnicodeError,
                json.JSONDecodeError,
            ):
                pass
        else:
            errors.append(f"images: dangling-image prune failed with {prune_code}")
            dangling_prune = f"failed with {prune_code}"
    return {
        "removed": removed,
        "SpaceReclaimed": reclaimed,
        "danglingPrune": dangling_prune,
    }, errors


def _purge_docker_locked() -> Tuple[int, Dict[str, Any]]:
    """Prune unused objects while retaining the images needed by saved recovery.

    All other tagged images are removed. Saved Compose image references remain
    available for boot-time recovery after stopped containers are purged.
    """
    results: Dict[str, Any] = {}
    errors: List[str] = []
    reclaimed = 0

    endpoints = [
        ("containers", "/containers/prune"),
        ("volumes", "/volumes/prune"),
        ("networks", "/networks/prune"),
        ("build", "/build/prune?all=true"),
    ]

    for key, path in endpoints:
        try:
            code, body = _docker_request("POST", path, timeout=PURGE_TIMEOUT_SECONDS)
            if code in (200, 204):
                try:
                    parsed = json.loads(body.decode("utf-8"))
                except Exception:
                    parsed = "cleared"
                results[key] = parsed
                if isinstance(parsed, dict):
                    space = parsed.get("SpaceReclaimed")
                    if isinstance(space, (int, float)):
                        reclaimed += int(space)
            else:
                errors.append(f"{key}: {code}")
        except Exception as e:
            errors.append(f"{key}: {str(e)}")

    try:
        image_result, image_errors = _prune_obsolete_images()
    except OSError as error:
        image_result = {"skipped": f"image cleanup transport failed: {error}"}
        image_errors = [f"images: cleanup transport failed: {error}"]
    results["images"] = image_result
    reclaimed += image_result.get("SpaceReclaimed", 0)
    errors.extend(image_errors)

    message = (
        f"Docker purge completed: {_human_bytes(reclaimed)} reported reclaimed by "
        "Docker prune operations; tagged image deletions are not included."
    )
    if errors:
        message += f" {len(errors)} step(s) failed."
    return 200, {
        "status": "success" if not errors else "partial",
        "message": message,
        "reclaimedBytes": reclaimed,
        "reclaimedHuman": _human_bytes(reclaimed),
        "reclaimedBytesScope": "Docker-reported prune space; excludes tagged image deletions",
        "details": results,
        "errors": errors if errors else None,
    }


def purge_docker() -> Tuple[int, Dict[str, Any]]:
    update_dir = Path(os.environ.get("PIB_UPDATE_DIR", "/app/.update"))
    try:
        update_dir.mkdir(parents=True, exist_ok=True)
        lock_path = update_dir / "deployment.lock"
        lock_fd = os.open(
            lock_path,
            os.O_CREAT | os.O_RDWR | os.O_APPEND | getattr(os, "O_NOFOLLOW", 0),
            0o660,
        )
        try:
            lock_info = os.fstat(lock_fd)
            if not stat.S_ISREG(lock_info.st_mode) or lock_info.st_nlink != 1:
                raise OSError("deployment lock must be a regular, singly linked file")
            directory_info = update_dir.stat()
            os.fchown(lock_fd, directory_info.st_uid, directory_info.st_gid)
            os.fchmod(lock_fd, 0o660)
            lock = os.fdopen(lock_fd, "a", encoding="utf-8")
            lock_fd = -1
        finally:
            if lock_fd >= 0:
                os.close(lock_fd)
        with lock:
            try:
                fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                return 409, {
                    "status": "error",
                    "message": "Docker purge cannot run while an update or recovery is in progress.",
                    "errors": ["deployment lock is held"],
                }
            try:
                return _purge_docker_locked()
            finally:
                fcntl.flock(lock.fileno(), fcntl.LOCK_UN)
    except OSError as error:
        return 500, {
            "status": "error",
            "message": f"Docker purge could not acquire deployment lock: {error}",
            "errors": [str(error)],
        }
