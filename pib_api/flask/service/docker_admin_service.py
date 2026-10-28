import os
import socket
import json
import logging
from urllib.parse import quote
from typing import Dict, Any, List, Tuple, Optional

DOCKER_SOCK = "/var/run/docker.sock"

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


def purge_docker() -> Tuple[int, Dict[str, Any]]:
    """Prune everything the device no longer needs, and report what that freed.

    Two Engine defaults made the old purge a no-op on an installed robot (PR-2099):
    ``POST /images/prune`` without a filter removes only *untagged* images, so the
    tagged leftovers of test runs stayed, and ``POST /build/prune`` prunes the
    legacy builder, not the BuildKit cache this installation actually fills.
    """
    results: Dict[str, Any] = {}
    errors: List[str] = []
    reclaimed = 0

    # filters must be the URL-encoded JSON the Engine expects.
    image_filters = quote(json.dumps({"dangling": ["false"]}))
    endpoints = [
        ("containers", "/containers/prune"),
        ("images", f"/images/prune?filters={image_filters}"),
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

    message = f"Docker purge completed: {_human_bytes(reclaimed)} freed."
    if errors:
        message += f" {len(errors)} step(s) failed."
    return 200, {
        "status": "success" if not errors else "partial",
        "message": message,
        "reclaimedBytes": reclaimed,
        "reclaimedHuman": _human_bytes(reclaimed),
        "details": results,
        "errors": errors if errors else None,
    }
