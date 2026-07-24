import logging
from typing import Any, Dict, List, Optional

import docker
from docker.errors import DockerException, NotFound


logger = logging.getLogger(__name__)


def _get_client():
    return docker.from_env()


def _health_from_container(container) -> str:
    state = container.attrs.get("State", {})
    health = state.get("Health")
    if health and health.get("Status"):
        return health["Status"]
    status = (state.get("Status") or container.status or "").lower()
    if status == "running":
        return "healthy"
    if status in ("exited", "dead", "restarting", "paused", "created"):
        return "unhealthy"
    return "unknown"


def _ports_from_container(container) -> List[str]:
    ports: List[str] = []
    network_settings = container.attrs.get("NetworkSettings", {}).get("Ports") or {}
    for container_port, bindings in network_settings.items():
        if not bindings:
            ports.append(container_port)
            continue
        for binding in bindings:
            host_ip = binding.get("HostIp") or "0.0.0.0"
            host_port = binding.get("HostPort")
            ports.append(f"{host_ip}:{host_port}->{container_port}")
    return ports


def list_containers() -> List[Dict[str, Any]]:
    try:
        client = _get_client()
        containers = client.containers.list(all=True)
    except DockerException as exc:
        logger.error("Unable to list docker containers: %s", exc)
        raise RuntimeError("Unable to access Docker. Is the Docker socket mounted?") from exc

    result: List[Dict[str, Any]] = []
    for container in containers:
        name = container.name.lstrip("/")
        image = container.image.tags[0] if container.image.tags else (
            container.attrs.get("Config", {}).get("Image") or container.image.short_id
        )
        state = container.attrs.get("State", {})
        result.append(
            {
                "id": container.short_id,
                "name": name,
                "image": image,
                "status": container.status,
                "health": _health_from_container(container),
                "startedAt": state.get("StartedAt"),
                "ports": _ports_from_container(container),
            }
        )
    result.sort(key=lambda item: item["name"])
    return result


def get_container_logs(name: str, tail: int = 200) -> Dict[str, Any]:
    try:
        client = _get_client()
        container = client.containers.get(name)
        logs = container.logs(tail=tail, timestamps=True).decode("utf-8", errors="replace")
        return {"name": name, "logs": logs}
    except NotFound as exc:
        raise LookupError(f"Container '{name}' not found") from exc
    except DockerException as exc:
        logger.error("Unable to read logs for %s: %s", name, exc)
        raise RuntimeError(f"Unable to read logs for container '{name}'") from exc


def get_container_inspect(name: str) -> Dict[str, Any]:
    try:
        client = _get_client()
        container = client.containers.get(name)
        return container.attrs
    except NotFound as exc:
        raise LookupError(f"Container '{name}' not found") from exc
    except DockerException as exc:
        raise RuntimeError(f"Unable to inspect container '{name}'") from exc


def get_docker_info() -> Dict[str, Any]:
    try:
        return _get_client().info()
    except DockerException as exc:
        raise RuntimeError(f"Unable to query Docker info: {exc}") from exc


def get_docker_version() -> Dict[str, Any]:
    try:
        return _get_client().version()
    except DockerException as exc:
        raise RuntimeError(f"Unable to query Docker version: {exc}") from exc


def list_images() -> List[Dict[str, Any]]:
    try:
        images = _get_client().images.list()
    except DockerException as exc:
        raise RuntimeError(f"Unable to list Docker images: {exc}") from exc

    result = []
    for image in images:
        result.append(
            {
                "id": image.short_id,
                "tags": image.tags,
                "created": image.attrs.get("Created"),
                "size": image.attrs.get("Size"),
                "labels": image.labels or {},
            }
        )
    result.sort(key=lambda item: (item["tags"][0] if item["tags"] else item["id"]))
    return result


def list_networks() -> List[Dict[str, Any]]:
    try:
        networks = _get_client().networks.list()
    except DockerException as exc:
        raise RuntimeError(f"Unable to list Docker networks: {exc}") from exc

    return [
        {
            "id": network.short_id,
            "name": network.name,
            "driver": network.attrs.get("Driver"),
            "scope": network.attrs.get("Scope"),
            "containers": list((network.attrs.get("Containers") or {}).keys()),
            "ipam": network.attrs.get("IPAM"),
        }
        for network in networks
    ]


def list_volumes() -> List[Dict[str, Any]]:
    try:
        volumes = _get_client().volumes.list()
    except DockerException as exc:
        raise RuntimeError(f"Unable to list Docker volumes: {exc}") from exc

    return [
        {
            "name": volume.name,
            "driver": volume.attrs.get("Driver"),
            "mountpoint": volume.attrs.get("Mountpoint"),
            "createdAt": volume.attrs.get("CreatedAt"),
            "labels": volume.attrs.get("Labels") or {},
        }
        for volume in volumes
    ]


def get_container_stats_snapshot(name: str) -> Optional[Dict[str, Any]]:
    try:
        client = _get_client()
        container = client.containers.get(name)
        if container.status != "running":
            return {"name": name, "status": container.status, "stats": None}
        stats = container.stats(stream=False)
        return {"name": name, "status": container.status, "stats": stats}
    except NotFound:
        return None
    except DockerException as exc:
        logger.warning("Unable to read stats for %s: %s", name, exc)
        return {"name": name, "error": str(exc)}
