import os
import platform
import shutil
import socket
import subprocess
from typing import Any, Dict, List, Optional


HOST_PROC = os.getenv("HOST_PROC_PATH", "/host/proc")
HOST_OS_RELEASE = os.getenv("HOST_OS_RELEASE_PATH", "/host/os-release")
HOST_THERMAL = os.getenv(
    "HOST_THERMAL_PATH", "/sys/class/thermal/thermal_zone0/temp"
)
HOST_ROOT = os.getenv("DIAGNOSTICS_HOST_ROOT", "/host/root")


def _read_file(path: str, max_bytes: Optional[int] = None) -> Optional[str]:
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as handle:
            if max_bytes is None:
                return handle.read().strip()
            return handle.read(max_bytes).strip()
    except OSError:
        return None


def _parse_meminfo(content: str) -> Dict[str, int]:
    values: Dict[str, int] = {}
    for line in content.splitlines():
        if ":" not in line:
            continue
        key, raw = line.split(":", 1)
        parts = raw.strip().split()
        if parts:
            try:
                values[key] = int(parts[0])
            except ValueError:
                continue
    total = values.get("MemTotal", 0)
    available = values.get("MemAvailable", values.get("MemFree", 0))
    used = max(total - available, 0)
    return {
        "totalMb": round(total / 1024) if total else 0,
        "usedMb": round(used / 1024) if used else 0,
        "availableMb": round(available / 1024) if available else 0,
        "swapTotalMb": round(values.get("SwapTotal", 0) / 1024),
        "swapFreeMb": round(values.get("SwapFree", 0) / 1024),
        "rawKb": values,
    }


def _parse_cpuinfo(content: str) -> Dict[str, Any]:
    model = "unknown"
    hardware = None
    revision = None
    serial = None
    cores = 0
    for line in content.splitlines():
        if line.startswith("model name") or line.startswith("Model"):
            model = line.split(":", 1)[1].strip()
        elif line.startswith("Hardware"):
            hardware = line.split(":", 1)[1].strip()
        elif line.startswith("Revision"):
            revision = line.split(":", 1)[1].strip()
        elif line.startswith("Serial"):
            serial = line.split(":", 1)[1].strip()
        elif line.startswith("processor"):
            cores += 1
    return {
        "model": model,
        "cores": cores or os.cpu_count() or 0,
        "hardware": hardware,
        "revision": revision,
        "serial": serial,
    }


def _parse_os_release(content: str) -> Dict[str, str]:
    data: Dict[str, str] = {}
    for line in content.splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        data[key] = value.strip().strip('"')
    return {
        "prettyName": data.get("PRETTY_NAME", platform.platform()),
        "versionId": data.get("VERSION_ID", ""),
        "id": data.get("ID", ""),
        "versionCodename": data.get("VERSION_CODENAME", ""),
        "raw": data,
    }


def _read_temperature_c() -> Optional[float]:
    raw = _read_file(HOST_THERMAL)
    if raw is None:
        return None
    try:
        return round(int(raw) / 1000.0, 1)
    except ValueError:
        return None


def _read_all_thermal_zones() -> List[Dict[str, Any]]:
    zones: List[Dict[str, Any]] = []
    thermal_root = "/sys/class/thermal"
    try:
        entries = sorted(os.listdir(thermal_root))
    except OSError:
        return zones
    for entry in entries:
        if not entry.startswith("thermal_zone"):
            continue
        temp = _read_file(os.path.join(thermal_root, entry, "temp"))
        zone_type = _read_file(os.path.join(thermal_root, entry, "type"))
        try:
            temp_c = round(int(temp) / 1000.0, 1) if temp is not None else None
        except ValueError:
            temp_c = None
        zones.append({"name": entry, "type": zone_type, "temperatureC": temp_c})
    return zones


def _read_uptime_seconds() -> Optional[float]:
    raw = _read_file(os.path.join(HOST_PROC, "uptime"))
    if raw is None:
        return None
    try:
        return float(raw.split()[0])
    except (ValueError, IndexError):
        return None


def _read_load_average() -> Optional[Dict[str, float]]:
    raw = _read_file(os.path.join(HOST_PROC, "loadavg"))
    if raw is None:
        return None
    try:
        one, five, fifteen, *_ = raw.split()
        return {
            "oneMinute": float(one),
            "fiveMinutes": float(five),
            "fifteenMinutes": float(fifteen),
        }
    except (ValueError, IndexError):
        return None


def _read_host_ip() -> Optional[str]:
    from app.app import app

    ip_file = app.config.get("HOST_IP_FILE", "host_ip.txt")
    return _read_file(ip_file)


def _disk_usage(path: str) -> Optional[Dict[str, Any]]:
    try:
        usage = shutil.disk_usage(path)
        return {
            "path": path,
            "totalMb": round(usage.total / (1024 * 1024)),
            "usedMb": round(usage.used / (1024 * 1024)),
            "freeMb": round(usage.free / (1024 * 1024)),
            "usedPercent": round((usage.used / usage.total) * 100, 1) if usage.total else 0,
        }
    except OSError:
        return None


def _run_command(command: List[str], timeout: int = 15) -> Dict[str, Any]:
    try:
        completed = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
        return {
            "command": command,
            "exitCode": completed.returncode,
            "stdout": completed.stdout,
            "stderr": completed.stderr,
        }
    except FileNotFoundError:
        return {"command": command, "error": "command not found"}
    except subprocess.TimeoutExpired:
        return {"command": command, "error": "timeout"}
    except OSError as exc:
        return {"command": command, "error": str(exc)}


def get_system_info() -> Dict[str, Any]:
    meminfo = _read_file(os.path.join(HOST_PROC, "meminfo"))
    cpuinfo = _read_file(os.path.join(HOST_PROC, "cpuinfo"))
    os_release = _read_file(HOST_OS_RELEASE)

    memory = (
        _parse_meminfo(meminfo)
        if meminfo
        else {"totalMb": 0, "usedMb": 0, "availableMb": 0}
    )
    cpu = (
        _parse_cpuinfo(cpuinfo)
        if cpuinfo
        else {
            "model": platform.processor() or "unknown",
            "cores": os.cpu_count() or 0,
        }
    )
    os_info = (
        _parse_os_release(os_release)
        if os_release
        else {
            "prettyName": platform.platform(),
            "versionId": "",
            "id": "",
        }
    )

    try:
        hostname = socket.gethostname()
    except OSError:
        hostname = "unknown"

    disks = [
        disk
        for disk in (
            _disk_usage("/"),
            _disk_usage("/app"),
            _disk_usage(os.path.expanduser("~")),
            _disk_usage(HOST_ROOT) if os.path.isdir(HOST_ROOT) else None,
        )
        if disk
    ]

    return {
        "hostname": hostname,
        "os": os_info,
        "arch": platform.machine(),
        "kernel": _read_file(os.path.join(HOST_PROC, "version")) or platform.release(),
        "pythonVersion": platform.python_version(),
        "cpu": cpu,
        "memory": memory,
        "temperatureC": _read_temperature_c(),
        "thermalZones": _read_all_thermal_zones(),
        "uptimeSeconds": _read_uptime_seconds(),
        "loadAverage": _read_load_average(),
        "hostIp": _read_host_ip(),
        "disks": disks,
        "procMounts": _read_file(os.path.join(HOST_PROC, "mounts"), max_bytes=100_000),
        "procNetDev": _read_file(os.path.join(HOST_PROC, "net/dev")),
        "cmdline": _read_file(os.path.join(HOST_PROC, "cmdline")),
    }


def collect_runtime_commands() -> Dict[str, Any]:
    commands = [
        ["uname", "-a"],
        ["df", "-h"],
        ["free", "-h"],
        ["uptime"],
        ["hostname", "-I"],
        ["ip", "addr"],
        ["ip", "route"],
        ["ip", "link"],
        ["lsusb"],
        ["vcgencmd", "measure_temp"],
        ["vcgencmd", "get_throttled"],
        ["vcgencmd", "measure_volts"],
        ["vcgencmd", "get_config", "int"],
        ["sh", "-c", "dmesg -T 2>/dev/null | tail -n 1000"],
        ["pip", "freeze"],
        ["python3", "--version"],
        ["ls", "-la", "/app"],
        ["ls", "-la", "/host/pib"],
        ["ls", "-la", "/sys/class/thermal"],
    ]
    results: Dict[str, Any] = {}
    for index, cmd in enumerate(commands):
        key = "_".join(cmd[:3]).replace("/", "_").replace(" ", "_")
        key = f"{index:02d}_{key}"[:80]
        results[key] = _run_command(cmd)
    return results
