"""Unit tests for diagnostics service CPU usage in summary (PR-1521)."""

from unittest.mock import patch

from service import diagnostics_service


def test_get_cpu_usage_percent_from_proc_stat():
    samples = [
        (1000, 2000),  # idle, total
        (1050, 2200),  # idle_delta=50, total_delta=200 -> 75% used
    ]

    with patch(
        "service.diagnostics_service._read_proc_stat_cpu_times",
        side_effect=samples,
    ), patch("service.diagnostics_service.time.sleep"):
        percent = diagnostics_service._get_cpu_usage_percent()

    assert percent == 75.0


def test_get_cpu_usage_percent_falls_back_to_loadavg():
    with patch(
        "service.diagnostics_service._read_proc_stat_cpu_times",
        return_value=None,
    ), patch("os.getloadavg", return_value=(1.5, 1.0, 0.5)), patch(
        "os.cpu_count", return_value=4
    ):
        percent = diagnostics_service._get_cpu_usage_percent()

    assert percent == 37.5


def test_get_system_telemetry_includes_cpu_usage_percent():
    with patch(
        "service.diagnostics_service._get_cpu_usage_percent",
        return_value=42.5,
    ), patch(
        "service.diagnostics_service._query_docker_containers",
        return_value=[],
    ):
        telemetry = diagnostics_service.get_system_telemetry()

    assert "cpuUsagePercent" in telemetry
    assert telemetry["cpuUsagePercent"] == 42.5


def test_get_summary_includes_cpu_usage_percent():
    fake_system = {
        "cpuTemperature": 50.0,
        "cpuUsagePercent": 18.25,
        "memoryUsage": {
            "total": "8.0 GB",
            "used": "3.2 GB",
            "free": "4.8 GB",
            "percentUsed": 40.0,
        },
        "diskSpace": {
            "total": "64.0 GB",
            "used": "22.4 GB",
            "free": "41.6 GB",
            "percentUsed": 35.0,
        },
        "containers": [
            {"name": "pib-backend", "status": "running", "health": "healthy"},
        ],
        "status": "ok",
    }

    with patch(
        "service.diagnostics_service.get_system_telemetry",
        return_value=fake_system,
    ), patch(
        "service.diagnostics_service.get_bricklets_telemetry",
        return_value=[],
    ):
        summary = diagnostics_service.get_summary()

    assert "cpuUsagePercent" in summary
    assert summary["cpuUsagePercent"] == 18.25
    assert summary["overallStatus"] == "ok"
    assert summary["cpuTemperature"] == 50.0
