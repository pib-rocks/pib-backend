from pathlib import Path

import pytest

from service import controller_service, diagnostics_service


@pytest.fixture(autouse=True)
def clear_detected_assignments():
    controller_service._detected_assignments.clear()
    yield
    controller_service._detected_assignments.clear()


def _assignment_defaults():
    return {
        "assignedControllerNumber": None,
        "matchCount": 0,
        "expectedCount": 0,
    }


def test_extract_serial_from_by_id_name_strips_interface_suffix():
    assert (
        controller_service.extract_serial_from_by_id_name(
            "usb-1a86_USB_Single_Serial_5B91030562-if00"
        )
        == "5B91030562"
    )
    assert (
        controller_service.extract_serial_from_by_id_name(
            "usb-Waveshare_USB_Single_Serial_5B79018060-if00-port0"
        )
        == "5B79018060"
    )


def test_get_connected_serial_controllers_from_fake_by_id_dir(tmp_path):
    entries = [
        ("usb-1a86_USB_Single_Serial_5B91030562-if00", "ttyACM0"),
        ("usb-Waveshare_USB_Single_Serial_5B79018060-if00-port0", "ttyACM1"),
        ("usb-1a86_USB_Single_Serial_5B79018101-if00", "ttyACM2"),
        ("usb-1a86_USB_Single_Serial_5B79014593-if00", "ttyACM3"),
    ]
    dev = tmp_path / "dev"
    dev.mkdir()
    by_id = tmp_path / "by-id"
    by_id.mkdir()
    for link_name, tty in entries:
        tty_path = dev / tty
        tty_path.write_text("")
        (by_id / link_name).symlink_to(Path("..") / "dev" / tty)

    controllers = controller_service.get_connected_serial_controllers(by_id)

    assert len(controllers) == 4
    serials = {item["serial"] for item in controllers}
    assert serials == {
        "5B91030562",
        "5B79018060",
        "5B79018101",
        "5B79014593",
    }
    for item in controllers:
        assert item["kind"] == "feetech_st_serial"
        assert item["name"] == "Waveshare Serial Controller"
        assert item["uid"] == item["serial"]
        assert item["assignedControllerNumber"] is None
        assert item["matchCount"] == 0
        assert item["expectedCount"] == 0
    ports = {item["serial"]: item["port"] for item in controllers}
    assert ports == {
        "5B91030562": "ttyACM0",
        "5B79018060": "ttyACM1",
        "5B79018101": "ttyACM2",
        "5B79014593": "ttyACM3",
    }


def test_get_connected_serial_controllers_merges_live_telemetry(monkeypatch):
    diagnostics_service._serial_pin_telemetry.clear()
    diagnostics_service.update_controller_pin_telemetry(
        [{"uid": "5BTELEMETRY01", "pin": 1, "voltage": 12.0, "current": 1.0}]
    )
    monkeypatch.setattr(
        controller_service,
        "SERIAL_BY_ID_DIR",
        Path("/nonexistent-serial-by-id"),
    )

    controllers = controller_service.get_connected_serial_controllers()

    assert controllers == [
        {
            "serial": "5BTELEMETRY01",
            "uid": "5BTELEMETRY01",
            "port": "",
            "kind": "feetech_st_serial",
            "name": "Waveshare Serial Controller",
            **_assignment_defaults(),
        }
    ]
    diagnostics_service._serial_pin_telemetry.clear()


def test_controller_connected_route(app, monkeypatch):
    monkeypatch.setattr(
        controller_service,
        "get_connected_serial_controllers",
        lambda serial_dir=None: [
            {
                "serial": "5B91030562",
                "uid": "5B91030562",
                "port": "ttyACM0",
                "kind": "feetech_st_serial",
                "name": "Waveshare Serial Controller",
            }
        ],
    )

    with app.test_client() as client:
        response = client.get("/controller/connected")

    assert response.status_code == 200
    body = response.get_json()
    assert set(body) == {"controllers"}
    assert len(body["controllers"]) == 1
    assert body["controllers"][0]["serial"] == "5B91030562"


def test_update_detected_assignments_accepts_valid_and_skips_malformed():
    updated = controller_service.update_detected_assignments(
        [
            {
                "serial": " 5B91030562 ",
                "controllerNumber": 2,
                "matchCount": 4,
                "expectedCount": 5,
            },
            {"serial": ""},
            {"serial": "BAD", "controllerNumber": "two"},
            "not-a-dict",
        ]
    )
    assert updated == 1
    stored = controller_service._detected_assignments["5B91030562"]
    assert stored["controllerNumber"] == 2
    assert stored["matchCount"] == 4
    assert stored["expectedCount"] == 5


def test_detected_assignments_ttl_drops_stale_entries(monkeypatch, tmp_path):
    controller_service.update_detected_assignments(
        [{"serial": "5BSTALE01", "controllerNumber": 1, "matchCount": 1}]
    )
    controller_service._detected_assignments["5BSTALE01"]["updated_at"] = 0.0
    monkeypatch.setattr(controller_service.time, "time", lambda: 10_000.0)

    entries = [
        ("usb-1a86_USB_Single_Serial_5BSTALE01-if00", "ttyACM9"),
    ]
    dev = tmp_path / "dev"
    dev.mkdir()
    by_id = tmp_path / "by-id"
    by_id.mkdir()
    for link_name, tty in entries:
        tty_path = dev / tty
        tty_path.write_text("")
        (by_id / link_name).symlink_to(Path("..") / "dev" / tty)

    controllers = controller_service.get_connected_serial_controllers(by_id)
    assert controllers[0]["assignedControllerNumber"] is None
    assert "5BSTALE01" not in controller_service._detected_assignments


def test_get_connected_serial_controllers_enriched_from_assignment(tmp_path):
    controller_service.update_detected_assignments(
        [
            {
                "serial": "5B91030562",
                "controllerNumber": 3,
                "matchCount": 5,
                "expectedCount": 6,
            }
        ]
    )
    dev = tmp_path / "dev"
    dev.mkdir()
    by_id = tmp_path / "by-id"
    by_id.mkdir()
    tty_path = dev / "ttyACM0"
    tty_path.write_text("")
    (by_id / "usb-1a86_USB_Single_Serial_5B91030562-if00").symlink_to(
        Path("..") / "dev" / "ttyACM0"
    )

    controllers = controller_service.get_connected_serial_controllers(by_id)
    assert controllers[0]["assignedControllerNumber"] == 3
    assert controllers[0]["matchCount"] == 5
    assert controllers[0]["expectedCount"] == 6


def test_post_detected_assignments_route(app):
    with app.test_client() as client:
        bad = client.post(
            "/controller/detected-assignments",
            json={"assignments": "not-a-list"},
        )
        assert bad.status_code == 400

        ok = client.post(
            "/controller/detected-assignments",
            json={
                "assignments": [
                    {"serial": "5BNEW01", "controllerNumber": 1, "matchCount": 2}
                ]
            },
        )
    assert ok.status_code == 200
    assert ok.get_json() == {"updated": 1}
