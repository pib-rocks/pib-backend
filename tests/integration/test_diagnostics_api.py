import pytest
import json
from unittest.mock import MagicMock, patch
from app.app import app


@pytest.fixture
def client():
    app.config["TESTING"] = True
    with app.test_client() as client:
        yield client


def test_get_diagnostics_summary(client):
    response = client.get("/api/v1/diagnostics/summary")
    assert response.status_code == 200
    data = response.get_json()
    assert "overallStatus" in data
    assert "cpuTemperature" in data
    assert "cpuUsagePercent" in data
    assert isinstance(data["cpuUsagePercent"], (int, float))
    assert 0.0 <= data["cpuUsagePercent"] <= 100.0
    assert "memoryUsage" in data
    assert "memoryStatus" in data
    assert "diskSpace" in data
    assert "brickletsStatus" in data


def test_get_diagnostics_bricklets(client):
    response = client.get("/api/v1/diagnostics/bricklets")
    assert response.status_code == 200
    data = response.get_json()
    assert "bricklets" in data
    assert isinstance(data["bricklets"], list)


def test_get_diagnostics_system(client):
    response = client.get("/api/v1/diagnostics/system")
    assert response.status_code == 200
    data = response.get_json()
    assert "cpuTemperature" in data
    assert "cpuUsagePercent" in data
    assert isinstance(data["cpuUsagePercent"], (int, float))
    assert 0.0 <= data["cpuUsagePercent"] <= 100.0
    assert "memoryUsage" in data
    assert "diskSpace" in data
    assert "containers" in data


def test_servo_bricklet_live_telemetry_query():
    from service import diagnostics_service

    mock_pin = MagicMock()
    mock_pin.pin = 0

    mock_bricklet = MagicMock()
    mock_bricklet.bricklet_number = 1
    mock_bricklet.uid = "SERVO123"
    mock_bricklet.type = "Servo Bricklet"
    mock_bricklet.bricklet_pins = [mock_pin]

    mock_ipcon_cls = MagicMock()
    mock_ipcon_inst = MagicMock()
    mock_ipcon_cls.return_value = mock_ipcon_inst

    mock_servo_cls = MagicMock()
    mock_servo_inst = MagicMock()
    mock_servo_inst.get_input_voltage.return_value = 5250  # 5.25 V
    mock_servo_inst.get_overall_current.return_value = 180  # 180 mA
    mock_servo_inst.get_servo_current.return_value = 45     # 45 mA
    mock_servo_cls.return_value = mock_servo_inst

    with app.app_context():
        with patch("service.bricklet_service.get_all_bricklets", return_value=[mock_bricklet]):
            with patch.dict("sys.modules", {
                "tinkerforge": MagicMock(),
                "tinkerforge.ip_connection": MagicMock(IPConnection=mock_ipcon_cls),
                "tinkerforge.bricklet_servo_v2": MagicMock(BrickletServoV2=mock_servo_cls),
            }):
                telemetry = diagnostics_service.get_bricklets_telemetry()
                assert len(telemetry) == 1
                assert telemetry[0]["uid"] == "SERVO123"
                assert telemetry[0]["voltage"] == 5.25
                assert telemetry[0]["current"] == 180.0
                assert len(telemetry[0]["pins"]) == 1
                assert telemetry[0]["pins"][0]["pin"] == 0
                assert telemetry[0]["pins"][0]["current"] == 45.0
                assert telemetry[0]["pins"][0]["voltage"] == 5.25


def test_rgb_led_button_live_telemetry_query():
    from service import diagnostics_service

    mock_bricklet = MagicMock()
    mock_bricklet.bricklet_number = 2
    mock_bricklet.uid = "BTN123"
    mock_bricklet.type = "RGB LED Button Bricklet"
    mock_bricklet.bricklet_pins = []

    mock_ipcon_cls = MagicMock()
    mock_ipcon_inst = MagicMock()
    mock_ipcon_cls.return_value = mock_ipcon_inst

    mock_btn_cls = MagicMock()
    mock_btn_cls.BUTTON_STATE_PRESSED = 0
    mock_btn_cls.BUTTON_STATE_RELEASED = 1
    mock_btn_inst = MagicMock()
    mock_btn_inst.get_color.return_value = (255, 0, 128)  # #FF0080
    mock_btn_inst.get_button_state.return_value = 0      # Pressed
    mock_btn_cls.return_value = mock_btn_inst

    with app.app_context():
        with patch("service.bricklet_service.get_all_bricklets", return_value=[mock_bricklet]):
            with patch.dict("sys.modules", {
                "tinkerforge": MagicMock(),
                "tinkerforge.ip_connection": MagicMock(IPConnection=mock_ipcon_cls),
                "tinkerforge.bricklet_rgb_led_button": MagicMock(BrickletRGBLEDButton=mock_btn_cls),
            }):
                telemetry = diagnostics_service.get_bricklets_telemetry()
                assert len(telemetry) == 1
                assert telemetry[0]["uid"] == "BTN123"
                assert telemetry[0]["color"] == "#FF0080"
                assert telemetry[0]["pressState"] == "Pressed"


def test_solid_state_relay_live_telemetry_query():
    from service import diagnostics_service

    mock_bricklet = MagicMock()
    mock_bricklet.bricklet_number = 3
    mock_bricklet.uid = "SSR123"
    mock_bricklet.type = "Solid State Relay Bricklet"
    mock_bricklet.bricklet_pins = []

    mock_ipcon_cls = MagicMock()
    mock_ipcon_inst = MagicMock()
    mock_ipcon_cls.return_value = mock_ipcon_inst

    mock_ssr_cls = MagicMock()
    mock_ssr_inst = MagicMock()
    mock_ssr_inst.get_state.return_value = True
    mock_ssr_cls.return_value = mock_ssr_inst

    with app.app_context():
        with patch("service.bricklet_service.get_all_bricklets", return_value=[mock_bricklet]):
            with patch.dict("sys.modules", {
                "tinkerforge": MagicMock(),
                "tinkerforge.ip_connection": MagicMock(IPConnection=mock_ipcon_cls),
                "tinkerforge.bricklet_solid_state_relay_v2": MagicMock(BrickletSolidStateRelayV2=mock_ssr_cls),
            }):
                telemetry = diagnostics_service.get_bricklets_telemetry()
                assert len(telemetry) == 1
                assert telemetry[0]["uid"] == "SSR123"
                assert telemetry[0]["relayState"] is True


def test_docker_containers_live_query():
    from service import diagnostics_service

    fake_containers = [
        {"Names": ["/pib-backend"], "State": "running", "Status": "Up 2 hours (healthy)"},
        {"Names": ["/rosbridge"], "State": "running", "Status": "Up 2 hours (unhealthy)"},
    ]
    body_str = json.dumps(fake_containers)
    http_response = (
        f"HTTP/1.1 200 OK\r\nContent-Length: {len(body_str)}\r\n\r\n{body_str}"
    ).encode("utf-8")

    mock_sock = MagicMock()
    mock_sock.recv.side_effect = [http_response, b""]

    with patch("os.path.exists", return_value=True):
        with patch("socket.socket", return_value=mock_sock):
            containers = diagnostics_service._query_docker_containers()
            assert len(containers) == 2
            assert containers[0]["name"] == "pib-backend"
            assert containers[0]["status"] == "running"
            assert containers[0]["health"] == "healthy"
            assert containers[1]["name"] == "rosbridge"
            assert containers[1]["health"] == "unhealthy"
