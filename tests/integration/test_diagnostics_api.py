import pytest
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
    assert "memoryUsage" in data
    assert "diskSpace" in data
    assert "containers" in data

def test_servo_bricklet_live_voltage_query():
    from service import diagnostics_service

    mock_bricklet = MagicMock()
    mock_bricklet.bricklet_number = 1
    mock_bricklet.uid = "SERVO123"
    mock_bricklet.type = "Servo Bricklet"
    mock_bricklet.bricklet_pins = []

    mock_ipcon_cls = MagicMock()
    mock_ipcon_inst = MagicMock()
    mock_ipcon_cls.return_value = mock_ipcon_inst

    mock_servo_cls = MagicMock()
    mock_servo_inst = MagicMock()
    mock_servo_inst.get_input_voltage.return_value = 5250  # 5.25 V
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


