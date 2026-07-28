import pytest
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
    assert "diskSpace" in data
    assert "containers" in data
