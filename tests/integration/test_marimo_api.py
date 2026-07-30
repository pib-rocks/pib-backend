import pytest
from app import app
from service import marimo_service


@pytest.fixture
def client():
    app.config["TESTING"] = True
    with app.test_client() as client:
        yield client


def test_list_marimo_notebooks(client):
    response = client.get("/v1/marimo/notebooks")
    assert response.status_code == 200
    data = response.get_json()
    assert data["status"] == "success"
    assert isinstance(data["notebooks"], list)


def test_create_and_delete_marimo_notebook(client):
    test_name = "test_auto_generated.py"
    # Ensure deleted
    marimo_service.delete_notebook(test_name)

    # Create
    response = client.post("/v1/marimo/notebooks", json={"name": test_name})
    assert response.status_code == 201
    data = response.get_json()
    assert data["status"] == "success"

    # Get
    response_get = client.get(f"/v1/marimo/notebooks/{test_name}")
    assert response_get.status_code == 200

    # Delete
    response_del = client.delete(f"/v1/marimo/notebooks/{test_name}")
    assert response_del.status_code == 200
