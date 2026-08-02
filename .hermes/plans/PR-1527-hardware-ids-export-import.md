# PR-1527 — Export and Import of Hardware IDs and Bricklet Mappings

Jira Ticket: https://pib-rocks.atlassian.net/browse/PR-1527
Category: Software
Branch: `PR-1527`

## Goals
Implement REST API endpoints and service logic for exporting and importing Tinkerforge Bricklet UIDs, Hardware IDs, and motor mappings:
- `GET /api/system/hardware-config/export`: Generates JSON file of active Bricklet UIDs, positions, and motor limits.
- `POST /api/system/hardware-config/import`: Validates JSON schema and imports hardware configurations into `pibdata.db`.

## Tasks
1. Service & Controller (`pib_api/flask/service/hardware_config_service.py` and `pib_api/flask/controller/system_controller.py`).
2. Test Basis `docs/test-basis/PR-1527-hardware-ids-export-import.md`.
3. Unit & Integration Tests (`tests/unit/test_hardware_config_service.py` & `tests/integration/test_hardware_config_api.py`).
4. Merge into `develop` upon success and push to `origin/develop` (DO NOT DEPLOY TO PI).
