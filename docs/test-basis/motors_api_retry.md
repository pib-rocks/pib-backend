# Test Basis: Motors API Bricklet Retry

**Repository:** `pib-backend`  
**Requirement:** Jira PR-1509  
**Components:** `pib_motors/bricklet.py`, `ros-motors` container startup

## Requirement

`pib_motors/bricklet.py` must retry connecting to `pib-api` during startup when `pib-api` is still starting up after a reboot, preventing `motor_control` node crashes caused by transient connection refused errors.

## Acceptance Criteria Traceability

| AC | Acceptance criterion | Coverage | Status |
|---|---|---|---|
| AC1 | `pib_motors/bricklet.py` retries connecting to `pib-api` with backoff (up to 60s) before raising RuntimeError. | `tests/unit/test_bricklet_retry.py::test_gives_up_with_runtime_error_after_the_retry_window`, `::test_retry_window_spans_at_least_a_minute` | Covered |
| AC2 | `motor_control` node successfully initializes once `pib-api` is ready. | `tests/unit/test_bricklet_retry.py::test_module_imports_once_pib_api_becomes_reachable`, `::test_bricklets_are_wired_up_after_a_retry` | Covered |
| AC3 | Unit tests in `tests/unit/test_bricklet_retry.py` cover retry behavior on transient connection failures. | `tests/unit/test_bricklet_retry.py::test_retries_when_the_request_raises_connection_refused`, `::test_each_retry_logs_a_warning` | Covered |
