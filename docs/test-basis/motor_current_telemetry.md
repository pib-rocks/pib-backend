# Test Basis: Motor Current Telemetry Timer

**Repository:** `pib-backend`  
**Requirement:** Jira PR-1510  
**Components:** `ros_packages/motors/motors/motor_current.py`, Cerebra UI motor currents display

## Requirement

`MotorCurrent` node must schedule a periodic 1.0-second ROS timer in `__init__` calling `publish_motor_current()`, ensuring motor current telemetry is published regularly to `/motor_current` and displayed in Cerebra UI.

## Acceptance Criteria Traceability

| AC | Acceptance criterion | Coverage | Status |
|---|---|---|---|
| AC1 | `MotorCurrent` node schedules a 1.0-second timer calling `publish_motor_current()` in `__init__`. | `tests/unit/test_motor_current_timer.py` | Planned |
| AC2 | Motor current telemetry is published periodically to `/motor_current`. | `tests/unit/test_motor_current_timer.py` | Planned |
| AC3 | Unit tests in `tests/unit/test_motor_current_timer.py` verify timer scheduling and publishing. | `tests/unit/test_motor_current_timer.py` | Planned |
