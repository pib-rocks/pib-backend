# Implementation Plan: PR-1510 Periodic Motor Current Telemetry Timer

**Repository:** `pib-backend`  
**Requirement:** Jira PR-1510  
**Branch:** `PR-1510`  

## Root Cause
In `ros_packages/motors/motors/motor_current.py`:
`publish_motor_current()` polls current for all motors and publishes to `/motor_current`, but it was never scheduled on a ROS timer in `MotorCurrent.__init__()`.

## Fix
In `ros_packages/motors/motors/motor_current.py`:
Add `self.timer = self.create_timer(1.0, self.publish_motor_current)` in `MotorCurrent.__init__()`.

## Unit Tests
Create `tests/unit/test_motor_current_timer.py` to verify that `MotorCurrent` creates a 1.0-second timer and that calling `publish_motor_current()` publishes `DiagnosticStatus` for connected motors.
