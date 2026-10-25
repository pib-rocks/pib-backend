# PR-1660: motor_control as software trajectory controller (multi-point + time_from_start, interpolation, via-point blend)

Ticket: https://pib-rocks.atlassian.net/browse/PR-1660
Repo: pib-backend — /home/pib/opencode/pib-backend, branch `PR-1660` (from origin/develop `ccea2b71`).
Instruction: implement; ON SUCCESS create a PR PR-1660 -> develop for the user to review & merge manually (do NOT merge).

## Goal
Extend the ROS `motor_control` node so it can execute a **multi-point JointTrajectory including
`time_from_start`**, interpolating in a fixed-rate software loop and blending at via-points, giving
smooth synchronized real-time motion. Backward compatible: existing single-point calls keep working.

## Verified facts (read this repo)
- `ros_packages/datatypes/srv/ApplyJointTrajectory.srv`: `trajectory_msgs/JointTrajectory joint_trajectory` / `bool successful`.
- `ros_packages/motors/motors/motor_control.py`:
  - `as_motor_positions(jt)` (line 30): reads `joint_names` + `points[0].positions[0]` only → single-point.
  - `apply_joint_trajectory` (line 174): iterates `as_motor_positions`, calls `motor.set_position(position)` per motor, publishes echo, sets `successful`.
- `ros_packages/motors/pib_motors/pib_motors/motor.py`:
  - `set_position(position)` (line 55): all bricklet-pins `set_position(position)`.
  - `get_position()` (line 64): returns target position (last commanded) or 0 if disconnected.
  - `has_reached_position()` (line 76): True if all pins reached target.
  - `check_if_motor_is_connected()` (line 88).
- `ros_packages/motors/pib_motors/pib_motors/startup_pose_executor.py`: shows the **settings backup/reduce/restore** pattern via `_backup_motor_settings()`, `_reduce_motor_speeds()` (velocity reduced + turnedOn False), `_restore_settings()` in a finally. Reuse this pattern for sharpening/restoring motion config during a trajectory.
- Bricklet speed control only via `set_motion_configuration(velocity, acceleration, deceleration)` (bricklet_pin.py apply_settings).

## Design

### New: `ros_packages/motors/pib_motors/pib_motors/trajectory_executor.py`
A `TrajectoryExecutor` class that owns the per-motor fixed-rate interpolation loop. Interface:

```python
class TrajectoryExecutor:
    def __init__(self, node, motor_lookup, rate_hz=100, motion_sharp: dict | None = None, restore: bool = True): ...
    def execute(jt: JointTrajectory) -> bool:
        # 1. validate: for each joint_names entry there is a known Motor; gather points.
        # 2. if len(points) > 1 and any time_from_start set -> multi-point mode:
        #    a. backup current motor settings (motion config) for the involved motors.
        #    b. apply 'sharp' motion config (high velocity/accel/decel) so software spline shapes speed.
        #    c. fixed-rate tick loop (rate_hz): at each tick, for the current time, compute the
        #       interpolated position for each motor between its via-points and set_position(it).
        #       Hold the final point when beyond the last time_from_start.
        #    d. finally restore the backup settings.
        # 3. else (single-point, no meaningful time) -> fall back to existing per-motor set_position once
        #    (backward compatible).
        # 4. return success.
```

- Interpolation: **joint-space interpolation per motor** over the provided via-points at `time_from_start`.
  Use linear between consecutive points OR a simple cubic (Catmull-Rom or monotone cubic). At least linear
  with velocity-continuous blending desired; prefer a small cubic for continuity at via-points.
- Via-point blend: to avoid full stops, don't require each point's full stop; interpolate through the interval
  with tangent continuity (e.g., monotone cubic Hermite). If linear is chosen, note the limit; implement blend
  if feasible in scope (the ticket says via-point blending).
- The loop runs in the node's process (spin), not blocking the ROS executor — run the loop on a dedicated thread
  with rospy/rclpy-safe sleep or schedule via `create_timer` at rate_hz. Prefer rclpy `create_timer` on the node
  for clean shutdown, or a daemon thread if simpler and clearly bounded.
- Settings for 'sharp': set `velocity`, `acceleration`, `deceleration` high (e.g. velocity 30000+) via
  `motor.apply_settings({...original, velocity:<sharp>, acceleration:<sharp>, deceleration:<sharp>})`. Restore
  the exact backup in a finally.

### Modify `motor_control.py`
- `apply_joint_trajectory`: if the request carries multiple points / time_from_start (a real trajectory),
  delegate to `TrajectoryExecutor.execute(jt)`. Else keep the existing single-point behavior
  (`as_motor_positions` + `set_position` once) unchanged → backward compatible.
- Keep the echo publisher and `successful` semantics.

### Tests
- Add `tests/unit/test_trajectory_executor.py` (pytest) — OFFLINE, with a fake/mock Motor object that records
  `set_position`/`apply_settings` calls and a fake `time_from_start`. Assert:
  1. multi-point trajectory calls set_position for the interpolated points in order (>= 2 calls, monotonic in time),
  2. motion config is sharpened before and restored after (apply_settings called with sharp then original),
  3. single-point path stays backward compatible (exactly one set_position, no sharp/restore), 
  4. via-point blending does not leave a full-stop (positions mid-segment are interpolated, not clamped).
  Run: `cd /home/pib/opencode/pib-backend && python3 -m pytest tests/unit/test_trajectory_executor.py -q` (
  use repo's venv if present; the code must be importable — likely needs `ros_packages` on sys.path OR run from
  within a package context. If pure ROS import is needed, gate the test to skip when rclpy not installed, OR
  structure TrajectoryExecutor to be dependency-light / testable with minimal imports).

## Hard constraints
1. Stay on branch `PR-1660`. Do NOT merge, do NOT push develop. You MAY commit (`feat: PR-1660 ...`). The
   orchestrator opens the PR after verifying.
2. Do NOT change existing single-point behavior. Do NOT change ApplyJointTrajectory.srv schema (standard
   trajectory_msgs already carries time_from_start). Do NOT break startup_pose_executor.py.
3. Reuse the settings-backup/restore pattern from startup_pose_executor.py (do not invent a parallel one).
4. Keep it readable; add module docstrings; type hints.
5. Do NOT commit `.hermes/plans/**`. Do NOT commit if tests fail.
6. Report REAL results.

## Verify (orchestrator)
- `python3 -m pytest tests/unit/test_trajectory_executor.py -q` passes (with a meaningful multi-point test).
- `git diff --stat origin/develop...PR-1660` shows trajectory_executor.py (new), motor_control.py, the test file.
- `grep -c "time_from_start" trajectory_executor.py` > 0; single-point path in motor_control unchanged.
- No routes/config/other modules changed.