# PR-2153: servo speed floor measurement and trajectory slow-path policy

This document mirrors the motion-design page section for slow STS paths. Fill measured
numbers on the robot using ``tools/measure_servo_speed_floor.py``; until then
``servo_speed_curve.py`` keeps ``None`` placeholders.

## Problem

Path-coupled trajectories (PR-2142) map path speed to an STS velocity setting via a measured
curve. Below setting 200 the curve was extrapolated to the origin. The servo accepts settings
only down to 50, so a slow path can force a setting whose real cruise rate is several times
faster than the path. The joint then reaches each 10 ms tick early and motion **steps**.

## Measurement

Run inside the **ros-motors** container on powered hardware:

```bash
source /opt/ros/jazzy/setup.bash && source /app/ros2_ws/install/setup.bash
python3 tools/measure_servo_speed_floor.py --joint turn_head_motor --family sts3095
python3 tools/measure_servo_speed_floor.py --joint index_right_stretch --family finger
```

- Velocity changes go through ``apply_motor_settings`` (not HTTP PUT).
- Moves are single-point ``apply_joint_trajectory`` commands.
- Cruise rate uses ``get_joint_position`` polling (default 30 Hz); state the poll rate in logs.
- After an SSR power cycle, wait until ``/joint_states`` is non-zero before measuring.

Paste printed rows into ``LOW_SPEED_CURVE_BY_FAMILY`` in ``servo_speed_curve.py``, then set
``MIN_CONTINUOUS_PATH_SPEED_CDPS_BY_FAMILY`` and ``MIN_RELIABLE_MOVE_TICKS_BY_FAMILY`` from the
script summary.

## Setting 0

Do not command velocity 0 until ``STS_SPEED_SETTING_ZERO_MEASURED_BEHAVIOR`` is filled in.
On some STS firmware generations 0 means **maximum** speed, not stop.

## Trajectory policy

When a software trajectory’s minimum path speed falls below
``MIN_CONTINUOUS_PATH_SPEED_CDPS_BY_FAMILY`` for a joint’s servo family, the executor emits
``[PERF_TRACE] TRAJECTORY_SLOW_PATH`` and **still runs** (``SLOW_PATH_BELOW_FLOOR_POLICY =
warn``). Rejecting would break valid slow motions without removing the physical limit; the trace
records the compromise. Switch to ``reject`` only if product owners prefer hard failure over
stepping.

Confluence: [Bewegungssteuerung in pib](https://pib-rocks.atlassian.net/wiki/spaces/kb/pages/1082753029/Bewegungssteuerung+in+pib+Konzepte+Industriestandards+und+Optionen)
