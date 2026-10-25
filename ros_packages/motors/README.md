# pib motors (ROS)

## Joint position units

| Layer | Unit | Notes |
| --- | --- | --- |
| ROS edge (`/apply_joint_trajectory`, `/joint_trajectory`) | **radians** | Matches `trajectory_msgs/JointTrajectory` and standard tools (rviz, plotjuggler). |
| HTTP API, database, `motor_control` logic, `TrajectoryExecutor` | **centidegrees** | Integer hundredths of a degree; ±9000 ≈ ±90°. |
| Feetech / bricklet drivers | **ticks** | See `pib_motors/feetech_actuator.py`. |

All conversions between radians and centidegrees go through `pib_motors.units`
(`rad_to_centideg`, `centideg_to_rad`). Convert at the ROS service entry and at
each `/joint_trajectory` publish site—never inside interpolation or motor loops.

Incoming `/apply_joint_trajectory` requests with non-finite positions or
centidegree equivalents outside a joint's configured range are rejected (not
clamped).
