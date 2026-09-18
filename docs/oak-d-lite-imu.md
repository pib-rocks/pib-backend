# OAK-D Lite IMU ROS interface

The camera node is the sole owner of the OAK-D Lite and publishes its on-board
BMI270 as `sensor_msgs/Imu` on `/imu` at no more than 10 Hz. The IMU is soldered
to the camera PCB in pib's forward-facing head camera module; consequently
`oak_imu_frame` is located at the camera module, not at the robot base or torso
origin.

Published vectors use REP-103 body axes (x forward, y left, z up).
`linear_acceleration` is in m/s² and `angular_velocity` is in rad/s. Orientation
comes from DepthAI `ROTATION_VECTOR` and is fused on the device; the ROS node
does not run a second fusion filter. The BMI270 reports no covariance estimate,
so element zero of each covariance array is `-1` rather than a guessed value.

The ROS header stamp is converted from DepthAI `getTimestampSystem()`, the host
clock shared with ROS. `getTimestampDevice()` is retained by the node only as
the correlation source for a later hardware camera/IMU synchronisation step.

The existing `/models_status` `ModelStatusArray` includes an additional entry
with `model_id == "imu"` and state `present`, `absent`, or `stale`. A camera
variant without an IMU starts without that branch, keeps all camera topics
running, and reports `absent`.
