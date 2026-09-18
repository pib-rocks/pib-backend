# OAK-D Lite IMU ROS interface

The camera node is the sole owner of the OAK-D Lite and publishes its on-board
BMI270 as `sensor_msgs/Imu` on `/imu` at no more than 10 Hz. The IMU is soldered
to the camera PCB in pib's forward-facing head camera module; consequently
`oak_imu_frame` is located at the camera module, not at the robot base or torso
origin.

Published vectors use REP-103 body axes (x forward, y left, z up).
`linear_acceleration` is in m/s² and `angular_velocity` is in rad/s.

**Orientation is not published.** The BMI270 on the OAK-D Lite refuses every
fused output: requesting `ROTATION_VECTOR` makes the device answer
`IMU invalid settings!: ROTATION_VECTOR output is unsupported. BMI270 supports
only ACCELEROMETER_RAW and/or GYROSCOPE_RAW outputs.` and that single rejected
sensor setting takes the whole pipeline start down, so only `ACCELEROMETER_RAW`
and `GYROSCOPE_RAW` are requested. The quaternion therefore stays at identity
and `orientation_covariance[0]` is `-1`, the sensor_msgs/Imu convention for
"orientation not available" — consumers must not read the quaternion as a
measurement. Fusing orientation on the host (complementary or Madgwick filter)
is a separate, not yet taken decision; at a 10 Hz publication rate it would be
coarse and the gyro drift would need its own treatment.

The BMI270 reports no covariance estimate, so element zero of each covariance
array is `-1` rather than a guessed value.

The ROS header stamp is the host wall-clock time at which the node received the
report, because that is the only host-time value available: measured on the
depthai 3.6.1 build in the camera container, the IMU reports expose
`getTimestamp()` (a `timedelta` "related to `dai::Clock::now()`", so not an epoch
stamp) and `getTimestampDevice()` (device monotonic clock, explicitly not
synchronized to host time), but no `getTimestampSystem()`. A device duration is
never reinterpreted as wall time. The drain interval bounds the stamping delay,
so a consumer should treat the stamp as host time with that latency rather than
as the exact measurement instant; `getTimestampDevice()` is retained by the node
only as a future camera/IMU hardware-correlation source.

The existing `/models_status` `ModelStatusArray` includes an additional entry
with `model_id == "imu"` and state `present`, `absent`, or `stale`. A camera
variant without an IMU starts without that branch, keeps all camera topics
running, and reports `absent`.
