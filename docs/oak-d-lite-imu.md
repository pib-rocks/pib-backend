# OAK-D Lite IMU ROS interface

The camera node is the sole owner of the OAK-D Lite and publishes its on-board
BMI270 as `sensor_msgs/Imu` on `/imu` at no more than 100 Hz. The IMU is soldered
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
is a separate, not yet taken decision; the 100 Hz publication rate makes it
practical, but the gyro drift would still need its own treatment.

The BMI270 reports no covariance estimate, so element zero of each covariance
array is `-1` rather than a guessed value.

## Timestamps

The ROS header stamp is the **measurement instant, expressed on the host
timeline**: `header.stamp = device timestamp + estimated clock offset`.

That construction exists because no clock on the reports is both uniformly
spaced and host wall time. Measured on the depthai 3.6.1 build in the camera
container, the IMU reports expose `getTimestamp()` (a `timedelta` "related to
`dai::Clock::now()`", so not an epoch stamp) and `getTimestampDevice()` (device
monotonic clock, explicitly not synchronized to host time), but no
`getTimestampSystem()`. The device clock is uniform but is a duration since
device boot; the host receipt time is real wall time but is quantised by the
poll timer and inflated by however long the report waited in the device queue.
Publishing receipt times produced the measured symptom this change removes: a
correct average rate with intervals between 35 ms and 200 ms and occasional
duplicate stamps.

The offset is estimated from the reports themselves. Every report contributes one
observation, `host receipt - device timestamp`, and the estimate is the
**minimum** over a sliding window of one second of reports. The minimum, not the
mean: transport, batching and the poll timer can only make a report arrive
later, never earlier, so the smallest observation is the one with the least
added latency, while a mean would track the average queueing delay and shift
every stamp into the future by it.

Accuracy limits, none of which this estimate hides:

- The offset can be no better than the fastest report in the window. The
  residual error is that report's remaining one-way latency, which is not
  observable from the host and is not corrected.
- A minimum only falls; it follows the clocks drifting apart no faster than the
  window discards old samples, so the estimate lags real drift by up to one
  window.
- The estimate is a pure offset. The device and host crystals run at slightly
  different rates and no skew term is fitted, so intervals between published
  stamps carry the device's notion of a second, not the host's.
- Precision is nanoseconds, resolution is the sensor period; the stamp does not
  claim to resolve anything finer than the 200 Hz sampling.

**Fallback.** While no offset can be established - fewer than two observations,
which is the case for the first report after a pipeline start - the host receipt
time is published instead: late by the transport latency, but real host time. The
same fallback catches a stamp that lands more than one second away from the
receipt time, which is what a device clock restart (a new device session begins
near zero) looks like. A device duration is therefore never published as wall
time, and both the throttle state and the offset window are discarded whenever
the pipeline is rebuilt.

`getTimestampDevice()` remains the correlation source for future camera/IMU
hardware alignment; it is also what the publication throttle selects on, because
it is spaced uniformly by the sensor.

## Rate and its cost

The sensor is configured for 200 Hz raw reports and the host deterministically
drops every second one to publish at 100 Hz, so a single dropped or late report
cannot make the throttle skip a 10 ms slot. The device-side queue holds 20
reports (100 ms at 200 Hz), the poll timer runs once per publication period
(10 ms), and the drain limit (256) exceeds the queue depth, so one tick always
empties a full queue.

This costs more than the previous 10 Hz did: ten times the number of host
iterations, ROS publications and rosbridge messages, and twice the number of USB
reports. The poll timer shares the node's single-threaded executor with the
colour-frame timer, so the IMU period is only honoured between colour encodes.
`/models_status` therefore reports the **measured** rate for `model_id == "imu"`
rather than the configured one - that field exists precisely because the rig once
measured 8.1 Hz while the configuration said 10 Hz. The `stale` threshold is
likewise tied to the period: 50 missed publications (0.5 s), instead of a round
second that at 100 Hz would mean a hundred lost samples.

The resulting CPU load on the Pi is a rig measurement and is deliberately not
stated here; it belongs in the acceptance notes of whoever measures it.

The pib SDK is deliberately unchanged and keeps `latest()`: rosbridge, not the
publication rate, is the limit for a consumer that wants every sample.

The existing `/models_status` `ModelStatusArray` includes an additional entry
with `model_id == "imu"` and state `present`, `absent`, or `stale`. A camera
variant without an IMU starts without that branch, keeps all camera topics
running, and reports `absent`.
