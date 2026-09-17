# PR-1701 imitation test basis

## Architecture and safety

`imitation` is an ament Python package and does not own the OAK device. The
long-lived node acquires `hand_tracking` from `/start_model`, subscribes to the
typed `/detections/hand_tracking` `datatypes/DetectionArray`, and releases its
stable per-process owner through `/stop_model` at shutdown.

The newest detection is coalesced at `max_update_hz`. Mapping, pose publication,
and IK rejection remain active in dry-run mode. Actuation is opt-in and
**disabled by default** (`enable_motion:=false`). When enabled, the first valid
IK solution only establishes a baseline; it does not send a command. Later
solutions become two-waypoint, full-joint `JointTrajectory` requests in
`ArmKinematics.motor_names` order, with positions in hundredths of a degree.
They use the existing `/apply_joint_trajectory`
`datatypes/ApplyJointTrajectory` service consumed by `TrajectoryExecutor`.
Only one trajectory request may be in flight.

No motion or live hardware test was run while implementing this package.

## Interfaces and parameters

Published observability topics:

- `/imitation/hand_pose` (`geometry_msgs/PoseStamped`, optical camera frame)
- `/imitation/target_pose` (`geometry_msgs/PoseStamped`, `robot_base`)

Both `PoseStamped` topics use metres as required by ROS geometry conventions;
the pure mapper and `pib_sdk` IK retain their documented millimetre units.

Main parameters and defaults:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `enable_motion` | `false` | Allow trajectory service requests |
| `arm` | `right` | SDK arm model (`left` or `right`) |
| `max_update_hz` | `10.0` | Callback coalescing/rate limit |
| `trajectory_duration_sec` | `0.25` | Absolute time of the second waypoint |
| `fallback_depth_mm` | `500.0` | Depth when no valid landmark Z exists |
| `palm_width_mm` | `80.0` | Scale reference for pixel X/Y |
| `min_detection_score` | `0.5` | Minimum accepted hand score |
| `target_origin_mm` | `[180,-180,220]` | Neutral point in pib base coordinates |
| `reference_depth_mm` | `500.0` | Neutral camera depth |
| `position_gain` | `1.0` | Hand-to-target translation gain |
| `use_orientation` | `false` | Include palm orientation in IK |

The mapping follows the neighboring camera package's canonical 21 MediaPipe
names, regardless of input array order. Palm centre is the mean of wrist and
the index/middle/pinky MCPs. Index-to-pinky defines palm X, wrist-to-middle
defines palm-up, and their cross product defines the normal.

The camera currently publishes `keypoint_z == 0`. Zero, negative, and non-finite
depth values are invalid. Each invalid value uses the median positive finite Z
from that hand; if none exists, it uses `fallback_depth_mm`. Pixel X/Y are
converted using the observed index-to-pinky width and `palm_width_mm`.

The selected optical-to-robot gesture mapping matches the existing SDK's
millimetre convention: camera depth controls robot X inversely, camera X
controls robot Y inversely, and camera Y controls robot Z inversely. The neutral
origin is configurable because no calibrated camera-to-base transform exists
in neighboring packages. Orientation is observable but position-only IK is the
default.

## Latency evidence

At debug log level each accepted frame records `processing_latency_ms`. If the
detection header has a non-zero stamp, it also records `source_latency_ms` from
the camera stamp through mapping/IK/request preparation. Callback coalescing
means stale intermediate frames are intentionally discarded.

The local synthetic benchmark uses only the pure landmark and target mapping
functions; it does not initialize ROS or contact hardware. The implementation
environment rejected process execution before the benchmark could start, so no
local number is claimed here. Pi end-to-end latency remains unmeasured.

## Pi-only operator commands

Build on the Pi:

```bash
cd ~/app/pib-backend
colcon build --base-paths ros_packages --packages-select datatypes imitation
source install/setup.bash
```

Non-actuating dry run on the Pi:

```bash
ros2 launch imitation hand_teleop.launch.py enable_motion:=false
ros2 topic echo /imitation/target_pose
```

Only after workspace checks, clearance around the robot, and an operator is
ready at the stop control, run the supervised motion trial:

```bash
ros2 launch imitation hand_teleop.launch.py enable_motion:=true arm:=right
```

`colcon build`, Pi end-to-end latency, and the supervised motion run were not
performed here. In particular, **“the robot follows the hand smoothly” is
unverified pending that supervised run**.

## Legacy replacement

`setup/setup-pib.sh` no longer clones or installs `~/imitation`. That retired
host script directly owned the OAK camera and must not run alongside the ROS
camera owner. The in-repository package consumes the camera owner's typed topic
instead and has no camera-runtime dependency.
