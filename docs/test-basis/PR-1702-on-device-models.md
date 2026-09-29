# PR-1702 on-device model test basis

## Scope and safety

Coverage exercises registry contracts, model reference counting, live
start/stop, detection/status messages, and a Blockly program that records a
named pose. Recording a pose only reads current motor telemetry.

Tests must never call `setPose`, `move_to_pose`, `apply_pose`,
`play_pose_sequence`, `/apply_joint_trajectory`, or publish a trajectory.
The Cerebra overlay E2E belongs to the Cerebra repository and is out of scope
for this backend PR.

## Automated coverage

| Level | Test | Evidence |
| --- | --- | --- |
| Unit | `test_camera_model_lifecycle.py` | Manifest parsing, fixed per-network SHAVEs, owner reference counts, failure/recovery, invalid requests, and exact model service/status contracts |
| Unit | `test_hand_tracking.py` | Palm/landmark coordinate behavior and exact generic detection message fields |
| Generator | Jest `model_generator.test.ts`, `pose_generator.test.ts` | Generated start/read/stop and named pose-recording code |
| Live E2E | `test_on_device_models_e2e.py` | List/start/status/detection/stop and compiled Blockly-to-pose chain |

## Live scenarios

### Model lifecycle and topics

```gherkin
Given pib-api and rosbridge are reachable on the live robot
And hand_tracking is available in /list_models
When the test starts hand_tracking with shaves 0 and a unique owner
Then /models_status reports its active runtime state
And /get_detections returns a DetectionArray with the documented fields
And any streamed /detections/hand_tracking payload follows the same contract
When the test stops hand_tracking with the same owner
Then the stop service succeeds
```

No hand is required in view. An empty latest detection array is valid.

### Blockly detection-to-pose chain

```gherkin
Given a uniquely named Blockly program and pose
When the stored program starts hand_tracking
And reads the label from detection item 0
And records current motor telemetry under the unique pose name
And stops hand_tracking
Then RunProgram proxy feedback contains the printed detection value
And RunProgram proxy result has exit code 0
And GET /pose contains the unique pose name
```

If no detection arrives, the generated read helper must print `0` and feedback
must include `no detection 0 received from model 'hand_tracking'`. If a hand is
present, the `hand` label is printed instead. Program stdout/stderr is asserted
from `/proxy_run_program_feedback`; Docker logs are not an oracle.

The E2E subscribes before starting the program and cleans up its program and
pose records. It skips only when the HTTP API, rosbridge, or the local
`websocket-client` prerequisite is unavailable. A reachable robot with a
missing service, compilation error, failed assertion, or non-zero program exit
is a failure.

## Verification commands

```bash
python3 -m pytest tests/unit -q
python3 -m pytest tests/e2e -q --collect-only
python3 -m pytest tests/e2e/test_on_device_models_e2e.py -q -s
python3 -m black --check --include '\.py$' .
```
