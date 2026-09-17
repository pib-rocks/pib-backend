# On-device model operations

The `ros-camera` node owns the OAK device and the model pipeline. Other
components request models through ROS services; they must not open DepthAI or
read blob files directly.

## Runtime contract

| Interface | Contract |
| --- | --- |
| `/list_models` | Registry metadata: `model_id`, task, licence, SHAVEs, bytes, availability, active state |
| `/start_model` | `{model_id, shaves, owner}`; `0` uses the registry SHAVE value |
| `/stop_model` | `{model_id, owner}`; releases that owner's reference |
| `/get_detections` | Latest `DetectionArray` for a model |
| `/detections/<model_id>` | Typed streamed `DetectionArray` |
| `/models_status` | Approximately 1 Hz; `idle`, `starting`, `running`, or `failed`, plus FPS and active state |

Use a stable, non-empty owner per consumer and release every model at shutdown.
Multiple owners share one loaded model. A model remains requested until its
last owner releases it.

Detection bounding boxes and X/Y keypoints are pixels in
`frame_width × frame_height`. Keypoint Z is millimetres; zero means invalid or
unavailable. Names and values use parallel arrays, so adding a model does not
change the ROS message schema.

## Shipping another model

The authoritative registry, blobs, provenance, and detailed field checklist
are in [`../models/README.md`](../models/README.md). In summary:

1. Compile each network for Myriad-X and preserve its per-network SHAVE count.
2. Vend it at `models/<model_id>/<model_id>.blob`.
3. Add all provenance, compile, dimensions, hash, and size fields to
   `models/manifest.yaml`. Composite entries reference their artifact IDs and
   declare a publish topic.
4. Run `python3 models/verify_models.py`.
5. On the robot, run `./setup/setup-pib.sh --models` before starting
   containers, then `./setup/setup-pib.sh --verify-models`.
6. Start `ros-camera`; verify `/list_models`, `/models_status`, and the typed
   detection output with `shaves=0`.

The persistent host store defaults to `/home/pib/app/pib-models` (override with
`PIB_MODEL_STORE`) and is mounted read-only at `/models` in `ros-camera`.

## Safe verification

Model start/stop and camera observation are non-actuating. Saving a pose records
current motor telemetry and is also non-actuating. Verification must not call
`move_to_pose`, `setPose`, `apply_pose`, `play_pose_sequence`,
`/apply_joint_trajectory`, or send a trajectory.

The live pytest is:

```bash
python3 -m pytest tests/e2e/test_on_device_models_e2e.py -q -s
```

It skips when the robot API or rosbridge is unreachable, or when the local
`websocket-client` test prerequisite is absent. Override the default live
target with `PIB_MODEL_E2E_HOST`, `PIB_MODEL_E2E_API_URL`, and
`PIB_MODEL_E2E_ROSBRIDGE_URL`.
