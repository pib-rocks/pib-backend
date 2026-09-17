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

### Curated-registry measurement

`tools/measure_on_device_models.py` performs a non-actuating, sequential run of
the available registry. It refuses to start if the available-entry count is not
13 or if `/list_models` reports an already-active entry. For every entry it:

1. requests the exact SHAVE count returned by `/list_models`;
2. records `/models_status` transitions, active state, and measured FPS;
3. counts `/detections/<model_id>` messages for the measurement window;
4. captures camera-container stage-counter and crash/reconnect evidence from
   the entry's remote timestamp onward; and
5. attempts to release the owner with `/stop_model` twice after a start or
   observation error, and aborts before the next entry if release is not
   confirmed.

Run it from a workstation that has `websocket-client`, `ssh`, and `sshpass`:

```bash
PIB_ROBOT_SSH_PASSWORD=pib python3 tools/measure_on_device_models.py \
  --host 192.168.1.92 \
  --output PR-1740-model-measurements.json | tee PR-1740-model-measurements.log
```

Each `RESULT` line is compact JSON containing the raw per-entry values and log
excerpts, including the remote Docker-log cutoff. The full formatted record is
written to `--output`. A zero detection count is not itself a failure:
standalone registry networks currently expose physical packet flow as nonzero
status FPS, while only entries with a declared publish topic emit
`DetectionArray`. A `confirmed` SHAVE basis means the pipeline produced packets
while running with the exact compiled allocation declared by the registry; it
does not claim to measure otherwise-idle hardware capacity. Likewise, the
`hand_tracking stage packets total: ...` line is specific to that composite
pipeline; its absence is retained as an empty list rather than interpreted as a
passing signal.

### Measured matrix (live run, 17 Sep 2026)

The initial sequential, non-actuating run measured all 14 entries with the
harness above (`--measure-seconds 8`); every entry was released with
`/stop_model` before the next one started. The failed gaze entry is now retained
in `/list_models` as unavailable, so subsequent curated-registry runs measure
the 13 functional entries.

| model_id | shaves | start | final | active | fps | msgs | warn | released | shave basis |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| palm_detection_128x128 | 4 | yes | running | yes | 84.6 | 0 | ok | yes | confirmed: requested=4, status=4, fps=84.593 |
| palm_detection_128x128_decoding | 1 | yes | running | yes | 133.2 | 0 | ok | yes | confirmed: requested=1, status=1, fps=133.155 |
| hand_landmark_224x224 | 4 | yes | running | yes | 86.8 | 0 | ok | yes | confirmed: requested=4, status=4, fps=86.84 |
| face_detection_yunet_160x120 | 4 | yes | running | yes | 136.5 | 0 | ok | yes | confirmed: requested=4, status=4, fps=136.484 |
| facemesh_192x192 | 4 | yes | running | yes | 167.2 | 0 | ok | yes | confirmed: requested=4, status=4, fps=167.25 |
| facial_landmarks_68_160x160 | 4 | yes | running | yes | 142.5 | 0 | ok | yes | confirmed: requested=4, status=4, fps=142.483 |
| face-recognition-arcface-112x112 | 4 | yes | running | yes | 57.6 | 0 | ok | yes | confirmed: requested=4, status=4, fps=57.598 |
| emotion_recognition_lfw_64x64 | 4 | yes | running | yes | 168.9 | 0 | ok | yes | confirmed: requested=4, status=4, fps=168.944 |
| gaze-estimation-adas-0002 | 4 | yes | running (ignored) | yes (ignored) | 0.0 | 0 | ok | yes | non-functional: multi-input model received only the generic single image input; 4-SHAVE runtime flow unconfirmed |
| head-pose-estimation-adas-0001 | 4 | yes | running | yes | 132.4 | 0 | ok | yes | confirmed: requested=4, status=4, fps=132.437 |
| yolov6n_coco_640x640 | 4 | yes | running | yes | 11.0 | 0 | ok | yes | confirmed: requested=4, status=4, fps=11.013 |
| qr_code_detection_384x384 | 4 | yes | running | yes | 76.0 | 0 | ok | yes | confirmed: requested=4, status=4, fps=75.969 |
| person-reidentification-retail-0031_96x48 | 4 | yes | running | yes | 129.8 | 0 | ok | yes | confirmed: requested=4, status=4, fps=129.769 |
| hand_tracking | 9 | yes | running | yes | 1.0 | 6 | ok | yes | confirmed: requested=9, status=9, fps=1.001 |

Findings from the run:

* **All 13 functional entries produced packet flow and released cleanly.** No crash, no `X_LINK_ERROR`
  and no reconnect warning occurred in the initial 14-entry run - the camera stayed on the stereo
  pipeline (`Stereo depth available - full colour + stereo pipeline active (mode=auto)`) throughout.
* **Shave budgets are confirmed for the 13 functional entries**: the pipeline produced packets while running with
  exactly the allocation declared in the registry (per-network 4 / 1 / 4; composite 9 = 4+1+4). The
  `shave_basis` column records the basis: requested shaves, shaves the status reported, and the observed
  FPS.
* **`gaze-estimation-adas-0002` is non-functional and unavailable**: its two eye-image inputs and
  head-pose-vector input cannot be supplied by the generic single-image pipeline. The initial run's
  status said `running`/`active`, but the arbiters were FPS **0.0**, zero published messages, and no
  model-specific stage-counter evidence. Its 4-SHAVE value describes the compiled blob and is not a
  validated runtime budget. `/start_model` rejects it with the manifest reason.
* `published_messages` is 0 for every standalone network by design - they have no `/detections/<id>`
  publisher, so their packet flow appears as the reported FPS. Only the composite `hand_tracking` published
  messages (6 `DetectionArray` messages in its window).
* The FPS column is the **pipeline's own reported rate** (e.g. 84.6 for palm detection, 11.0 for
  yolov6n_coco_640x640), not a benchmark of otherwise-idle hardware capacity.
* One registry id differs in spelling from the curated list: the registry exposes
  `person-reidentification-retail-0031_96x48` (the ticket draft said `..._96`) - read ids from
  `/list_models`, never from prose.
