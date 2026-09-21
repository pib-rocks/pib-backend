# PR-1809 — face crop chain for the crop-consuming models

Branch `PR-1809` (worktree `~/opencode/pib-backend-PR-1809`), base `origin/develop`
`f603167f` (contains PR-1798 and PR-1805).

## The problem

Six selectable models take a **face crop**, not a full frame:
`emotion_recognition_lfw_64x64` (64x64), `face-recognition-arcface-112x112`,
`gaze-estimation-adas-0002`, `head-pose-estimation-adas-0001`,
`facial_landmarks_68_160x160` and `facemesh_192x192`. Feeding them a 1280x720 frame
produces garbage, so their overlay stories (PR-1799..PR-1804) cannot be single-network
stories. This story builds the shared two-stage chain.

## The pattern to copy (already in develop, and measured on this device)

`imitation.py` + the hand chain in `stereo.py` do exactly this shape:

- `imitation.py:13` `ProcessDetections(dai.node.HostNode)` — turns every parsed
  detection into a timestamped `dai.ImageManipConfig` for the device-side crop.
- `stereo.py` `_build_hand_mp_pipeline` on branch `PR-1791-hand-mp` (NOT in develop yet,
  open as PR #336 — read it as the reference implementation): palm detector via
  `ParsingNeuralNetwork` (`task_archives.py` builds the archive), `ProcessDetections`
  into `FrameCropper.fromManipConfigs(...)`, then the second network. It also contains
  the measured traps listed below.
- `task_archives.py` — `create_archive(model_id, blob_path)`; PR-1805 added the YOLO
  entry, PR-1798 the YuNet one. The face detector's archive already exists
  (`face_detection_yunet_160x120`).
- `parsed_detections.py` — `translate_detections(...)`, the DetectionArray translation.
- `stereo.py:828` `_publish_parsed_detections` and the `detection_publishers` dict built
  from `publish_topic` (manifest-driven).

## Traps that are already paid for — do not rediscover them

1. **A new composite model id must be known to `stereo.py:2130` `_verify_model_frames`.**
   That function accepts `hand_tracking`, `imitation`, `hand_tracking_mp` and otherwise
   demands the model appear in `nn_queues`. A composite has no such entry, so the start
   fails with the misleading `no frames arrived within 3.0s` while the chain is running.
   Add the new id to the branch **and** to the generic `not in (...)` tuple, and wait on
   the chain's own queue rather than only the colour frame.
2. **The crop input is bounded at 4 frames, never 1.** Crops are paired to frames by
   exact timestamp; a single-slot input drops the very frame a config refers to
   (measured: 7.85 detections/s in, 1.00 results/s out).
3. **No `dai.node.Script`.** A Script node reboots this device without a Python error.
4. **Do not request a camera branch with its own fps** while the node's raw ISP output
   exists: a second rate on the same camera leaves the colour queue empty.
5. **Parsed detections carry a `RotatedRect`** (`getBoundingBox()`), not x_min/y_min/...
6. `PIB_CAMERA_STEREO=auto` means depth while idle; a running model means colour only.
   Do not change that.

## What to build

1. A composite in `models/manifest.yaml`, e.g. `emotion_recognition_crop`:
   artifacts `face_detection_yunet_160x120` + `emotion_recognition_lfw_64x64`,
   `publish_topic: detections/emotion_recognition_crop`, `selectable: true` is NOT
   wanted (the crop chain is one model id the user selects; decide and document it),
   and the shave total that the two blobs need (read both blob headers, do not guess).
2. `stereo.py`: a `_build_face_crop_pipeline(composite)` mirroring the hand chain -
   YuNet through `ParsingNeuralNetwork`, `ProcessDetections` with the face-crop padding,
   `FrameCropper`, then the emotion model. The second stage should be a **plain**
   `dai.node.NeuralNetwork` plus a host softmax/argmax with the class list, not a
   parsing archive: it keeps the archive work out of this story and mirrors how the hand
   chain reads its raw output.
3. The translation: one `DetectionArray` per detected face with `label` = the winning
   emotion, `score` = its probability, the **face** box in frame pixels, no keypoints,
   and `scalar_names/scalar_values` carrying the per-class probabilities.
4. Tests: the crop-config generation (padding, target size, timestamp), the softmax
   argmax translation, and that a model without an archive still takes the old path.

## Verification to report verbatim

1. `python3 -m pytest tests/unit -q`
2. `python3 -m black --check --include '\.py$' .`
3. Live on the robot (the Pi is reachable, sshpass password file `/tmp/.pipw`, the camera
   container is `multirepo-ros-camera-1`, the store at `/home/pib/app/pib-models`):
   deploy by copying the changed files to `/home/pib/app/pib-backend`, then
   `docker compose --profile all build ros-camera && docker compose --profile all up -d --force-recreate ros-camera`,
   then `docker compose restart angular-app` in `/home/pib/app/cerebra`, then start the
   model through the service and capture the topic **natively inside the container**
   (`ros2 topic hz`, or an rclpy subscriber) - rosbridge is unreliable for large payloads.
   Report the rate and one full message.

## Hard constraints

- Only this chain. Do not touch the hand chains, other models, shave budgets of existing
  models, the camera rate, or the stereo rule.
- Stay on `PR-1809`: no merge, no push, no branch switching.
- Report real command output.
