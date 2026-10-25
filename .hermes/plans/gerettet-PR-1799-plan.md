# PR-1799 — facemesh overlay, and the generalisation the other crop models need

Branch `PR-1799`, worktree `~/opencode/pib-backend-PR-1799`, base `origin/develop` `64b9b5b0`.
Cerebra side: worktree `~/opencode/cerebra-PR-1799` on the same base discipline.

Read the story comment (Jira PR-1799) first: it carries the blob facts, the decidable
tesselation source and the live scope check. Then follow the PR-1809 plan in
`~/opencode/pib-backend-PR-1809/.hermes/plans/PR-1809-plan.md` for the chain's shape - this
story builds directly on it and inherits all of its traps.

## Why this story is worth more than its title

The face-crop chain from PR-1809 is hard-wired to one classifier in three places:

* `_build_face_crop_pipeline` (stereo.py ~2094) requires exactly
  `{"face_detection_yunet_160x120", "emotion_recognition_lfw_64x64"}` and fetches that
  classifier from the registry by name.
* the crop target size comes from that classifier's `input_width/height` (already generic).
* `_publish_face_crop_detection` calls `translate_emotion(...)` and stamps
  `model_id = "emotion_recognition_crop"` on the message.

So generalising it is step one: the classifier artifact, its translation and the model id /
topic must all come from the composite entry instead of from the source. Four of the six
crop-consuming stories (PR-1800, PR-1802, PR-1803, PR-1804) depend on exactly this
generalisation, which is why it belongs in this story and not in a later one.

**Constraint: the emotion behaviour must not change.** Every existing emotion test stays
green, and the published message for `emotion_recognition_crop` stays bit-for-bit what it is
today. The generalisation is a refactor plus one new classifier.

## Model facts (read from the blob, do not assume)

`models/facemesh_192x192/facemesh_192x192.blob`, via `dai.OpenVINO.Blob`:

```
numShaves: 4
inputs : {'input_1': [192, 192, 3, 1]}
outputs: {'conv2d_31': [1, 1, 1, 1], 'conv2d_210': [1404, 1, 1, 1]}
```

1404 = 468 x 3: the MediaPipe mesh with x, y, z per point. `conv2d_31` is the detector
confidence the model exports; ignore it unless you find it useful for a score.

Two things to establish **empirically on the device**, not by assumption, and to state in the
PR with the numbers you measured:

1. Are the 1404 values normalised (0..1) or in pixels of the 192x192 crop? If pixels, divide
   by 192 before mapping.
2. Does the z coordinate need the same treatment (it is relative depth, not normalised).

## What to build

### 1. Generalise the chain (no behaviour change for emotion)

A mapping from classifier artifact id to its translation function, plus deriving the
required artifacts and the published model id / topic from the composite. Keep the padding
and geometry helpers exactly as they are. If you add a per-classifier table, make an unknown
classifier fail loudly at build time rather than silently publishing nothing.

### 2. The facemesh translation

`conv2d_210` -> 468 keypoints, x/y/z, into `DetectionArray` with:

* `label`: something stable, e.g. `Face` (the overlay keys the skeleton off the model id, not
  the label - check `camera.component.ts` before choosing).
* `keypoint_names`: 468 names, `landmark_0` ... `landmark_467`, in the model's own order.
* keypoint x/y mapped from the crop back into the published frame, the same way
  `_hand_mp_detection` does it for the hand chain - read that function and reuse its geometry
  rather than inventing a second mapping. The crop carries `FACE_CROP_PADDING`, so the
  padding is part of the mapping.
* the box: the detected face's box, exactly as the emotion translation publishes it.
* z: pass the model's z through in `keypoint_z`. The overlay draws x/y only, but the z stays
  in the message (same decision as the hand chain).

### 3. Manifest

A composite entry following the emotion one: detector `face_detection_yunet_160x120` plus
classifier `facemesh_192x192`, crop target 192x192, `publish_topic: detections/<id>`,
`selectable` left as it is elsewhere in the file. Name it after the pattern of
`emotion_recognition_crop`.

### 4. Cerebra: the mesh topology

`src/app/camera/detection_topology.ts` already carries `topologyConnections` per model id
(from cerebra PR #190). Add the face mesh segments for the new model id.

The source is the canonical table, already validated: `google-ai-edge/mediapipe`,
`mediapipe/python/solutions/face_mesh_connections.py`, constant `FACEMESH_TESSELATION` - 2556
index pairs stored as triangles, maximum index 467, so it covers all 468 points. Because they
are stored as triangles, **every interior edge appears twice**: deduplicate before drawing or
the mesh renders twice as thick on those edges. Keep the generator (or the fetched table plus
a short dedup script) next to the data so the provenance is visible, and add a Cerebra unit
test that the segment list has no duplicates and no index >= 468.

## Verification to report verbatim

1. `python3 -m pytest tests/unit -q` in pib-backend (all existing tests plus yours)
2. `python3 -m black --check --include '\.py$' .` in pib-backend
3. the Cerebra test command you use, plus `ng build` if it is quick enough

The reviewer does the robot deploy and the live measurement (the Pi is shared and running a
user-facing model right now); do not touch it.

## Hard constraints

* Do not change the emotion path's output, the camera rate, the shave budgets, or the stereo
  rule (`PIB_CAMERA_STEREO=auto` = depth while idle, colour only with a model running).
* No device-side Script node: measured as not viable on this board.
* The crop input bound stays at 4 (`FACE_CROPPER_QUEUE`): a single held frame drops seven of
  eight crops when pairing is by exact timestamp.
* Do not add the new composite id to the verification branch in a way that waits for a keypoint
  result - the facemesh only answers when a face is present, exactly like the classifier proves
  the emotion chain. The detector stage proves the chain.
* Stay on your branch: no merge, no push, no branch switching.
* Report real command output, never a summary of what you intended to run.
