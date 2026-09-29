# PR-1798 — overlay foundation: parser + DetectionArray publisher for plain models

Branch: `PR-1798` (worktree `~/opencode/pib-backend-PR-1798`), base `origin/develop` `38d04061`.

## Verified facts (read in this worktree)

**The gap.** `models/manifest.yaml` gives a `publish_topic` only to the three hand
composites. In the camera node, every other selectable model falls into the generic
branch at `ros_packages/camera/oak_d_lite/stereo.py:1567`:

```python
self.nn_queues[model.model_id] = neural_network.out.createOutputQueue(
    maxSize=BRANCH_OUTPUT_QUEUE_DEPTH, blocking=False
)
```

That drains raw NNData and publishes nothing, so Cerebra has nothing to draw.

**The publishing path already exists and is manifest-driven.**
`stereo.py:251-256`:

```python
self.detection_publishers = {
    model.model_id: self.create_publisher(
        DetectionArray, model.publish_topic, 10
    )
    for model in ...
    if model.publish_topic
}
```

`model_registry.py:25/51/158` carries `publish_topic` through from the manifest. So
adding a `publish_topic` to a model lights up its publisher **and** Cerebra's layer
(`camera.component.ts` `ensureDetectionLayer(model_id)`) with no further glue.

**Message contract** — `ros_packages/datatypes/msg/DetectionArray.msg` and
`Detection.msg`: `header`, `model_id`, `frame_width`, `frame_height`, `detections[]`
with `label`, `score`, `x_min/y_min/x_max/y_max` (pixels of that frame), parallel
arrays `keypoint_names/keypoint_x/keypoint_y/keypoint_z`, `scalar_names/scalar_values`.
The hand chain's `_publish_hand_mp_detections` (`stereo.py`, search for it) is the
pattern to copy for filling it.

**Archive/parser pattern to copy** — `ros_packages/camera/oak_d_lite/imitation_archive.py`:
`_image_input(name, size)`, `_output(name, shape)`, `_write_archive(...)`, and the
v1 config with `"parser": "<ParserName>"` per head. `create_palm_archive` builds one
for the zoo palm blob; add a sibling for the YuNet face detector the same way.

**Available parsers** (installed `depthai_nodes` 0.5.2, inside the container):
`detection.py yolo.py ppdet.py rf_detr.py scrfd.py yunet.py` (detection),
`classification.py classification_sequence.py embeddings.py`,
`keypoints.py mediapipe_palm_detection.py hrnet.py`, `regression.py`,
`xfeat.py lane_detection.py mlsd.py segmentation.py fastsam.py`.

**The model**: `face_detection_yunet_160x120`, task `face_detection`, no
`publish_topic` today. Its blob is in the store at
`/home/pib/app/pib-models/face_detection_yunet_160x120/`; the manifest entry carries
`shaves` and the input size. Read the blob's output names from the model metadata
rather than guessing them.

**Cerebra side** (`~/opencode/cerebra`, develop):
- `src/app/camera/camera.component.html:158-209` draws, per model layer: box
  (`showsBox`), label (`detectionLabel`), connections (`connections`), keypoint
  circles.
- `src/app/camera/camera.component.ts`: `keypoints()` (around line 218),
  `showsBox()` (233), `labelAnchor()` (245), `connections()` (251),
  `detectionLabel()` (255).
- **`connections()` currently applies `handSkeletonConnections` to every model** —
  `src/app/camera/hand-skeleton.ts`. That is the piece to generalise: a per-task
  topology lookup with "no connections" as the default for box-only models.
- `showsBox()` returns false as soon as a detection carries keypoints (a hand
  workaround from PR-1781). Keep that behaviour for hands but make it a
  per-task property, otherwise a face-landmark model loses the box forever.

## What to build (this story)

1. `ros_packages/camera/oak_d_lite/task_archives.py`: a small registry
   `create_archive(model_id, blob_path)` returning a depthai NNArchive for models
   that have a parser, `None` otherwise. First entry: YuNet face detection.
2. `stereo.py`, generic model branch: when `create_archive(...)` returns an archive,
   build `ParsingNeuralNetwork` instead of the plain `NeuralNetwork`, translate the
   parsed result into `DetectionArray` (label from the model's class list, score,
   pixel box, keypoints/scalars when present) and publish on the model's
   `publish_topic`. Models without an archive keep today's behaviour.
3. `models/manifest.yaml`: `publish_topic: detections/face_detection_yunet_160x120`
   for that model.
4. Cerebra: replace the hardcoded `handSkeletonConnections(...)` in `connections()`
   with a per-task topology map (new file next to `hand-skeleton.ts`, hand topology
   moved there, everything else no connections), and make `showsBox()` consult the
   same per-task map. Keep the hand overlay pixel-identical.
5. Tests: backend unit tests for the translation (label/score/box/keypoints/scalars)
   and for "model without archive still behaves as before"; a Cerebra spec for the
   topology lookup. `black --check --include '\.py$' .` must stay clean, and the
   Cerebra build (`npx ng build`) must pass.

## Hard constraints

- Stay on `PR-1798`. Do not merge, do not push, do not touch other models' pipelines
  or shave budgets (a shave value belongs to the compiled blob and a mismatched
  start request is rejected).
- Do not lower the camera rate and do not enable stereo depth: `PIB_CAMERA_STEREO=auto`
  means "depth while idle" (a model running ⇒ colour only). See the chain reference
  in the skill for why.
- Report REAL results: the exact test commands you ran and their output. Do not
  summarise as "all tests pass" without the command.
