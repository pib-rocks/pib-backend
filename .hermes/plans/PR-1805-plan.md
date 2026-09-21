# PR-1805 — overlay for yolov6n_coco_640x640 (boxes with COCO labels)

Branch `PR-1805` (worktree `~/opencode/pib-backend-PR-1805`), base `origin/develop`
`83f5e29b` (already contains merged PR-1798).

## What already exists (read it before writing anything)

PR-1798 landed the whole publishing path. Copy its shape, do not invent a new one:

- `ros_packages/camera/oak_d_lite/task_archives.py` — the registry. `_ARCHIVE_BUILDERS`
  maps a model id to a builder, `_MODEL_LABELS` maps it to the class list,
  `create_archive(model_id, blob_path)` returns an NNArchive or `None`,
  `labels_for_model(model_id)` returns the labels for the translation. The YuNet
  entry is the working example.
- `ros_packages/camera/oak_d_lite/parsed_detections.py` — `translate_detections(packet,
  labels, frame_w, frame_h)` turns the parsed `dai.ImgDetections` into the
  `DetectionArray` entries.
- `stereo.py` — `_build_single_network_pipeline` (parsed when an archive exists, plain
  fallback otherwise) and `_publish_parsed_detections`; the loop publishes parsed
  models from `nn_queues`.
- `models/manifest.yaml` — `publish_topic` on the model creates its publisher
  (`stereo.py:251-256`). That is the only manifest change needed.

**Cerebra needs no change for this story**: PR-1798 generalised the overlay, and
`detectionLabel()` already renders `label + score`. Boxes and labels are generic.

## Verified facts about the model (read from the blob, not assumed)

`models/yolov6n_coco_640x640/yolov6n_coco_640x640.blob`, inspected with
`dai.OpenVINO.Blob`:

```
numShaves: 4
inputs : {'images': [640, 640, 3, 1]}
outputs: {'output3_yolov6': [20, 20, 85, 1],
          'output2_yolov6': [40, 40, 85, 1],
          'output1_yolov6': [80, 80, 85, 1]}
```

85 channels = 4 box + 1 objectness + **80 COCO classes**, three detection scales, so
`n_classes = 80` and the model is YOLOv6.

Parser available in the installed `depthai_nodes` 0.5.2 (`node/parsers/yolo.py`):
**`YOLOExtendedParser`** is the only class there; it takes `conf_threshold`,
`iou_threshold`, `n_classes`, `label_names`, `subtype` and the output layer names.
Read its `__init__` for the exact parameter names before writing the archive.

## What to build

1. A COCO class list as a checked-in constant with a comment naming where it came
   from (the Luxonis YOLO example under
   `oak-examples/neural-networks/object-detection/yolo-p`, or the model zoo's
   `model.yml` for this model). 80 names, in the model's own order - a wrong order
   mislabels every box, so state the source.
2. A YOLO entry in `task_archives.py` following the YuNet one: build the archive from
   the blob's real tensor names and shapes (never hardcode 'images' without checking),
   declare one head with `YOLOExtendedParser`, `n_classes=80`, `subtype` for v6, the
   three `output*_yolov6` layer names, and the COCO list in `label_names`.
3. `publish_topic: detections/yolov6n_coco_640x640` for that model in
   `models/manifest.yaml`.
4. Unit tests: the archive builds from the real blob (tensor names/shapes match the
   blob, parser name, n_classes), `labels_for_model` returns 80 names, and a
   translation test that a two-class detection gets the right labels and pixel boxes.

## Hard constraints

- Stay on `PR-1805`: no merge, no push, no branch switching. Do not touch the YuNet
  entry or any other model.
- Do not change shave budgets, do not change the camera rate, do not enable stereo
  depth (`PIB_CAMERA_STEREO=auto` means depth only while idle).
- Report REAL command output, not summaries.

## Acceptance criteria (from the ticket)

- The model publishes on `detections/yolov6n_coco_640x640`; a captured message carries
  label, score, pixel box and (for YOLO) no keypoints.
- Camera and other models unaffected; no `PARSED_DROP` in the log.
- `python3 -m pytest tests/unit -q` and
  `python3 -m black --check --include '\.py$' .` clean.
