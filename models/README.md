# Vendored OAK / RVC2 model blobs (PR-1693 S1)

This directory is the on-device model registry: one folder per `model_id`, a
`.blob` compiled for Myriad-X (OpenVINO compile_tool, 4 SHAVE), plus
`manifest.yaml`.

## Where the artefacts come from

Weights are **not** invented here. IR (`.xml` / `.bin`) comes from:

- **luxonis/depthai-model-zoo** (deprecated GitHub zoo; `model.yml` still
  lists GitHub raw XML + RobotHub DigitalOcean CDN BIN).
- **OpenVINO Open Model Zoo** for `gaze-estimation-adas-0002` and
  `head-pose-estimation-adas-0001` (`zoo_type=intel` on blobconverter).
  `person-reidentification-retail-0031_96x48` is an OMZ model that Luxonis
  already packaged in the DepthAI zoo at 96×48.

The **vendored files** are compiled `.blob` outputs from the still-live
BlobConverter service (`https://blobconverter.luxonis.com`, Python package
`blobconverter==1.4.3`). HubAI (`models.luxonis.com`) is the replacement
zoo UI; unauthenticated HubAI SDK downloads require `HUBAI_API_KEY` and
were **not** used.

## Hand path (three artefacts)

Palm / hand tracking needs **three** blobs, not two:

1. `palm_detection_128x128` — MediaPipe palm **detector** (the network that
   actually finds palms).
2. `palm_detection_128x128_decoding` — **decoding head only**. Upstream README:
   "This model decodes the palm_detection_128x128, so everything can run on
   the edge. It returns only the TOP 10 most confident results." It is not a
   detector and does nothing useful without (1).
3. `hand_landmark_224x224` — hand landmark network on a cropped palm.

Licences are recorded in `manifest.yaml` (not used as a download gate).
Where the upstream `model.yml` `license` field is empty, the entry is
`unknown - see source`.

## How these blobs were produced (2026-09-15)

```bash
python3 -m venv /tmp/bcvenv
/tmp/bcvenv/bin/pip install blobconverter==1.4.3
```

```python
import blobconverter
blobconverter.set_defaults(shaves=4, version="2022.1")

depthai = [
    "palm_detection_128x128",
    "palm_detection_128x128_decoding",
    "hand_landmark_224x224",
    "face_detection_yunet_160x120",
    "facemesh_192x192",
    "face-recognition-arcface-112x112",
    "emotion_recognition_lfw_64x64",
    "yolov6n_coco_640x640",
    "qr_code_detection_384x384",
    "person-reidentification-retail-0031_96x48",
]
for name in depthai:
    blobconverter.from_zoo(name=name, zoo_type="depthai", shaves=4, version="2022.1")

blobconverter.from_zoo(
    name="facial_landmarks_68_160x160",
    zoo_type="depthai",
    shaves=4,
    version="2021.4",  # 2022.1 compile_tool fails: duplicate VariadicSplit_949
)

for name in ("gaze-estimation-adas-0002", "head-pose-estimation-adas-0001"):
    blobconverter.from_zoo(name=name, zoo_type="intel", shaves=4, version="2022.1")
```

Default compile flags from blobconverter: `compile_params=["-ip U8"]`.
Cached blobs land in `~/.cache/blobconverter/` as
`<name>_openvino_<version>_4shave.blob`. Copy each file to
`models/<model_id>/<model_id>.blob`.

## How to regenerate

1. Re-run the commands above (same `shaves`, `version`, and `zoo_type` as
   in `manifest.yaml`).
2. Copy the new blobs into the per-model directories.
3. Update `sha256` and `size_bytes` in `manifest.yaml`.
4. Run `python3 models/verify_models.py` from the repository root (or
   from this directory).

`facial_landmarks_68_160x160` **must** stay on OpenVINO **2021.4**. A 2022.1
compile returns HTTP 400 from blobconverter (`Function contains several
inputs and outputs with one friendly name: VariadicSplit_949`).

## Check

Provision the host store **before starting Docker containers**. This creates
the bind-mount source with the invoking user's ownership and installs both the
blobs and the manifest that the camera registry reads at `/models/manifest.yaml`.

```bash
./setup/setup-pib.sh --models
docker compose --profile camera up -d
```

Provisioning is idempotent. To check an existing store without changing it:

```bash
./setup/setup-pib.sh --verify-models
```

To verify the vendored source artefacts themselves:

```bash
python3 models/verify_models.py
```

The script needs no third-party packages. It exits non-zero if a listed
file is missing or the SHA-256 does not match.
