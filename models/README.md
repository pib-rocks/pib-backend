# OAK / RVC2 model registry (PR-1693 S1, asset PR-1943)

`models/manifest.yaml` is the on-device model registry: one entry per
`model_id`, the per-network SHAVE count, and the sha256 of the `.blob`
compiled for Myriad-X. The `.blob` files themselves are not in git.

The compiled blobs are not reproducible on the robot. BlobConverter is an
external service, and the camera runtime never downloads or compiles a model.
The published GitHub release asset is the master copy. This manifest is the
checksum source of truth: `./setup/setup-pib.sh --verify-models` compares the
host store to it and does not use the network.

## Release asset (master copy)

| Field | Value |
| --- | --- |
| Asset tag | `model-registry-2026-09-15` |
| Asset file | `models-2026-09-15.tar.gz` |
| Default URL | `https://github.com/pib-rocks/pib-backend/releases/download/model-registry-2026-09-15/models-2026-09-15.tar.gz` |
| Tarball sha256 | `07357ec99e0b91014285aad2e6a03fcd1e6ca95fd7ed686670f7ab73c1f3ff02` |
| Tarball size | 34581804 bytes |

The archive root matches this directory: `manifest.yaml` plus
`<model_id>/<model_id>.blob` for each network. It does not contain this
README or `verify_models.py`.

`setup/setup-pib.sh` reads those four values from `PIB_MODEL_ASSET_TAG`,
`PIB_MODEL_ASSET_NAME`, `PIB_MODEL_ASSET_URL`, and `PIB_MODEL_ASSET_SHA256`. The
defaults are the table above, so publishing this file on that tag needs no code
change. Point the same variables at a mirror or a newer asset when the file moves.

### Produce the tarball

From a tree that contains the current blobs and `manifest.yaml` (an unpacked
copy of the previous asset, or the tree that compiled them). The flags below
make the bytes reproducible; the sha256 above is the result.

```bash
list=$(mktemp)
find models -name '*.blob' -printf '%P\n' | sort > "$list"
tar -C models --sort=name --mtime='2026-09-15T00:00:00Z' \
  --owner=0 --group=0 --numeric-owner \
  -cf - manifest.yaml -T "$list" | gzip -n > models-2026-09-15.tar.gz
rm -f "$list"
sha256sum models-2026-09-15.tar.gz
```

Do not commit the `.blob` files or the tarball. Publish
`models-2026-09-15.tar.gz` as a GitHub release asset on tag
`model-registry-2026-09-15`, and mark that release as a **pre-release**: GitHub
excludes pre-releases from `releases/latest`, so a data release never becomes the
repository's latest release and never enters the version pairing guard.

### Where setup puts it

`--models` downloads the asset into
`${PIB_MODEL_CACHE:-$HOME/app/.cache/pib-models}`, checks the tarball sha256
before extracting, then copies any missing or mismatched blob into
`${PIB_MODEL_STORE:-/home/pib/app/pib-models}`. The runtime mount is unchanged:
that store is bind-mounted read-only at `/models`. `PIB_MODEL_STORE` is the
store the camera sees, not the download cache.

### Repair a missing or corrupt store

`--verify-models` is offline. It exits non-zero when a store blob is missing
or its sha256 does not match `models/manifest.yaml`. Repair by provisioning
again. If the cache does not match the manifest, `--models` downloads the
asset again and replaces the bad blobs:

```bash
./setup/setup-pib.sh --verify-models
./setup/setup-pib.sh --models
./setup/setup-pib.sh --verify-models
```

If the download fails and the store is empty, `--models` exits non-zero and
prints the asset URL, tag, and checksum. It does not install an empty store.
To use a mirror, set `PIB_MODEL_ASSET_URL` (and `PIB_MODEL_ASSET_SHA256` when
the file differs) and run `--models` again. A store that already matches the
manifest is left in place when the download fails.

## Where the artefacts come from

Weights are **not** invented here. IR (`.xml` / `.bin`) comes from:

- **luxonis/depthai-model-zoo** (deprecated GitHub zoo; `model.yml` still
  lists GitHub raw XML + RobotHub DigitalOcean CDN BIN).
- **OpenVINO Open Model Zoo** for `gaze-estimation-adas-0002` and
  `head-pose-estimation-adas-0001` (`zoo_type=intel` on blobconverter).
  `person-reidentification-retail-0031_96x48` is an OMZ model that Luxonis
  already packaged in the DepthAI zoo at 96×48.

The **published blobs** are compiled `.blob` outputs from the still-live
BlobConverter service (`https://blobconverter.luxonis.com`, Python package
`blobconverter==1.4.3`). HubAI (`models.luxonis.com`) is the replacement
zoo UI; unauthenticated HubAI SDK downloads require `HUBAI_API_KEY` and
were **not** used. Those outputs are the release asset above, not files in
git.

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
2. Copy the new blobs into the per-model directories. Those paths are
   gitignored; do not `git add` them.
3. Update `sha256` and `size_bytes` in `manifest.yaml`.
4. Run `python3 models/verify_models.py` from the repository root (or
   from this directory) while the blobs are on disk next to the manifest.
5. Rebuild and publish `models-<produced-date>.tar.gz` with the command in
   **Produce the tarball**, and set `PIB_MODEL_ASSET_TAG`,
   `PIB_MODEL_ASSET_NAME`, `PIB_MODEL_ASSET_URL`, and `PIB_MODEL_ASSET_SHA256`
   (or the defaults in `setup/setup-pib.sh`) to that new asset.

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

`python3 models/verify_models.py` checks blob files that sit next to the
manifest (the producer tree, or an unpacked asset copied there). It needs no
third-party packages and exits non-zero if a listed file is missing or the
SHA-256 does not match. A fresh git checkout has no `.blob` files, so that
command fails until the asset has been unpacked into `models/`. Devices do
not use it; they use `--verify-models`, which stays offline against the host
store.

## Add and vend a model

Models are immutable runtime inputs. The camera runtime does not download or
compile a model; the published release asset is the master copy. Add a new
single-network model as follows:

1. Confirm the upstream source and licence, then compile the IR for Myriad-X
   with the network's chosen OpenVINO version and SHAVE count.
2. Create `models/<model_id>/` and copy exactly one blob to
   `models/<model_id>/<model_id>.blob`. The blob stays out of git
   (see `.gitignore`).
3. Add the blob entry to `manifest.yaml`. Do not silently change an existing
   network's `shaves`: the value is part of the compiled blob and a mismatched
   `/start_model` request is rejected.
4. Calculate the recorded values:

   ```bash
   sha256sum models/<model_id>/<model_id>.blob
   stat --printf='%s\n' models/<model_id>/<model_id>.blob
   ```

5. Run `python3 models/verify_models.py` against the local blobs, rebuild the
   release tarball, publish it, then provision the host store with
   `./setup/setup-pib.sh --models` and confirm it with
   `./setup/setup-pib.sh --verify-models`.
6. Restart `ros-camera`, call `/list_models`, and start the model with
   `shaves=0` (registry default). Verify `/models_status` and the model's typed
   detection topic before adding a consumer.

Every blob entry requires:

| Field | Meaning |
| --- | --- |
| `model_id` | Stable service/API identifier and directory name |
| `task` | Machine-readable task category |
| `source_upstream`, `source_url` | Provenance for the original model |
| `licence` | Upstream licence, or `unknown - see source` |
| `file` | Path below `models/`; normally `<model_id>/<model_id>.blob` |
| `sha256`, `size_bytes` | Provisioning integrity values |
| `shaves` | SHAVEs used when compiling this exact network |
| `input_width`, `input_height` | Network input dimensions |
| `format` | `blob` |
| `openvino_version` | Version used by the compiler |
| `zoo_type` | BlobConverter source (`depthai` or `intel`) when applicable |
| `functional` | Optional `false` keeps a known non-functional blob listed but unavailable |
| `unavailable_reason` | Required when `functional` is `false`; surfaced by model status and `/start_model` |
| `notes` | Input names/layout, colour/range, outputs, and exceptions |

An executable composite model has no blob of its own. Give it `model_id`,
`task`, `composite: true`, an ordered `artifacts` list containing blob entry
IDs, and `publish_topic`. All artifacts must be present. Its displayed size and
SHAVE budget are the sums of its networks; each artifact keeps its own measured
`shaves` value. `hand_tracking` is the reference composite.

The provisioned host layout mirrors the repository:

```text
${PIB_MODEL_STORE:-/home/pib/app/pib-models}/
├── manifest.yaml
└── <model_id>/
    └── <model_id>.blob
```

Docker mounts that directory read-only at `/models`. Provision it before
starting containers so Docker does not create a root-owned bind-mount source.

## The imitation model set (2026-09-18)

`imitation` is a second, independent hand path. It exists because the DepthAI zoo decoding
head cannot be used as vendored: its blob metadata, read on the device, is

```
palm_detection_128x128           OUT regressors FP16 [18,896,1]  classificators FP16 [1,896,1]
palm_detection_128x128_decoding  IN  regressors U8F  [18,896,1]  classificators U8F  [1,896,1]
```

The decoding head was compiled with `-ip U8`, which is right for an image-input detector and
wrong for a decoding head whose inputs are float tensors. Feeding FP16 into U8 inputs
destroys them; the head then emits values quantised to 1/128 steps, a score column stuck at a
constant 1.0 and coordinates above 1.0 that correlate with nothing in the frame.

The upstream reference (`geaxgx/depthai_hand_tracker`, pib fork at `/home/pib/imitation` on
the robot) avoids that by generating its own post-processing blob. Its three blobs are
vendored here, all OpenVINO 2022.1, MIT licensed:

| model_id | shaves | inputs | outputs |
| --- | --- | --- | --- |
| `palm_detection_sh4` | 4 | `input` U8F 128x128x3 | `regressors` FP16, `classificators` FP16 |
| `pd_postprocessing_top2_sh1` | 1 | `regressors` FP16, `classificators` FP16 | `result` FP16 [8,2] |
| `hand_landmark_full_sh4` | 4 | `input_1` U8F 224x224x3 | `Identity_1`, `Identity_2`, `Identity_dense/BiasAdd/Add`, `Identity_3_dense/BiasAdd/Add` |

The composite `imitation` requests 4 + 1 + 4 = 9 shaves and publishes on
`detections/imitation`.

`result` carries the TOP 2 palms as eight square-normalised values each:
`score, box_x, box_y, box_size, kp0_x, kp0_y, kp2_x, kp2_y`. The reference derives the
landmark crop from them with `rotation = 0.5*pi - atan2(-(kp2_y-kp0_y), kp2_x-kp0_x)`,
`center = box + 0.5*box_size*(sin(rotation), -cos(rotation))` and `size = 2.9 * box_size`,
and it gates on `score >= 0.5` for the palm and `Identity_1 >= 0.5` for the landmarks.

**Before trusting any vendored helper blob, read its input datatypes:**

```python
import depthai as dai
b = dai.OpenVINO.Blob("/models/<id>/<id>.blob")
for n, t in b.networkInputs.items():
    print(n, t.dataType, list(t.dims))
```
