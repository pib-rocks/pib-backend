# PR-1688 analysis: leftover depthai 2.x vs camera 3.x (and pib-sdk)

**Date:** 2026-09-15  
**Scope:** read-only search of `pib-backend` plus sibling `../pib-sdk`  
**Method:** repository `rg` / file reads only. No builds, tests, git writes, or package installs.

---

## One-line verdict (question 7)

**NO — remaining depthai 2.x usage does not originate from pib-sdk.**  
pib-sdk declares only `pin`, `numpy`, and `roslibpy` (`/home/pib/opencode/pib-sdk/pyproject.toml` lines 17–21). Its camera API uses `roslibpy` against ROS services (`/home/pib/opencode/pib-sdk/src/pib_sdk/features/camera.py` lines 28, 57–78). A search of that tree for `depthai` / `blobconverter` / `luxonis` returned no matches. The 2.x pins live in **pib-backend** program-runtime / legacy-install files (`ros_packages/programs/Dockerfile` line 32, `ros_packages/requirements.txt` line 6, `setup/installation_scripts/local_install.sh` line 191).

---

## 1. Every `depthai` occurrence in pib-backend

Untracked Hermes prompt/plan files are listed last. Third-party `node_modules` was not treated as product source.

| Path | Line(s) | Text / role |
|---|---|---|
| `ros_packages/camera/oak_d_lite/stereo.py` | 5 | `import depthai as dai` |
| `ros_packages/camera/oak_d_lite/stereo.py` | 182, 184, 204 | comments / log: “depthai v3 API”, “DepthAI v3” |
| `ros_packages/camera/Dockerfile` | 36–40 | comment “depthai upgraded to 3.8.0…”; `pip install … depthai==3.8.0` |
| `ros_packages/camera/requirements.txt` | 51 | `depthai==3.8.0` |
| `ros_packages/programs/Dockerfile` | 32 | `"depthai<3"` in the same `pip install` as `pib-sdk` |
| `ros_packages/requirements.txt` | 6 | `depthai==2.25.1.0` |
| `setup/installation_scripts/local_install.sh` | 154 | `python3 -m pip install depthai` (unpinned) |
| `setup/installation_scripts/local_install.sh` | 155–156 | clone `luxonis/depthai-python` |
| `setup/installation_scripts/local_install.sh` | 159–160 | clone `depthai_hand_tracker` |
| `setup/installation_scripts/local_install.sh` | 165 | commented `#  sudo apt install -y ros-jazzy-depthai-ros` |
| `setup/installation_scripts/local_install.sh` | 191 | `python3 -m pip install depthai==2.24.0.0` (programs venv) |
| `setup/setup-pib.sh` | 253–256, 306–307 | udev helper named `install_depthai_udev_rules`; comment that imitation drives OAK via depthai |
| `setup/dev_tools/health-check-pib.sh` | 119 | host `pip show` list includes `depthai` |
| `tests/unit/test_stereo_camera_optimization.py` | 19 | `depthai = pytest.importorskip("depthai")` |
| `tests/unit/test_stereo_camera_optimization.py` | 116 | comment “Mock DepthAI queue” |
| `tests/unit/test_stereo_camera_optimization.py` | 199–211 | mocks `stereo.dai` / `mock_dai.Pipeline` |
| `pib_blockly/pib_blockly_server/src/pib-blockly/program-generators/util/definitions.ts` | 6 | `export const IMPORT_DEPTHAI = "import depthai as dai";` |
| `pib_blockly/pib_blockly_server/dist/pib-blockly/program-generators/util/definitions.d.ts` | 4 | compiled declaration of the same constant |
| `docs/test-basis/stereo_cpu_optimization.md` | 5, 9 | “DepthAI pipeline” (capital D; not the pip name) |
| `.hermes/plans/PR-1688-analysis-prompt.txt` | 4–32 | this analysis prompt |
| `.hermes/plans/PR-1674-backend-delegation-prompt.txt` | 14, 26 | depthai v3 conventions for stereo |

**Not found (product YAML / CI / pyproject):** `docker-compose.yaml` never names `depthai`. `.github/` and root `pyproject.toml` have no `depthai` hits. `pib_api/flask/requirements.txt` has none. `ros_packages/camera/package.xml` and `ros_packages/camera/setup.py` do not declare a depthai dependency (runtime is expected from the Docker `pip` layer).

**Related Blockly leftovers (not the string `depthai`):**  
`pib_blockly/.../reserved-words.ts` line 8 `"dai"` and line 14 `"blobconverter"`.  
`pib_blockly_server_bundle.js` line 1 contains `blobconverter` (minified). A search of that bundle for `depthai` returned **no** matches (unused `IMPORT_DEPTHAI` appears tree-shaken from the bundle).

---

## 2. Docker image / service mapping and 2.x vs 3.x

Compose service names from `docker-compose.yaml`.

### `ros-camera` → image `ros-camera` (`docker-compose.yaml` 62–67)

- Build: context `./ros_packages`, dockerfile `./camera/Dockerfile`.
- **Installs 3.x:** `ros_packages/camera/Dockerfile` lines 39–40 `depthai==3.8.0`.
- COPY chain: builder copies `./camera` into `ros2_ws/camera` (lines 14–15) and `colcon build`; runtime copies only `/app/ros2_ws/install` (line 55). Runtime `pip` does **not** use `-r camera/requirements.txt`.
- Runtime command: `ros2 run oak_d_lite stereo` (`docker-compose.yaml` line 68) → `oak_d_lite.stereo:main` (`ros_packages/camera/setup.py` line 21).
- USB: `/dev/bus/usb` (`docker-compose.yaml` 73–75).

### `ros-programs` → image `ros_programs` (`docker-compose.yaml` 163–167)

- Build: context repo root, dockerfile `./ros_packages/programs/Dockerfile`.
- **Installs 2.x:** lines 23–33 `pip install … pib-sdk … "depthai<3" blobconverter`.
- Does **not** COPY `ros_packages/requirements.txt` or `camera/requirements.txt`.
- User programs run with `PYTHON_BINARY=/usr/bin/python3` (`docker-compose.yaml` 184; `programs/program.py` 25–28, 144–150), i.e. the same interpreter that received `depthai<3`.
- USB is also mounted into this service (`docker-compose.yaml` 192–194).

### Other compose images

`pib-blockly-server`, `flask-app`, `rosbridge-ws`, `ros-motors`, `ros-voice-assistant`, `ros-display`, etc.: their Dockerfiles’ `pip install` lines (see grep of `**/Dockerfile*`) do **not** install depthai.

### Host / legacy (not a compose image)

- `setup/installation_scripts/local_install.sh` is sourced only when `INSTALL_METHOD=legacy` (`setup/setup-pib.sh` 644–646). That path: unpinned `pip install depthai` (line 154) plus programs venv `depthai==2.24.0.0` (line 191).
- Default Ubuntu/Raspbian install uses Docker (`setup/setup-pib.sh` 647–649), not that venv.

### `ros_packages/camera/requirements.txt`

Contains **3.x** `depthai==3.8.0` (line 51) **and** `blobconverter==1.4.3` (line 18). The camera Dockerfile never `pip install -r` this file. The file looks like a pip/ROS environment freeze (ament, colcon, cv-bridge, …), not the live install list.

---

## 3. Is `ros_packages/requirements.txt` orphaned?

**For Docker and install scripts: yes, it is not an input to any `pip install -r`.**

Grep for `ros_packages/requirements.txt`:

| File | Line | How it is used |
|---|---|---|
| `docs/test-basis/pib_sdk_integration.md` | 67 | Gherkin text claims “`ros_packages/programs/Dockerfile` **and** `ros_packages/requirements.txt` build configuration” |
| `.hermes/plans/PR-1657-*.md/txt` | various | planning notes that treated it as “program-runtime requirement” |
| `.hermes/plans/PR-1688-analysis-prompt.txt` | 14, 24 | this task |

Grep of Dockerfiles for `requirements.txt` COPY/install:

- `pib_api/flask/Dockerfile` lines 9, 18 → `./pib_api/flask/requirements.txt` only  
- `ros_packages/voice_assistant/Dockerfile` line 23 → voice_assistant requirements only  
- **No** Dockerfile copies `ros_packages/requirements.txt`

Grep of `pip install -r` in setup/tests: `local_install.sh` 161 is `depthai_hand_tracker`’s own `requirements.txt` after `cd`; 175 and 201 are voice_assistant and motors files. Tests use `tests/integration/requirements.txt` and `tests/infrastructure/requirements.txt`.

**Fact:** `docs/test-basis/pib_sdk_integration.md` lines 67–70 treat `ros_packages/requirements.txt` as if it built `ros-programs`. **Fact:** `ros_packages/programs/Dockerfile` installs `pib-sdk` inline (line 27) and never references that file. The test-basis wording is stale relative to the Dockerfile.

Contents of the orphaned file (`ros_packages/requirements.txt` 1–11) mix voice (`openai`, `google-cloud-texttospeech`, `pyaudio`, `SpeechRecognition`, `google-genai`), **2.x** `depthai==2.25.1.0`, `blobconverter==1.4.2`, `paramiko`, and `pib-sdk>=0.5.0`. That does not match either the camera image (3.8.0, no blobconverter pip) or the programs Dockerfile (unpinned `depthai<3`, unpinned `blobconverter`, no openai).

---

## 4. Does the programs image (or generated Blockly) import / need depthai at runtime?

### Programs package Python (image code)

`ros_packages/programs/` has four `.py` files. Imports:

**`programs/program.py`**

- 1 `import os`
- 2–11 `rclpy` / action / qos / duration
- 13–18 `threading`, `queue`, `subprocess`, `typing`, `time`, `tempfile`
- 20–21 `datatypes.action`, `datatypes.msg`
- 23 `import pib_blockly_client`

**`programs/proxy_program.py`**

- 1–17 `rclpy` / action / qos / duration / Future
- 10–16 `datatypes.action` / `msg` / `srv`
- 19 `uuid`

**`launch/launch.py`**, **`setup.py`:** packaging/launch only; no depthai.

**No `import depthai` and no `import blobconverter` in this package.**

### Generated Blockly (what `program.py` executes after `code_visual_to_python`)

Camera blocks emit **pib-sdk over rosbridge**, not depthai:

- `camera-generators.ts` lines 3, 10–13: `IMPORT_OS`, `IMPORT_PIB_SDK_CAMERA` (`from pib_sdk.features.camera import Camera`)
- `function-declarations.ts` 112–133: `Camera(host=…).get_depth_frame()` / `get_distance_at_px()`

Face detector uses `/face_center` (`function-declarations.ts` 307–327), which is published by `stereo.py` in the **camera** container, not by importing depthai in the program.

`IMPORT_DEPTHAI` / `IMPORT_BLOBCONVERTER` are defined in `definitions.ts` lines 6–7. **No other `.ts` file imports those symbols** (grep of `pib_blockly` for `IMPORT_DEPTHAI` / `IMPORT_BLOBCONVERTER` only hits `definitions.ts`). Camera generators do not add them.

**Fact:** first-party generated programs found in this repo do not import depthai.  
**Assumption (not observed in generated templates):** a user could type `import depthai` in a custom Python program stored under `PROGRAM_DIR`; the programs image currently makes 2.x importable. That is environment capability, not a first-party import.

---

## 5. blobconverter: used? requires depthai&lt;3?

### Occurrences

| Path | Line | Role |
|---|---|---|
| `ros_packages/programs/Dockerfile` | 33 | `pip install … blobconverter` (unpinned) |
| `ros_packages/requirements.txt` | 8 | `blobconverter==1.4.2` (orphaned file) |
| `ros_packages/camera/requirements.txt` | 18 | `blobconverter==1.4.3` (not pip-installed by camera Dockerfile) |
| `setup/installation_scripts/local_install.sh` | 192 | programs venv `blobconverter==1.4.2` |
| `definitions.ts` | 7 | unused `IMPORT_BLOBCONVERTER` |
| `definitions.d.ts` | 5 | compiled unused constant |
| `reserved-words.ts` | 14 | `"blobconverter"` in Blockly reserved words |
| `pib_blockly_server_bundle.js` | 1 | minified reserved-word string |

Grep of `*.py` and Blockly generators for `import blobconverter` / `blobconverter.` besides those constants: **no Python usage**. Camera `stereo.py` does not import it.

### Does blobconverter require depthai&lt;3?

**In this repository: no statement of that constraint.** The only coupling is juxtaposition: programs Dockerfile lines 32–33 install `"depthai<3"` and `blobconverter` in one `pip` command; `local_install.sh` 191–192 install `depthai==2.24.0.0` then `blobconverter==1.4.2`.

**Fact:** nothing in-repo documents blobconverter’s own `install_requires`. This analysis did not query PyPI or install the wheel.  
**Assumption:** historically blobconverter converted OpenVINO models to `.blob` for **depthai 2.x** NN nodes; that would explain the pairing. It is **not** proven here, and the camera 3.x node does not use blobconverter in `stereo.py`.

---

## 6. depthai v2 Pipeline API vs v3 Node API

**Only Python module that imports depthai:** `ros_packages/camera/oak_d_lite/stereo.py` line 5 (plus the unit test that mocks `dai`).

What `stereo.py` actually calls (v3-style, as annotated in-file):

```152:202:ros_packages/camera/oak_d_lite/stereo.py
    def _init_stereo_depth(self):
        """Add StereoDepth outputs to the existing pipeline (no extra start)."""
        mono_left = self.pipeline.create(dai.node.Camera)
        ...
        stereo = self.pipeline.create(dai.node.StereoDepth)
        ...
            self.pipeline = dai.Pipeline()
            self.camRgb = self.pipeline.create(dai.node.Camera)
            self.camRgb.build(dai.CameraBoardSocket.CAM_A)
            self.isp_out = self.camRgb.requestIspOutput()
            self.queue = self.isp_out.createOutputQueue()
            ...
            self.pipeline.start()
```

Comments at 182–186 state ColorCamera is replaced, XLinkOut is removed in v3, queues come from `requestIspOutput()`, start is `pipeline.start()`.

**Not found in camera code:** `dai.node.ColorCamera`, `XLinkOut`, `dai.Device(...)`.

`pipeline.stop()` at `stereo.py` 315 is used when preview size changes, then `init_pipeline()` again.

The unit test (`test_stereo_camera_optimization.py` 211–215) mocks `mock_dai.Pipeline` and `mock_dai.node.Camera` / `StereoDepth`, matching this v3 node graph.

`dai.Pipeline()` exists in both generations; the **v2-specific** surface (ColorCamera / XLinkOut / Device wrapping) is not present in the live camera path. The programs image still *installs* 2.x but has no first-party Pipeline code.

---

## Sibling pib-sdk (reachable)

Path `/home/pib/opencode/pib-sdk` was readable.

- `pyproject.toml` 17–21: `pin>=3.1`, `numpy>=1.22`, `roslibpy>=1.6` only. Optional: pytest/ruff/build/twine; drawing extra Pillow.
- `src/pib_sdk/features/camera.py`: `import roslibpy` (line 28); services `get_camera_image`, `get_depth_frame`, `get_distance_at_px` (lines 64–78). Docstring lines 3–7: snapshots/depth over rosbridge, “no on-board vision/AI here”.
- Repo-wide search for `depthai`, `blobconverter`, `luxonis`: **no matches**.

Installing `pib-sdk` on the programs image (`programs/Dockerfile` line 27) does **not** pull depthai. The `"depthai<3"` extra is a **backend Dockerfile choice**, sitting on the next lines after `pib-sdk`.

---

## Why 2.x artifacts still exist (synthesis)

Observed causes, not guesses about intent:

1. **Camera migration was local to the camera image and `stereo.py`.** 3.8.0 is pinned in `camera/Dockerfile` 36–40. The programs image was not updated to 3.x or stripped.
2. **`ros_packages/requirements.txt` was left as a stale combined pin list** (2.25.1.0) and is not wired into Docker. Docs still mention it as if it were the programs build config (`pib_sdk_integration.md` 67).
3. **Legacy `local_install.sh` still installs 2.24.0.0** into a programs venv (191) and unpinned depthai for “Camera Dependencies” (154), even though current `setup-pib.sh` prefers Docker.
4. **Blockly still exports unused depthai/blobconverter import strings** from an older generator style; live generators use `pib_sdk.features.camera`.
5. **USB on `ros-programs`** (`docker-compose.yaml` 192) is consistent with a former design where user programs talked to the OAK directly; first-party code no longer does that.

None of these are caused by pib-sdk’s declared dependencies.

---

## 8. Risks and suggested follow-up tickets

### Risks of leaving 2.x in place

- Two DepthAI major versions on one robot: `ros-camera` 3.8.0 vs `ros-programs` `depthai<3`. If a user program opens the OAK while `stereo.py` holds it, device exclusive-access failures are likely (USB is mounted into **both** services: compose 73–75 and 192).
- Docs/health-check drift: `health-check-pib.sh` 119 expects **host** `pip show depthai`; Docker camera install is inside `ros-camera`, not necessarily host Python.
- Stale pins disagree with each other: 2.25.1.0 (orphaned requirements), 2.24.0.0 (legacy local_install), `depthai<3` (programs image), 3.8.0 (camera).
- Extra image size and unmaintained 2.x / blobconverter attack surface in `ros_programs` with no first-party import.
- `pib_sdk_integration.md` 67–70 can mislead future tickets into editing the orphaned file instead of `programs/Dockerfile`.

### Risks of removing 2.x / blobconverter from programs

- **User** Python under `PROGRAM_DIR` that still `import depthai` or `import blobconverter` would fail at runtime. This repo does not contain those user files.
- Legacy `INSTALL_METHOD=legacy` `local_install.sh` would still install 2.x unless updated in the same change.
- If blobconverter truly needs a depthai 2.x extra (unproven here), removing only one of the two packages could break `pip install` or runtime of that unused library.
- Host `health-check-pib.sh` would still fail if operators expect a global `depthai` package after Docker-only camera.

### Prioritised ticket titles (do not create)

1. **Remove unused `depthai<3` and `blobconverter` from `ros-programs` (confirm no user-program imports)**
2. **Retire or retarget orphaned `ros_packages/requirements.txt`; fix `pib_sdk_integration.md` Scenario 5**
3. **Align `camera/requirements.txt` with `camera/Dockerfile` (drop freeze-only blobconverter or document freeze-only)**
4. **Delete unused Blockly `IMPORT_DEPTHAI` / `IMPORT_BLOBCONVERTER` and reserved words `dai` / `blobconverter`**
5. **Update or gate `local_install.sh` camera/programs depthai 2.24 pins behind legacy-only, matching 3.x Docker**
6. **Scope `health-check-pib.sh` depthai check to the camera container, not host pip**
7. **Document OAK USB exclusivity: `ros-camera` vs `ros-programs` `/dev/bus/usb`**
8. **Confirm blobconverter PyPI deps (out of tree) before any 3.x co-install experiment**
