# Automated Test Suite

Generated from `docs/test-basis/` specifications.

## Layout

| Directory | Framework | Scope |
|---|---|---|
| `integration/` | Pytest | Flask API contracts, BDD error handling, motor clamping |
| `infrastructure/` | Pytest | docker-compose.yaml contract + optional live Docker tests |
| `frontend/` | Robot Framework + Browser | Cerebra UI and Blockly interactions |
| `e2e/` | Robot Framework and Pytest | Robot suites are full-system BDD. `test_*_e2e.py` is the live robot suite and reads `PIB_ROBOT_URL`. |
| `blockly_generator/` | Jest | Blockly Python code generator unit tests |
| `resources/` | Python/Robot | Shared keywords and `ROS2TestLibrary.py` |

## Run everything (Pi)

From the `tests/` directory on the Pi:

```bash
cd ~/app/pib-backend/tests
chmod +x run_all_tests.sh   # once
./run_all_tests.sh
```

Options: `--skip-docker`, `--skip-jest`, `--skip-robot`, `--include-frontend`, `--help`.

Results are written to `tests/results/run-<timestamp>/`.

## Pytest

The suite needs four requirement files, not one: the camera and unit files carry
imports the collection step hits before any test runs (`cv2` in
`tests/unit/test_stereo_camera_optimization.py`, for example). Skipping them turns
into a collection ERROR that reads like a broken test.

```bash
cd <repo-root>
python3 -m venv .test-venv
source .test-venv/bin/activate
pip install --prefer-binary \
  -r tests/integration/requirements.txt \
  -r tests/infrastructure/requirements.txt \
  -r tests/requirements-camera.txt \
  -r tests/requirements-unit.txt \
  playwright lark
pip install -e ./pib_hermes_config -e ./pib_mcp_server -e ./public_api_client
playwright install chromium        # skipped on arm64; the tests then use system chromium

# The live suites read PIB_ROBOT_URL and refuse to guess localhost.
export PIB_ROBOT_URL=http://localhost
PYTHONPATH="$PWD:$PWD/pib_api/flask:$PWD/pib_api/client:$PWD/pib_hermes_config:$PWD/pib_mcp_server:$PWD/public_api_client" \
  python -m pytest tests -q -m "not docker" --ignore=tests/blockly_generator
```

`tests/run_all_tests.sh` does all of this for you and now fails fast when one of
the required imports is missing, naming the module instead of letting pytest
report a collection error later.

Live Docker deployment tests (slow, builds images):

```bash
set RUN_DOCKER_TESTS=1
pytest tests/infrastructure/test_docker_deployment.py -m docker
```

## Live pytest E2E

The pytest modules in `tests/e2e/` (`test_*_e2e.py`) run against one live robot.
Set `PIB_ROBOT_URL` to that robot's address. The API address is derived from it
(`$PIB_ROBOT_URL/api`) and is not a separate setting.

```bash
PIB_ROBOT_URL=http://192.168.1.172 python3 -m pytest tests/e2e -q
```

With no address configured, the suite stops and names `PIB_ROBOT_URL`. It does
not try `http://localhost`.

Older names still work as fallbacks when `PIB_ROBOT_URL` is unset:
`PIB_API_URL`, `PIB_E2E_BASE_URL`, `PIB_MODEL_E2E_HOST`, and
`PIB_MODEL_E2E_API_URL`. The run prints which variable it used. If more than
one of these is set, they must describe the same robot; the suite does not
pick between them.

On the robot itself, set `PIB_ROBOT_URL=http://localhost`. Robot Framework
suites in this directory do not read `PIB_ROBOT_URL`.

## Robot Framework

Requires Flask on `:5000`. Cerebra on `:4200` for frontend suites. Set `ROS2_TEST_MOCK=true` (default) for E2E without hardware.

```bash
pip install -r tests/requirements-robot.txt
rfbrowser init
robot --outputdir tests/results tests/e2e/system_e2e.robot
```

## Jest (Blockly generators)

```bash
cd tests/blockly_generator
npm install
npm test
```

Requires the `pib-blockly` submodule at `pib_blockly/pib_blockly_server/src/pib-blockly`.

## CI note

Only Black lint runs in GitHub Actions today; execute these suites locally or wire them into CI separately.
