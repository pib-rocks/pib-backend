# Automated Test Suite

Generated from `docs/test-basis/` specifications.

## Layout

| Directory | Framework | Scope |
|---|---|---|
| `integration/` | Pytest | Flask API contracts, BDD error handling, motor clamping |
| `infrastructure/` | Pytest | docker-compose.yaml contract + optional live Docker tests |
| `frontend/` | Robot Framework + Browser | Cerebra UI and Blockly interactions |
| `e2e/` | Robot Framework | Full-system REST/ROS safety and infrastructure BDD |
| `blockly_generator/` | Jest | Blockly Python code generator unit tests |
| `resources/` | Python/Robot | Shared keywords and `ROS2TestLibrary.py` |

## Pytest

```bash
pip install -r tests/integration/requirements.txt
pip install -r tests/infrastructure/requirements.txt
pytest tests/ -q
```

Live Docker deployment tests (slow, builds images):

```bash
set RUN_DOCKER_TESTS=1
pytest tests/infrastructure/test_docker_deployment.py -m docker
```

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
