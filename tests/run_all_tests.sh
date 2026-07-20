#!/usr/bin/env bash
# Run the full backend test suite from the tests/ directory (Pi or dev host).
#
# Usage (from repo tests/ directory):
#   ./run_all_tests.sh
#   ./run_all_tests.sh --skip-docker
#   ./run_all_tests.sh --include-frontend
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
VENV_DIR="${REPO_ROOT}/.test-venv"
RESULTS_DIR="${SCRIPT_DIR}/results/run-$(date +%Y%m%d-%H%M%S)"

SKIP_DOCKER=0
SKIP_JEST=0
SKIP_ROBOT=0
INCLUDE_FRONTEND=0

usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Run pytest, Jest (Blockly), and Robot E2E tests. Intended for use on the Pi
from the tests/ directory; also works on a dev machine with Docker and Python.

Options:
  --skip-docker       Skip live Docker deployment tests (pytest -m docker)
  --skip-jest         Skip Blockly generator tests
  --skip-robot        Skip Robot Framework E2E suites
  --include-frontend  Also run tests/frontend/ (needs Cerebra :4200 + rfbrowser)
  -h, --help          Show this help

Environment:
  RUN_DOCKER_TESTS=1  Required for live Docker tests (set automatically unless --skip-docker)
  ROS2_TEST_MOCK=true Set for Robot E2E without ROS hardware (set automatically)
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --skip-docker) SKIP_DOCKER=1; shift ;;
        --skip-jest) SKIP_JEST=1; shift ;;
        --skip-robot) SKIP_ROBOT=1; shift ;;
        --include-frontend) INCLUDE_FRONTEND=1; shift ;;
        -h | --help) usage; exit 0 ;;
        *) echo "Unknown option: $1" >&2; usage; exit 2 ;;
    esac
done

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

section() {
    echo ""
    echo "========== $1 =========="
}

failures=0
run_step() {
    local label=$1
    shift
    section "${label}"
    if "$@"; then
        echo -e "${GREEN}OK:${NC} ${label}"
    else
        echo -e "${RED}FAILED:${NC} ${label}" >&2
        failures=$((failures + 1))
    fi
}

ensure_venv() {
    section "Python virtualenv"
    if [[ ! -d "${VENV_DIR}" ]]; then
        echo "Creating ${VENV_DIR} ..."
        python3 -m venv "${VENV_DIR}"
    fi
    # shellcheck source=/dev/null
    source "${VENV_DIR}/bin/activate"
    # --prefer-binary: use prebuilt wheels instead of compiling C extensions
    # (e.g. grpcio) from source, which is slow and fragile on arm64 / Raspberry Pi.
    pip install -q --prefer-binary -r "${SCRIPT_DIR}/integration/requirements.txt" \
        -r "${SCRIPT_DIR}/infrastructure/requirements.txt"
    if [[ ${SKIP_ROBOT} -eq 0 ]] || [[ ${INCLUDE_FRONTEND} -eq 1 ]]; then
        pip install -q --prefer-binary -r "${SCRIPT_DIR}/requirements-robot.txt"
    fi
    echo -e "${GREEN}OK:${NC} venv ready"
}

check_flask() {
    section "Flask preflight"
    if curl -sf --max-time 5 "http://localhost:5000/motor" >/dev/null; then
        echo -e "${GREEN}OK:${NC} Flask responding on :5000"
        return 0
    fi
    echo -e "${YELLOW}WARN:${NC} Flask not reachable on http://localhost:5000 — Robot E2E may fail"
    return 0
}

run_pytest_integration() {
    cd "${REPO_ROOT}"
    PYTHONPATH="${REPO_ROOT}/pib_api/flask" \
        pytest "${SCRIPT_DIR}" -q \
        --ignore="${SCRIPT_DIR}/blockly_generator" \
        -m "not docker"
}

run_pytest_docker() {
    cd "${REPO_ROOT}"
    export RUN_DOCKER_TESTS=1
    pytest "${SCRIPT_DIR}/infrastructure/test_docker_deployment.py" -m docker -q
}

run_jest() {
    local jest_dir="${SCRIPT_DIR}/blockly_generator"
    if command -v node >/dev/null 2>&1 && command -v npm >/dev/null 2>&1; then
        cd "${jest_dir}"
        npm install --silent
        # npm install can drop the exec bit on the jest launcher (observed on
        # the Pi filesystem), which makes `npx jest` fail with "Permission
        # denied". Restore it defensively before running.
        chmod +x node_modules/.bin/jest 2>/dev/null || true
        npx jest --config jest.config.js
        return
    fi
    if ! command -v docker >/dev/null 2>&1; then
        echo "Neither node/npm nor docker found — cannot run Jest" >&2
        return 1
    fi
    docker run --rm \
        -v "${REPO_ROOT}:/work" \
        -w "/work/tests/blockly_generator" \
        node:18-bookworm \
        bash -lc 'rm -rf node_modules && npm install --silent && npx jest --config jest.config.js'
}

run_robot_e2e() {
    cd "${REPO_ROOT}"
    export ROS2_TEST_MOCK=true
    mkdir -p "${RESULTS_DIR}"
    local suites=(
        "${SCRIPT_DIR}/e2e/system_e2e.robot"
        "${SCRIPT_DIR}/e2e/safety_scenarios.robot"
        "${SCRIPT_DIR}/e2e/infrastructure_recovery.robot"
    )
    if [[ ${SKIP_DOCKER} -eq 0 ]]; then
        export RUN_DOCKER_TESTS=1
    fi
    robot --outputdir "${RESULTS_DIR}" "${suites[@]}"
}

run_robot_frontend() {
    cd "${REPO_ROOT}"
    # `import Browser` succeeds even when the Node.js wrapper dependencies are
    # missing, so check for the actual node_modules directory instead.
    local browser_wrapper
    browser_wrapper="$(python -c 'import os, Browser; print(os.path.join(os.path.dirname(Browser.__file__), "wrapper", "node_modules"))' 2>/dev/null)"
    if [[ -z "${browser_wrapper}" ]] || [[ ! -d "${browser_wrapper}" ]]; then
        echo "Installing Browser library node dependencies ..."
        # --skip-browsers: do not download Playwright's bundled Chromium (the
        # download is unsupported on Raspberry Pi / arm64). The tests use the
        # system Chromium via the BROWSER_EXECUTABLE variable in
        # tests/resources/frontend_keywords.robot instead.
        rfbrowser init --skip-browsers || rfbrowser init
    fi
    mkdir -p "${RESULTS_DIR}"
    robot --outputdir "${RESULTS_DIR}" "${SCRIPT_DIR}/frontend/"
}

main() {
    echo "Repo:    ${REPO_ROOT}"
    echo "Results: ${RESULTS_DIR}"
    cd "${SCRIPT_DIR}"

    ensure_venv
    check_flask

    run_step "Pytest (integration + compose)" run_pytest_integration

    if [[ ${SKIP_DOCKER} -eq 0 ]]; then
        if command -v docker >/dev/null 2>&1; then
            run_step "Pytest (live Docker)" run_pytest_docker
        else
            echo -e "${YELLOW}SKIP:${NC} live Docker tests (docker not installed)"
        fi
    else
        echo -e "${YELLOW}SKIP:${NC} live Docker tests (--skip-docker)"
    fi

    if [[ ${SKIP_JEST} -eq 0 ]]; then
        run_step "Jest (Blockly generators)" run_jest
    else
        echo -e "${YELLOW}SKIP:${NC} Jest (--skip-jest)"
    fi

    if [[ ${SKIP_ROBOT} -eq 0 ]]; then
        run_step "Robot (E2E + infrastructure)" run_robot_e2e
    else
        echo -e "${YELLOW}SKIP:${NC} Robot (--skip-robot)"
    fi

    if [[ ${INCLUDE_FRONTEND} -eq 1 ]]; then
        run_step "Robot (frontend / Cerebra)" run_robot_frontend
    fi

    section "Summary"
    if [[ ${failures} -eq 0 ]]; then
        echo -e "${GREEN}All test steps passed.${NC}"
        exit 0
    fi
    echo -e "${RED}${failures} test step(s) failed.${NC}" >&2
    exit 1
}

main "$@"
