#!/usr/bin/env bash
# Canonical unit-test recipe. CI calls this instead of its own pytest command.
# No --ignore and no --deselect: every test under tests/unit is in the run.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

exec python -m pytest tests/unit -q "$@"
