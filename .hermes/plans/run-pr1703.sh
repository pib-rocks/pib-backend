#!/usr/bin/env bash
set -euo pipefail
cd /home/pib/opencode/pib-backend
export PATH="$HOME/.local/bin:$PATH"
export CURSOR_API_KEY="${CURSOR_API_KEY:-}"
test -n "${CURSOR_API_KEY:-}" || { echo "CURSOR_API_KEY fehlt"; exit 1; }
echo "BRANCH_BEFORE=$(git branch --show-current)"
timeout 1800 cursor-agent --print --force --output-format stream-json --stream-partial-output -p "$(cat .hermes/plans/PR-1703-delegation-prompt.txt)" > /tmp/pr1703_raw.log 2>&1 || echo "CURSOR_EXIT=$?"
echo "CURSOR_EXIT=$?"
echo "=== git status after ==="
git status --short --untracked-files=no