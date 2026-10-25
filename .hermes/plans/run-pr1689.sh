#!/usr/bin/env bash
set -euo pipefail
cd /home/pib/opencode/pib-backend
export PATH="$HOME/.local/bin:$PATH"
export CURSOR_API_KEY="${CURSOR_API_KEY:-}"
test -n "${CURSOR_API_KEY:-}" || { echo "CURSOR_API_KEY not set"; exit 2; }
echo "BRANCH_BEFORE=$(git branch --show-current)"
timeout 2400 cursor-agent --print --force --output-format stream-json --stream-partial-output \
  "$(cat .hermes/plans/PR-1689-delegation-prompt.txt)"
echo "CURSOR_EXIT=$?"
echo "=== git status after ==="
git status --short --untracked-files=no