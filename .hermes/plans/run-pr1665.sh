#!/usr/bin/env bash
set -euo pipefail
cd /home/pib/opencode/pib-backend
export PATH="$HOME/.local/bin:$PATH"
export CURSOR_API_KEY="${CURSOR_API_KEY:-}"
test -n "${CURSOR_API_KEY:-}" || { echo "CURSOR_API_KEY not set"; exit 2; }
echo "BRANCH_BEFORE=$(git branch --show-current)"
timeout 1100 cursor-agent -p "$(cat .hermes/plans/PR-1665-delegation-prompt.txt)" \
  --output-format text --force 2>&1 | tail -180
echo "CURSOR_EXIT=${PIPESTATUS[0]}"
echo "=== git status after ==="
git status --short --untracked-files=no