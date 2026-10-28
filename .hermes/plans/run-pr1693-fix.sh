#!/usr/bin/env bash
set -euo pipefail
cd /home/pib/opencode/pib-backend
export PATH="$HOME/.local/bin:$PATH"
export CURSOR_API_KEY="${CURSOR_API_KEY:-}"
test -n "${CURSOR_API_KEY:-}" || { echo "CURSOR_API_KEY fehlt"; exit 1; }
BRANCH_BEFORE="$(git branch --show-current)"
echo "BRANCH_BEFORE=$BRANCH_BEFORE"
timeout 2400 cursor-agent --print --force --output-format stream-json --stream-partial-output \
  "$(cat .hermes/plans/PR-1693-fix-prompt.txt)" > /tmp/pr1693_fix_raw.log 2>&1
CURSOR_EXIT=$?
echo "CURSOR_EXIT=$CURSOR_EXIT"
echo "=== git status after ==="
git status --short --untracked-files=no | head -20