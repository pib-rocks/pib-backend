#!/bin/bash
# Delegate PR-1798 to cursor-agent and print what actually changed.
set -u
cd /home/pib/opencode/pib-backend-PR-1798
export CURSOR_API_KEY="${CURSOR_API_KEY:-$(grep -E '^CURSOR_API_KEY=' ~/.hermes/.env | cut -d= -f2- | tr -d '"' | tr -d "'")}"
echo "=== PR-1798 Delegation $(date -Is) ==="
echo "BRANCH_BEFORE=$(git rev-parse --abbrev-ref HEAD) HEAD_BEFORE=$(git rev-parse --short HEAD)"
timeout 3300 /home/pib/.local/bin/cursor-agent --trust --model auto \
    --output-format stream-json --stream-partial-output \
    -p "$(cat .hermes/plans/PR-1798-delegation-prompt.txt)" 2>&1 | tail -160
echo "CURSOR_EXIT=$?"
echo "BRANCH_AFTER=$(git rev-parse --abbrev-ref HEAD) HEAD_AFTER=$(git rev-parse --short HEAD)"
echo "--- Status ---"
git status --short
echo "--- Commits auf dem Branch ---"
git log --oneline origin/develop..HEAD | head -10
echo "--- Geaenderte Dateien gegen develop ---"
git diff --name-only origin/develop...HEAD
echo "=== ENDE $(date -Is) ==="
