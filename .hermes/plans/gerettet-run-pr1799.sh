#!/bin/bash
# Delegate PR-1799 (facemesh + chain generalisation) to cursor-agent.
set -u
cd /home/pib/opencode/pib-backend-PR-1799
export CURSOR_API_KEY="${CURSOR_API_KEY:-$(grep -E '^CURSOR_API_KEY=' ~/.hermes/.env | cut -d= -f2- | tr -d '"')}"
echo "=== PR-1799 Delegation $(date -Is) ==="
echo "BRANCH_BEFORE=$(git rev-parse --abbrev-ref HEAD) HEAD_BEFORE=$(git rev-parse --short HEAD)"
echo "CEREBRA_BEFORE=$(git -C /home/pib/opencode/cerebra-PR-1799 rev-parse --short HEAD)"
timeout 3000 /home/pib/.local/bin/cursor-agent -p --force --trust \
  "$(cat .hermes/plans/PR-1799-delegation-prompt.txt)" 2>&1 | tail -40
echo "CURSOR_EXIT=$?"
echo "--- Backend Status ---"
git status --short
echo "--- Backend Commits ---"
git log --oneline -3
echo "--- Cerebra Status ---"
git -C /home/pib/opencode/cerebra-PR-1799 status --short
echo "--- Cerebra Commits ---"
git -C /home/pib/opencode/cerebra-PR-1799 log --oneline -3
echo "=== ENDE $(date -Is) ==="
