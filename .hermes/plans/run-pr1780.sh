#!/bin/bash
# Run the PR-1780 delegation to cursor-agent and print the real result.
cd /home/pib/opencode/pib-backend || exit 1
export CURSOR_API_KEY="$(grep '^CURSOR_API_KEY=' /home/pib/.hermes/.env | cut -d= -f2)"
if [ -z "$CURSOR_API_KEY" ]; then
  echo "FATAL: CURSOR_API_KEY not found in /home/pib/.hermes/.env"
  exit 2
fi
echo "=== branch ==="
git rev-parse --abbrev-ref HEAD
echo "=== cursor-agent start: $(date -u) ==="
timeout 2400 /home/pib/.local/bin/cursor-agent \
  -p "$(cat .hermes/plans/PR-1780-delegation-prompt.txt)" \
  --output-format text --force 2>&1 | tail -220
echo "=== cursor-agent exit: ${PIPESTATUS[0]} at $(date -u) ==="
echo "=== git status ==="
git status --short
echo "=== git log ==="
git log --oneline -3
echo "=== diff --stat vs develop ==="
git diff --stat origin/develop...HEAD
