#!/bin/bash
cd /home/pib/opencode/pib-backend || exit 1
export CURSOR_API_KEY="$(grep '^CURSOR_API_KEY=' /home/pib/.hermes/profiles/remy/.env | cut -d= -f2)"
[ -z "$CURSOR_API_KEY" ] && { echo "FATAL: kein CURSOR_API_KEY"; exit 2; }
echo "=== branch: $(git rev-parse --abbrev-ref HEAD) @ $(git log --oneline -1) ==="
echo "=== start: $(date -u) ==="
timeout 3000 /home/pib/.local/bin/cursor-agent \
  -p "$(cat .hermes/plans/PR-1781-delegation-prompt.txt)" \
  --output-format text --force 2>&1 | tail -200
echo "=== exit:  at $(date -u) ==="
git status --short | head
git log --oneline -3
git diff --stat origin/develop...HEAD
