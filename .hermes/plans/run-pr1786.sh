#!/bin/bash
cd /home/pib/opencode/pib-backend || exit 1
export CURSOR_API_KEY="$(grep '^CURSOR_API_KEY=' /home/pib/.hermes/profiles/remy/.env | cut -d= -f2)"
[ -z "$CURSOR_API_KEY" ] && { echo "FATAL: kein CURSOR_API_KEY"; exit 2; }
echo "=== branch: $(git rev-parse --abbrev-ref HEAD) ==="
echo "=== start: $(date -u) ==="
timeout 2400 /home/pib/.local/bin/cursor-agent -p "$(cat .hermes/plans/PR-1786-delegation-prompt.txt)" --output-format text --force 2>&1 | tail -180
echo "=== exit: ${PIPESTATUS[0]} at $(date -u) ==="
git log --oneline -3; git diff --stat origin/develop...HEAD | tail -6
