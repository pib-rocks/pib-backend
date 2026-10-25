#!/usr/bin/env bash
set -euo pipefail
cd /home/pib/opencode/pib-backend
export PATH="$HOME/.local/bin:$PATH"
export CURSOR_API_KEY="${CURSOR_API_KEY:-}"
test -n "${CURSOR_API_KEY:-}" || { echo "CURSOR_API_KEY missing"; exit 1; }
cat /tmp/p1705.txt .hermes/plans/PR-1705-rules.txt > /tmp/p1705-final.txt
echo "=== starting cursor-agent (trust) ==="
cursor-agent --trust --model auto --output-format stream-json --stream-partial-output \
  -p "$(cat /tmp/p1705-final.txt)" 2>&1 | tail -40
echo "=== cursor-agent rc=$? ==="
