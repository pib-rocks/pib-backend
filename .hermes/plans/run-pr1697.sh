#!/usr/bin/env bash
set -euo pipefail
cd /home/pib/opencode/pib-backend
export PATH="$HOME/.local/bin:$PATH"
export CURSOR_API_KEY="${CURSOR_API_KEY:-}"
test -n "${CURSOR_API_KEY:-}" || { echo "CURSOR_API_KEY fehlt"; exit 1; }
P=/tmp/pr1697_prompt.txt
cat .hermes/plans/PR-1697-prompt-a.txt .hermes/plans/PR-1697-prompt-b.txt > "$P"
echo "PROMPT_LINES=$(wc -l < "$P")"
timeout 3000 cursor-agent -p --force --output-format stream-json --stream-partial-output "$(cat "$P")" > /tmp/pr1697_raw.log 2>&1
echo "CURSOR_EXIT=$?"
