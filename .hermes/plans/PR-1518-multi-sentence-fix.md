# PR-1518 — Fix: Voice Assistant loses tail of multi-sentence responses

Jira: https://pib-rocks.atlassian.net/browse/PR-1518
Repo: `pib-backend` (ros_packages/voice_assistant)
Target file: `ros_packages/voice_assistant/voice_assistant/chat.py`
Function: `_stream_chunks_to_goal` (approx. lines 700-750)

## Problem (verified by read-only analysis)

The Voice Assistant ROS node loses the tail of multi-sentence assistant responses when Hermes Agent uses MCP tools. Only the first sentence of the final synthesized response is persisted and displayed.

## Root cause (identified)

In `_stream_chunks_to_goal`:

1. Streaming chunks are accumulated and sent via **UPDATE** to an existing assistant message (id=2)
2. When a sentence terminator (`.?!/:`) is detected, a **CREATE** inserts a NEW assistant message (id=3) with only the **FIRST sentence** of the synthesized response
3. No further **UPDATE** targets id=3 — the rest of the response is lost
4. Additionally, leftover text without a terminator is discarded at the end (lines 733-742), because the Action result prefers `prev_text` over `curr_text`

## What is NOT the problem (do NOT touch)
- cerebra `handleStreamedMessages()` — same-id overwrite / new-id append logic is correct
- `hermes_agent_client.py` — by design one assistant reply per turn
- Flask `chat_service.py` — id policy is correct

## Required fix (minimal, backend only)

### 1. Serialize CREATE/UPDATE in `_stream_chunks_to_goal`
Ensure that when a sentence terminator triggers a CREATE, that CREATE **completes** before any subsequent UPDATEs target the new messageId. Options:
- Await the `create_chat_message` future before continuing the loop
- Use a single ordered queue for DB operations
- Or restructure: accumulate full response, then single CREATE at end (simpler, but loses streaming feel)

### 2. Persist leftover `curr_text` after stream ends
After the streaming loop, if `curr_text` is non-empty, persist it (CREATE new or UPDATE existing) rather than discarding it. The Action result currently prefers `prev_text`.

### 3. Keep streaming behavior for UX
The partial UPDATEs to the existing message (id=2) during streaming should continue to work — user sees incremental progress. The fix only ensures the final synthesized response is fully persisted under its own messageId.

## Files to modify
- `pib-backend/ros_packages/voice_assistant/voice_assistant/chat.py` — `_stream_chunks_to_goal` function

## Verification
After fix, run E2E test against live Pi 5 (192.168.1.28):
```bash
/home/pib/.hermes/hermes-agent/venv/bin/pytest tests/e2e/test_voice_assistant_hermes_e2e.py -k persists_reply -v
```
Expected: Full synthesized response displayed in UI (no truncation after first sentence).

Note: Full pass also requires PR-1517 (SQLite lock) to be fixed, but the message loss should be fixed independently.

## Constraints
- DO NOT modify cerebra, hermes_agent_client.py, or Flask chat_service.py
- DO NOT change the messageId contract (one assistant reply per turn is the design)
- Keep streaming UX (partial updates during generation)
- Prettier: 4-space indent for Python