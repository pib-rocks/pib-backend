# Test Basis: Long-Lived Hermes Agent Daemon for Low-Latency Voice Assistant Turns (PR-1535)

**Jira Story:** PR-1535  
**Repository:** `pib-backend`  

## Objectives & Scope

Eliminate the ~3–5 second Python interpreter and FastMCP cold-start overhead per turn on the Raspberry Pi 5 by introducing a long-lived, warm Hermes Agent daemon service.

### Requirements & Acceptance Criteria

1. **Hermes Agent Daemon (`hermes_daemon.py`):**
   - Create a lightweight long-lived daemon service (running under `ros-voice-assistant` container) that keeps the Hermes Agent process, module imports, and FastMCP `pib` tools warm in memory.
   - Expose a simple internal HTTP/socket endpoint (`/turn`) to accept conversational turn requests (`text`, `chat_id`, `personality_id`).

2. **Client Integration (`hermes_agent_client.py`):**
   - Update `run_turn()` to query the warm daemon via fast HTTP/socket request.
   - Fall back to oneshot subprocess or graceful fallback if the daemon is unavailable.

3. **Performance Target:**
   - Eliminate Python interpreter cold-start (~1.5–3.5s) and FastMCP stdio spawn (~1–2s) on every turn on the Raspberry Pi 5.
   - Reduce overall turn latency by **3 to 5 seconds**.

4. **Testing:**
   - Add unit and integration tests in `tests/unit/test_hermes_daemon.py` and `tests/unit/test_hermes_agent_client.py`.
   - Update E2E tests to verify warm daemon turn execution.
