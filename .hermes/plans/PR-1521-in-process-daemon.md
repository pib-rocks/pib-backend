# PR-1521 — In-Process Warm Hermes Agent Execution in Daemon (< 1s Response Time)

Jira Ticket: https://pib-rocks.atlassian.net/browse/PR-1521
Category: Software
Base Branch: `PR-1520`
Target Branch: `PR-1521` (DO NOT MERGE TO DEVELOP)

## Goals
Convert `hermes_daemon.py` from spawning cold subprocesses (`subprocess.run([hermes_bin, ...])`) to executing Hermes Agent turns **in-process** via Python API (`from hermes.run_agent import run_agent`). This eliminates subprocess spawn overhead, Python import delays, and profile reloading, dropping turn response latency (TTFT) to < 1s.

## Components to implement

1. `public_api_client/public_api_client/hermes_daemon.py`:
   - Implement `run_turn_in_process(text: str, chat_id: str, personality_id: Optional[str] = None, timeout: Optional[int] = None) -> str`:
     - Resolves personality profile directory and ensures profile credentials/SOUL.md (`ensure_profile`).
     - Imports `run_agent` from `hermes.run_agent` in-process.
     - Executes `run_agent(prompt=text, session_id=f"pib_chat_{chat_id}", profile=profile_name, timeout=timeout)` directly in Python.
     - Catches and logs errors gracefully.
   - Use `run_turn_in_process` as the default `turn_runner` in `hermes_daemon.py`.

2. Logging & Tracing:
   - Retain `[PERF_TRACE]` timing logs (`DAEMON_RECV`, `DAEMON_TURN_START`, `DAEMON_FIRST_TOKEN`, `DAEMON_DONE`).

3. Tests:
   - Update `tests/unit/test_hermes_daemon.py` and `tests/unit/test_chat_hermes_routing.py` to test in-process daemon execution and fallback.

## Constraints
- Branch off `PR-1520` to `PR-1521`.
- DO NOT MERGE TO DEVELOP.
- Ensure all pytest unit and integration tests pass.
