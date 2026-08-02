# PR-1520 — Voice Assistant Chat Performance Profiling & Latency Reduction (< 1s Response Time)

Jira Ticket: https://pib-rocks.atlassian.net/browse/PR-1520
Category: Software
Base Branch: `PR-1519`
Target Branch: `PR-1520` (DO NOT MERGE TO DEVELOP)

## Goals
Reduce Voice Assistant chat response latency (Time-To-First-Token / TTFT) from > 10s to < 1s by adding end-to-end tracing and optimizing the execution pipeline on the Raspberry Pi 5.

## Components to implement

1. **High-Precision End-to-End Tracing (`[PERF_TRACE]`)**:
   - `pib_api/flask/service/chat_service.py`: Log `[PERF_TRACE] API_ENTRY` and `[PERF_TRACE] API_EXIT` with elapsed ms.
   - `ros_packages/voice_assistant/voice_assistant/chat.py`: Log `[PERF_TRACE] ROS_SERVICE_RECV`, `[PERF_TRACE] FIRST_CHUNK_EMITTED`, `[PERF_TRACE] ROS_SERVICE_DONE` with elapsed ms.
   - `public_api_client/public_api_client/hermes_agent_client.py`: Log `[PERF_TRACE] HERMES_CLIENT_START`, `[PERF_TRACE] DAEMON_HTTP_START`, `[PERF_TRACE] DAEMON_TTFT_MS`, `[PERF_TRACE] HERMES_CLIENT_DONE`.
   - `public_api_client/public_api_client/hermes_daemon.py`: Log `[PERF_TRACE] DAEMON_RECV`, `[PERF_TRACE] DAEMON_TURN_START`, `[PERF_TRACE] DAEMON_FIRST_TOKEN`, `[PERF_TRACE] DAEMON_DONE`.

2. **Latency Optimizations (< 1s TTFT)**:
   - Ensure `hermes_agent_client.py` uses persistent `requests.Session` pooling for HTTP daemon requests (`127.0.0.1:8088`).
   - Bypass expensive filesystem profile re-validation or subprocess execution when the warm daemon is active.
   - Stream first generated tokens immediately through `_stream_chunks_to_goal` without sentence-buffering delay for TTFT.

3. **Tests**:
   - `tests/unit/test_hermes_daemon.py`: verify daemon fast-path and `[PERF_TRACE]` logging.
   - `tests/unit/test_chat_hermes_routing.py`: verify trace logging in `ChatNode`.

## Constraints
- Branch off `PR-1519` to `PR-1520`.
- DO NOT MERGE TO DEVELOP.
- Ensure all pytest unit and integration tests pass.
