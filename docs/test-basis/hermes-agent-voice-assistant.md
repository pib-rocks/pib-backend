# Test Basis: Hermes Agent Voice Assistant

**Repository:** `pib-backend`  
**Requirement:** Jira PR-1505  
**Components:** Flask voice-assistant API, ROS 2 `ChatNode`, Hermes CLI, Cerebra personality editor

## Requirement

A personality can select `hermes-agent` as its conversation backend. Each pib
chat uses a distinct persistent Hermes session, while each personality uses a
Hermes profile whose `SOUL.md` mirrors `personality.description`. The existing
ROS chat action, persisted messages, TTS, and Blockly behavior remain compatible
with legacy assistant models. Hermes failures return a speakable fallback and do
not fail the chat action. Self-learning SOUL changes are append-only and bounded.

The live E2E test requires a reachable robot with the voice-assistant and
rosbridge services running, a seeded `hermes-agent` model, a personality, and a
working cloned Hermes profile with provider credentials. It skips with an
explicit reason when those prerequisites are absent. Set
`PIB_HERMES_E2E_PERSONALITY_ID` to select the personality and optionally
`PIB_E2E_BASE_URL` and `PIB_E2E_ROSBRIDGE_URL` for non-default robot addresses.

## Acceptance-criteria traceability

| AC | Acceptance criterion | Coverage | Status |
|---|---|---|---|
| AC1 | `hermes-agent` model is selectable. | `tests/unit/test_hermes_assistant_model_seed.py::test_seed_includes_hermes_agent_assistant_model`; live selection in `tests/e2e/test_voice_assistant_hermes_e2e.py::test_voice_assistant_hermes_persists_reply_and_recalls_prior_fact` | Automated; live path is prerequisite-gated |
| AC2 | Editor text is written to the profile `SOUL.md`. | `tests/unit/test_personality_soul_sync.py::test_update_description_writes_soul_file`; path and round-trip tests in `tests/unit/test_soul_service.py` | Automated backend synchronization; Cerebra editor interaction is covered in the Cerebra repository |
| AC3 | A non-empty assistant reply is persisted and published. | Live persistence assertion in `test_voice_assistant_hermes_persists_reply_and_recalls_prior_fact`; publication/action result assertions in `tests/unit/test_chat_hermes_routing.py::test_stream_chunks_to_goal_publishes_prior_sentence_as_feedback` and `::test_chat_routes_hermes_without_replaying_history` | Automated; live path is prerequisite-gated |
| AC4 | Memory is durable across turns and after restart. | Earlier-turn recall assertion in `test_voice_assistant_hermes_persists_reply_and_recalls_prior_fact`; deterministic named-session coverage in `tests/unit/test_hermes_agent_client.py::test_build_command_uses_oneshot_named_session_and_profile` | Cross-turn automated; restart verification manual/pending |
| AC5 | Chats have isolated Hermes sessions. | Session derivation in `tests/unit/test_hermes_agent_client.py::test_session_name_is_prefixed_and_sanitized` and `::test_session_name_strips_unsafe_chars` | Mapping automated; live two-chat non-leakage test pending |
| AC6 | The ROS chat action contract is unchanged. | Chunk/result/feedback tests in `tests/unit/test_chat_hermes_routing.py`, including `::test_stream_chunks_to_goal_extracts_pib_program` | Automated |
| AC7 | Legacy assistant models remain backwards compatible. | `tests/unit/test_chat_hermes_routing.py::test_chat_legacy_path_still_uses_public_api` | Automated |
| AC8 | Timeout produces a graceful fallback and the action still succeeds. | Fallback in `tests/unit/test_hermes_agent_client.py::test_run_turn_on_timeout_returns_fallback`; successful Hermes action path in `tests/unit/test_chat_hermes_routing.py::test_chat_routes_hermes_without_replaying_history` | Partial automated; combined timeout-through-action test pending |
| AC9 | SOUL append is append-only and oversized input returns HTTP 400. | `tests/unit/test_soul_append.py::test_soul_append_is_bounded_and_appends` and `::test_soul_append_rejects_oversized_lesson` (also checks empty input) | Automated |
| AC10 | Deleting a chat deletes its Hermes session. | `tests/unit/test_hermes_agent_client.py::test_delete_session_invokes_hermes_sessions_delete`; `chat_service.delete_chat` invokes that helper best-effort | Helper automated; service-level wiring test pending |
| AC11 | Full suites pass. | `python -m pytest tests/unit -v`; `python -m pytest tests/e2e/test_voice_assistant_hermes_e2e.py -v`; robot and Cerebra suites from the implementation plan | Local commands automated; full live-robot and Cerebra runs are environment-dependent/manual |
| AC12 | The requirement and coverage are documented in the test basis. | This document and the traceability table above | Documented |

## Manual checks still required

1. Complete one turn, restart the voice-assistant process, and verify the same
   chat recalls the established fact.
2. Establish different facts in two chats and verify neither fact leaks into
   the other session.
3. Force a Hermes timeout through the ROS action and verify the fallback is
   persisted/published while the action reports success.
4. Delete a chat on a live robot and verify its `pib_chat_<chat_id>` session is
   absent from `hermes sessions list`.
