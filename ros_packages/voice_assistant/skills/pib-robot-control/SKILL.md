---
name: pib-robot-control
description: Interim reference for pib voice-assistant REST and Blockly controls.
---

# pib robot control — INTERIM

> **INTERIM ONLY:** This skill is a bridge for PR-1505 and will be superseded by
> the schema-validated pib MCP server in Jira PR-1506. The `terminal` toolset
> stays disabled for now. Do not use a shell, `curl`, or arbitrary commands to
> reach these APIs.

Use only the HTTP capability explicitly supplied by the runtime and only against
the robot's configured API origin. Paths below are rooted at `/api/v1`. Treat
actuator writes and program execution as side effects: perform them only when
the user clearly requests them, preserve configured limits, and never invent
identifiers or payload fields. Prefer a GET before a write.

## Voice-assistant REST surface

- `GET /api/v1/assistant-model` — list selectable assistant models.
- `GET /api/v1/voice-assistant/personality` — list personalities.
- `GET /api/v1/voice-assistant/personality/{personality_id}` — read one
  personality, including its description/SOUL and assistant model.
- `PUT /api/v1/voice-assistant/personality/{personality_id}` — update supplied
  personality fields. Do not use this to self-modify the SOUL.
- `POST /api/v1/voice-assistant/personality/{personality_id}/soul/append` with
  `{"lesson": "..."}` — append one non-empty durable lesson of at most 500
  characters. This endpoint is append-only; it cannot replace the existing
  SOUL.
- `GET /api/v1/voice-assistant/chat` and
  `GET /api/v1/voice-assistant/chat/{chat_id}` — list/read chats.
- `POST /api/v1/voice-assistant/chat` with
  `{"topic": "...", "personalityId": "..."}` — create a chat.
- `GET /api/v1/voice-assistant/chat/{chat_id}/messages` — read persisted
  messages.
- `POST /api/v1/voice-assistant/chat/{chat_id}/messages` with
  `{"isUser": true|false, "content": "..."}` — persist a message; this does not
  itself run a conversational turn.
- `DELETE /api/v1/voice-assistant/chat/{chat_id}` — delete a chat and
  best-effort delete its backing Hermes session.

## Motors, poses, and programs

- `GET /api/v1/motor` and `GET /api/v1/motor/{name}` — inspect motors.
- `GET|PUT /api/v1/motor/{name}/settings` — inspect or update motor settings.
- `GET|PUT /api/v1/motor/{name}/bricklet-pins` — inspect or update pin mapping.
- `GET /api/v1/pose` and `GET /api/v1/pose/{pose_id}` — inspect poses.
- `GET|PATCH /api/v1/pose/{pose_id}/motor-positions` — inspect or update a
  pose's motor positions.
- `GET|POST /api/v1/program`, `GET|PUT|DELETE /api/v1/program/{program_number}`,
  and `GET|PUT /api/v1/program/{program_number}/code` — manage stored programs
  and their Blockly visual code.

Program execution is a ROS 2 interface, not a REST write: stored programs run
through the `run_program` action (directly or via `proxy_run_program_start`);
temporary visual code uses `RunProgram.Goal.SOURCE_CODE_VISUAL`. With
`terminal` disabled, do not attempt to invoke ROS commands directly.

## `<pib-program>` Blockly response contract

To ask the existing voice-assistant pipeline to execute generated Blockly,
return the complete visual-code payload exactly once inside:

```text
<pib-program>COMPLETE_BLOCKLY_CODE_VISUAL</pib-program>
```

The content between the tags is passed unchanged as
`RunProgram.Goal.SOURCE_CODE_VISUAL` to the Blockly compiler/executor. Do not
wrap the tags in Markdown fences, escape the visual code, omit the closing tag,
or place explanatory prose inside the tags. Put spoken explanation outside the
block. Emit a program only when program generation is requested and execution
is safe.
