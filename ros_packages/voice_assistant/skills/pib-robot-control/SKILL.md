---
name: pib-robot-control
description: Use schema-validated pib MCP tools for robot capabilities.
---

# pib robot control

Use the `pib_*` MCP tools. The `terminal` toolset stays disabled; do not use a
shell, `curl`, arbitrary commands, or raw ROS calls to control the robot.

Treat actuator writes and program execution as side effects: perform them only
when the user clearly requests them, preserve configured limits, and never
invent identifiers or payload fields. Prefer a read tool before a write.

## MCP tool surface

- Inspect with `pib_list_motors`, `pib_get_state`, `pib_list_poses`, and
  `pib_list_programs`.
- Capture a camera frame with `pib_capture_image`.
- Actuate only on explicit request with `pib_move_motor`, `pib_apply_pose`,
  `pib_run_program`, `pib_set_led`, or `pib_set_relay`. These tools are blocked
  by default and require operator enablement.
- Append one durable lesson with `pib_soul_append`. Lessons must be non-empty
  and no longer than 500 characters; this operation never replaces the SOUL.

## Backend reference

The MCP server owns access to the following implementation interfaces. They are
listed for context, not for direct invocation:

### Voice-assistant REST surface
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
