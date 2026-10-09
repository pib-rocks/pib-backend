# pib MCP Server Runbook

`pib_mcp_server` is a stdio MCP server for pib's REST API and ROS 2 services.
It is installed in the `ros-voice-assistant` image and is normally launched by
Hermes with `python3 -m pib_mcp_server`.

## Safety model

The actuator tools are discoverable, and the server-side gate that allows them
to change hardware is open by default (`PIB_MCP_ENABLE_ACTUATION=true`):

- `pib_move_motor`
- `pib_apply_pose`
- `pib_run_program`
- `pib_set_led`
- `pib_set_relay`

The gate is global by design. Every personality that has the pib tools
registered can move hardware. A personality payload cannot carry the gate:
`personality_service._reject_actuation_request` rejects the request with
"The actuation gate is an installation setting", and
`pib_hermes_config/live_interaction.py` lists `pib_mcp_enable_actuation` among
the fields a personality payload must not carry. A per-personality actuation
switch is out of scope.

`hermes mcp configure pib` can hide tools from a profile. Hiding a tool is not
the gate. Selecting an actuator there does not bypass a closed gate.

`pib_soul_append` remains available because it is an append-only write. Its
`lesson` is schema-capped at 500 characters and the API enforces the same limit.

## Runtime configuration

| Variable | Container value | Purpose |
|---|---|---|
| `FLASK_API_BASE_URL` | `http://flask-app:5000` | Persisted motors, poses, programs, diagnostics, and SOUL |
| `PIB_MCP_ROSBRIDGE_URL` | `ws://rosbridge-ws:9090` | Live ROS services |
| `PIB_MCP_REQUEST_TIMEOUT` | `10` | HTTP/ROS request timeout in seconds |
| `PIB_MCP_ENABLE_ACTUATION` | `true` | Server-side actuator gate, open by default |

`PIB_MCP_API_BASE_URL` may be set to override `FLASK_API_BASE_URL`.

## Register for one personality

Hermes profiles are isolated directories. Run registration with that profile as
`HERMES_HOME`, as the same OS user that runs Hermes:

```bash
PROFILE=/home/pib/.hermes/profiles/pib_<personality_id>

sudo -u pib -H env HERMES_HOME="$PROFILE" \
  hermes mcp add pib \
  --command python3 \
  --connect-timeout 15 \
  --env FLASK_API_BASE_URL=http://flask-app:5000 \
        PIB_MCP_ROSBRIDGE_URL=ws://rosbridge-ws:9090 \
        PIB_MCP_ENABLE_ACTUATION=true \
  --args -m pib_mcp_server
```

Keep `--args` last; Hermes treats everything after it as server arguments.
Repeat registration for each personality that should receive pib tools.

Verify discovery:

```bash
sudo -u pib -H env HERMES_HOME="$PROFILE" hermes mcp list
sudo -u pib -H env HERMES_HOME="$PROFILE" hermes mcp test pib
```

The test should discover eleven tools. A standalone protocol smoke test is:

```bash
FLASK_API_BASE_URL=http://flask-app:5000 \
PIB_MCP_ROSBRIDGE_URL=ws://rosbridge-ws:9090 \
PIB_MCP_ENABLE_ACTUATION=true \
python3 -m pib_mcp_server
```

The standalone process waits for MCP JSON-RPC on stdin; silence is normal.

## Toggle tools

Use Hermes' interactive selector to disable tools that a personality should not
see:

```bash
sudo -u pib -H env HERMES_HOME="$PROFILE" hermes mcp configure pib
```

For a read-only profile, leave only `pib_list_motors`, `pib_get_state`,
`pib_list_poses`, `pib_list_programs`, and `pib_capture_image` selected.
That hides actuator tools from that profile. It does not close the gate, and
it is not a per-personality actuation setting. The gate stays the installation
variable `PIB_MCP_ENABLE_ACTUATION`, which defaults to `true` for every
registration.

## Deregister

```bash
sudo -u pib -H env HERMES_HOME="$PROFILE" hermes mcp remove pib
sudo -u pib -H env HERMES_HOME="$PROFILE" hermes mcp list
```

Removal changes only that personality's MCP configuration. It does not delete
the personality, SOUL, sessions, stored poses/programs, or robot data.

## Troubleshooting

- `api_unavailable`: verify `flask-app` and `FLASK_API_BASE_URL`.
- `ros_unavailable` or `ros_timeout`: verify `rosbridge-ws`, port 9090, and the
  target ROS service.
- `actuation_disabled`: the registration environment has the gate closed.
  The shipped default is enabled. An unset, empty, or other non-true value
  closes it.
- `position_out_of_range`: inspect the motor's `rotationRangeMin` and
  `rotationRangeMax`; the server rejects rather than clamps unsafe values.
- `camera_empty`: verify `ros-camera` and `/get_camera_image`.

Server logs must go to stderr because stdout is reserved for the MCP stdio
transport.
