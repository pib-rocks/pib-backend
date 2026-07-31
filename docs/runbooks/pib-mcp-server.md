# pib MCP Server Runbook

`pib_mcp_server` is a stdio MCP server for pib's REST API and ROS 2 services.
It is installed in the `ros-voice-assistant` image and is normally launched by
Hermes with `python3 -m pib_mcp_server`.

## Safety model

The five actuator tools are discoverable but reject every call by default:

- `pib_move_motor`
- `pib_apply_pose`
- `pib_run_program`
- `pib_set_led`
- `pib_set_relay`

Set `PIB_MCP_ENABLE_ACTUATION=true` in the MCP server's environment only for a
personality that is allowed to change hardware. This server-side gate is
independent of Hermes tool selection. `hermes mcp configure pib` can hide tools
from a profile, but selecting an actuator there does not bypass the gate.

`pib_soul_append` remains available because it is an append-only write. Its
`lesson` is schema-capped at 500 characters and the API enforces the same limit.

## Runtime configuration

| Variable | Container value | Purpose |
|---|---|---|
| `FLASK_API_BASE_URL` | `http://flask-app:5000` | Persisted motors, poses, programs, diagnostics, and SOUL |
| `PIB_MCP_ROSBRIDGE_URL` | `ws://rosbridge-ws:9090` | Live ROS services |
| `PIB_MCP_REQUEST_TIMEOUT` | `10` | HTTP/ROS request timeout in seconds |
| `PIB_MCP_ENABLE_ACTUATION` | `false` | Server-side actuator gate |

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
        PIB_MCP_ENABLE_ACTUATION=false \
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

To permit hardware changes, remove and re-register the server with
`PIB_MCP_ENABLE_ACTUATION=true`, then select only the required actuator tools
with `hermes mcp configure pib`. Do not enable the gate globally merely to grant
one personality access.

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
- `actuation_disabled`: expected until the registration environment explicitly
  enables actuation.
- `position_out_of_range`: inspect the motor's `rotationRangeMin` and
  `rotationRangeMax`; the server rejects rather than clamps unsafe values.
- `camera_empty`: verify `ros-camera` and `/get_camera_image`.

Server logs must go to stderr because stdout is reserved for the MCP stdio
transport.
