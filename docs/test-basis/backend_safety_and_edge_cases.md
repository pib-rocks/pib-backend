# Backend Safety Mechanisms and Edge Cases

**Repository:** `pib-backend`  
**Format:** BDD Gherkin (`Feature` / `Scenario`)  
**Scope:** Application layer (Flask, ROS2, Blockly) + infrastructure layer (Docker, boot, hardware)

---

## Feature: Flask HTTP error handling

### Scenario: Schema validation failure on POST

```gherkin
Given the Flask server receives a POST with invalid JSON body for a Marshmallow-backed endpoint
When schema loading raises ValidationError
Then the response status is 400
And the response body is {"error": "Bad request."}
And the database session is rolled back
```

### Scenario: Entity not found

```gherkin
Given a REST client requests a resource with an unknown path parameter
When SQLAlchemy raises NoResultFound
Then the response status is 404
And the response body is {"error": "Entity not found. Please check your path parameter."}
```

### Scenario: Database integrity violation

```gherkin
Given a REST client submits a duplicate unique field
When SQLAlchemy raises IntegrityError
Then the response status is 400
And the response body is {"error": "Bad request."}
```

### Scenario: Blockly compilation failure on program save

```gherkin
Given the client sends PUT /program/{programNumber}/code
When pib_blockly_client.code_visual_to_python returns (False, None)
Then the response status is 500
And the response body contains {"error": "an unknown error occured."}
```

### Scenario: Non-deletable pose deletion

```gherkin
Given a pose exists with deletable=false
When the client sends DELETE /pose/{poseId}
Then pose_service raises ValueError
And the response status is 500
```

### Scenario: Host IP file missing

```gherkin
Given HOST_IP_FILE does not exist on disk
When the client sends GET /host-ip
Then the response status is 500
And the response body is {"error": "Unable to retrieve host IP"}
```

### Scenario: Missing Content-Type on JSON POST

```gherkin
Given the Flask server receives POST /program without Content-Type application/json
When Werkzeug raises UnsupportedMediaType (415)
Then the generic Exception handler returns status 500
And the response is not 400
```

---

## Feature: ROS2 action and service timeouts (application)

### Scenario: ROS2 Action Server does not respond — actual Flask behavior

```gherkin
Given a user initiates motor movement through ROS2 apply_joint_trajectory
When the action or service call blocks or times out inside ROS nodes
Then no Flask endpoint is invoked
And no HTTP 503 is returned to the frontend
And the failure is contained within the ROS2 process graph
```

**Test oracle:** HTTP **503 is not implemented** in Flask for ROS timeouts. Do not assert 503 on REST for move commands.

### Scenario: Blockly TF button service timeout

```gherkin
Given a Blockly program calls blockly_client._call with default timeout_sec=10.0
When /tf_button/read is unavailable for 10 seconds
Then wait_for_service returns false
And _call raises RuntimeError or TimeoutError
And the user program process may exit with non-zero code
And Flask is not notified
```

### Scenario: rgb_button_control blocks on proxy services at startup

```gherkin
Given the rgb_button_control node is starting
When proxy_run_program_start is not yet available
Then wait_for_service blocks indefinitely
And the node does not finish __init__ until proxy_program is running
And Docker restart policy may restart the container until dependencies are up
```

### Scenario: Audio mic configuration timeout

```gherkin
Given audio_recorder calls request_mic_configuration with timeout_sec=25.0
When get_mic_configuration service is unavailable for 25 seconds
Then the method returns False
And a warning is logged with the timeout duration
```

---

## Feature: Motor position and hardware limits

### Scenario: Joint command exceeds configured rotation range

```gherkin
Given motor_control receives apply_joint_trajectory for tilt_forward_motor
And the requested position is 5000 outside seeded range -4500..4500
When motor.set_position is invoked
Then Motor._validate_position clamps the value to 4500
And no HTTP error is generated
```

### Scenario: Joint command exceeds global hardware envelope

```gherkin
Given a motor has rotationRangeMin=-9000 and rotationRangeMax=9000
When apply_joint_trajectory requests position 12000
Then the effective command is clamped to 9000
```

### Scenario: Unknown joint name

```gherkin
Given a client calls get_joint_position with joint_name "nonexistent_joint"
When the joint is not in name_to_motors
Then response.successful is false
And response.message is "unknown joint name 'nonexistent_joint'"
```

### Scenario: Motor without connected bricklet pins

```gherkin
Given a motor has no connected bricklet pins
When apply_joint_trajectory targets that motor
Then motor.set_position returns false
And response.successful is false after aggregation
```

### Scenario: Startup pose motor wait timeout

```gherkin
Given StartupPoseExecutor commanded multiple motors to startup positions
When not all motors reach target within ALL_MOTORS_TIMEOUT (15.0 seconds)
Then a warning is logged
And original motor velocity settings are restored anyway
```

---

## Feature: pib_api_client HTTP failure handling

### Scenario: Flask returns 404 during ROS poll

```gherkin
Given rgb_button_control calls GET /button-programs
When Flask responds with HTTP 404
Then send_request returns (False, None)
And rgb_button_control logs "Failed to load button programs from backend."
And button presses find no program assignment
```

### Scenario: Flask unreachable at pib_motors import

```gherkin
Given the ros-motors container starts before flask-app is ready
When pib_motors.motor module calls GET /motor at import time
And send_request returns (False, None)
Then RuntimeError "failed to load motors from pib-api..." is raised
And the container exits and Docker restarts it due to restart: always
```

### Scenario: Voice assistant cannot persist chat message

```gherkin
Given chat node calls POST /voice-assistant/chat/{chatId}/messages
When Flask returns an HTTP error
Then send_request returns (False, None)
And the message is not stored in SQLite
```

---

## Feature: Program execution safety

### Scenario: Program cancelled via proxy stop

```gherkin
Given a program is running with proxy_goal_id "abc-123"
When proxy_run_program_stop is called with that ID
Then the run_program goal is cancelled
And the subprocess is terminated
And RunProgram.Result exit_code is 2
And proxy_run_program_result is published
```

### Scenario: Program non-zero exit sets button red

```gherkin
Given proxy_run_program_result is received with exit_code != 0
When rgb_button_control handles the result
Then the button LED is set to red (255, 0, 0)
And sticky manual colors may be restored after program end per rgb_button_control logic
```

### Scenario: Program stdin to dead process

```gherkin
Given program_input is published for an unknown mpid
When program_input_callback runs
Then a warning is logged
And the program node continues running
```

### Scenario: Visual code compile failure at runtime

```gherkin
Given run_program action receives SOURCE_CODE_VISUAL
When pib_blockly_client.code_visual_to_python fails
Then the goal is aborted
And RunProgram.Result exit_code is 2
```

---

## Feature: Button color and sticky state

### Scenario: set_button_color published without subscriber

```gherkin
Given blockly_client publishes to set_button_color
When no subscriber is present within 3 seconds
Then a warning is logged "No subscribers on set_button_color after 3s; publishing anyway"
And the message is still published
```

### Scenario: Sticky color survives poll cycle

```gherkin
Given a sticky color is set for button UID via set_button_color with sticky=true
When update_button_colors poll runs and a program is assigned
Then the sticky color is not overwritten by default blue assignment logic
```

### Scenario: Unknown bricklet UID for color set

```gherkin
Given set_button_color targets an unknown UID
When rgb_led_bricklets.get(uid) returns None
Then the call returns silently without raising
```

---

## Feature: Solid state relay and startup gating

### Scenario: SSR off delays startup pose

```gherkin
Given solid_state_relay_state.turned_on is false
When motor_control receives the SSR state message
Then startup pose is not executed
```

### Scenario: SSR turns on triggers startup pose once

```gherkin
Given startup has not completed (_startup_done=false)
When solid_state_relay_state arrives with turned_on=true
Then _execute_startup_pose is invoked once
And _startup_done is set true
```

### Scenario: No SSR bricklet configured

```gherkin
Given solid_state_relay_bricklet is None
When motor_control initializes
Then startup pose executes immediately
```

---

## Feature: Docker container lifecycle (infrastructure)

### Scenario: ROS container crashes during operation

```gherkin
Given the system is running with docker compose --profile all
When the ros-motors container process crashes
Then Docker restart policy always restarts the container
And ros-motors re-imports pib_motors requiring Flask GET /motor and GET /bricklet
And rgb_button_control blocks until proxy_run_program services return
And Flask does not automatically reconnect because it never connected to ROS
```

### Scenario: Flask container restart while ROS is running

```gherkin
Given ros-motors is running and connected to hardware
When flask-app container is recreated
Then SQLite data persists if the database file is on a preserved volume or bind mount
And ROS nodes continue running with cached motor settings until restart
And the next pib_api_client call uses the new flask-app DNS endpoint on pib-network
```

### Scenario: Cold boot race — motors before Flask

```gherkin
Given docker compose starts all profile services simultaneously
When ros-motors starts before flask-app finishes seed_db
Then pib_motors import may fail with RuntimeError
And ros-motors container restarts until flask-app responds on port 5000
And no explicit healthcheck coordinates the startup
```

### Scenario: rosbridge unavailable

```gherkin
Given Cerebra frontend connects via WebSocket to localhost:9090
When rosbridge-ws container is stopped
Then the WebSocket connection fails
And Flask REST on port 5000 may still respond independently
And ROS nodes on pib-network continue DDS communication without rosbridge
```

### Scenario: Missing hardware profile services

```gherkin
Given docker compose is started without --profile all
When only default services are running
Then pib-blockly-server, flask-app, and rosbridge-ws are up
And ros-motors, ros-programs, and ros-camera are not started
And motor/program ROS services are unavailable to Cerebra
```

### Scenario: Tinkerforge host unreachable from container

```gherkin
Given ros-motors uses TINKERFORGE_HOST=host.docker.internal
When the Tinkerforge daemon is not listening on port 4223
Then bricklet enumeration fails
And motor hardware commands fail at the Tinkerforge layer
And apply_joint_trajectory may return successful=false
```

### Scenario: Shared programs volume desync

```gherkin
Given flask-app writes {programNumber}.py to the programs named volume
When ros-programs executes run_program with that program_number
Then it reads from PROGRAM_DIR=/ros2_ws/cerebra_programs on the same volume
And a missing file causes program execution failure with non-zero exit_code
```

---

## Feature: pib-blockly-server failures

### Scenario: Invalid visual JSON

```gherkin
Given pib-blockly-server receives POST with non-JSON body
When codeVisualToPython throws
Then the server responds with HTTP 400
And body is "failed to compile visual-code."
```

### Scenario: Blockly server down during Flask program save

```gherkin
Given flask-app calls PIB_BLOCKLY_SERVER_URL during PUT /program/{id}/code
When the blockly server connection fails
Then pib_blockly_client returns (False, None)
And Flask returns HTTP 500
```

---

## Feature: Voice and audio infrastructure

### Scenario: PulseAudio socket missing in voice container

```gherkin
Given ros-voice-assistant mounts Pulse native socket from host
When the socket path is absent or wrong UID
Then audio playback or recording fails at ALSA/Pulse layer
And voice assistant nodes log hardware errors
And Flask chat persistence may still succeed independently
```

### Scenario: Missing password.env for voice assistant

```gherkin
Given ros-voice-assistant references env_file password.env
When password.env was not generated by docker_install.sh
Then docker compose may fail to start ros-voice-assistant
Or public API token flows fail at runtime
```

---

## Feature: Camera infrastructure

### Scenario: OAK-D camera not available

```gherkin
Given ros-camera profile is active
When camera hardware init fails
Then get_camera_image service is not created
And camera_topic publishes error strings
And chat vision path falls back without image context
```

---

## Feature: CI pipeline

### Scenario: Black formatting violation on push

```gherkin
Given a developer pushes Python code with Black-incompatible formatting
When the Lint workflow runs on ubuntu-latest
Then psf/black@stable fails the job
And no automated integration or E2E tests run in CI
```

---

## Test ID Mapping

| Prefix | Framework | Source scenarios |
|---|---|---|
| `API-ERR-` | Pytest | Flask HTTP error handling |
| `API-HC-` | Pytest | Hard constraints in api_contracts.md |
| `ROS-LIM-` | Pytest / ROS2TestLibrary | Motor clamping, joint services |
| `ROS-TMO-` | Robot / ROS2TestLibrary | Service/action timeouts |
| `INF-DOCK-` | Testcontainers / Compose | Container lifecycle |
| `INF-BOOT-` | Shell + Compose | Startup race, profiles |
| `E2E-BDD-` | Robot Framework | End-to-end Gherkin above |

---

## Assertion Anti-Patterns

| Do not assert | Reason |
|---|---|
| HTTP 503 from Flask on ROS timeout | Not implemented |
| Flask validates motor angles on pose POST | Clamping is ROS-only |
| `depends_on` guarantees Flask readiness | No healthcheck in compose |
| pib_api_client propagates HTTP status | Always `(False, None)` on failure |
| Immediate cross-container recovery without restart | `restart: always` + blocking waits |
