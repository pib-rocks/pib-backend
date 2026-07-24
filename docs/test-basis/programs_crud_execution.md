# Test Basis: Program CRUD Operations and ROS 2 Program Execution

**Repository:** `pib-backend`  
**Branch:** `PR-1492`  
**Components:** `pib_api/flask`, `pib_blockly/pib_blockly_server`, `pib_blockly_client`, `ros_packages/programs` (`ProgramNode`, `ProxyProgramNode`)

---

## Overview

The Program Management & Execution subsystem in `pib-backend` allows users to create, read, update, compile, delete, and execute robotic behavior programs. Programs consist of visual code structures (Blockly XML/JSON) compiled into executable Python scripts.

The architecture comprises three main tiers:
1. **Flask REST API (`pib_api/flask`)**: Provides HTTP CRUD endpoints for program metadata and visual code, interacting with SQLite and managing local script files in `PYTHON_CODE_DIR`.
2. **Blockly Compilation Server (`pib_blockly_server` / `pib_blockly_client`)**: Transforms visual block representations into executable Python scripts via HTTP POST requests on port 2442.
3. **ROS 2 Execution Nodes (`ros_packages/programs`)**:
   - `ProgramNode`: Exposes the `run_program` Action server (`SOURCE_PROGRAM_NUMBER` or `SOURCE_CODE_VISUAL`), executes compiled scripts in subprocesses, captures unbuffered stdout/stderr, and supports interactive stdin input via `program_input`.
   - `ProxyProgramNode`: Provides ROS 2 services (`proxy_run_program_start`, `proxy_run_program_stop`) and topics (`proxy_run_program_feedback`, `proxy_run_program_status`, `proxy_run_program_result`) bridging frontend WebSocket clients (e.g., `roslibjs`) to the `run_program` action.
4. **Shared Docker Volume (`programs`)**: Mounts `/root/cerebra_programs` in `flask-app` and `/ros2_ws/cerebra_programs` in `ros-programs` to guarantee zero-latency file synchronization between code generation and ROS 2 execution.

---

## Component Matrix

| Component | Protocol / Interface | Endpoint / Name | Responsibility |
|---|---|---|---|
| Flask API | HTTP REST | `POST /program` | Create program metadata & empty script |
| Flask API | HTTP REST | `GET /program`, `GET /program/{id}` | List / retrieve program metadata |
| Flask API | HTTP REST | `PUT /program/{id}` | Update program metadata |
| Flask API | HTTP REST | `PUT /program/{id}/code` | Compile & save Python script from visual code |
| Flask API | HTTP REST | `DELETE /program/{id}` | Delete program record & script file |
| pib-blockly-server | HTTP Plaintext | `POST /` (port 2442) | Visual code -> Python compilation |
| ProgramNode | ROS 2 Action Server | `run_program` | Subprocess execution & output streaming |
| ProgramNode | ROS 2 Topic Subscription | `program_input` | Send stdin data to active subprocess (`mpid`) |
| ProxyProgramNode | ROS 2 Service | `proxy_run_program_start` | Trigger `run_program` action asynchronously |
| ProxyProgramNode | ROS 2 Service | `proxy_run_program_stop` | Cancel ongoing action goal |
| ProxyProgramNode | ROS 2 Topic Publisher | `proxy_run_program_feedback` | Stream execution stdout/stderr lines |
| ProxyProgramNode | ROS 2 Topic Publisher | `proxy_run_program_status` | Goal state transitions |
| ProxyProgramNode | ROS 2 Topic Publisher | `proxy_run_program_result` | Final execution exit code |

---

## BDD Specifications

### Scenario 1: Successful Program Creation
```gherkin
Given the Flask REST API is operational
When a user sends a POST request to "/program" with JSON payload '{"name": "test_program"}'
Then the API responds with HTTP status 201 Created
And the response payload contains "name": "test_program" and a generated "programNumber"
And an empty Python file named "<programNumber>.py" is created in PYTHON_CODE_DIR
```

### Scenario 2: Program Creation Failure with Missing or Duplicate Name
```gherkin
Given a program named "unique_program" already exists in the database
When a user sends a POST request to "/program" with '{"name": "unique_program"}'
Then the API responds with HTTP status 400 Bad Request
And the response body contains the standardized error message '{"error": "Bad request."}'
When a user sends a POST request to "/program" with empty payload '{}'
Then the API responds with HTTP status 400 Bad Request
```

### Scenario 3: Program Retrieval and Listing
```gherkin
Given multiple programs exist in the database
When a user sends a GET request to "/program"
Then the API responds with HTTP status 200 OK and a JSON array under key "programs"
And each listed item contains metadata ("programNumber", "name") without full visual code
When a user sends a GET request to "/program/<programNumber>"
Then the API returns the specific program's metadata
And requesting a non-existent "<programNumber>" returns HTTP 404 Not Found
```

### Scenario 4: Program Visual Code Compilation and Storage
```gherkin
Given an existing program with ID "<programNumber>"
When a user sends a PUT request to "/program/<programNumber>/code" with '{"codeVisual": "<valid_xml_or_json>"}'
And pib-blockly-server compiles the visual block structure successfully
Then the API updates "code_visual" in the database
And writes the generated Python source code into "PYTHON_CODE_DIR/<programNumber>.py"
And responds with HTTP status 200 OK returning '{"codeVisual": "<valid_xml_or_json>"}'
```

### Scenario 5: Program Visual Code Compilation Failure Handling
```gherkin
Given an existing program with ID "<programNumber>"
When a user sends a PUT request to "/program/<programNumber>/code" with invalid visual code
And pib-blockly-server returns an HTTP error or compilation fails
Then the API catches the error and aborts transaction
And responds with HTTP status 500 Internal Server Error
And the code file on disk remains unchanged
```

### Scenario 6: Program Deletion and File Cleanup
```gherkin
Given an existing program with ID "<programNumber>" and file "PYTHON_CODE_DIR/<programNumber>.py"
When a user sends a DELETE request to "/program/<programNumber>"
Then the API removes the database entry for "<programNumber>"
And deletes file "PYTHON_CODE_DIR/<programNumber>.py" from disk
And responds with HTTP status 204 No Content
And subsequent GET requests for "<programNumber>" return HTTP 404 Not Found
```

### Scenario 7: Direct ROS 2 Program Execution via Action Goal
```gherkin
Given ROS 2 ProgramNode is active and file "PROGRAM_DIR/<programNumber>.py" exists
When a client sends a "RunProgram" action goal with source_type=SOURCE_PROGRAM_NUMBER and source="<programNumber>"
Then ProgramNode assigns a unique Process ID (mpid)
And publishes an initial feedback message containing mpid
And launches a Python subprocess to execute "PROGRAM_DIR/<programNumber>.py"
And streams stdout and stderr output lines as "ProgramOutputLine" objects in feedback
And completes with an Action Result containing the subprocess exit code (e.g. 0)
```

### Scenario 8: Direct ROS 2 Execution of Temporary Visual Code
```gherkin
Given ROS 2 ProgramNode is active and pib-blockly-server is reachable
When a client sends a "RunProgram" action goal with source_type=SOURCE_CODE_VISUAL and source="<visual_code>"
Then ProgramNode compiles the visual code via pib_blockly_client
And writes the compiled Python code to a temporary file
And executes the temporary Python script in a subprocess
And streams execution output feedback and returns exit code upon completion
```

### Scenario 9: Proxy Service Execution and Topic Forwarding
```gherkin
Given ProxyProgramNode and ProgramNode are both active
When a client calls service "proxy_run_program_start" with program_number="<programNumber>"
Then ProxyProgramNode generates a unique "proxy_goal_id"
And asynchronously triggers the "run_program" Action goal on ProgramNode
And returns "proxy_goal_id" in the service response
And forwards received action feedback to topic "proxy_run_program_feedback" with "proxy_goal_id"
And publishes goal status updates to topic "proxy_run_program_status"
And publishes final exit status to topic "proxy_run_program_result"
```

### Scenario 10: Execution Cancellation and Termination
```gherkin
Given a program is actively running under mpid and proxy_goal_id
When a client calls service "proxy_run_program_stop" with "proxy_goal_id"
Or cancels the "run_program" Action goal directly
Then ProgramNode terminates the subprocess via process.terminate()
And the Action Server transitions goal status to CANCELED / exit code 2
And ProxyProgramNode publishes updated CANCELED status on "proxy_run_program_status"
```

### Scenario 11: Interactive Stdin Stream Routing
```gherkin
Given a running program process registered in "mpid_to_process" under mpid
When a message is published to topic "program_input" with target mpid and input string
Then ProgramNode locates the active process by mpid
And writes input + newline to the process's standard input pipe (stdin)
```

---

## Non-Functional Requirements (NFR)

- **NFR-1: Storage & Docker Volume Synchronization**
  - Code generated by `flask-app` in `/root/cerebra_programs` must be instantly readable by `ros-programs` at `/ros2_ws/cerebra_programs` via shared Docker volume `programs`.
- **NFR-2: Thread Safety and Process Concurrency**
  - `ProgramNode` must synchronize thread access to `next_mpid` and `mpid_to_process` dictionary using reentrant or standard mutex locks (`Lock`).
  - Output streams (`stdout`, `stderr`) must be read asynchronously in dedicated worker threads with line buffering disabled (`bufsize=1`, unbuffered `-u` Python execution).
- **NFR-3: Error Contract Consistency**
  - All Flask REST API failures must strictly follow `{ "error": "<message>" }` JSON envelope with matching HTTP status codes (400 for validation/integrity, 404 for missing resource, 500 for backend compilation/runtime errors).
- **NFR-4: Resource Cleanup & Isolation**
  - `ProgramNode` periodic cleanup timer must purge terminated process references from `mpid_to_process`.
  - Temporary files created during `SOURCE_CODE_VISUAL` execution must be properly managed.
