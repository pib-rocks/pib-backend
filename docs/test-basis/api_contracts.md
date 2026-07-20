# API Contracts

**Repository:** `pib-backend`  
**Branch baseline:** `PR-1453` (includes PR-1461 button-color merge)  
**Sources:** `pib_api/flask/`, `pib_blockly/pib_blockly_server/src/app.ts`, `pib_api/client/`

---

## Web Framework Inventory

| Component | Protocol | Port (default) | Role |
|---|---|---|---|
| Flask REST API | HTTP/JSON | `5000` | Configuration, persistence, CRUD |
| pib-blockly-server | HTTP (plain text body) | `2442` | Visual-code → Python compilation |
| rosbridge_server | WebSocket (ROS2 bridge) | `9090` | Frontend ↔ ROS2 (not Flask) |

**Flask does not expose WebSocket endpoints.** Flask does not call ROS2 directly.

---

## Global Conventions (Flask)

| Property | Value |
|---|---|
| Base URL (Docker) | `http://flask-app:5000` |
| Base URL (host) | `http://localhost:5000` |
| Content-Type | `application/json` |
| JSON key casing | **camelCase** (`SQLAutoWithCamelCaseSchema`) |
| CORS | Enabled globally |
| DB commit policy | HTTP **2xx** → commit; else rollback (`app.after_request`) |
| Database | SQLite (`SQLALCHEMY_DATABASE_URI`) |
| Program files | `PYTHON_CODE_DIR/{programNumber}.py` |

### Standard Error Envelope

```json
{ "error": "<string>" }
```

| Status | Trigger | `error` value |
|---|---|---|
| 400 | `marshmallow.ValidationError`, `IntegrityError` | `"Bad request."` |
| 404 | `sqlalchemy.exc.NoResultFound` | `"Entity not found. Please check your path parameter."` |
| 500 | `abort(500)` / Werkzeug HTTPException | `error.description` or `"Internal Server Error, please try later again."` |
| 500 | Unhandled `Exception` | `"an unknown error occured."` |
| 501 | `NotImplementedError` | `"Not implemented."` |
| 415 | Missing/wrong `Content-Type` on JSON endpoints | Handled as unhandled → **500** (not 400) |

**Not implemented:** HTTP **503** for downstream ROS/blockly failures.

### Hard Constraints

| ID | Location | Rule |
|---|---|---|
| HC-MOTOR-GLOBAL | `pib_motors/motor.py` | `MIN_ROTATION=-9000`, `MAX_ROTATION=9000` (1/100° units) |
| HC-MOTOR-DB | `motor.rotation_range_min/max` | Seed default `-9000..9000`; `tilt_forward_motor` seed `-4500..4500` |
| HC-MOTOR-CLAMP | `Motor._validate_position()` | Positions clamped in ROS; **not validated** at Flask on pose write |
| HC-POSE-COUNT | `pose_service.update_motor_positions_of_pose` | Motor position count must match existing pose |
| HC-POSE-DELETE | `pose_service.delete_pose` | `deletable=false` poses reject delete (`ValueError` → 500) |
| HC-POSE-RENAME | `pose_service.rename_pose` | Non-deletable poses reject rename (`ValueError` → 500) |
| HC-PROGRAM-NAME | `program.name` | Unique, required, max 255 |
| HC-PROGRAM-CODE | `program.code_visual` | Max 100000 chars |
| HC-CHAT-CONTENT | `chatMessage.content` | Max 100000 chars |
| HC-PERSONALITY-DESC | `personality.description` | Max 38000 chars |
| HC-BRICKLET-UID | `bricklet.uid` | Max 30 chars, unique when set |
| HC-CAMERA-RES | `cameraSettings.resolution` | Max 3 chars |
| HC-BUTTON-MAP | `button_programs_load_schema` | `brickletNumber` required integer; `programNumber` nullable string |

---

## pib-blockly-server

### `POST /` (implicit — listens on port 2442)

| | |
|---|---|
| **Method** | POST |
| **Content-Type** | `text/plain` (Blockly JSON string in body) |
| **200 Response** | `text/plain` — compiled Python source |
| **400 Response** | `"failed to compile visual-code."` |
| **500** | Uncaught generator errors → connection error |

**Called by:** `pib_blockly_client.code_visual_to_python()` via `PIB_BLOCKLY_SERVER_URL`.

---

## Flask Endpoints

### `/program`

#### `POST /program`

| | |
|---|---|
| **Request** | `{ "name": string }` |
| **201** | `{ "name": string, "programNumber": string }` |
| **400** | Missing/duplicate name |
| **500** | Unhandled errors |

Side effect: creates empty `{programNumber}.py`.

#### `GET /program`

| | |
|---|---|
| **200** | `{ "programs": [{ "name", "programNumber" }, ...] }` |

#### `GET /program/{programNumber}`

| | |
|---|---|
| **200** | `{ "name", "programNumber" }` |
| **404** | Unknown program |

#### `PUT /program/{programNumber}`

| | |
|---|---|
| **Request** | `{ "name": string }` |
| **200** | Updated program object |

#### `DELETE /program/{programNumber}`

| | |
|---|---|
| **204** | Empty body |
| **404** | Unknown program |

#### `GET /program/{programNumber}/code`

| | |
|---|---|
| **200** | `{ "codeVisual": string }` |

#### `PUT /program/{programNumber}/code`

| | |
|---|---|
| **Request** | `{ "codeVisual": string }` |
| **200** | `{ "codeVisual": string }` |
| **500** | Blockly compilation failure |

Side effect: writes Python to `PYTHON_CODE_DIR/{programNumber}.py`.

---

### `/motor`

**MotorObject (response):**

```json
{
  "name": "string",
  "pulseWidthMin": "integer",
  "pulseWidthMax": "integer",
  "rotationRangeMin": "integer",
  "rotationRangeMax": "integer",
  "velocity": "integer",
  "acceleration": "integer",
  "deceleration": "integer",
  "period": "integer",
  "turnedOn": "boolean",
  "visible": "boolean",
  "invert": "boolean",
  "brickletPins": [
    { "pin": "integer", "invert": "boolean", "bricklet": { "uid": "string|null" } }
  ]
}
```

| Method | Path | Status | Notes |
|---|---|---|---|
| GET | `/motor` | 200 | `{ "motors": [MotorObject] }` |
| GET | `/motor/{name}` | 200/404 | Single motor |
| PUT | `/motor/{name}` | 200/400/404 | Full MotorObject; updates settings + pins |
| GET | `/motor/{name}/settings` | 200/404 | Without `brickletPins` |
| PUT | `/motor/{name}/settings` | 200/404 | Settings only |
| GET | `/motor/{name}/bricklet-pins` | 200/404 | `{ "name", "brickletPins" }` |
| PUT | `/motor/{name}/bricklet-pins` | 200/404 | Pin assignment only |

---

### `/pose`

| Method | Path | Request | Success | Errors |
|---|---|---|---|---|
| POST | `/pose` | `{ "name", "motorPositions": [{ "position": int, "motorName": string }] }` | **201** full pose | 400 duplicate/FK |
| GET | `/pose` | — | **200** `{ "poses": [{ "poseId", "name", "deletable" }] }` | |
| GET | `/pose/by-name/{name}` | URL-encoded name | **200** full pose + positions | 404 |
| GET | `/pose/{poseId}/motor-positions` | — | **200** `{ "motorPositions": [...] }` | 404 |
| DELETE | `/pose/{poseId}` | — | **204** | 404; **500** if non-deletable |
| PATCH | `/pose/{poseId}` | `{ "name" }` | **200** | **500** if non-deletable |
| PATCH | `/pose/{poseId}/motor-positions` | `{ "motorPositions": [...] }` | **200** | **500** count mismatch / unknown motor |

Seeded non-deletable poses: `Startup/Resting`, `Calibration`.

---

### `/bricklet`

| Method | Path | Request | Response |
|---|---|---|---|
| GET | `/bricklet` | — | `{ "bricklets": [{ "brickletNumber", "uid", "type" }] }` |
| GET | `/bricklet/{brickletNumber}` | — | `{ "uid" }` |
| PUT | `/bricklet/{brickletNumber}` | `{ "uid": string }` | Full bricklet; **400** duplicate UID |

**`type` enum:** `Solid State Relay Bricklet` | `Servo Bricklet` | `RGB LED Button Bricklet`

---

### `/button-programs`

| Method | Path | Request | Response |
|---|---|---|---|
| GET | `/button-programs` | — | `{ "buttonPrograms": [{ "brickletNumber", "brickletUid", "programNumber" }] }` |
| PUT | `/button-programs` | `{ "buttonProgramUpdates": [{ "brickletNumber": int, "programNumber": string|null }] }` | **200** updated list |

---

### `/camera-settings`

| Method | Path | Response fields |
|---|---|---|
| GET/POST | `/camera-settings` | `{ "resolution", "refreshRate", "qualityFactor", "resX", "resY" }` |
| PUT | `/camera-settings` | Same shape |

Seed: `resolution="SD"`, `refreshRate=0.1`, `qualityFactor=80`, `resX=640`, `resY=480`.

---

### `/voice-assistant/personality`

**PersonalityObject:** `{ "personalityId", "name", "gender", "description", "pauseThreshold", "messageHistory", "assistantModelId" }`

| Method | Path | Status |
|---|---|---|
| GET | `/voice-assistant/personality` | 200 list |
| GET | `/voice-assistant/personality/{personalityId}` | 200/404 |
| POST | `/voice-assistant/personality` | 201 |
| PUT | `/voice-assistant/personality/{personalityId}` | 200/404 |
| DELETE | `/voice-assistant/personality/{personalityId}` | 204 |

---

### `/voice-assistant/chat`

**ChatObject:** `{ "chatId", "topic", "personalityId" }`  
**ChatMessage:** `{ "messageId", "timestamp", "isUser", "content" }`

| Method | Path | Status |
|---|---|---|
| POST | `/voice-assistant/chat` | 201 |
| GET | `/voice-assistant/chat` | 200 `{ "voiceAssistantChats": [...] }` |
| GET | `/voice-assistant/chat/{chatId}` | 200/404 |
| PUT | `/voice-assistant/chat/{chatId}` | 200/404 |
| DELETE | `/voice-assistant/chat/{chatId}` | 204 |
| POST | `/voice-assistant/chat/{chatId}/messages` | 201 |
| GET | `/voice-assistant/chat/{chatId}/messages` | 200 all messages |
| GET | `/voice-assistant/chat/{chatId}/messages/{messageHistory}` | 200 limited |
| GET | `/voice-assistant/chat/{chatId}/messages/history/{messageHistory}` | 200 limited |
| GET | `/voice-assistant/chat/{chatId}/messages/{messageId}` | 200/404/500 |
| PUT | `/voice-assistant/chat/{chatId}/messages/{messageId}` | 200 |
| DELETE | `/voice-assistant/chat/{chatId}/messages/{messageId}` | 204 |

---

### `/assistant-model`

| Method | Path | Response |
|---|---|---|
| GET | `/assistant-model` | `{ "assistantModels": [{ "id", "apiName", "visualName", "hasImageSupport" }] }` |
| GET | `/assistant-model/{assistantModelId}` | Single model object |

---

### `/host-ip`

| Method | Path | Response |
|---|---|---|
| GET | `/host-ip` | **200** `{ "host_ip": string }` from `HOST_IP_FILE` (default `host_ip.txt`) |
| | | **500** `{ "error": "Unable to retrieve host IP" }` if file missing |

---

## HTTP Client Contract (`pib_api_client`)

| Env var | Default | Purpose |
|---|---|---|
| `FLASK_API_BASE_URL` | `http://localhost:5000` | Base URL for all ROS-side HTTP calls |

**Failure semantics:** `send_request()` → `(False, None)` on HTTP or network error. **No status code propagation** to ROS callers.

| Client module | Methods used |
|---|---|
| `motor_client` | GET/PUT `/motor`, `/motor/{name}/settings` |
| `pose_client` | GET `/pose/by-name/{name}`, `/pose/{id}/motor-positions` |
| `button_programs_client` | GET `/button-programs` |
| `bricklet_client` | GET `/bricklet` |
| `voice_assistant_client` | GET/POST/PUT personality, chat, messages, assistant-model |
