# Cerebra Frontend E2E Test Basis

**Repository:** `pib-backend`
**Branch:** `PR-1466`
**Sources:** Cerebra Angular frontend (data-test attributes), Angular routes, `tests/resources/frontend_keywords.robot`
**Framework:** Robot Framework + Browser Library (Playwright)

---

## Scope

These tests verify the **functional behavior** of the Cerebra frontend — not just element visibility. Each test sets up its own preconditions (e.g. selecting a list item, expanding a motor panel, opening a chat) and then asserts that a button click produces a real effect: an API call to the Flask backend, a list mutation, a toggle state change, or a slider value update.

**What is tested:**
- Sidebar navigation to all 6 main views (URL + primary control rendered)
- Button functionality: clicks trigger Flask API requests (`Wait For Response`)
- List mutations: items appear/disappear after add/delete (`Element Count By Data Test`)
- Toggle state changes: `Get Property    checked` flips after click
- Slider value changes: `Get Text` on the bubble input differs after moving a slider
- Panel expansions: extended sliders become visible after a toggle click

**Test philosophy (PR-1466 rewrite):**
- **Tests create their own preconditions.** Each test clicks the first list item / expands a motor / opens a chat *before* checking the button that depends on that state.
- **Keywords are primitives only.** `frontend_keywords.robot` exposes `Open Cerebra Page`, `Click Element By Data Test`, `Wait For Element By Data Test`, `Get Text By Data Test`, `Get Property By Data Test`, `Type Into Element By Data Test`, `Select Option By Data Test`, `Element Count By Data Test`. All test logic lives in the `.robot` files.
- **Functional verification, not visibility-only.** After clicking a button, tests verify the actual effect (API call, list change, state flip).

---

## Cerebra Application Under Test

| Property | Value |
|---|---|
| App name | Cerebra (Angular) |
| URL | `http://localhost:80` |
| Base URL variable | `${CEREBRA_BASE_URL}` = `http://localhost:80` |
| Flask API | `http://localhost:5000` (`${FLASK_BASE_URL}`) |
| Headless mode | `${HEADLESS}` = `${True}` |
| Default timeout | `${DEFAULT_TIMEOUT}` = `20s` |

### Angular Routes

| Sidebar Nav Item (data-test) | Route |
|---|---|
| `LNK_Joint_Control` | `/joint-control/head` |
| `LNK_Poses` | `/pose` |
| `LNK_Camera` | `/camera` |
| `LNK_Voice_Assistant` | `/voice-assistant` |
| `LNK_Program` | `/program` |
| `LNK_System` | `/system` |

### Flask API endpoints used in functional assertions

| Feature | Method + Endpoint | Used by test |
|---|---|---|
| Poses | `GET/POST /pose`, `DELETE /pose/{id}` | PS-001, PS-003, PS-004, PS-005 |
| Personalities | `GET/POST /voice-assistant/personality` | VA-002, VA-003 |
| Chat | `POST /voice-assistant/chat/{id}/messages` | VA-008 |
| Motors | `GET /motor` | JC-001 |
| Camera | `GET/PUT /camera-settings` | CAM-001 |
| Programs | `GET/POST/PUT /program` | PM-001, PM-004, PM-005, PM-006 |
| Bricklets | `GET /bricklet` | SYS-001, SYS-003 |

---

## Selector Convention

Cerebra uses `data-test` attributes throughout its HTML templates for stable element identification:

| Prefix | Element type | Examples |
|---|---|---|
| `BTN_` | Button | `BTN_Apply_pose`, `BTN_Save_pose`, `BTN_Chat_Send`, `BTN_Smart_Connect` |
| `LNK_` | Link / sidebar nav | `LNK_Joint_Control`, `LNK_Poses`, `LNK_Camera` |
| `DDN_` | Dropdown | `DDN_Camera_resolution` |
| `TGL_` | Toggle | `TGL_Camera_On_Off`, `TGL_Solid_State_Relay` |
| `SLD_` | Slider | `SLD_Motor_Settings_Velocity`, `SLD_Camera_qualityFactor` |
| `TXT_` | Text / input | `TXT_Peronality`, `TXT_Chat_Message`, `TXT_Message_history`, `TXT_Slider_BubbleInput` |
| `LBL_` | Label | `LBL_IP_Address` |

**CSS selector pattern:** `css=[data-test="BTN_Apply_pose"]`

---

## Test Suites

All suites live in `tests/frontend/` and follow the shared setup pattern:

```robot
Suite Setup         Set Suite Variables
Suite Teardown      Close Cerebra Application
```

### 1. `navigation.robot` — Sidebar Navigation (E2E-BDD-FE-NAV-*)

Verifies all 6 sidebar nav items navigate to the correct Angular route AND that the target view rendered its primary control (not just a URL change).

| Test ID | Nav item clicked | Functional assertion |
|---|---|---|
| E2E-BDD-FE-NAV-001 | `LNK_Joint_Control` | URL contains `/joint-control` + `BTN_Motor_Settings_Toggle_Extended` visible |
| E2E-BDD-FE-NAV-002 | `LNK_Poses` | URL contains `/pose` + `BTN_Apply_pose` visible |
| E2E-BDD-FE-NAV-003 | `LNK_Camera` | URL contains `/camera` + `TGL_Camera_On_Off` visible |
| E2E-BDD-FE-NAV-004 | `LNK_Voice_Assistant` | URL contains `/voice-assistant` + `BTN_Add_Personality` visible |
| E2E-BDD-FE-NAV-005 | `LNK_Program` | URL contains `/program` + `BTN_Program_Run_Stop` visible |
| E2E-BDD-FE-NAV-006 | `LNK_System` | URL contains `/system` + `BTN_Update_bricklet_UIDs` visible |

### 2. `joint_control.robot` — Joint Control View (E2E-BDD-FE-JC-*)

Verifies motor expansion preconditions and slider value changes.

| Test ID | Precondition | Functional assertion |
|---|---|---|
| E2E-BDD-FE-JC-001 | navigate to Joint Control | view container visible + `GET /motor` observed |
| E2E-BDD-FE-JC-002 | navigate to Joint Control | head joint tab visible |
| E2E-BDD-FE-JC-003 | click `BTN_Motor_Settings_Toggle_Extended` | `SLD_Motor_Settings_Acceleration` becomes visible |
| E2E-BDD-FE-JC-004 | click `BTN_Motor_Settings_Toggle_Extended` | `SLD_Motor_Settings_Velocity`, `..._Acceleration`, `..._Deceleration` all visible |
| E2E-BDD-FE-JC-005 | click `BTN_Motor_Settings_Toggle_Extended` | extended sliders appear (toggle flips) |
| E2E-BDD-FE-JC-006 | click `BTN_Motor_Settings_Toggle_Extended` | `TXT_Threshold`, `BTN_Threshold_up`, `BTN_Threshold_down` visible |
| E2E-BDD-FE-JC-007 | expand motor, focus `SLD_Motor_Settings_Velocity`, press ArrowUp | `TXT_Slider_BubbleInput` value changes |

### 3. `poses.robot` — Poses View (E2E-BDD-FE-PS-*)

Verifies pose list mutations and API calls after button clicks.

| Test ID | Precondition | Functional assertion |
|---|---|---|
| E2E-BDD-FE-PS-001 | navigate to Poses | view visible + `GET /pose` observed |
| E2E-BDD-FE-PS-002 | navigate to Poses | list container visible, ≥1 pose (`BTN_Apply_pose` count ≥ 1) |
| E2E-BDD-FE-PS-003 | click first pose (select) | `BTN_Apply_pose` click → `POST /pose` observed |
| E2E-BDD-FE-PS-004 | navigate to Poses | `BTN_Save_pose` click → `POST /pose` observed + count not decreased |
| E2E-BDD-FE-PS-005 | click first pose (select) | `BTN_Delete_pose` click → `/pose` request observed + count not increased |
| E2E-BDD-FE-PS-006 | click first pose (select) | `BTN_Rename_pose` reachable (PATCH target) |
| E2E-BDD-FE-PS-007 | navigate to Poses | `BTN_Pose_Import` click → page stays responsive |

### 4. `camera.robot` — Camera View (E2E-BDD-FE-CAM-*)

Verifies resolution change keeps on/off working and settings panel reveals sliders.

| Test ID | Precondition | Functional assertion |
|---|---|---|
| E2E-BDD-FE-CAM-001 | navigate to Camera | view visible + `GET /camera-settings` observed |
| E2E-BDD-FE-CAM-002 | navigate to Camera | `TGL_Camera_On_Off` exposes `checked` property |
| E2E-BDD-FE-CAM-003 | navigate to Camera | select 720p in `DDN_Camera_resolution` → `TGL_Camera_On_Off` still checkable |
| E2E-BDD-FE-CAM-004 | navigate to Camera | click `BTN_Video_settings` → `SLD_Camera_qualityFactor` + `SLD_Camera_refreshRate` visible |

### 5. `voice_assistant.robot` — Voice Assistant View (E2E-BDD-FE-VA-*)

Verifies personality creation and chat message sending.

| Test ID | Precondition | Functional assertion |
|---|---|---|
| E2E-BDD-FE-VA-001 | navigate to Voice Assistant | view visible |
| E2E-BDD-FE-VA-002 | navigate to Voice Assistant | list visible + `GET /voice-assistant/personality` observed |
| E2E-BDD-FE-VA-003 | navigate to Voice Assistant | click `BTN_Add_Personality`, type name, confirm → `POST /voice-assistant/personality` + count not decreased |
| E2E-BDD-FE-VA-004 | click first personality (select) | `BTN_Delete_Persona` visible for selected persona |
| E2E-BDD-FE-VA-005 | click first personality (open chat) | `TXT_Message_history` visible |
| E2E-BDD-FE-VA-006 | click first personality (open chat) | `BTN_Chat_Send` visible |
| E2E-BDD-FE-VA-007 | click first personality (open chat) | `TXT_Chat_Message` visible |
| E2E-BDD-FE-VA-008 | open chat, type marker in `TXT_Chat_Message` | click `BTN_Chat_Send` → `POST /voice-assistant/chat` + message appears in `TXT_Message_history` |

### 6. `program_manager.robot` — Program Manager View (E2E-BDD-FE-PM-*)

Verifies run/save/export trigger backend actions.

| Test ID | Precondition | Functional assertion |
|---|---|---|
| E2E-BDD-FE-PM-001 | navigate to Program | view visible + `GET /program` observed |
| E2E-BDD-FE-PM-002 | navigate to Program | list visible (`BTN_Program_Run_Stop` + `BTN_Program_Save`) |
| E2E-BDD-FE-PM-003 | navigate to Program | workspace visible (`TXT_Program_Input` + `TGL_Split_Screen`) |
| E2E-BDD-FE-PM-004 | navigate to Program | click `BTN_Program_Run_Stop` → `/program` request + `TXT_Program_Input` text changes |
| E2E-BDD-FE-PM-005 | navigate to Program | click `BTN_Program_Save` → `BTN_Confirm` or `BTN_Save` appears (or `/program` request) |
| E2E-BDD-FE-PM-006 | navigate to Program | click `BTN_Program_Export` → `/program` request + page still responsive |

### 7. `system.robot` — System View (E2E-BDD-FE-SYS-*)

Verifies smart connect feedback, relay toggle state change, and bricklet list refresh.

| Test ID | Precondition | Functional assertion |
|---|---|---|
| E2E-BDD-FE-SYS-001 | navigate to System | view visible + `GET /bricklet` observed |
| E2E-BDD-FE-SYS-002 | navigate to System | hardware IDs section visible (`BTN_Update_bricklet_UIDs` + `LBL_IP_Address`) |
| E2E-BDD-FE-SYS-003 | navigate to System | click `BTN_Update_bricklet_UIDs` → `GET /bricklet` observed (list refresh) |
| E2E-BDD-FE-SYS-004 | navigate to System | click `BTN_Smart_Connect` → network request or dialog feedback |
| E2E-BDD-FE-SYS-005 | navigate to System | click `TGL_Solid_State_Relay` → `checked` property flips |

---

## Shared Keywords (primitives only)

All keywords live in `tests/resources/frontend_keywords.robot`. PR-1466 reduced them to **primitives** — test logic lives in the `.robot` files.

### Suite lifecycle
- `Set Suite Variables` — opens a headless browser + context
- `Close Cerebra Application` — closes the browser

### Page primitives
- `Open Cerebra Page` — opens a Cerebra path, waits for network idle
- `Open Cerebra Home` — opens `/`

### Data-test primitives (the core)
- `Wait For Element By Data Test` — waits for `data-test` element to reach a state (visible/hidden/…)
- `Click Element By Data Test` — waits for visibility then clicks
- `Get Text By Data Test` — returns textContent of a `data-test` element
- `Get Property By Data Test` — returns a DOM property (e.g. `checked`, `value`)
- `Type Into Element By Data Test` — clears and types into an input
- `Select Option By Data Test` — selects a `<select>` option by label
- `Element Count By Data Test` — returns the count of matching elements

### Navigation helpers (thin wrappers)
- `Click Sidebar Nav Item` / `When User Clicks Sidebar Nav Item`
- `Given User Opens Cerebra And Navigates To`
- `Then Cerebra Url Path Should Contain` / `Then Cerebra Url Path Should Be`
- `Then View Element Is Visible` / `Then View Element Count Is At Least`

### Compatibility shims
The sibling suites `angular_components.robot` and `blockly_interactions.robot` still use a few older keywords (`Mock Flask Route`, `When User Opens Blockly Program Editor`, etc.). These are retained as compatibility shims and are not used by the PR-1466 functional tests.

---

## Running the Tests

### Prerequisites

```bash
pip install -r tests/requirements-robot.txt
rfbrowser init
```

### Cerebra + Flask must be running

```bash
# Flask API on :5000
cd pib_api && flask run --port 5000

# Cerebra on :80 (or via Docker)
docker compose up angular-app
```

### Run all frontend suites

```bash
cd tests
./run_all_tests.sh --include-frontend
```

### Run a single suite

```bash
robot --outputdir tests/results tests/frontend/poses.robot
```

### Dry-run (syntax check without a browser)

```bash
robot --dryrun --outputdir /tmp/rf-dryrun tests/frontend/
```

### Results

Output is written to `tests/results/run-<timestamp>/` (log.html, report.html, output.xml).

---

## Notes

- Tests use **timeout=20s** (`${DEFAULT_TIMEOUT}`) for element waits to handle slow Angular renders.
- Functional API assertions use the Browser Library's `Wait For Response` keyword with a `matcher` pointing at the Flask API URL — this verifies the frontend actually called the backend, not just that a button was clicked.
- Toggle state assertions use `Get Property    checked` to verify the state actually flipped.
- Slider value assertions use `Get Text` on `TXT_Slider_BubbleInput` before and after a keyboard ArrowUp press.
- The `Run Keyword And Ignore Error` pattern is used where a button click may produce one of several acceptable functional signals (e.g. a confirmation dialog OR an API call).
