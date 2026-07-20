# Cerebra Frontend E2E Test Basis

**Repository:** `pib-backend`
**Branch:** `PR-1466`
**Sources:** Cerebra Angular frontend (data-test attributes), Angular routes, `tests/resources/frontend_keywords.robot`
**Framework:** Robot Framework + Browser Library (Playwright)

---

## Scope

These tests verify the **UI structure and presence** of Cerebra frontend elements — not their functional behavior. Each test asserts that an element identified by a `data-test` attribute is **visible** after navigating to the corresponding Angular route.

**What is tested:**
- Sidebar navigation to all 6 main views
- Presence of view containers, lists, buttons, dropdowns, sliders, and input fields
- Element visibility with resilient timeouts (20s) for slow Angular renders

**What is NOT tested (out of scope):**
- Functional behavior (clicking a button performs an action)
- API integration (mocked or real)
- Data correctness
- Visual/layout regression

---

## Cerebra Application Under Test

| Property | Value |
|---|---|
| App name | Cerebra (Angular) |
| Dev URL | `http://localhost:4200` |
| Docker container | `angular-app` (port 4200) |
| Base URL variable | `${CEREBRA_BASE_URL}` = `http://localhost:4200` |
| Headless mode | `${HEADLESS}` = `${True}` |

### Angular Routes

| Sidebar Nav Item (data-test) | Route |
|---|---|
| `LNK_Joint_Control` | `/joint-control/head` |
| `LNK_Poses` | `/pose` |
| `LNK_Camera` | `/camera` |
| `LNK_Voice_Assistant` | `/voice-assistant` |
| `LNK_Program` | `/program` |
| `LNK_System` | `/system` |

---

## Selector Convention

Cerebra uses `data-test` attributes throughout its HTML templates for stable element identification:

| Prefix | Element type | Examples |
|---|---|---|
| `BTN_` | Button | `BTN_Apply_pose`, `BTN_Save_pose`, `BTN_Chat_Send`, `BTN_Smart_Connect` |
| `LNK_` | Link / sidebar nav | `LNK_Joint_Control`, `LNK_Poses`, `LNK_Camera` |
| `DDN_` | Dropdown | `DDN_Camera_resolution` |
| `CHK_` | Checkbox / toggle | `CHK_Head_joint`, `CHK_Relay_control` |
| `SLD_` | Slider | `SLD_Motor_slider`, `SLD_Motor_threshold_min` |
| `IMG_` | Image | `IMG_Camera_stream` |
| `INP_` | Input field | `INP_Chat_Message` |

**CSS selector pattern:** `css=[data-test="BTN_Apply_pose"]`

Some view containers use fallback selectors (class-based or testid) where `data-test` is not yet available, following the existing pattern in `angular_components.robot`.

---

## Test Suites

All suites live in `tests/frontend/` and follow the shared setup pattern:

```robot
Suite Setup         Set Suite Variables
Suite Teardown      Close Cerebra Application
```

### 1. `navigation.robot` — Sidebar Navigation (E2E-BDD-FE-NAV-*)

Verifies all 6 sidebar nav items navigate to the correct Angular route.

| Test ID | Nav item clicked | Expected URL path |
|---|---|---|
| E2E-BDD-FE-NAV-001 | `LNK_Joint_Control` | `/joint-control/head` |
| E2E-BDD-FE-NAV-002 | `LNK_Poses` | `/pose` |
| E2E-BDD-FE-NAV-003 | `LNK_Camera` | `/camera` |
| E2E-BDD-FE-NAV-004 | `LNK_Voice_Assistant` | `/voice-assistant` |
| E2E-BDD-FE-NAV-005 | `LNK_Program` | `/program` |
| E2E-BDD-FE-NAV-006 | `LNK_System` | `/system` |

### 2. `joint_control.robot` — Joint Control View (E2E-BDD-FE-JC-*)

Verifies joint tabs, motor sliders, motor settings panel, and threshold controls.

| Test ID | What is verified |
|---|---|
| E2E-BDD-FE-JC-001 | Joint Control view container renders after navigation |
| E2E-BDD-FE-JC-002 | Head joint tab (`CHK_Head_joint`) is visible |
| E2E-BDD-FE-JC-003 | Motor slider control (`SLD_Motor_slider`) is visible |
| E2E-BDD-FE-JC-004 | Motor settings panel is visible |
| E2E-BDD-FE-JC-005 | Motor settings toggle button (`BTN_Motor_settings_toggle`) expands settings |
| E2E-BDD-FE-JC-006 | Threshold controls (`SLD_Motor_threshold_min`, `SLD_Motor_threshold_max`) are visible |

### 3. `poses.robot` — Poses View (E2E-BDD-FE-PS-*)

Verifies the pose list and action buttons are present.

| Test ID | What is verified |
|---|---|
| E2E-BDD-FE-PS-001 | Poses view container renders after navigation |
| E2E-BDD-FE-PS-002 | Pose list container is visible |
| E2E-BDD-FE-PS-003 | Apply Pose button (`BTN_Apply_pose`) is visible |
| E2E-BDD-FE-PS-004 | Save Pose button (`BTN_Save_pose`) is visible |
| E2E-BDD-FE-PS-005 | Delete Pose button (`BTN_Delete_pose`) is visible |
| E2E-BDD-FE-PS-006 | Rename Pose button (`BTN_Rename_pose`) is visible |
| E2E-BDD-FE-PS-007 | Pose Import button (`BTN_Pose_Import`) is visible |

### 4. `camera.robot` — Camera View (E2E-BDD-FE-CAM-*)

Verifies the video stream, resolution dropdown, and settings button.

| Test ID | What is verified |
|---|---|
| E2E-BDD-FE-CAM-001 | Camera view container renders after navigation |
| E2E-BDD-FE-CAM-002 | Camera video stream element is visible |
| E2E-BDD-FE-CAM-003 | Camera resolution dropdown (`DDN_Camera_resolution`) is visible |
| E2E-BDD-FE-CAM-004 | Camera settings button (`BTN_Video_settings`) is visible |

### 5. `voice_assistant.robot` — Voice Assistant View (E2E-BDD-FE-VA-*)

Verifies personality list, add/delete buttons, and chat window elements.

| Test ID | What is verified |
|---|---|
| E2E-BDD-FE-VA-001 | Voice Assistant view container renders after navigation |
| E2E-BDD-FE-VA-002 | Personality list is visible |
| E2E-BDD-FE-VA-003 | Add Personality button (`BTN_Add_Personality`) is visible |
| E2E-BDD-FE-VA-004 | Delete Persona button (`BTN_Delete_Persona`) is visible |
| E2E-BDD-FE-VA-005 | Chat window is visible |
| E2E-BDD-FE-VA-006 | Chat send button (`BTN_Chat_Send`) is visible |
| E2E-BDD-FE-VA-007 | Chat input field (`INP_Chat_Message`) is visible |

### 6. `program_manager.robot` — Program Manager View (E2E-BDD-FE-PM-*)

Verifies program list, workspace, and run/stop/save/export buttons.

| Test ID | What is verified |
|---|---|
| E2E-BDD-FE-PM-001 | Program Manager view container renders after navigation |
| E2E-BDD-FE-PM-002 | Program list is visible |
| E2E-BDD-FE-PM-003 | Program workspace (Blockly canvas) is visible |
| E2E-BDD-FE-PM-004 | Run/Stop button (`BTN_Program_Run_Stop`) is visible |
| E2E-BDD-FE-PM-005 | Save button (`BTN_Program_Save`) is visible |
| E2E-BDD-FE-PM-006 | Export button (`BTN_Program_Export`) is visible |

### 7. `system.robot` — System View (E2E-BDD-FE-SYS-*)

Verifies hardware IDs, bricklet update, smart connect, and relay controls.

| Test ID | What is verified |
|---|---|
| E2E-BDD-FE-SYS-001 | System view container renders after navigation |
| E2E-BDD-FE-SYS-002 | Hardware IDs section is visible |
| E2E-BDD-FE-SYS-003 | Update Bricklet UIDs button (`BTN_Update_bricklet_UIDs`) is visible |
| E2E-BDD-FE-SYS-004 | Smart Connect button (`BTN_Smart_Connect`) is visible |
| E2E-BDD-FE-SYS-005 | Relay control toggle (`CHK_Relay_control`) is visible |

---

## Shared Keywords

All keywords live in `tests/resources/frontend_keywords.robot`. Key keywords added by PR-1466:

### Navigation
- `Open Cerebra Home` — navigates to `/` and waits for network idle
- `Click Sidebar Nav Item` — clicks a sidebar link by data-test, waits for network idle
- `Given User Opens Cerebra And Navigates To` — Given-style step: open home, click nav, verify URL
- `When User Clicks Sidebar Nav Item` — When-style wrapper for `Click Sidebar Nav Item`
- `Then Cerebra Url Path Should Be` — asserts URL ends with expected route path
- `Then View Element Is Visible` — asserts element by data-test is visible (timeout=20s)
- `Then View Element Count Is At Least` — asserts at least N matching elements exist

### Joint Control
- `Then Joint Control View Is Visible`
- `Then Joint Tab Is Visible`
- `When User Selects Joint Tab`
- `Then Motor Slider Is Visible`
- `Then Motor Settings Panel Is Visible`
- `When User Toggles Motor Settings`
- `Then Threshold Control Is Visible`

### Poses
- `Then Poses View Is Visible`
- `Then Pose List Is Visible`
- `Then Pose Button Is Visible`

### Camera
- `Then Camera View Is Visible`
- `Then Camera Video Stream Is Visible`
- `Then Camera Resolution Dropdown Is Visible`
- `Then Camera Settings Button Is Visible`

### Voice Assistant
- `Then Voice Assistant View Is Visible`
- `Then Personality List Is Visible`
- `Then Personality Button Is Visible`
- `Then Chat Window Is Visible`
- `Then Chat Send Button Is Visible`
- `Then Chat Input Field Is Visible`

### Program Manager
- `Then Program Manager View Is Visible`
- `Then Program List Is Visible`
- `Then Program Workspace Is Visible`
- `Then Program Button Is Visible`

### System
- `Then System View Is Visible`
- `Then Hardware Id Section Is Visible`
- `Then Bricklet Update Button Is Visible`
- `Then Smart Connect Button Is Visible`
- `Then Relay Control Is Visible`

---

## Running the Tests

### Prerequisites

```bash
pip install -r tests/requirements-robot.txt
rfbrowser init
```

### Cerebra must be running

```bash
# Dev mode
cd cerebra && ng serve  # serves on http://localhost:4200

# OR via Docker
docker compose up angular-app
```

### Run all frontend suites

```bash
cd tests
./run_all_tests.sh --include-frontend
```

### Run a single suite

```bash
robot --outputdir tests/results tests/frontend/navigation.robot
```

### Results

Output is written to `tests/results/run-<timestamp>/` (log.html, report.html, output.xml).

---

## Notes

- Tests use **timeout=20s** for element waits to handle slow Angular renders and initial load.
- View container selectors include fallbacks (e.g. `[data-test="view-poses"], [data-test="poses-view"]`) where the exact data-test attribute name may vary — adjust as the Cerebra codebase stabilizes its data-test conventions.
- Tests assert **visibility only** — they do not interact with elements beyond navigation and tab/settings toggles. Functional E2E tests are a future concern (separate ticket).
