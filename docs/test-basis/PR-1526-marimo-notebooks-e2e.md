# Test Basis: Robust Marimo Notebooks E2E UI Test (PR-1526)

**Repository:** `pib-backend`  
**Requirement:** Jira [PR-1526](https://pib-rocks.atlassian.net/browse/PR-1526)  
**Branch:** `PR-1526` (do not merge to develop)  
**Components:** `tests/e2e/test_programs_component_e2e.py`, Cerebra `/program/marimo` (`app-marimo` iframe → `/marimo-server/`)

---

## Overview & Scope

PR-1526 requires a fail-safe Playwright E2E UI suite for the Marimo Notebooks editor that proves—via real browser and iframe DOM nodes—that the component loads correctly, supports full CRUD, and executes Python cells with rendered output verification.

Previous E2E coverage could pass while the iframe was empty, proxied incorrectly, or returned gateway errors. Assertions must therefore target concrete Marimo DOM elements (`.marimo-app`, `marimo-code-editor`, cell output nodes) so empty or error views fail immediately.

**Target environment:** live Pi 5 at `http://192.168.1.28`.

---

## Acceptance Criteria Traceability

| AC | Acceptance criterion | Coverage | Status |
|---|---|---|---|
| AC1 | Deep iframe DOM: `iframe[src*='marimo-server']` contains visible `.marimo-app` or `marimo-code-editor` | `test_01b_marimo_iframe_inner_content_loads`, `test_10_marimo_notebooks_strict_crud_and_execution` | Implemented |
| AC2 | Create & Read: `"New notebook"` creates a `.py` file visible in the sidebar and openable in the editor | `test_06_create_rename_delete_workbook_via_ui`, `test_10_…` | Implemented |
| AC3 | Update & Run: write Python into a cell, execute, assert rendered DOM output (`.marimo-output-cell` / cell-output / text) | `test_09_…`, `test_10_…` | Implemented |
| AC4 | Delete: remove notebook via UI; gone from list (and backend) | `test_06_…`, `test_10_…` | Implemented |
| AC5 | Error-state detection: empty iframe / 404 / 502 / missing editor nodes fail the test | Strict selectors in `test_01b` / `test_10` (no `body`-only fallback) | Implemented |

---

## Test Cases (from Jira PR-1526)

### Testfall 1 — Deep Iframe DOM Rendering

**Objective:** Verify the Marimo iframe is not empty and contains visible editor DOM nodes (`.marimo-app` / `marimo-code-editor` / equivalent editor root).

**Preconditions:** Cerebra UI and `pib-marimo` (proxied as `/marimo-server/`) are reachable on the Pi.

**Steps:**
1. Open `/program/marimo`.
2. Locate `iframe[src*="marimo-server"]` (or `app-marimo iframe` with that `src`).
3. Switch into the iframe context and wait for `.marimo-app` or `marimo-code-editor` (or `.cm-editor` as the live editor surface) to become visible.

**Expected result:**
- Iframe `src` contains `marimo-server`.
- At least one Marimo editor root / code editor node is visible inside the iframe.
- A bare `body` or nginx error page (`502 Bad Gateway`, empty shell) must **not** satisfy the assertion.

**Automated coverage:** `tests/e2e/test_programs_component_e2e.py` (`test_01b_…`, `test_10_…`).

---

### Testfall 2 — Notebook Creation & File Persistence

**Objective:** Create a new notebook via **"New notebook"** and verify the `.py` file exists in the UI list and on the backend.

**Preconditions:** Marimo tab and right sidebar (`app-sidebar-right`) are loaded.

**Steps:**
1. Click `[data-test="BTN_New notebook"]`.
2. Enter a unique name (without or with `.py`) in `#input-name` and save via `#modal-save-button`.
3. Assert a sidebar link `a[href$="/program/marimo/<name>.py"]` appears.
4. Optionally confirm via `GET /api/v1/marimo/notebooks` that the notebook is listed.

**Expected result:**
- New `.py` notebook appears in the sidebar.
- Backend notebook list includes the same filename.
- Opening the link navigates to `/program/marimo/<file>` and loads that file in the iframe (`?file=`).

**Automated coverage:** `test_06_…`, `test_10_…`.

---

### Testfall 3 — Interactive Code Execution & DOM Output Assert

**Objective:** Write Python into a cell, run it, and validate the exact output string in a rendered DOM output node.

**Preconditions:** A notebook is open in the Marimo iframe with a visible CodeMirror editor (`.cm-editor`).

**Steps:**
1. Focus the first `.cm-editor` / `.cm-content` inside the iframe.
2. Replace cell contents with deterministic code, e.g. `print(21 * 2)` or `print(f'PIB_EXEC_{…}')`.
3. Execute via visible `[data-testid="run-button"]` or `Control+Enter` / `Shift+Enter`.
4. Assert the output text appears in a DOM output node: `.marimo-output-cell`, `[data-testid='cell-output']`, `.cell-output`, or equivalent Marimo output container (not merely anywhere in `body`).

**Expected result:**
- Expected output string (e.g. `42` or `PIB_EXEC_…`) is visible inside an output cell node.
- Failure to render output (or only matching text in the editor source) fails the test.

**Automated coverage:** `test_09_…`, `test_10_…`.

---

### Testfall 4 — Notebook Deletion

**Objective:** Delete the created test notebook via the UI and ensure it is removed from the list and filesystem/backend.

**Preconditions:** A test notebook exists in the sidebar (from Testfall 2 / 3).

**Steps:**
1. Open the row dropdown for the notebook (`button[id^="dropdownbutton-"]`).
2. Click the delete action (`button[id="sidebar-right-delete-<Title>"]`); accept `confirm()`.
3. Assert the sidebar link is gone.
4. Confirm `GET /api/v1/marimo/notebooks` no longer lists the file (or `DELETE` cleanup is idempotent).

**Expected result:**
- Notebook disappears from the UI list.
- Backend no longer returns the notebook metadata.

**Automated coverage:** `test_06_…`, `test_10_…`.

---

### Testfall 5 — Error-State Detection

**Objective:** Ensure connection errors, proxy 404/500/502 responses, or empty default views are treated as test failures (not silent passes).

**Preconditions:** Same suite as Testfall 1; assertions must be strict.

**Steps:**
1. Use deep iframe selectors that exclude a lone `body` match.
2. Assert iframe `src` includes `marimo-server`.
3. Assert Marimo editor / app nodes are visible within a bounded timeout.
4. For execution tests, require output inside output-cell selectors.

**Expected result:**
- If `/marimo-server/` returns `502 Bad Gateway`, empty HTML, or no editor nodes, Playwright expectations time out / assert and the test fails.
- No soft fallback that treats an error page as a successful Marimo load.

**Automated coverage:** Strict selectors and helpers in `test_01b_…` and `test_10_…` (regression of soft `…, body` fallbacks).

---

## Mapping to Playwright Suite

| Jira test case | Primary test method(s) |
|---|---|
| Testfall 1 | `test_01b_marimo_iframe_inner_content_loads`, `test_10_marimo_notebooks_strict_crud_and_execution` |
| Testfall 2 | `test_06_create_rename_delete_workbook_via_ui`, `test_10_…` |
| Testfall 3 | `test_09_create_notebook_write_time_program_and_run_in_marimo`, `test_10_…` |
| Testfall 4 | `test_06_…`, `test_10_…` |
| Testfall 5 | Strict iframe/output assertions shared by `test_01b` / `test_10` |

Run against the live Pi:

```bash
pytest tests/e2e/test_programs_component_e2e.py -v
```
