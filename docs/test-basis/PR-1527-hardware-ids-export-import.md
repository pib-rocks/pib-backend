# Test Basis: Export and Import of Hardware IDs and Bricklet Mappings (PR-1527)

**Jira Story:** [PR-1527](https://pib-rocks.atlassian.net/browse/PR-1527)  
**Repositories:** `pib-backend` (Flask REST API) & `cerebra` (Angular UI)  
**Branch:** `PR-1527`

---

## 1. Overview & Scope

PR-1527 adds backup/restore for Tinkerforge Bricklet UIDs, motor pin mappings, and motor limits so hardware configurations can be cloned or restored without manual UID entry.

**Backend (`pib-backend`):**
- `GET /api/system/hardware-config/export` — JSON snapshot of active Bricklet UIDs, pin mappings, and motor limits.
- `POST /api/system/hardware-config/import` — validate schema and apply configuration into `pibdata.db`.

**Frontend (`cerebra`):** Export / Import Hardware-IDs controls under System → Hardware Settings (covered separately in Cerebra).

---

## 2. Requirements & Acceptance Criteria

### 2.1 Backend API Contracts (`pib-backend`)

#### `GET /api/system/hardware-config/export`
* **Response Status:** `200 OK`
* **Content-Type:** `application/json`
* **Content-Disposition:** `attachment; filename=hardware-config.json`
* **Response Schema (excerpt):**
  ```json
  {
    "version": 1,
    "bricklets": [
      { "brickletNumber": 1, "uid": "ABC123", "type": "Servo Bricklet" }
    ],
    "motors": [
      {
        "name": "elbow_left",
        "pulseWidthMin": 700,
        "pulseWidthMax": 2500,
        "rotationRangeMin": -9000,
        "rotationRangeMax": 9000,
        "velocity": 16000,
        "acceleration": 10000,
        "deceleration": 5000,
        "period": 19500,
        "turnedOn": true,
        "visible": true,
        "invert": false,
        "brickletPins": [
          { "brickletNumber": 3, "pin": 8, "invert": false }
        ]
      }
    ]
  }
  ```
* **Behavior:** Includes all configured Bricklets (including empty UIDs) and all motors with limits and pin mappings.

#### `POST /api/system/hardware-config/import`
* **Request Body:** Hardware-config JSON document (same schema as export).
* **Response Status:** `200 OK` with the applied configuration (export shape).
* **Validation / Safety:**
  - Reject non-object / missing `bricklets` or `motors` arrays (`400`).
  - Reject unsupported `version` (`400`).
  - Reject invalid UID format (non-alphanumeric or longer than 6 characters) (`400`).
  - Reject duplicate UID assignments (`400`).
  - Reject unknown Bricklet types and type mismatches against existing DB rows (`400`).
  - Reject unknown motor names (`400`).
  - Accept empty UID strings (clears UID).
  - Remain backward compatible with existing Bricklet types (Servo, Solid State Relay, RGB LED Button).

Also registered under `/system/...`, `/v1/system/...`, and `/api/v1/system/...` for proxy parity.

---

## 3. Acceptance Criteria Traceability (from Jira PR-1527)

| AC | Acceptance criterion (Jira test case) | Coverage | Status |
|---|---|---|---|
| AC1 / TC1 | Hardware ID Export: `GET /api/system/hardware-config/export` returns valid JSON with all active Bricklet UIDs | `tests/unit/test_hardware_config_service.py`, `tests/integration/test_hardware_config_api.py` | Implemented |
| AC2 / TC2 | Hardware ID Import & Validation: `POST .../import` accepts valid JSON, updates `pibdata.db`, rejects invalid schemas | Unit + integration import tests | Implemented |
| AC3 / TC3 | UI Export/Import: browser download and upload workflows | Cerebra (`hardware-id` component / E2E) | Frontend |
| AC4 / TC4 | Backward Compatibility: import files remain compatible with existing Bricklet types | `test_import_rejects_unknown_bricklet_type`, `test_roundtrip_preserves_seeded_types` | Implemented |

---

## 4. Test Cases (from Jira PR-1527)

### Test Case 1 — Hardware ID Export

**Objective:** `GET /api/system/hardware-config/export` returns a valid JSON document containing all active Bricklet UIDs (plus pin mappings and motor limits).

**Preconditions:** Seeded `pibdata.db` with Bricklets and motors.

**Steps:**
1. Set one or more Bricklet UIDs via existing bricklet APIs or service helpers.
2. Call `GET /api/system/hardware-config/export`.
3. Parse the JSON body.

**Expected result:**
- HTTP `200`.
- Document has `version`, `bricklets[]`, `motors[]`.
- Every DB Bricklet appears with `brickletNumber`, `uid`, `type`.
- Motors include limit fields and `brickletPins`.

**Automated coverage:** `test_export_includes_bricklets_and_motors`, `test_export_endpoint_returns_attachment`.

---

### Test Case 2 — Hardware ID Import & Validation

**Objective:** `POST /api/system/hardware-config/import` accepts valid JSON backups, updates database records, and rejects invalid schemas.

**Preconditions:** Seeded database.

**Steps:**
1. POST a valid export-shaped document that changes UIDs and a motor limit.
2. POST an invalid document (bad UID, duplicate UIDs, missing arrays).
3. Re-export and verify persisted changes.

**Expected result:**
- Valid import → HTTP `200`, DB updated.
- Invalid import → HTTP `400` with `{ "error": "..." }`, DB unchanged for that request.

**Automated coverage:** `test_import_updates_uids_and_motor_limits`, `test_import_rejects_invalid_uid`, `test_import_rejects_duplicate_uids`, `test_import_endpoint_rejects_invalid_json_body`.

---

### Test Case 3 — UI Export/Import

**Objective:** Verify browser UI download and upload workflows (Cerebra).

**Preconditions:** Cerebra System → Hardware IDs page; backend endpoints available.

**Steps:**
1. Click **Export Hardware-IDs** and confirm a `.json` download.
2. Open **Import Hardware-IDs**, select a valid file, preview/validate, confirm apply.
3. Retry with an invalid file and confirm error feedback.

**Expected result:** Download succeeds; valid upload updates UIDs; invalid upload is rejected without corrupting config.

**Automated coverage:** Cerebra unit/E2E (out of scope for this backend test basis beyond API contracts).

---

### Test Case 4 — Backward Compatibility

**Objective:** Import files remain backward compatible with existing Bricklet types.

**Preconditions:** Database contains Servo / SSR / RGB LED Button bricklets.

**Steps:**
1. Export current config.
2. Re-import unchanged export.
3. Attempt import with an unsupported `type` string.

**Expected result:**
- Round-trip succeeds without altering types.
- Unknown types are rejected with `400`.

**Automated coverage:** `test_roundtrip_preserves_seeded_types`, `test_import_rejects_unknown_bricklet_type`.

---

## 5. Automated Suites (`pib-backend`)

- `tests/unit/test_hardware_config_service.py` — service export/import/validation.
- `tests/integration/test_hardware_config_api.py` — HTTP endpoints under `/api/system/hardware-config/*`.
