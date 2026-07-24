# Frontend E2E — Camera Stream Verification

**Repository:** `pib-backend`
**Jira:** PR-1489
**Format:** BDD Gherkin (`Feature` / `Scenario`)
**Scope:** Cerebra frontend — Camera view (`/camera`)
**Test file:** `tests/frontend/camera.robot` (E2E-BDD-FE-CAM-005)
**Library:** Robot Framework Browser Library (Playwright)

---

## Background

The pib camera stream is delivered via ROS2 `camera_topic` (base64-encoded JPEG
frames) through the rosbridge WebSocket (`ws://localhost:9090`). Cerebra
subscribes to the topic and renders each frame into an `<img>` element
(`img#camera-stream`).

A "black screen" or "stuck stream" failure mode occurs when the `<img>` element
is present in the DOM but no real frame data has been received — the element
exists but `naturalWidth` / `naturalHeight` remain `0`, or `complete` is
`false`. The existing E2E tests (CAM-001 through CAM-004) verify that camera
**controls** render correctly but do **not** assert that the video stream itself
is delivering active frames.

---

## Feature: Camera Video Stream Active-Frame Verification

### Scenario: E2E-BDD-FE-CAM-005 — Video stream has active frames

```gherkin
Given Cerebra is open and the user has navigated to the Camera view (/camera)
  And the TGL_Camera_On_Off toggle is visible and checked (camera is enabled)
When  the browser waits for the img#camera-stream element to appear
Then  the image element satisfies ALL of the following conditions:
  | Property      | Expected Value |
  | complete      | true           |
  | naturalWidth  | > 0            |
  | naturalHeight | > 0            |
And   the test FAILS if the stream is black or fails to deliver real bytes
```

---

## Acceptance Criteria

| # | Criterion |
|---|-----------|
| 1 | The automated E2E test **fails** when the camera stream image element has `naturalWidth == 0` or `naturalHeight == 0` (black / no-frame state). |
| 2 | The automated E2E test **fails** when `complete` is `false` (image load incomplete). |
| 3 | The test uses Robot Framework Browser Library keywords (`Evaluate JavaScript` or equivalent) to inspect DOM properties at runtime. |
| 4 | The test waits up to **15 seconds** for the stream to deliver at least one valid frame before failing. |
| 5 | The test is tagged `E2E-BDD-FE-CAM-005` and is located in `tests/frontend/camera.robot`. |

---

## Technical Notes

- **Selector:** `css=img#camera-stream` — the `<img>` element into which Cerebra
  renders the base64 JPEG frames received from the ROS2 `camera_topic`.
- **JavaScript assertion evaluated on the element:**
  ```javascript
  (img) => img.complete && img.naturalWidth > 0 && img.naturalHeight > 0
  ```
- **Precondition:** The camera toggle (`TGL_Camera_On_Off`) must be visible and
  in the `checked` state, confirming the camera stream has been requested.
- **Failure mode:** If the camera hardware is disconnected, the ROS2 camera node
  has crashed, or the rosbridge WebSocket is unavailable, the `<img>` element
  will either not appear or will remain in a zero-dimension / incomplete state,
  causing the test to fail.
