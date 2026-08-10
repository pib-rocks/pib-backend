# Test Basis: Voice Assistant Name Contrast

**Repository:** `pib-backend` (E2E + this document); fix lives in `cerebra`  
**Requirement:** Jira PR-1580  
**Components:** Cerebra `chat-window-deep-chat` (deep-chat 2.5.0 open Shadow DOM), live-robot Playwright E2E

## Requirement

Conversation partner name labels inside the voice-assistant chat
(`deep-chat` Shadow DOM `.name` elements) must be readable on the chat
background `#041939`. Before the fix both labels computed to `rgb(0, 0, 0)`
(~1.20:1 contrast). After the fix they must use a light colour (`#ffffff`)
via deep-chat's `names[*].style` API and a belt-and-braces
`auxiliaryStyle` rule `.name { color: #ffffff; }`, meeting WCAG 2.1 AA
contrast (≥ 4.5:1) for normal text.

The live E2E test requires a reachable robot with the Cerebra frontend
serving a chat that already contains at least one user message and one AI
message (so both `.name.end-item-position` and `.name.start-item-position`
render). It discovers such a chat via `{BASE}/api` (messages endpoint) and
falls back to the verified Eva / Nuernberg pair when needed. It skips with
an explicit reason when those prerequisites are absent. Set
`PIB_E2E_BASE_URL` for a non-default robot address; optionally set
`PIB_NAME_CONTRAST_E2E_PERSONALITY_ID` and `PIB_NAME_CONTRAST_E2E_CHAT_ID`
to force a specific chat.

**Verified DOM facts (do not redesign):** name labels are only reliably
reachable via `document.querySelector('deep-chat').shadowRoot.querySelectorAll('.name')`
inside `page.evaluate`; Playwright locators do not reach them reliably.
Read `getComputedStyle(n).color` and compare against `#041939`.

## Acceptance-criteria traceability

| AC | Acceptance criterion | Coverage | Status |
|---|---|---|---|
| AC1 | User and AI name labels are styled light (`#ffffff`), not inherited black. | Cerebra unit tests on `applyNames` / `NAME_STYLE` in `chat-window-deep-chat.component.spec.ts` (repo `cerebra`, branch `PR-1580`) | Automated in cerebra |
| AC2 | `applyNames` always configures **both** `names.user` and `names.ai`, including when `personalityName` is still undefined. | Cerebra unit tests covering early-return regression and `names.user.text === "User"` / `names.ai.text` fallback `"pib"` | Automated in cerebra |
| AC3 | `auxiliaryStyle` includes `.name { color: #ffffff; }` without removing existing `code` / `blockquote` rules. | Cerebra unit test asserting `auxiliaryStyle` contents | Automated in cerebra |
| AC4 | Live UI: every `.name` label inside deep-chat Shadow DOM has computed colour ≠ `rgb(0, 0, 0)`. | `tests/e2e/test_voice_assistant_name_contrast_e2e.py::test_voice_assistant_name_labels_meet_wcag_aa_contrast` | Automated; live path is prerequisite-gated; requires redeployed frontend |
| AC5 | Live UI: every `.name` label contrast ratio against `#041939` is ≥ 4.5:1 (WCAG 2.1 AA). | Same E2E test (explicit sRGB → linear → relative luminance → `(Lmax+0.05)/(Lmin+0.05)` helper) | Automated; live path is prerequisite-gated; requires redeployed frontend |
| AC6 | At least two name labels are present in the probed chat (user + AI); the test must not pass vacuously. | Same E2E test asserts `len(name_colors) >= 2` | Automated; live path is prerequisite-gated |
| AC7 | Visual confirmation that labels are legible on the dark chat background. | Manual screenshot of Eva / Nuernberg chat after frontend redeploy | Manual |
| AC8 | Existing chat palette (`#041939`, `#344864`, bubble/`#fff` text) is unchanged aside from name label colour. | Cerebra review of `applyStyles` / palette constants; manual visual check | Manual / review |
| AC9 | Requirement and coverage are documented in this test basis. | This document and the traceability table above | Documented |

## Manual checks still required

1. Rebuild and redeploy Cerebra to the robot so Phase 1 (`applyNames` +
   `auxiliaryStyle`) is live, then re-run the E2E test.
2. Open
   `/voice-assistant/8f73b580-927e-41c2-98ac-e5df070e7288/chat/b4f01552-0c09-401c-8fde-fda753fb0261`
   and confirm "User" and the personality name (e.g. "Eva") are clearly
   readable on the dark blue chat background.
