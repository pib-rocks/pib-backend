# PR-1514 Option B — rewrite E2E suites onto real deep-chat selectors

Related: https://pib-rocks.atlassian.net/browse/PR-1514
Repos: `pib-backend` (test suites) **and** `cerebra` (remove the obsolete hook code)

## Why option B

The original cut-over tried to preserve the legacy test hooks by stamping
`id="message-input"` / `id="chat-send-button"` and the `data-test` attributes onto
deep-chat's internal elements from inside the Angular component. That stamping
does **not** work reliably in the live app (verified on the Pi 5: the properties
`connect`, `loadHistory`, `avatars`, `validateInput` are all set — so
`ngAfterViewInit` runs — yet `#message-input` never appears in the shadow root).

Option B drops the stamping entirely and points the test suites at deep-chat's
real DOM instead.

## VERIFIED FACTS (measured live against the Pi 5, deep-chat 2.5.0)

Do not re-derive these; they were probed in a real browser.

### 1. deep-chat uses an OPEN shadow root and Playwright CSS pierces it

| Selector | count | visible |
|---|---|---|
| `deep-chat #text-input` | 1 | true |
| `#text-input` | 1 | true |
| `deep-chat #submit-icon` | 1 | true |
| `deep-chat div[contenteditable=true]` | 1 | true |
| `[data-test='TXT_Chat_Message']` | 0 | false |
| `[data-test='BTN_Chat_Send']` | 0 | false |
| `#message-input` | 0 | false |
| `#chat-send-button` | 0 | false |

So plain CSS selectors work — no `>>>` / shadow-piercing syntax needed.

### 2. Shadow DOM structure

```
deep-chat (shadowRoot, open)
└── div#container
    └── div#chat-view
        ├── div#messages
        └── div#input
            ├── div#file-attachment-container
            └── div#text-input-container
                ├── div#text-input          <- contenteditable, THE INPUT
                └── div.input-button-container.inner-button-container
                    └── div.input-button...  <- THE STATE CARRIER (wrapper)
                        └── svg#submit-icon  <- the icon only
```

### 3. The input is a `contenteditable` div, NOT a textarea/input

- `fill()` and `press_sequentially()` do **not** work on it.
- Use `click()` then `page.keyboard.type("...")`.
- Read it back with `inner_text()` (verified: typing `abc` yields `'abc'`).

### 4. Enabled/disabled state lives on the WRAPPER, not on `#submit-icon`

Measured transition at the 2 → 3 character boundary:

```
wrapper.class: 'input-button inside-end disabled-button input-button-svg'
            -> 'input-button inside-end submit-button   input-button-svg'
wrapper.aria-disabled: 'true' -> None (attribute removed)
```

Therefore:
- **disabled** ⇔ wrapper has class `disabled-button` (and `aria-disabled="true"`)
- **enabled**  ⇔ wrapper has class `submit-button` (and no `aria-disabled`)

`#submit-icon` itself carries NO state (`class=null`, `aria=null`,
`disabled=false` in both states) — asserting on it is meaningless.

**Playwright `is_enabled()` / `is_disabled()` must NOT be used here**: these are
`div`/`svg` elements, not form controls, so Playwright reports them as always
enabled. Assert on the class / `aria-disabled` instead.

### 5. Recommended stable selectors

```python
CHAT_INPUT   = "deep-chat #text-input"
SUBMIT_WRAP  = "deep-chat .input-button.input-button-svg"   # state carrier
SUBMIT_CLICK = "deep-chat #submit-icon"                     # click target
MESSAGES     = "deep-chat #messages"
```

## Work items

### Part 1 — `cerebra`: delete the obsolete stamping code

In `src/app/voice-assistant/voice-assistant-chat/chat-window-deep-chat/chat-window-deep-chat.component.ts`
remove the whole hook-stamping mechanism, because the suites no longer need it:

- `applyE2eHooks`, `tryStampE2eHooks`, `observeE2eHooks`,
  `teardownE2eHooksWatchers`, `syncSubmitButtonDisabled`
- the fields `stampedSubmitButton`, `e2eHooksObserver`, `e2eHooksRetryTimeout`,
  `currentInputText` (only if it is not used elsewhere)
- the `applyE2eHooks(el)` call in `ngAfterViewInit` and the teardown call in `ngOnDestroy`
- the `onComponentRender` override that only existed for stamping
- `wireOnInput` **only if** it exists solely to feed `syncSubmitButtonDisabled`

Keep everything functional: `connect.handler`, `loadHistory`, streaming/`overwrite`,
`validateInput` (the >2-char rule must stay — it is the real behaviour under test),
`textInput` gating, styles, avatars, names.

Delete the corresponding specs (`describe("applyE2eHooks")` and the
`aria-disabled` mirroring spec). The remaining suite must stay green.

### Part 2 — `pib-backend`: rewrite `tests/e2e/test_voice_assistant_hermes_e2e.py`

Rewrite `test_chat_send_button_activation_with_smartconnect`:

1. Wait for readiness with
   `page.wait_for_selector("deep-chat #text-input", state="visible", timeout=30000)`.
2. Type via `click()` + `page.keyboard.type(...)`; clear via
   `page.keyboard.press("Control+a")` then `page.keyboard.press("Delete")`
   (NOT `fill("")`).
3. Assert the ≤2 / >2 rule on the wrapper:
   ```python
   wrap = page.locator("deep-chat .input-button.input-button-svg")
   cls = wrap.get_attribute("class") or ""
   assert "disabled-button" in cls        # for "12"
   assert "submit-button" in cls          # for "12345678"
   ```
   Optionally also assert `wrap.get_attribute("aria-disabled")`.
4. Submit by clicking `deep-chat #submit-icon`.
5. Verify the message actually lands: assert the typed marker appears in
   `deep-chat #messages` (this is stronger than the old test, which only clicked).
6. Update the docstring — it still mentions `#message-input` / `#chat-send-button`.

Keep the existing SmartConnect setup, the persona/chat creation via REST, the
sidebar-click navigation and the cleanup in `finally` exactly as they are.
The other two tests in the file must keep passing untouched.

### Part 3 — `pib-backend`: rewrite `tests/frontend/voice_assistant.robot`

The Robot suite uses the Browser library (Playwright-based), so `css=` selectors
also pierce the open shadow root. Affected cases: **VA-006, VA-007, VA-008**.

- Replace `Wait For Element By Data Test    BTN_Chat_Send` with
  `Wait For Element By Css Selector    deep-chat #submit-icon    visible`
- Replace `Wait For Element By Data Test    TXT_Chat_Message` with
  `Wait For Element By Css Selector    deep-chat #text-input    visible`
- VA-008: `Type Into Element By Data Test    TXT_Chat_Message` uses
  `Clear Text` + `Type Text`, which do not work on a contenteditable div.
  Use the Browser library's `Click` + `Keyboard Input`/`Type Text` on
  `css=deep-chat #text-input`, then click `css=deep-chat #submit-icon`, and
  assert the marker inside `css=deep-chat #messages` instead of
  `TXT_Message_history`.
- Reuse the existing generic keywords in `../resources/frontend_keywords.robot`
  where possible (`Wait For Element By Css Selector`, `Click Element By Css Selector`).
  Add new keywords there only if strictly necessary — do not break other suites
  that rely on those keywords.

### Part 4 — `pib-backend`: update the test-basis documentation

`tests/docs/test-basis/frontend_e2e.md` documents `BTN_Chat_Send` /
`TXT_Chat_Message` for cases E2E-BDD-FE-VA-006/007/008 and lists them in the
`data-test` naming tables. Update those rows to state that the Voice-Assistant
chat is now a deep-chat web component addressed by CSS selectors
(`deep-chat #text-input`, `deep-chat #submit-icon`, `deep-chat #messages`),
and note the `disabled-button` / `submit-button` class contract.

## Constraints

- Do NOT change chat behaviour in `cerebra` (`connect.handler`, `loadHistory`,
  streaming/`overwrite`, `validateInput` >2 chars, SmartConnect gating must all stay).
- Do NOT touch `chat.service.ts`, `ros.service.ts`, `token.service.ts`,
  `voice-assistant-chat.component.*`.
- Do NOT weaken assertions into unconditional `Pass Execution` just to make tests
  green — the precondition guards that already exist may stay, but the actual
  assertions must remain real.
- Prettier for TS: 4-space indent, double quotes, trailing commas.

## Verification required

- `cerebra`: `CHROME_BIN=/usr/bin/chromium-browser npm test -- --watch=false` and
  `npm run build` (baseline before removal: 370 SUCCESS; expect fewer after the
  stamping specs are deleted — report the exact number).
- `pib-backend`: the Playwright test must pass against the live Pi 5 at
  `http://192.168.1.28`:
  `pytest tests/e2e/test_voice_assistant_hermes_e2e.py -k chat_send_button -v`
  (use `/home/pib/.hermes/hermes-agent/venv/bin/pytest`).
- Report the exact selectors used and the final pass/fail counts.
