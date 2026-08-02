# Test Basis: Hermes Agent Speed Optimization (Gemini Flash / Flash-Lite)

**Repository:** `pib-backend`  
**Requirement:** Jira [PR-1524](https://pib-rocks.atlassian.net/browse/PR-1524)  
**Components:** `public_api_client/hermes_agent_client.py`, `pib_hermes_config`

## Requirement

Define and seed high-speed Hermes Agent configuration defaults for Gemini 3.5 Flash and Gemini 3.5 Flash-Lite so voice-assistant turns achieve minimal TTFT and maximum generation speed:

| Parameter | Value |
|---|---|
| `model` | `gemini-3.5-flash` / `gemini-3.5-flash-lite` |
| `reasoning_effort` | `low` |
| `max_tokens` | `1024` |
| `temperature` | `0.3` |
| `context_compress_threshold` | `0.7` |

## Acceptance Criteria Traceability

| AC | Acceptance criterion | Coverage | Status |
|---|---|---|---|
| AC1 | High-speed defaults (`reasoning_effort`, `max_tokens`, `temperature`, Flash models) are defined in `hermes_agent_client.py` and `pib_hermes_config`. | `tests/unit/test_hermes_speed_optimization.py` | Planned |
| AC2 | `ensure_profile` seeds profile `config.yaml` with `reasoning_effort: low`, `max_tokens: 1024`, `temperature: 0.3`. | `tests/unit/test_hermes_speed_optimization.py` | Planned |
| AC3 | `gemini-3.5-flash-lite` model string is supported as `DEFAULT_HERMES_LITE_MODEL`. | `tests/unit/test_hermes_speed_optimization.py` | Planned |

## Test Cases (from Jira PR-1524)

### Testfall 1 — Low Reasoning Effort / Thinking Off

**Objective:** Verify that Gemini does not prepend Thinking tokens in the stream and that TTFT latency stays under 200 ms.

**Preconditions:** Hermes profile configured with `reasoning_effort: "low"` (or equivalent thinking budget off) and model `gemini-3.5-flash` or `gemini-3.5-flash-lite`.

**Steps:**
1. Seed / confirm profile `config.yaml` has `reasoning_effort: low`.
2. Send a short conversational turn through the Hermes agent.
3. Observe the response stream and measure time-to-first-token.

**Expected result:**
- No Thinking / extended-reasoning tokens appear before the spoken reply.
- TTFT is under 200 ms under normal network conditions.

**Automated coverage:** Unit tests assert `DEFAULT_REASONING_EFFORT == "low"` and that `ensure_profile` writes `reasoning_effort: low` into profile `config.yaml`. Live TTFT measurement is a manual / integration check on the Pi.

---

### Testfall 2 — Max Tokens Bounding

**Objective:** Verify that responses are limited to at most 1024 tokens and that unbounded generation does not occur.

**Preconditions:** Profile `config.yaml` contains `max_tokens: 1024`.

**Steps:**
1. Confirm `DEFAULT_MAX_TOKENS == 1024` and that `ensure_profile` seeds it.
2. Prompt the agent with a request that would otherwise produce a very long answer.
3. Observe that generation stops within the configured token budget.

**Expected result:**
- Responses are bounded by `max_tokens: 1024`.
- No endless token streaming.

**Automated coverage:** Unit tests assert constant value and profile seeding of `max_tokens: 1024`.

---

### Testfall 3 — Gemini 3.5 Flash Lite Interoperability

**Objective:** Verify that the model string `gemini-3.5-flash-lite` is accepted by Hermes and answers successfully with `reasoning_effort: "low"`.

**Preconditions:** `DEFAULT_HERMES_LITE_MODEL = "gemini-3.5-flash-lite"` is defined; a profile may use this model string.

**Steps:**
1. Confirm the lite model constant equals `gemini-3.5-flash-lite` in both packages.
2. (Manual / on-device) Configure a profile with `model: gemini-3.5-flash-lite` and `reasoning_effort: low`.
3. Send a turn and confirm a normal reply without provider/model errors.

**Expected result:**
- Hermes accepts `gemini-3.5-flash-lite`.
- The turn completes without model-string or reasoning-effort errors.

**Automated coverage:** Unit tests assert `DEFAULT_HERMES_LITE_MODEL == "gemini-3.5-flash-lite"` in `hermes_agent_client` and `pib_hermes_config`.

---

### Testfall 4 — System Prompt & Toolset Size Trimming

**Objective:** Verify that the request payload stays under 5 KB when using a focused voice-assistant toolset.

**Preconditions:** Voice-assistant profile uses a slim enabled toolset (not the full Hermes tool catalogue).

**Steps:**
1. Build or inspect the outbound request payload for a typical voice turn.
2. Measure serialized prompt + tool definitions size.

**Expected result:**
- Request payload remains under 5 KB with the focused toolset.

**Automated coverage:** Documented here as an on-device / integration check; constants and profile seeding tests do not measure live payload size.

---

### Testfall 5 — Dauerhafte Profil-Vererbung (Persistent Profile Inheritance)

**Objective:** Verify that newly created personalities automatically inherit the optimized settings (`reasoning_effort: "low"`, `max_tokens: 1024`) from base / ensure_profile seeding.

**Preconditions:** No pre-existing profile directory for the personality under test.

**Steps:**
1. Call `ensure_profile` for a new personality id.
2. Load the created profile `config.yaml`.
3. Assert speed defaults are present (and model/provider are pinned).

**Expected result:**
- New profiles contain `reasoning_effort: low`, `max_tokens: 1024`, `temperature: 0.3`.
- Existing profiles missing these keys are repaired on the next `ensure_profile` call.

**Automated coverage:** `tests/unit/test_hermes_speed_optimization.py` covers fresh and existing profile seeding.
