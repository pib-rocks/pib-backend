# Hermes Agent as Voice-Assistant Conversation Partner — Implementation Plan

> **For Hermes:** Use subagent-driven-development to implement task-by-task.

**Goal:** Replace the stateless single round-trip LLM chat in the pib voice assistant with a persistent, tool-capable, self-learning Hermes Agent — one Hermes session per pib chat — and repurpose the Personality Editor to edit that personality's `SOUL.md`.

**Architecture:** A new `HermesAgentClient` (subprocess wrapper around `hermes -z … -c <session>`) becomes an alternative backend inside the existing ROS 2 `ChatNode`. The `chat` action contract (goal → streamed sentence/code feedback → result) stays **byte-identical**, so `assistant.py`, TTS, audio-loop, and Cerebra chat UI need no changes. Backend selection is per-personality via a new `assistant_model` entry (`hermes-agent`). Each pib `chat_id` maps 1:1 to a Hermes session name (`pib_chat_<chat_id>`), giving durable cross-turn memory. The personality `description` column is repurposed as the personality's SOUL text, materialized to a per-personality `SOUL.md` that Hermes loads via `--soul`/profile injection.

**Tech Stack:** ROS 2 Jazzy (rclpy), Python 3.11, Flask + SQLAlchemy (pib_api), Angular 17 (Cerebra), Hermes Agent v0.18.2 CLI, pytest + Playwright, Karma/Jasmine.

---

## Current State Analysis

### Today's conversation flow (verified by reading source)

```
User speaks
  → audio_recorder.py (RecordAudio action)
  → stt_transcription.py  →  transcribed text
  → assistant.py :: on_transcribed_text_received()
      → self.chat(text, chat_id, generate_code=True, cb_sentence, cb_code)
        → ROS Action "chat"  (datatypes/action/Chat)
          → chat.py :: ChatNode.chat(goal_handle)          [THE REPLACEMENT TARGET]
              1. create_chat_message(user text)            → pib_api DB + ROS topic
              2. voice_assistant_client.get_personality_from_chat(chat_id)
                 → description, message_history (N), assistant_model.api_name, gender, language
              3. voice_assistant_client.get_chat_history(chat_id, history_length)
                 → last N messages, replayed as PublicApiChatMessage[]
              4. optional camera frame if assistant_model.has_image_support
              5. public_voice_client.chat_completion(text, description,
                     message_history, image_base64, model, public_api_token)
                 → SSE token stream from TRYB public-api
              6. regex chunking: sentences  +  <pib-program>…</pib-program> blocks
              7. per chunk: goal_handle.publish_feedback(Chat.Feedback{text,text_type})
                            + create_chat_message(assistant chunk, update=True)
              8. goal_handle.succeed() → Chat.Result{text,text_type}
  → assistant.py :: on_sentence_received()  → play_audio_from_speech (TTS)
    assistant.py :: on_code_visual_received() → run_program (Blockly)
```

### Key files (current)

| File | Role |
|---|---|
| `ros_packages/voice_assistant/voice_assistant/chat.py` | `ChatNode`: `chat` action, chunking, persistence. **556 lines — main change site** |
| `ros_packages/voice_assistant/voice_assistant/assistant.py` | `VoiceAssistantNode`: VA state machine, calls `chat` action. **Should stay unchanged** |
| `public_api_client/public_api_client/public_voice_client.py` | `chat_completion()` → TRYB SSE stream |
| `pib_api/client/pib_api_client/voice_assistant_client.py` | `Personality`, `get_chat_history`, `create/update_chat_message` |
| `pib_api/flask/model/personality_model.py` | `description` column `db.String(38000)` |
| `pib_api/flask/service/personality_service.py` | personality CRUD |
| `cerebra/src/app/voice-assistant/personality-description/personality-description.component.ts` | Personality editor (textarea → `description`, 1s debounce autosave) |

### Structural limitations to fix

1. **No memory beyond N messages** — `history_length` replays the last N rows each turn; nothing is learned or carried forward.
2. **No tools** — the LLM can only emit text or a `<pib-program>` block. It cannot search, read files, or act.
3. **No self-improvement** — `description` is a static prompt; pib never updates its own understanding.
4. **Single-shot statelessness** — every turn rebuilds full context; no reflection, no persistent notes.

### Verified Hermes integration facts (tested on this machine)

```bash
hermes -z "prompt"                       # one-shot; prints ONLY final text to stdout; exit 0
hermes -z "prompt" -c <session_name>     # SAME named session across invocations → durable memory
hermes -p <profile> -z "prompt"          # run under an isolated profile (own SOUL.md, .env, config)
hermes -z "prompt" --pass-session-id     # include session id in system prompt
hermes --ignore-rules                    # disables AGENTS.md / SOUL.md auto-injection
hermes sessions list|delete|prune        # session store management
hermes profile create|delete <name>      # isolated profile lifecycle
hermes mcp add|list|test|configure       # MCP server wiring (stdio or HTTP/SSE)
```

Continuity proof (actually executed):
- turn 1: `hermes -z "Remember this codeword: ZEBRA42…" -c pibtest_chat_demo` → `ZEBRA42 — acknowledged.`
- turn 2: `hermes -z "What was the codeword…" -c pibtest_chat_demo` → `ZEBRA42`

### OQ-1 RESOLVED — per-personality SOUL.md via Hermes profiles

**Verified experimentally** (profile created, SOUL.md written, agent queried, profile deleted):

1. `hermes profile create pibsoultest --no-skills` creates `~/.hermes/profiles/pibsoultest/`
   containing its **own** `SOUL.md`, `.env`, `config.yaml`, `memories/`, `cron/`, `logs/`.
2. It also installs a wrapper script `~/.local/bin/pibsoultest` whose body is
   `exec /home/pib/.local/bin/hermes -p pibsoultest "$@"` — proving the supported
   selector is the **`-p <profile>` flag** (there is no `--profile` long form, and no
   `--soul` flag).
3. Writing a marker into `~/.hermes/profiles/pibsoultest/SOUL.md` and asking
   `hermes -p pibsoultest -z "Wie lautet dein geheimes Kennwort?"` returned
   **`SOULMARKER-7731`** → the profile-local SOUL.md is genuinely injected.
4. **Profile + named session compose correctly.** With `-p pibsoultest -c pib_chat_verify`:
   turn 1 stored "Lieblingsmotor ist Bricklet 29F3"; turn 2 answered
   `Mein Kennwort ist SOULMARKER-7731.` **and** `Dein Lieblingsmotor ist der Bricklet 29F3.`
   → persona *and* durable memory work together.

**Pitfall found:** a fresh profile has an **isolated `.env`**, so it starts with no
credentials and fails with *"No LLM provider configured"*. The provisioning step MUST
copy/symlink `config.yaml` and the provider key into the profile (or use
`hermes profile create --clone`).

**Consequence for the design:** the SOUL is materialized to
`~/.hermes/profiles/pib_<personality_id>/SOUL.md` and the agent is invoked as
`hermes -p pib_<personality_id> -z <text> -c pib_chat_<chat_id>`.
This supersedes the earlier `HERMES_SOUL_PATH` placeholder in Task 2.2.

---

## Design Decisions

**D1 — Wrap the CLI, don't import Hermes.** `hermes -z … -c <name>` is a stable, documented contract. Importing Hermes internals into a ROS node would couple pib to Hermes' Python env and break ROS' rclpy env. Subprocess isolation also prevents a Hermes crash from taking down `ChatNode`.

**D2 — Keep the `chat` action contract identical.** `assistant.py` (625 lines of state machine), TTS, Blockly execution, and the Cerebra chat UI all depend on `Chat.Feedback{text,text_type}` + `Chat.Result`. Preserving it means the whole downstream stack is untouched → far lower risk.

**D3 — One Hermes session per pib chat.** Session name `pib_chat_<chat_id>`. This is what makes pib "remember" across turns and across restarts. Deleting a pib chat deletes the Hermes session.

**D4 — `description` becomes the SOUL text.** No schema migration needed (`db.String(38000)` is ample). The Personality Editor keeps writing `description`; the backend materializes it to `SOUL.md`.

**D5 — Backend selection via `assistant_model`.** Add a `hermes-agent` row to `assistant_model`. `ChatNode` routes on `api_name == "hermes-agent"`, so old personalities keep the legacy public-api path. **Fully backwards compatible + instant rollback.**

**D6 — Streaming via stdout line-buffering.** `hermes -z` prints only the final text. To keep TTS responsive, chunk the returned text with the **existing** sentence regex and publish feedback per sentence. (Real token streaming is a follow-up ticket — see Out of Scope.)

**D7 — Timeout + fallback.** Hermes with tools can take longer than a plain LLM. Hard timeout (default 120 s, configurable); on timeout/non-zero exit, speak a graceful fallback sentence rather than aborting the action.

---

## Step-by-Step Plan

### Phase 1 — Personality SOUL storage & materialization

#### Task 1.1: Add SOUL directory constant and path helper

**Objective:** One canonical place that resolves a personality's SOUL.md path.

**Files:**
- Create: `pib_api/flask/service/soul_service.py`
- Test: `tests/unit/test_soul_service.py`

**Step 1: Write failing test**

```python
# tests/unit/test_soul_service.py
import os
from service.soul_service import soul_path_for, write_soul, read_soul

def test_soul_path_is_inside_the_personality_profile(tmp_path, monkeypatch):
    monkeypatch.setattr("service.soul_service.HERMES_HOME", str(tmp_path))
    p = soul_path_for("abc-123")
    assert p == os.path.join(str(tmp_path), "profiles", "pib_abc-123", "SOUL.md")

def test_write_then_read_roundtrip(tmp_path, monkeypatch):
    monkeypatch.setattr("service.soul_service.HERMES_HOME", str(tmp_path))
    write_soul("abc-123", "Du bist pib.")
    assert read_soul("abc-123") == "Du bist pib."

def test_read_missing_returns_empty(tmp_path, monkeypatch):
    monkeypatch.setattr("service.soul_service.HERMES_HOME", str(tmp_path))
    assert read_soul("nope") == ""
```

**Step 2: Run to verify failure**

Run: `cd pib_api/flask && python -m pytest ../../tests/unit/test_soul_service.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'service.soul_service'`

**Step 3: Implement**

```python
# pib_api/flask/service/soul_service.py
"""Materializes a personality's SOUL text to its Hermes profile SOUL.md.

The Hermes Agent reads the SOUL.md of the profile it runs under (-p) to establish
pib's identity/persona. The authoritative copy lives in the personality.description
DB column; this module mirrors it into the profile directory.

Verified: ~/.hermes/profiles/<profile>/SOUL.md is injected when running
`hermes -p <profile> -z ...`.
"""
import os

HERMES_HOME = os.environ.get("HERMES_HOME", "/home/pib/.hermes")
PROFILE_PREFIX = "pib_"

DEFAULT_SOUL = "Du bist pib, ein humanoider Roboter."


def profile_name_for(personality_id: str) -> str:
    """Name of the Hermes profile that hosts this personality."""
    return PROFILE_PREFIX + personality_id


def soul_path_for(personality_id: str) -> str:
    """Absolute path of the SOUL.md belonging to one personality."""
    return os.path.join(
        HERMES_HOME, "profiles", profile_name_for(personality_id), "SOUL.md"
    )


def write_soul(personality_id: str, text: str) -> str:
    """Write the SOUL text to the profile, creating parent dirs. Returns the path."""
    path = soul_path_for(personality_id)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(text or DEFAULT_SOUL)
    return path


def read_soul(personality_id: str) -> str:
    """Read the SOUL text, or '' when it does not exist yet."""
    path = soul_path_for(personality_id)
    if not os.path.isfile(path):
        return ""
    with open(path, encoding="utf-8") as fh:
        return fh.read()
```

**Step 4: Run to verify pass**

Run: `cd pib_api/flask && python -m pytest ../../tests/unit/test_soul_service.py -v`
Expected: 3 passed

**Step 5: Commit**

```bash
git add pib_api/flask/service/soul_service.py tests/unit/test_soul_service.py
git commit -m "feat(soul): add per-personality SOUL.md materialization service"
```

---

#### Task 1.2: Materialize SOUL.md whenever a personality is created/updated

**Objective:** Editing the description in Cerebra writes SOUL.md automatically.

**Files:**
- Modify: `pib_api/flask/service/personality_service.py`
- Test: `tests/unit/test_personality_soul_sync.py`

**Step 1: Write failing test**

```python
# tests/unit/test_personality_soul_sync.py
def test_update_description_writes_soul_file(tmp_path, monkeypatch, app_ctx, make_personality):
    monkeypatch.setattr("service.soul_service.SOUL_DIR", str(tmp_path))
    from service import personality_service, soul_service

    p = make_personality(description="alt")
    personality_service.update_personality(p.personality_id, {"description": "neu"})

    assert soul_service.read_soul(p.personality_id) == "neu"
```

**Step 2: Run to verify failure**

Run: `cd pib_api/flask && python -m pytest ../../tests/unit/test_personality_soul_sync.py -v`
Expected: FAIL — SOUL file not written (`read_soul` returns `''`)

**Step 3: Implement** — in `personality_service.py`, after the description is assigned in both create and update paths:

```python
from service import soul_service   # add to imports

# ... inside create_personality(), after `personality.description = ...`:
    soul_service.write_soul(personality.personality_id, personality.description)

# ... inside update_personality(), inside the `if "description" in personality_dto:` block:
        personality.description = personality_dto["description"]
        soul_service.write_soul(personality.personality_id, personality.description)
```

**Step 4: Run to verify pass** → 1 passed

**Step 5: Commit**

```bash
git add pib_api/flask/service/personality_service.py tests/unit/test_personality_soul_sync.py
git commit -m "feat(soul): sync personality description to SOUL.md on create/update"
```

---

#### Task 1.3: Expose SOUL path on the personality read model

**Objective:** `ChatNode` must know which SOUL.md to hand Hermes.

**Files:**
- Modify: `pib_api/client/pib_api_client/voice_assistant_client.py:21-37`

**Step 3: Implement** — add to `Personality.__init__`:

```python
        self.personality_id = personality_dto.get("personalityId")
        self.soul_path = personality_dto.get("soulPath")
```

And include `soulPath` in the personality DTO serializer (`pib_api/flask/dto/` or wherever `assistantModelId` is emitted), using `soul_service.soul_path_for(personality_id)`.

**Step 5: Commit**

```bash
git commit -am "feat(soul): expose personality_id and soul_path on Personality model"
```

---

### Phase 2 — HermesAgentClient

#### Task 2.1: Session-name mapping

**Objective:** Deterministic, filesystem-safe pib-chat → Hermes-session mapping.

**Files:**
- Create: `public_api_client/public_api_client/hermes_agent_client.py`
- Test: `tests/unit/test_hermes_agent_client.py`

**Step 1: Write failing test**

```python
from public_api_client.hermes_agent_client import session_name_for

def test_session_name_is_prefixed_and_sanitized():
    assert session_name_for("abc-123") == "pib_chat_abc-123"

def test_session_name_strips_unsafe_chars():
    assert session_name_for("a/b c!") == "pib_chat_ab_c"
```

**Step 2: Run** → FAIL (module missing)

**Step 3: Implement**

```python
# public_api_client/public_api_client/hermes_agent_client.py
"""Runs the Hermes Agent as the conversation partner for a pib chat.

One pib chat_id maps to exactly one persistent Hermes session, so the agent
retains memory across turns and across robot restarts.
"""
import logging
import os
import re
import subprocess
from typing import Optional

HERMES_BIN = os.environ.get("PIB_HERMES_BIN", "/home/pib/.local/bin/hermes")
SESSION_PREFIX = "pib_chat_"
DEFAULT_TIMEOUT_SECONDS = int(os.environ.get("PIB_HERMES_TIMEOUT", "120"))

_UNSAFE = re.compile(r"[^A-Za-z0-9_-]")


def session_name_for(chat_id: str) -> str:
    """Deterministic Hermes session name for a pib chat id."""
    return SESSION_PREFIX + _UNSAFE.sub("", (chat_id or "").replace(" ", "_"))
```

**Step 4: Run** → 2 passed

**Step 5: Commit**

```bash
git add public_api_client/public_api_client/hermes_agent_client.py tests/unit/test_hermes_agent_client.py
git commit -m "feat(hermes): add session-name mapping for pib chats"
```

---

#### Task 2.2: Build the Hermes command line

**Objective:** Assemble argv, including SOUL and toolset restriction.

**Step 1: Write failing test**

```python
from public_api_client.hermes_agent_client import build_command, profile_name_for

def test_profile_name_is_derived_from_personality():
    assert profile_name_for("abc-123") == "pib_abc-123"

def test_build_command_uses_oneshot_named_session_and_profile():
    cmd = build_command("hallo", "chat-1", personality_id="p-9")
    assert cmd[0].endswith("hermes")
    assert "-p" in cmd and "pib_p-9" in cmd          # profile carries the SOUL.md
    assert "-z" in cmd and "hallo" in cmd
    assert "-c" in cmd and "pib_chat_chat-1" in cmd  # durable per-chat session

def test_build_command_without_personality_omits_profile():
    cmd = build_command("hallo", "chat-1", personality_id=None)
    assert "-p" not in cmd
```

**Step 3: Implement** — append to `hermes_agent_client.py`:

```python
PROFILE_PREFIX = "pib_"


def profile_name_for(personality_id: str) -> str:
    """Hermes profile that holds this personality's SOUL.md."""
    return PROFILE_PREFIX + _UNSAFE.sub("", (personality_id or "").replace(" ", "_"))


def build_command(
    text: str,
    chat_id: str,
    personality_id: Optional[str] = None,
    toolsets: Optional[str] = None,
) -> list[str]:
    """argv for one one-shot turn in this chat's persistent session.

    The personality's persona comes from the Hermes PROFILE
    (~/.hermes/profiles/pib_<personality_id>/SOUL.md), selected via -p.
    Conversation memory comes from the named SESSION, selected via -c.
    Verified: -p and -c compose correctly (persona + memory together).
    """
    cmd = [HERMES_BIN]
    if personality_id:
        cmd += ["-p", profile_name_for(personality_id)]
    cmd += ["-z", text, "-c", session_name_for(chat_id)]
    if toolsets:
        cmd += ["-t", toolsets]
    return cmd
```

**Step 4: Run** → 3 passed

**Step 5: Commit**

```bash
git commit -am "feat(hermes): build one-shot command with per-personality profile and chat session"
```

---

#### Task 2.2b: Provision the personality's Hermes profile

**Objective:** Create the profile (with credentials!) and write its SOUL.md.

> **Critical pitfall, found during OQ-1 verification:** a freshly created profile has an
> **isolated `.env`** and therefore *no* LLM credentials — the agent aborts with
> *"No LLM provider configured"*. Provisioning MUST seed `config.yaml` + the provider key.

**Files:**
- Modify: `public_api_client/public_api_client/hermes_agent_client.py`
- Test: `tests/unit/test_hermes_profile_provisioning.py`

**Step 1: Write failing test**

```python
from unittest.mock import patch, call
from public_api_client.hermes_agent_client import ensure_profile

def test_ensure_profile_creates_profile_when_missing(tmp_path):
    with patch("os.path.isdir", return_value=False), \
         patch("subprocess.run") as run:
        ensure_profile("p-9", soul_text="Du bist pib.")
        args = run.call_args_list[0].args[0]
        assert "profile" in args and "create" in args and "pib_p-9" in args

def test_ensure_profile_is_idempotent_when_present(tmp_path):
    with patch("os.path.isdir", return_value=True), \
         patch("subprocess.run") as run, \
         patch("builtins.open"):
        ensure_profile("p-9", soul_text="Du bist pib.")
        run.assert_not_called()          # no re-create
```

**Step 3: Implement**

```python
HERMES_HOME = os.environ.get("HERMES_HOME", "/home/pib/.hermes")


def profile_dir_for(personality_id: str) -> str:
    return os.path.join(HERMES_HOME, "profiles", profile_name_for(personality_id))


def ensure_profile(personality_id: str, soul_text: str, timeout: int = 60) -> str:
    """Create the personality's Hermes profile if needed and write its SOUL.md.

    Uses --clone so config.yaml AND the provider credentials are inherited from
    the active profile; without this the profile has no LLM provider configured.
    Returns the profile directory.
    """
    pdir = profile_dir_for(personality_id)
    if not os.path.isdir(pdir):
        subprocess.run(
            [HERMES_BIN, "profile", "create", profile_name_for(personality_id),
             "--clone", "--no-alias",
             "--description", f"pib personality {personality_id}"],
            capture_output=True, text=True, timeout=timeout, check=False,
        )
    os.makedirs(pdir, exist_ok=True)
    with open(os.path.join(pdir, "SOUL.md"), "w", encoding="utf-8") as fh:
        fh.write(soul_text or "Du bist pib, ein humanoider Roboter.")
    return pdir


def delete_profile(personality_id: str, timeout: int = 60) -> bool:
    """Remove a personality's Hermes profile (best-effort).

    NOTE: `hermes profile delete` prompts for confirmation — feed the name on stdin.
    """
    name = profile_name_for(personality_id)
    try:
        result = subprocess.run(
            [HERMES_BIN, "profile", "delete", name],
            input=name + "\n",
            capture_output=True, text=True, timeout=timeout, check=False,
        )
        return result.returncode == 0
    except Exception as exc:
        logging.warning("could not delete hermes profile %s: %s", name, exc)
        return False
```

**Step 4: Run** → 2 passed

**Step 5: Commit**

```bash
git add public_api_client/public_api_client/hermes_agent_client.py tests/unit/test_hermes_profile_provisioning.py
git commit -m "feat(hermes): provision per-personality profile with cloned credentials and SOUL.md"
```

---

#### Task 2.3: Execute a turn with timeout and error handling

**Objective:** Return the agent's reply text; never raise into the ROS action.

**Step 1: Write failing test**

```python
import subprocess
from unittest.mock import patch
from public_api_client.hermes_agent_client import run_turn

def test_run_turn_returns_stdout():
    completed = subprocess.CompletedProcess(args=[], returncode=0, stdout="Hallo!\n", stderr="")
    with patch("subprocess.run", return_value=completed):
        assert run_turn("hi", "c1") == "Hallo!"

def test_run_turn_on_timeout_returns_fallback():
    with patch("subprocess.run", side_effect=subprocess.TimeoutExpired(cmd="x", timeout=1)):
        out = run_turn("hi", "c1")
        assert out  # non-empty graceful sentence
        assert "moment" in out.lower() or "später" in out.lower()

def test_run_turn_on_error_returns_fallback():
    completed = subprocess.CompletedProcess(args=[], returncode=1, stdout="", stderr="boom")
    with patch("subprocess.run", return_value=completed):
        assert run_turn("hi", "c1")
```

**Step 3: Implement**

```python
FALLBACK_REPLY = "Entschuldige, das hat gerade zu lange gedauert. Frag mich bitte noch einmal."


def run_turn(
    text: str,
    chat_id: str,
    soul_path: Optional[str] = None,
    toolsets: Optional[str] = None,
    timeout: int = DEFAULT_TIMEOUT_SECONDS,
) -> str:
    """Run one conversational turn. Always returns speakable text."""
    cmd = build_command(text, chat_id, soul_path, toolsets)
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout, check=False
        )
    except subprocess.TimeoutExpired:
        logging.warning("hermes turn timed out after %ss (chat=%s)", timeout, chat_id)
        return FALLBACK_REPLY
    except Exception as exc:
        logging.error("hermes turn failed (chat=%s): %s", chat_id, exc)
        return FALLBACK_REPLY

    if result.returncode != 0:
        logging.error(
            "hermes exited %s (chat=%s): %s",
            result.returncode, chat_id, (result.stderr or "")[:500],
        )
        return FALLBACK_REPLY

    reply = (result.stdout or "").strip()
    return reply or FALLBACK_REPLY
```

**Step 4: Run** → 3 passed

**Step 5: Commit**

```bash
git commit -am "feat(hermes): run_turn with timeout and graceful fallback"
```

---

#### Task 2.4: Delete the Hermes session when a pib chat is deleted

**Objective:** No orphaned sessions.

**Files:**
- Modify: `public_api_client/public_api_client/hermes_agent_client.py`
- Modify: chat-deletion service in `pib_api/flask/service/` (the handler behind `DELETE /voice-assistant/chat/<id>`)

**Step 3: Implement**

```python
def delete_session(chat_id: str, timeout: int = 30) -> bool:
    """Remove the Hermes session backing this pib chat. Best-effort."""
    try:
        result = subprocess.run(
            [HERMES_BIN, "sessions", "delete", session_name_for(chat_id)],
            capture_output=True, text=True, timeout=timeout, check=False,
        )
        return result.returncode == 0
    except Exception as exc:
        logging.warning("could not delete hermes session for %s: %s", chat_id, exc)
        return False
```

**Step 5: Commit**

```bash
git commit -am "feat(hermes): delete backing session on chat deletion"
```

---

### Phase 3 — Wire the agent into ChatNode

#### Task 3.1: Register the `hermes-agent` assistant model

**Objective:** Make the backend selectable per personality.

**Files:**
- Modify: DB seed / migration for `assistant_model` (see `pib_api/flask/` seed used by `flask --app run seed_db`)

**Step 3: Implement** — add a seed row:

```python
{
    "api_name": "hermes-agent",
    "visual_name": "Hermes Agent (selbstlernend)",
    "has_image_support": True,
}
```

**Step 4: Verify**

Run: `curl -s http://192.168.1.28/api/v1/assistant-model | grep -i hermes`
Expected: the new model appears

**Step 5: Commit**

```bash
git commit -am "feat(hermes): seed hermes-agent assistant model"
```

---

#### Task 3.2: Extract the chunk-and-publish logic (refactor, no behavior change)

**Objective:** Reuse the existing sentence/`<pib-program>` chunker for both backends. **DRY.**

**Files:**
- Modify: `ros_packages/voice_assistant/voice_assistant/chat.py`

**Step 1:** Extract the body of the `for token in tokens:` loop (lines ~454-521) into:

```python
def _stream_chunks_to_goal(self, goal_handle, chat_id: str, tokens) -> tuple[Optional[str], Optional[int], str]:
    """Consume a token iterable, publishing sentence/code chunks as feedback.

    Returns (prev_text, prev_text_type, curr_text) so the caller can build Chat.Result.
    Behavior is identical to the previous inline implementation.
    """
```

Then call it from `chat()`. **No logic changes** — pure extraction.

**Step 2: Verify no regression**

Run the existing VA tests + a manual chat round-trip:
`./run_all_tests.sh` on the Pi (see Testing section)
Expected: unchanged results

**Step 5: Commit**

```bash
git commit -am "refactor(chat): extract chunk-streaming helper for backend reuse"
```

---

#### Task 3.3: Route to Hermes when the personality selects it

**Objective:** The actual swap — with the action contract preserved.

**Files:**
- Modify: `ros_packages/voice_assistant/voice_assistant/chat.py`

**Step 3: Implement** — inside `chat()`, replace the single `chat_completion` call site with a branch:

```python
from public_api_client import hermes_agent_client

# ... after personality + history are loaded:
is_hermes = personality.assistant_model.api_name == "hermes-agent"

if is_hermes:
    # Hermes keeps its own durable memory per chat → do NOT replay history.
    # Persona comes from the personality's Hermes profile (-p), memory from the
    # named session (-c). Verified: both compose correctly.
    reply_text = await asyncio.get_running_loop().run_in_executor(
        None,
        lambda: hermes_agent_client.run_turn(
            text=content,
            chat_id=chat_id,
            personality_id=getattr(personality, "personality_id", None),
        ),
    )
    tokens = [reply_text]          # single chunk; chunker splits into sentences
else:
    with self.public_voice_client_lock:
        tokens = public_voice_client.chat_completion(
            text=content,
            description=description,
            message_history=message_history,
            image_base64=image_base64,
            model=personality.assistant_model.api_name,
            public_api_token=self.token,
        )

prev_text, prev_text_type, curr_text = self._stream_chunks_to_goal(
    goal_handle, chat_id, tokens
)
```

Notes:
- `run_in_executor` keeps the ROS executor responsive during the blocking subprocess.
- Reusing `_stream_chunks_to_goal` means `<pib-program>` blocks emitted by Hermes still drive Blockly execution for free.
- **Measure the real turn latency here** — it decides OQ-6 (subprocess vs. persistent `hermes serve`).

**Step 4: Verify**

Manual: set a personality to `hermes-agent`, speak to pib, confirm TTS answers and the Cerebra chat shows the message.

**Step 5: Commit**

```bash
git commit -am "feat(chat): route conversation to Hermes Agent when selected"
```

---

#### Task 3.4: Make the timeout configurable per personality

**Objective:** Tool-using turns need headroom; keep it tunable without a redeploy.

**Files:**
- Modify: `ros_packages/voice_assistant/voice_assistant/chat.py` (read `PIB_HERMES_TIMEOUT`)
- Modify: `ros_packages/voice_assistant/launch/launch.py` (surface the env var)

**Step 5: Commit**

```bash
git commit -am "feat(hermes): make agent turn timeout configurable"
```

---

### Phase 4 — Personality Editor becomes the SOUL editor

#### Task 4.1: Relabel the editor to SOUL.md

**Objective:** Make the semantics explicit to the user.

**Files:**
- Modify: `cerebra/src/app/voice-assistant/personality-description/personality-description.component.html`

**Step 3: Implement** — change the heading/label from the description wording to `SOUL.md`, and add a short hint that this text defines pib's identity and is loaded by the agent. Keep `[(ngModel)]="textAreaContent"` and `updateDescription()` **unchanged** (the DB field is the same).

**Step 4: Verify**

Run: `cd cerebra && CHROME_BIN=/usr/bin/chromium-browser npm test -- --watch=false --browsers=ChromeHeadless`
Expected: 346 passed (unchanged)

**Step 5: Commit**

```bash
git commit -am "feat(cerebra): present personality editor as SOUL.md editor"
```

---

#### Task 4.2: Monospace + larger editing surface

**Objective:** SOUL.md is markdown; make it comfortable to edit.

**Files:**
- Modify: `.../personality-description.component.scss`

**Step 5: Commit**

```bash
git commit -am "style(cerebra): monospace SOUL editor with larger textarea"
```

---

#### Task 4.3: Unit-test the SOUL editor autosave

**Objective:** Lock in the debounce + save behavior.

**Files:**
- Modify: `.../personality-description.component.spec.ts`

**Step 1: Write test**

```typescript
it("saves the SOUL text after the debounce", fakeAsync(() => {
    component.personality = personality;
    component.textAreaContent = "Du bist pib.";
    component.updateDescription();
    tick(1000);
    expect(voiceAssistantServiceSpy.updatePersonalityById).toHaveBeenCalled();
    expect(personality.description).toBe("Du bist pib.");
}));
```

**Step 4: Run** → passes

**Step 5: Commit**

```bash
git commit -am "test(cerebra): cover SOUL editor debounced autosave"
```

---

### Phase 5 — Tool use and self-learning

#### Task 5.1: Interim tool access via a pib skill (bridge until the MCP server exists)

**Objective:** Give the agent a documented, read-mostly view of pib's control surface now,
without waiting for the MCP server. This is deliberately the **interim** solution —
**PR-1506 (follow-up story) replaces it with a proper MCP server.**

**Files:**
- Create: `~/.hermes/profiles/pib_<personality_id>/skills/pib-robot-control/SKILL.md`
  (seeded from a template in `ros_packages/voice_assistant/skills/pib-robot-control/SKILL.md`)

**Step 3: Implement** — document the pib REST surface the agent may call
(`/api/v1/voice-assistant/...`, motor endpoints, program execution) and the
`<pib-program>` contract for Blockly.

**Constraint:** ship with `terminal` **disabled** (`-t` allow-list) until the MCP server
provides properly scoped, validated tools.

**Step 5: Commit**

```bash
git commit -m "feat(hermes): interim pib-robot-control skill for the voice agent"
```

---

#### Task 5.2: Let pib refine its own SOUL.md (the self-learning core)

**Objective:** After a conversation, pib may append durable lessons about the user to its SOUL.

**Design:** Hermes' own `memory` tool already persists cross-session facts. For SOUL edits,
expose a narrow, **append-only** endpoint so the agent cannot destroy its persona:

- Add `POST /api/v1/voice-assistant/personality/<id>/soul/append` → appends a bounded
  (`<= 500` chars) line to `description`, re-materializes the profile SOUL.md.
- Document it in the skill (later: expose it as an MCP tool in PR-1506).

**Guardrails:** append-only, size-capped, and the full SOUL stays user-editable in Cerebra
(the user always wins).

**Step 1: Write failing test**

```python
def test_soul_append_is_bounded_and_appends(client, make_personality):
    p = make_personality(description="Basis.")
    r = client.post(f"/voice-assistant/personality/{p.personality_id}/soul/append",
                    json={"lesson": "Der Nutzer heißt Jürgen."})
    assert r.status_code == 200
    assert "Jürgen" in get_description(p.personality_id)

def test_soul_append_rejects_oversized_lesson(client, make_personality):
    p = make_personality(description="Basis.")
    r = client.post(f"/voice-assistant/personality/{p.personality_id}/soul/append",
                    json={"lesson": "x" * 5000})
    assert r.status_code == 400
```

**Step 4: Run** → 2 passed

**Step 5: Commit**

```bash
git commit -m "feat(soul): append-only self-learning endpoint with size guard"
```

---

### Phase 6 — Tests, docs, rollout

#### Task 6.1: E2E — talk to a Hermes-backed personality

**Files:**
- Create: `tests/e2e/test_voice_assistant_hermes_e2e.py`

Covers: set a personality to `hermes-agent` → send a chat message via the API → assert an assistant reply is persisted → assert the reply references something only a *previous* turn established (proves durable memory).

#### Task 6.2: Document the new requirement in the test basis

**Files:**
- Create: `docs/test-basis/hermes-agent-voice-assistant.md`

Per project convention, every new requirement is documented in the test basis and mapped to automated tests.

#### Task 6.3: Rollout & rollback runbook

**Files:**
- Create: `docs/runbooks/hermes-voice-agent.md`

Contents: how to switch a personality to `hermes-agent`; how to revert (select the old model — no redeploy); where sessions live; how to inspect/prune them; how to read SOUL.md on disk.

---

## Files Likely to Change

**pib-backend**
```
public_api_client/public_api_client/hermes_agent_client.py      (new)
pib_api/flask/service/soul_service.py                           (new)
pib_api/flask/service/personality_service.py                    (modify)
pib_api/flask/controller/personality_controller.py              (modify: soul/append)
pib_api/client/pib_api_client/voice_assistant_client.py         (modify: soul_path)
ros_packages/voice_assistant/voice_assistant/chat.py            (modify: routing + refactor)
ros_packages/voice_assistant/launch/launch.py                   (modify: env)
tests/unit/test_soul_service.py                                 (new)
tests/unit/test_hermes_agent_client.py                          (new)
tests/unit/test_personality_soul_sync.py                        (new)
tests/e2e/test_voice_assistant_hermes_e2e.py                    (new)
docs/test-basis/hermes-agent-voice-assistant.md                 (new)
docs/runbooks/hermes-voice-agent.md                             (new)
```

**cerebra**
```
src/app/voice-assistant/personality-description/personality-description.component.html   (modify)
src/app/voice-assistant/personality-description/personality-description.component.scss   (modify)
src/app/voice-assistant/personality-description/personality-description.component.spec.ts (modify)
```

**Explicitly NOT changed:** `assistant.py`, `audio_loop.py`, `tts_synthesis.py`, `audio_player.py`, `audio_recorder.py`, `stt_transcription.py`, Cerebra chat UI, `datatypes/action/Chat.action`.

---

## Tests / Validation

```bash
# pib-backend unit + integration (on the Pi — arm64 venv)
sshpass -p 'pib' ssh pib@192.168.1.28 'cd /home/pib/app/pib-backend && ./run_all_tests.sh'

# targeted new unit tests
cd pib_api/flask && python -m pytest ../../tests/unit -v -k "soul or hermes"

# E2E
python -m pytest tests/e2e/test_voice_assistant_hermes_e2e.py -v

# cerebra
cd cerebra && CHROME_BIN=/usr/bin/chromium-browser npm test -- --watch=false --browsers=ChromeHeadless
cd cerebra && npm run build
```

**Manual acceptance:**
1. Cerebra → Voice Assistant → new personality → model `Hermes Agent (selbstlernend)`.
2. Edit SOUL.md text; confirm `/home/pib/pib_souls/<id>/SOUL.md` matches.
3. Speak to pib; confirm spoken answer + message in chat window.
4. Ask pib to remember a fact; **restart the VA**; ask again → it still knows (proves durable memory).
5. Switch the personality back to a legacy model → old behavior returns (rollback works).

---

## Risks & Tradeoffs

| Risk | Impact | Mitigation |
|---|---|---|
| Hermes turn latency (tools) breaks conversation flow | High | Configurable timeout (120 s default) + graceful fallback sentence; measure real latency in Task 3.3 |
| No true token streaming → TTS starts later | Medium | Chunk the final text with the existing sentence regex; real streaming is a follow-up ticket |
| Subprocess spawn cost per turn | Medium | Measure; if too slow, move to `hermes serve` JSON-RPC (port 9119) as a persistent backend |
| Agent with tools does something unintended | High | Restrict toolsets via `-t`; append-only + size-capped SOUL endpoint; no `--yolo` |
| Session store growth | Low | `hermes sessions prune`; delete session on chat deletion (Task 2.4) |
| SOUL.md drift between DB and disk | Medium | DB is authoritative; re-materialize on every write; `read_soul` tolerates missing files |
| Hermes CLI unavailable/misconfigured on the Pi | High | Preflight check at node startup; fall back to legacy path and log loudly |

---

## Open Questions

- ~~**OQ-1**~~ **RESOLVED** — per-personality SOUL.md is delivered via a Hermes **profile**
  (`~/.hermes/profiles/pib_<personality_id>/SOUL.md`, selected with `-p`). Verified
  experimentally, including that `-p` and `-c` compose. Provisioning must use
  `--clone` so the profile inherits credentials.
- **OQ-2:** Which toolsets should pib's agent get by default? (Current plan: `terminal`
  **off**; interim skill only; proper scoped tools arrive with the MCP server in PR-1506.)
- **OQ-3:** Should camera images be forwarded to Hermes (it has `vision_analyze`)? Current
  plan drops `image_base64` on the Hermes path; the MCP server could expose a
  `pib_capture_image` tool instead.
- **OQ-4:** Per-personality Hermes model override, or always the globally configured model?
  (Note: each profile has its own `config.yaml`, so per-personality models are *possible*.)
- **OQ-5:** Multi-user robots — should each *chat* get its own SOUL, or is SOUL per
  *personality* (current plan) correct?
- **OQ-6:** Is `hermes serve` (persistent JSON-RPC) preferable to per-turn subprocess, given
  latency? Measure in Task 3.3 before deciding.

---

## Follow-up Story: pib MCP Server (PR-1506)

Tool use in this story is intentionally minimal (a documented skill, `terminal` disabled).
The **proper** mechanism is a dedicated **MCP server** that exposes pib's capabilities as
first-class, schema-validated tools. This is scoped as its own story because it is a
separate deliverable with its own test surface.

**Why MCP rather than more skills or new core tools:**
- Tools get **typed input schemas** → the model cannot invent malformed motor commands.
- **Scoped and auditable** — the server decides what is allowed; no shell access needed.
- Reusable by **any** MCP client (Claude Desktop, other agents), not just pib's voice path.
- Keeps the Hermes core narrow: capability lives at the edge, exactly as intended.

**Verified integration surface (already available in Hermes v0.18.2):**

```bash
hermes mcp add pib --command python --args -m pib_mcp_server   # stdio transport
hermes mcp add pib --url http://127.0.0.1:8765/sse             # or HTTP/SSE
hermes mcp list | test pib | configure pib                     # inspect / toggle tools
```

MCP config is **per profile**, so each personality can be granted a different tool subset.

**Proposed tool surface for `pib_mcp_server`:**

| Tool | Purpose |
|---|---|
| `pib_list_motors` | enumerate motors/bricklets with current positions |
| `pib_move_motor` | move one motor to a validated position (range-checked) |
| `pib_apply_pose` | apply a stored pose by name |
| `pib_list_poses` | enumerate available poses |
| `pib_capture_image` | grab a camera frame (answers OQ-3) |
| `pib_run_program` | execute a stored Blockly/Python program by id |
| `pib_list_programs` | enumerate programs |
| `pib_set_led` / `pib_set_relay` | actuator control incl. solid-state relay |
| `pib_soul_append` | the append-only self-learning endpoint from Task 5.2 |
| `pib_get_state` | battery/diagnostics/joint state snapshot |

**Sketched acceptance criteria for PR-1506:**
1. `pib_mcp_server` starts standalone and passes `hermes mcp test pib`.
2. Every tool has a JSON schema; invalid input is rejected **before** reaching hardware.
3. Motor positions are range-validated against the motor's configured limits.
4. A destructive/actuating tool requires explicit enablement (not on by default).
5. The voice agent can move a motor **through MCP** with no `terminal` access.
6. The interim `pib-robot-control` skill from Task 5.1 is removed/reduced to a pointer.
7. Unit tests for schema validation + integration test against a running server.
8. Documented in `docs/test-basis/` per project convention.
9. Runbook: how to add/remove the MCP server per personality profile.

**Sequencing:** PR-1506 depends on this story (needs the agent wired in), and it
**supersedes** Task 5.1.

---

## Out of Scope (follow-up tickets)

1. **pib MCP server for tool use — see PR-1506 above (planned follow-up).**
2. True token-level streaming from Hermes into `Chat.Feedback` (needs a streaming Hermes API).
3. Replacing the Gemini live `audio_loop.py` path with Hermes (voice-to-voice).
4. Multi-agent / MoA personalities.
5. Fine-tuning pib on its own conversation history.
