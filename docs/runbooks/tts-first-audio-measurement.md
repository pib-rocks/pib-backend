# TTS first-audio measurement (PR-2074)

Repeatable spoken-turn series for the concept budget row **TTS first audio**
(proposed ≤ **600 ms** from `first_sentence_flushed` to the first answer PCM
chunk). The voice assistant logs one line per spoken turn:

```text
[PERF_TRACE] TTS_FIRST_AUDIO chat=<uuid> elapsed_ms=<ms>
```

Surrounding pipeline detail is in the matching `[TURN_LATENCY]` line
(`first_sentence_flushed_ms`, `first_audio_ms`, and `deltas=`).

## What needs the robot

| Step | Robot required? |
|------|-----------------|
| Unit tests (`tests/unit/test_measure_tts_first_audio.py`) | No |
| Parse a saved log (`--log-file`) | No |
| Drive typed turns + collect logs (`--host`, `--chat-id`) | Yes |
| Observe microphone turns (`--observe-only`) | Yes |

The harness is read-only: no container restart, deploy, or config change.

## Preconditions

- Voice stack running (`multirepo-ros-voice-assistant-1` by default).
- A **chat id** for the personality under test (Cerebra → chat UUID). Pass
  `--personality` only as a label in JSON output.
- SSH as `pib` to the robot (`PIB_ROBOT_SSH_PASSWORD` or key-based auth).
- Tooling inside the container at `/app/ros2_ws/tools/measure_turn_latency.py`
  (same image as production).

Typed prompts use `send_chat_message` and exercise streaming + TTS + playback.
They do **not** open the microphone.

**Audio files:** there is no supported path to inject WAV files as user speech
from this script. `play_audio_from_file` plays files through the player but does
not complete an assistant turn for measurement. For WAV/mic scenarios, use
`--observe-only` and perform the turns yourself while the tool tails logs.

## Run an automated series

From a dev machine with repo checkout:

```bash
python tools/measure_tts_first_audio.py \
  --host 192.168.1.172 \
  --chat-id <uuid> \
  --personality Eva \
  --turns 5 \
  --json /tmp/tts-first-audio-series.json
```

Optional prompts:

```bash
python tools/measure_tts_first_audio.py \
  --host 192.168.1.172 \
  --chat-id <uuid> \
  --turns 3 \
  --prompt "Hallo." \
  --prompt "Erzähl einen Witz."
```

## Observe operator turns

```bash
python tools/measure_tts_first_audio.py \
  --host 192.168.1.172 \
  --chat-id <uuid> \
  --turns 5 \
  --observe-only
```

Speak at the robot between start and timeout; the tool polls
`docker logs --timestamps --since <epoch>` until enough `TTS_FIRST_AUDIO` lines
appear.

## Parse logs without SSH (fallback)

When JSONL is mounted locally, you can still use container logs — the trace is
only emitted on the spoken path:

```bash
ssh pib@<robot> 'docker logs --timestamps multirepo-ros-voice-assistant-1' \
  > /tmp/voice.log

python tools/measure_tts_first_audio.py \
  --log-file /tmp/voice.log \
  --chat-id <uuid> \
  --turns 5
```

## Reading the output

Per turn:

```text
turn=1 chat=<uuid> first_sentence_flushed_ms=... first_audio_ms=... tts_first_audio_ms=...
```

Series:

```text
series count=5 median_ms=... p90_ms=... worst_ms=... worst_turn=... worst_dominating_delta=...
```

- **Good series:** `median_ms` and `p90_ms` at or below **600 ms**; worst case
  dominated by `sentence_to_first_audio` (TTS/queue) rather than upstream LLM
  gaps (`headers_to_first_token`, etc.).
- **Bad series:** medians above budget, or worst turn dominated by upstream
  stages (model/API) or missing `TTS_FIRST_AUDIO` entirely.

Exit code **3** means the run completed but **no** `TTS_FIRST_AUDIO` lines were
found — treat that as a measurement finding (silent assistant, muted output, or
trace not reached), not a tool crash.

## Related tools

- `tools/measure_turn_latency.py` — per-turn driver inside the container.
- `tools/turn_latency_report.py` — JSONL stage percentiles (typed turns; no
  `TTS_FIRST_AUDIO` unless the answer is spoken).
