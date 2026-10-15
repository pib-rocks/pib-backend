# Test Basis: Local Speech-to-Text (STT) Transcription with faster-whisper

## Overview
This document specifies the requirements and acceptance criteria for the local Speech-to-Text (STT) transcription system in `pib-backend` (`ros_packages/voice_assistant`). The STT engine is powered by `faster-whisper` using the **`small`** model size for offline speech recognition on Raspberry Pi 5 ARM64 CPU. Weights are read from `/data/voice/models/whisper/` (`voice/whisper-model.yaml`). `medium` is an override, not the default. Compute type stays `int8`.

---

## BDD Specifications

### Scenario 1: Offline Local Audio Transcription with "small" Model
```gherkin
Given the voice assistant STT engine is initialized
When an incoming 16kHz mono PCM or WAV audio buffer is received
Then the "FasterWhisperSTTEngine" transcribes the audio locally without external API calls
And the default model size used is "small" with INT8 quantization and four CPU threads
And a missing model directory does not download weights by model name
And the recognized text string and detected language are returned
```

### Scenario 2: Multilingual Auto-Detection and Language Override
```gherkin
Given a speech audio sample in German or English
When transcribing without specifying an explicit language
Then the engine auto-detects the language (e.g. "de" or "en")
When an explicit language code is provided (e.g., language="de")
Then transcription forces the specified language model context
```

### Scenario 3: Robust Fallback and Error Handling
```gherkin
Given missing model weights or an invalid audio buffer
When calling the transcribe method
Then the engine catches the exception gracefully
And falls back to public API client or empty response contract without crashing the ROS node
```

### Scenario 4: Boundary Cases and Signal Edge Conditions
```gherkin
Given silent audio, empty bytes, or whitespace-only inputs
When processed by the STT engine
Then the engine returns a clean empty string or silent response
And no uncaught exceptions are raised
```


## Cloud STT and visible local fallback (PC-44)

Selecting `cloud_whisper` posts one WAV multipart `file` to the sold cloud API
`/v1/voice-assistant/speech-to-text`, with optional pinned `language`.
`PIB_CLOUD_BASE_URL` is the cloud service root or `/v1` base (LAN default
`http://192.168.1.252:8090/v1`), **not** `FLASK_API_BASE_URL`, which is robot-local.
`PIB_CLOUD_MODE=production` requires HTTPS. Userinfo, query, fragment and unrelated
paths are refused. No bearer token, provider credential, proxy environment or
redirect is used. `X-Request-ID` is a UUID logged on both robot and API. The action
has no personality-id field, so this client deliberately omits `X-Personality-ID`
rather than claiming an identifier it did not receive.

`PIB_CLOUD_STT_TIMEOUT_SECONDS` defaults to 3 and must be finite and positive.
It bounds the entire async connect/upload/read, not only a socket inactivity gap.
Responses are bounded to 64 KiB; non-200, malformed, empty and invalid text trigger
local fallback. Each request owns and closes its async HTTP client; the persistent
provider HTTP/2 pool remains server-side. Robot-to-cloud pooling is not claimed.

Cloud failure publishes `voice_using_fallback=true` **before** local inference,
then transcribes the exact same WAV with the existing pinned-language local
engine. Local success does not clear fallback; a subsequent cloud success does.
Cloud and local failure (including empty fallback text) publish the combined
technical cause on `voice_stt_failure`, mark fallback and abort the action. The
existing assistant subscriber presents the cause and opens no chat turn.
Local-only and legacy tryb routing are unchanged. Foreign engines never upload.
Logs contain correlation, byte count, status and duration, not audio, transcript,
provider body or credential. The existing local engine start-up self-check is
unchanged; lazy loading and memory savings are not claimed by this slice.

Verification: `tests/unit/test_cloud_stt_client.py` executes the actual recorder
method with ROS boundary substitutes and exercises real TCP multipart/error/drip
responses. It does not claim acoustic or provider acceptance. Real robot acoustic
chat and controlled fallback measurements are recorded separately in PC-44.
PC-45 is the host egress policy in `docs/runbooks/voice-egress.md` and
`setup/voice_egress/`. PC-46 owns broader latency/cost/FLAC evaluation.
