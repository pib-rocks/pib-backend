# Test Basis: Voice Assistant STT Engine Configuration and Routing

## Overview
This document specifies the functional and non-functional requirements in BDD (Gherkin) format for STT engine configuration, DB persistence, Flask API CRUD operations, and dynamic routing in `audio_recorder.py`.

---

## Requirements & Acceptance Criteria

### Functional Requirements
1. **Default Follows the Model**: A personality created without an STT choice (omitted or blank `sttEngine`) stores `cloud_whisper` when its model is pib.Cloud (directly, or through the `default` policy while it resolves to pib.Cloud) and pib.Cloud carries the `stt` capability. Every other model stores `local_whisper`. An explicit choice always wins, and an update that does not name `sttEngine` keeps the stored engine, also when the model changes.
2. **Database Persistence**: The personality stores `stt_engine` as `local_whisper` (faster-whisper), `cloud_whisper` (pib.Cloud, only while that row carries `stt`; the bare pib.Cloud row id is stored as `cloud_whisper`), or the id of another provider whose `stt` capability is set, and `tts_engine` as `supertone` or the id of a provider whose `tts` capability is set. ElevenLabs is not a value.
3. **API Endpoint Support**: The Flask API (`GET`, `POST`, `PUT` on `/voice-assistant/personality`) serializes the fields as `sttEngine` and `ttsEngine`. `GET /provider/voice-backends` lists the local engines, `cloud_whisper` while pib.Cloud carries `stt`, and other capable providers by row id.
4. **Dynamic Routing**: `audio_recorder.py` uses the personality's `stt_engine` (the record goal) and transcribes with local faster-whisper. `resolve_stt_route("cloud_whisper")` is `cloud`, distinct from the legacy `tryb_api` value (`tryb`), which still calls the existing public voice client. Cloud route execution is PC-44 and remains unimplemented; until it lands the recorder has no client for the `cloud` route.

### Non-Functional Requirements
1. **Backward Compatibility**: Existing database records without explicit `stt_engine` values default to `"local_whisper"`, and stored engines are never rewritten by the new pib.Cloud default.
2. **Resilience & Fallback**: Network failures or API lookup errors in `audio_recorder.py` must default safely to local `FasterWhisperSTTEngine` without crashing ROS2 node execution.

---

## BDD Specifications

### Feature: Personality STT Engine Configuration Persistence

#### Scenario 1: Default STT Engine Assignment on Personality Creation
```gherkin
Given a request to create a new personality without specifying an sttEngine
When the POST request is processed by /voice-assistant/personality
Then a personality on pib.Cloud is created with stt_engine "cloud_whisper"
And a personality on any other model is created with stt_engine "local_whisper"
And the response JSON reports that value as "sttEngine"
```

#### Scenario 2: Validating and Updating STT Engine Configuration
```gherkin
Given an existing personality in the database
And pib.Cloud carries the stt capability
When a PUT request is sent to /voice-assistant/personality/{personalityId} with "sttEngine": "cloud_whisper"
Then the database record is updated with stt_engine="cloud_whisper"
And a subsequent GET request returns "sttEngine": "cloud_whisper"
When a PUT request is sent with "sttEngine": "tryb_api"
Then the API responds with a 400 Bad Request status code
And the stored stt_engine is left unchanged
When a PUT request is sent with an invalid sttEngine value "invalid_engine"
Then the API responds with a 400 Bad Request status code
And the stored stt_engine is left unchanged
```

#### Scenario 3: STT Engine Retrieval via API
```gherkin
Given personalities configured with different stt_engine values ("local_whisper" and "cloud_whisper")
When querying GET /voice-assistant/personality or GET /voice-assistant/personality/{personalityId}
Then the returned JSON representation correctly serializes the sttEngine field for each personality
```

### Feature: Dynamic STT Provider Routing in Audio Recorder

#### Scenario 4: Dynamic STT Routing to Local Faster-Whisper
```gherkin
Given the audio_recorder node is initialized
And the active personality stt_engine setting is "local_whisper" (or default)
When audio recording finishes and transcription is triggered
Then audio_recorder routes the recorded audio to FasterWhisperSTTEngine
And transcription is processed locally without calling external public voice APIs
```

#### Scenario 5: Legacy stored tryb_api still dispatches to the public voice client
```gherkin
Given the audio_recorder node is initialized
And the active personality already stores the legacy stt_engine value "tryb_api"
And "tryb_api" is not selectable through the personality API
When audio recording finishes and transcription is triggered
Then resolve_stt_route("tryb_api") is "tryb"
And audio_recorder routes the recorded audio to public_voice_client.speech_to_text
And transcription is completed using the existing public voice client
And cloud route execution for "cloud_whisper" is PC-44 and remains unimplemented
```

#### Scenario 6: Non-Functional Requirement - Fallback on Network or Service Failure
```gherkin
Given stt_engine is configured
When API configuration fetch or local model fails unexpectedly
Then audio_recorder handles the failure gracefully
And defaults safely to local_whisper or error response without crashing the ROS node
```
