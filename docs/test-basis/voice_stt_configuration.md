# Test Basis: Voice Assistant STT Engine Configuration and Routing

## Overview
This document specifies the functional and non-functional requirements in BDD (Gherkin) format for STT engine configuration, DB persistence, Flask API CRUD operations, and dynamic routing in `audio_recorder.py`.

---

## Requirements & Acceptance Criteria

### Functional Requirements
1. **Local Faster-Whisper Default**: Personalities must default to `stt_engine="local_whisper"` upon creation if no STT engine preference is specified.
2. **Database Persistence**: The personality entity in DB and Flask API must persist `stt_engine` as either `"local_whisper"` or `"tryb_api"`.
3. **API Endpoint Support**: The Flask API (`GET`, `POST`, `PUT` on `/voice-assistant/personality`) must serialize `stt_engine` as `sttEngine` in camelCase and validate input values.
4. **Dynamic Routing**: `audio_recorder.py` must dynamically query the `stt_engine` setting from DB/API and route audio transcription to either local `FasterWhisperSTTEngine` or remote `public_voice_client.speech_to_text`.

### Non-Functional Requirements
1. **Backward Compatibility**: Existing database records without explicit `stt_engine` values must default to `"local_whisper"`.
2. **Resilience & Fallback**: Network failures or API lookup errors in `audio_recorder.py` must default safely to local `FasterWhisperSTTEngine` without crashing ROS2 node execution.

---

## BDD Specifications

### Feature: Personality STT Engine Configuration Persistence

#### Scenario 1: Default STT Engine Assignment on Personality Creation
```gherkin
Given a request to create a new personality without specifying an sttEngine
When the POST request is processed by /voice-assistant/personality
Then the personality is created with stt_engine default value "local_whisper"
And the response JSON includes "sttEngine": "local_whisper"
```

#### Scenario 2: Validating and Updating STT Engine Configuration
```gherkin
Given an existing personality in the database
When a PUT request is sent to /voice-assistant/personality/{personalityId} with "sttEngine": "tryb_api"
Then the database record is updated with stt_engine="tryb_api"
And a subsequent GET request returns "sttEngine": "tryb_api"
When a PUT request is sent with an invalid sttEngine value "invalid_engine"
Then the API responds with a 400 Bad Request status code
```

#### Scenario 3: STT Engine Retrieval via API
```gherkin
Given personalities configured with different stt_engine values ("local_whisper" and "tryb_api")
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

#### Scenario 5: Dynamic STT Routing to Cloud API (tryb_api)
```gherkin
Given the audio_recorder node is initialized
And the active personality stt_engine setting is updated to "tryb_api"
When audio recording finishes and transcription is triggered
Then audio_recorder routes the recorded audio to public_voice_client.speech_to_text
And transcription is completed using the cloud API service
```

#### Scenario 6: Non-Functional Requirement - Fallback on Network or Service Failure
```gherkin
Given stt_engine is configured
When API configuration fetch or local model fails unexpectedly
Then audio_recorder handles the failure gracefully
And defaults safely to local_whisper or error response without crashing the ROS node
```
