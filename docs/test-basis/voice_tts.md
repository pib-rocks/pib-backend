# Test Basis: Expressive Local TTS System (Supertone supertonic-3)

**Repository:** `pib-backend`  
**Package:** `ros_packages/voice_assistant`  
**Engine:** Supertone supertonic-3 Expressive Offline TTS  
**Default Model Path:** `/data/voice/models/supertone/` (Overridable via `SUPERTONE_MODEL_PATH`)

---

## 1. Overview & System Context

The `voice_assistant` ROS 2 package provides voice interactions for the PIB humanoid robot. To guarantee privacy, zero cloud latency dependencies, and air-gapped offline operational capabilities, local text-to-speech (TTS) synthesis is powered by the **Supertone supertonic-3** expressive neural TTS engine.

The system synthesizes raw text strings into high-quality 16-bit PCM audio encapsulated in WAV containers, supporting expressiveness parameters (emotion, speed, pitch, language) and maintaining robust fallback mechanisms when model files are missing or runtime errors occur.

---

## 2. Functional Requirements (BDD / Gherkin Format)

### Feature: Local Supertone supertonic-3 TTS Initialization & Path Resolution

  As the voice assistant service
  I want to load the Supertone supertonic-3 model from a configurable directory
  So that offline synthesis operates seamlessly across different environments.

  Scenario: Default model path resolution
    Given the environment variable "SUPERTONE_MODEL_PATH" is unset
    When the Supertone TTS engine is initialized
    Then the model directory should default to "/data/voice/models/supertone/"

  Scenario: Environment variable path override
    Given the environment variable "SUPERTONE_MODEL_PATH" is set to "/custom/models/supertone/"
    When the Supertone TTS engine is initialized
    Then the model directory should resolve to "/custom/models/supertone/"

  Scenario: Explicit model path constructor parameter
    Given an explicit model path "/explicit/path/supertone/"
    When the Supertone TTS engine is initialized with model_path="/explicit/path/supertone/"
    Then the explicit model path should override both environment variable and default path

---

### Feature: Offline Capability & Model Loading State

  As an autonomous robot system
  I want 100% offline TTS synthesis capabilities
  So that voice feedback functions without cloud or internet connectivity.

  Scenario: Successful local model loading
    Given valid Supertone supertonic-3 model artifacts exist in the model directory
    When the TTS engine attempts model initialization
    Then the engine state "is_loaded" should be True
    And the active synthesis backend should be "supertone-supertonic-3"

  Scenario: Missing model directory or missing model files
    Given no model directory exists at the target path
    When the TTS engine attempts model initialization
    Then the engine state "is_loaded" should be False
    And the engine should activate fallback mode without throwing an unhandled exception

---

### Feature: Fallback Synthesis Mechanism

  As the audio pipeline
  I want automatic fallback synthesis when primary model synthesis is unavailable
  So that the robot never fails silently when requested to speak.

  Scenario: Fallback on missing model files
    Given the Supertone supertonic-3 model is not loaded
    When speech synthesis is requested for text "Roboter bereit"
    Then the engine should execute fallback synthesis
    And return valid WAV audio data with a 44-byte header and non-zero audio bytes

  Scenario: Fallback on primary synthesis runtime exception
    Given the Supertone supertonic-3 model is loaded
    And the primary inference routine encounters a simulated runtime memory error
    When speech synthesis is requested for text "Systemfehler erkannt"
    Then the engine should catch the exception
    And automatically degrade to fallback synthesis
    And return valid WAV audio data

---

### Feature: Audio Output Format & Quality Verification

  As an audio playback node
  I want synthesized audio in strict WAV/PCM specifications
  So that the sound hardware can play the audio stream without distortion or decoding errors.

  Scenario: WAV Header and PCM Encoding Compliance
    Given a valid synthesis request for "Hallo PIB"
    When speech synthesis completes
    Then the output bytes should start with RIFF header magic bytes "RIFF" and "WAVE"
    And the audio format encoding should be 1-channel mono 16-bit PCM
    And the audio sample rate should equal 16000 Hz

  Scenario: Audio Energy and Non-Zero Byte Validation
    Given a valid synthesis request for non-empty text "Guten Tag"
    When speech synthesis completes
    Then the audio payload should contain non-zero bytes
    And the calculated RMS (Root Mean Square) audio amplitude should be greater than 0.001

---

### Feature: Boundary Conditions & Edge Case Handling

  As a robust TTS engine
  I want to process unusual inputs gracefully
  So that the engine never crashes on edge case inputs.

  Scenario: Empty text or whitespace input
    Given input text containing only whitespace "   " or empty string ""
    When speech synthesis is requested
    Then the engine should return a valid WAV container
    And the synthesis process should complete without throwing an exception

  Scenario: Numbers and currency symbols
    Given input text containing numerical digits and special symbols "Version 2.0 kostet 150 Euro"
    When speech synthesis is requested
    Then the engine should sanitize and synthesize the text
    And produce valid WAV audio output

  Scenario: Multilingual and mixed language synthesis
    Given input text in German "Guten Morgen", English "Hello world", or mixed "PIB Backend System Start"
    When speech synthesis is requested with language parameter "de" or "en"
    Then the engine should process the text
    And yield valid synthesized speech

  Scenario: Very long sentence processing
    Given input text exceeding 500 characters containing multiple long sentences
    When speech synthesis is requested
    Then the engine should chunk or process the text seamlessly
    And return consolidated valid WAV audio data

---

## 3. Non-Functional Requirements

| Requirement ID | Category | Description | Target Metric |
|---|---|---|---|
| NFR-01 | Latency | Time to first audio byte (TTFB) for 10-word sentence in offline mode | < 250 ms |
| NFR-02 | Reliability | Zero crash guarantee on malformed, long, or empty text strings | 100% uptime |
| NFR-03 | Air-gap Security | Zero outbound network calls during synthesis execution | 0 network bytes sent |
| NFR-04 | Compatibility | Output WAV files directly consumable by ROS `audio_player` and PyAudio | 100% compliance |
