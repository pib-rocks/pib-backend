# Test Basis: Blockly Supertonic-3 TTS Integration

## Overview
This test basis specifies the integration of the local Supertone Supertonic-3 expressive Text-to-Speech (TTS) engine into the `play_audio_from_speech` ("Say") Blockly block.

## Functional Requirements
- **10 Supertonic Voice Styles:** Dropdown options for Female 1–5 (`F1`–`F5`) and Male 1–5 (`M1`–`M5`).
- **Language Selection:** Dropdown options for German (`de`), English (`en`), and Auto Detect (`auto` / `na`).
- **ROS 2 Service Call:** Parameter transmission via `PlayAudioFromSpeech.srv` (`request.gender` = voice, `request.language` = language).
- **Audio Output:** 44.1 kHz 16-bit PCM audio played back locally on target hardware.

## Scenarios (BDD/Gherkin)

### Scenario 1: Female voice F2 and German language
```gherkin
Given a user configures the "Say" block with text "Hallo Robot", voice "Female 2 (F2)", and language "Deutsch (DE)"
When the Blockly Python code is generated
Then the generated Python code contains 'play_audio_from_speech("Hallo Robot", \'F2\', \'de\')'
And the request.gender is set to "F1".."F5" or "M1".."M5"
```

### Scenario 2: Male voice M3 and Auto language detection
```gherkin
Given a user configures the "Say" block with text "Good morning", voice "Male 3 (M3)", and language "Auto"
When the Blockly Python code is generated
Then the generated Python code contains 'play_audio_from_speech("Good morning", \'M3\', \'auto\')'
```
