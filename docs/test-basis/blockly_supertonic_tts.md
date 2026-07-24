# Test Basis: Blockly Supertonic-3 Local TTS Speech Generation

## Overview
This document specifies the requirements and acceptance criteria for the Blockly `play_audio_from_speech` block generators in `pib_blockly` (`pib-backend` repository). The speech generation blocks generate Python code executing text-to-speech output using the local Supertone Supertonic-3 expressive TTS engine introduced in PR-1487.

---

## BDD Specifications

### Scenario 1: Speech Output Generation with Local Supertonic-3 TTS Engine
```gherkin
Given a user configures a Blockly "as <voice> say <text>" speech block
When the Blockly program is compiled into Python
Then the generated Python code creates a client for the "play_audio_from_speech" ROS 2 service
And the request payload specifies the speech text and voice selection
And the voice assistant processes the request using the local Supertone Supertonic-3 TTS engine
```

### Scenario 2: Voice Parameter Mapping and Backward Compatibility
```gherkin
Given existing Blockly programs with legacy voice names ("Hannah", "Daniel", "Emma", "Brian")
When the generator compiles the speech block
Then legacy voice names map cleanly to Supertonic gender and language parameters
And newly added Supertonic voices ("supertonic_female_de", "supertonic_male_en") generate valid service calls
And no runtime exceptions occur due to missing or unknown voice identifiers
```

### Scenario 3: Automated Generator Unit Testing
```gherkin
Given the Jest test suite in "tests/blockly_generator/"
When running "npm test" or Jest runner
Then the generated Python code for "play_audio_from_speech" matches expected service contract signatures
And special characters in speech strings are properly escaped
```
