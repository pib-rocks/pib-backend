# PR-1519 — Microphone Array (Seeed ReSpeaker) REST API & Driver Service

Jira Ticket: https://pib-rocks.atlassian.net/browse/PR-1519
Category: Software
Branch: `PR-1519`

## Goals
Implement REST API endpoints and driver service in `pib-backend` for configuring and reading telemetry from the Seeed ReSpeaker 4-Mic Array (USB XMOS XVF3000 / UAC1.0).

## Components to implement

1. `pib_api/flask/service/microphone_array_service.py`:
   - Hardware driver using `pyusb` / `usb.core` (Vendor ID `0x2886`, Product ID `0x0018`) to read/write XMOS DSP tuning parameters (`AGCONOFF`, `AGCMAXGAIN`, `AGCDESIREDLEVEL`, `AGCTIME`, `STATNOISEONOFF`, `NONSTATNOISEONOFF`, `ECHOONOFF`, `HPFONOFF`, `DOAANGLE`, `VOICEACTIVITY`, `SPEECHDETECTED`).
   - Preset manager: "Standard", "Noisy Environment / ASR", "Loud Speaker Playback", "Raw Pass-Through", "Custom".
   - Fallback simulation / mock mode when USB device is absent, returning valid telemetry and persisting settings in memory/config.

2. `pib_api/flask/controller/microphone_array_controller.py`:
   - `GET /api/system/microphone-array/telemetry`: returns `{ "doa_angle": 180, "voice_activity": false, "speech_detected": false, "audio_levels": [0.05, 0.02, 0.02, 0.03, 0.02] }`.
   - `GET /api/system/microphone-array/tuning`: returns current DSP tuning & LED ring configuration.
   - `POST /api/system/microphone-array/tuning`: updates DSP tuning parameters, preset, or LED ring mode.

3. `pib_api/flask/app/__init__.py`:
   - Register `microphone_array_blueprint` at `/system/microphone-array` and `/v1/system/microphone-array`.

4. Tests:
   - `tests/unit/test_microphone_array_service.py`: unit tests for service parameters & presets.
   - `tests/integration/test_microphone_array_api.py`: integration tests for telemetry and tuning endpoints.

## Important Constraints
- Keep on branch `PR-1519`. DO NOT MERGE INTO DEVELOP.
- Ensure all unit & integration tests pass with pytest.
