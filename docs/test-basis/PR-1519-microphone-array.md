# Test Basis: Microphone Array (Seeed ReSpeaker) Configuration & Live Telemetry (PR-1519)

**Jira Story:** PR-1519  
**Repositories:** `cerebra` (Angular UI) & `pib-backend` (Flask REST API + Driver Service)  

---

## 1. Overview & Scope

PR-1519 introduces a dedicated **"Microphone Array"** configuration and live telemetry tab under the **System** component in Cerebra for managing the **Seeed ReSpeaker 4-Mic Array** (XMOS XVF3000 / USB UAC1.0).

---

## 2. Requirements & Acceptance Criteria

### 2.1 Backend API Contracts (`pib-backend`)

#### `GET /api/system/microphone-array/telemetry`
* **Response Status:** `200 OK`
* **Response Schema:**
  ```json
  {
    "doa_angle": 180,
    "voice_activity": false,
    "speech_detected": false,
    "audio_levels": [0.05, 0.02, 0.02, 0.03, 0.02]
  }
  ```
* **Behavior:**
  - `doa_angle`: Direction of arrival angle in degrees (0–359).
  - `voice_activity`: Boolean indicator for Voice Activity Detection (VAD).
  - `speech_detected`: Boolean indicator for sustained speech detection.
  - `audio_levels`: Array of 5 float values (0.0 to 1.0) representing RMS audio levels for the processed main channel (index 0) and the 4 raw microphone channels (indices 1–4).

#### `GET /api/system/microphone-array/tuning`
* **Response Status:** `200 OK`
* **Response Schema:**
  ```json
  {
    "preset": "Standard",
    "agc_on_off": true,
    "agc_max_gain": 30.0,
    "agc_desired_level": 0.01,
    "agc_time": 0.5,
    "stat_noise_on_off": true,
    "non_stat_noise_on_off": true,
    "echo_on_off": true,
    "hpf_on_off": 1,
    "led_mode": "DOA Trace",
    "led_brightness": 80,
    "led_color": "#00ff88"
  }
  ```

#### `POST /api/system/microphone-array/tuning`
* **Request Body:** Partial or complete tuning object.
* **Response Status:** `200 OK` with updated full tuning object.
* **Presets:**
  - `Standard`: Balanced AGC (30 dB), Stationary NS ON, Non-stationary NS ON, AEC ON, HPF ON (125 Hz).
  - `Noisy Environment / ASR`: High AGC (50 dB), Aggressive NS ON, AEC ON, HPF ON (150 Hz).
  - `Loud Speaker Playback`: Moderate AGC (20 dB), NS ON, Max AEC ON, HPF ON (150 Hz).
  - `Raw`: AGC OFF, NS OFF, AEC OFF, HPF OFF.
  - `Custom`: User-defined custom parameters.

---

### 2.2 Frontend UI Specification & Selectors (`cerebra`)

* **Navigation Tab:**
  - Selector: `a#tab-microphone-array` or `[data-test="TAB_MicrophoneArray"]`
  - Text label: `Microphone Array` (or `Microphone`)
* **DOA Compass / Radar Visualizer:**
  - Element: `<svg id="mic-doa-radar">` displaying 360° compass dial and dynamic indicator needle/dot at `doa_angle`.
* **Status Badges:**
  - `#badge-vad-status`: Active / Inactive VAD status badge.
  - `#badge-speech-detected`: Speech Detected / Silent status badge.
* **Audio Level Meters:**
  - Progress bars / meters `#meter-channel-0` through `#meter-channel-4`.
* **Preset Dropdown:**
  - Selector: `select#select-mic-preset`
  - Values: `Standard`, `Noisy Environment / ASR`, `Loud Speaker Playback`, `Raw`, `Custom`.
* **DSP Parameter Controls:**
  - AGC Toggle: `input#input-agc-toggle`
  - AGC Max Gain Slider: `input#slider-agc-max-gain` (0–60 dB)
  - Stationary Noise Suppression Toggle: `input#input-stat-noise-toggle`
  - Non-stationary Noise Suppression Toggle: `input#input-nonstat-noise-toggle`
  - AEC Echo Cancellation Toggle: `input#input-aec-toggle`
  - High-Pass Filter Cutoff Dropdown: `select#select-hpf-cutoff` (0=Off, 1=70Hz, 2=125Hz, 3=150Hz)
* **LED Ring Controls:**
  - LED Mode Dropdown: `select#select-led-mode` (`DOA Trace`, `Listening Pulse`, `Solid Color`, `Mute Indicator`, `Off`)
  - LED Brightness Slider: `input#slider-led-brightness` (0–100%)
  - LED Color Picker: `input#input-led-color`

---

## 3. Test Suites & Test Cases

### 3.1 Backend Integration & Unit Tests (`pib-backend`)
- `tests/unit/test_microphone_array_service.py`:
  - `test_service_get_telemetry_returns_defaults_when_simulated`
  - `test_service_update_tuning_applies_preset_correctly`
  - `test_service_handles_custom_parameter_overrides`
- `tests/integration/test_microphone_array_api.py`:
  - `test_api_get_telemetry_success`
  - `test_api_get_tuning_success`
  - `test_api_post_tuning_updates_preset`

### 3.2 Frontend Karma Unit Tests (`cerebra`)
- `src/app/system/microphone-array/microphone-array.component.spec.ts`:
  - `should create MicrophoneArrayComponent`
  - `should load telemetry and tuning on init`
  - `should render 360 DOA radar compass`
  - `should update sliders when preset changes`
  - `should send POST tuning payload on control change`

### 3.3 Automated E2E UI Test (`tests/e2e/test_microphone_array_e2e.py`)
- **Test Scenario:**
  1. Open Cerebra web UI and navigate to the **System** component.
  2. Click the **Microphone Array** tab (`#tab-microphone-array`).
  3. Verify that the **DOA Radar Compass** (`#mic-doa-radar`), VAD badges, and Audio Level Meters are visible and rendering values.
  4. Select preset **"Noisy Environment / ASR"** from `#select-mic-preset`.
  5. Assert that AGC Max Gain slider (`#slider-agc-max-gain`) updates to `50` and the API confirms the updated tuning.
  6. Modify LED Brightness slider (`#slider-led-brightness`) to `90` and verify the POST request payload.
