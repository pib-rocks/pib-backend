import base64
import json
import os
import re
import socket
import time

import pytest
from playwright.sync_api import sync_playwright, Page, expect

from robot_address import robot_base_url, robot_hostname

# Live UI waits on a Pi that is running the whole suite: 10-15 s was measured to be
# too short while whisper and headless Chromium compete for the CPU (PR-2176).
UI_TIMEOUT_MS = int(os.environ.get("PIB_MIC_E2E_TIMEOUT_MS", "45000"))

BASE_URL = robot_base_url()
ROBOT_HOST = robot_hostname()
ROSBRIDGE_PORT = int(os.getenv("PIB_ROSBRIDGE_PORT", "9090"))


def _get_doa_param(param_name: str, timeout: float = 5.0) -> str:
    """Read a live ROS parameter from /doa_publisher over rosbridge."""
    sock = socket.create_connection((ROBOT_HOST, ROSBRIDGE_PORT), timeout=timeout)
    try:
        key = base64.b64encode(os.urandom(16)).decode("ascii")
        handshake = (
            f"GET / HTTP/1.1\r\n"
            f"Host: {ROBOT_HOST}:{ROSBRIDGE_PORT}\r\n"
            f"Upgrade: websocket\r\n"
            f"Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            f"Sec-WebSocket-Version: 13\r\n\r\n"
        )
        sock.sendall(handshake.encode("ascii"))
        resp = b""
        while b"\r\n\r\n" not in resp:
            chunk = sock.recv(4096)
            if not chunk:
                raise RuntimeError("WebSocket handshake closed unexpectedly")
            resp += chunk

        payload = json.dumps(
            {
                "op": "call_service",
                "id": f"get_{param_name}_{int(time.time() * 1000)}",
                "service": "/rosapi/get_param",
                "type": "rosapi_msgs/srv/GetParam",
                "args": {"name": f"/doa_publisher:{param_name}", "default_value": ""},
            }
        ).encode("utf-8")

        mask = os.urandom(4)
        header = bytearray([0x81])
        length = len(payload)
        if length < 126:
            header.append(0x80 | length)
        else:
            header.append(0x80 | 126)
            header.extend(length.to_bytes(2, "big"))
        header.extend(mask)
        masked = bytearray(b ^ mask[i % 4] for i, b in enumerate(payload))
        sock.sendall(header + masked)

        first2 = sock.recv(2)
        ln = first2[1] & 0x7F
        if ln == 126:
            ln = int.from_bytes(sock.recv(2), "big")
        elif ln == 127:
            ln = int.from_bytes(sock.recv(8), "big")
        body = b""
        while len(body) < ln:
            body += sock.recv(ln - len(body))
        msg = json.loads(body.decode("utf-8"))
        assert msg.get("result") is True, f"get_param failed for {param_name}: {msg}"
        values = msg.get("values") or {}
        assert values.get("successful") is True, f"get_param unsuccessful: {msg}"
        return str(values.get("value", "")).strip().strip('"')
    finally:
        sock.close()


@pytest.fixture(scope="function")
def page():
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        context = browser.new_context(viewport={"width": 1400, "height": 900})
        page = context.new_page()
        page.on("dialog", lambda dialog: dialog.accept())
        page.goto(f"{BASE_URL}/system/diagnostics", wait_until="domcontentloaded")
        page.wait_for_selector("#system-nav", timeout=UI_TIMEOUT_MS)
        yield page
        context.close()
        browser.close()


class TestMicrophoneArrayE2E:

    def test_01_microphone_array_tab_navigation_and_rendering(self, page: Page):
        """
        Navigates to the Microphone Array tab under System and verifies:
        1. URL changes to /system/microphone-array.
        2. The <app-microphone-array> component renders.
        3. The 360° DOA Radar Compass is visible.
        4. VAD badges and Audio Level Meters exist.
        """
        # 1. Click System in left navigation
        page.locator("#system-nav").click()
        page.wait_for_selector("ul.nav-tabs", timeout=UI_TIMEOUT_MS)

        # 2. Click Microphone Array tab
        mic_tab = page.locator(
            "a:has-text('Microphone Array'), a:has-text('Microphone')"
        ).first
        expect(mic_tab).to_be_visible(timeout=UI_TIMEOUT_MS)
        mic_tab.click()

        # 3. Assert URL is /system/microphone-array
        expect(page).to_have_url(
            re.compile(r".*/system/microphone-array$"), timeout=UI_TIMEOUT_MS
        )

        # 4. Assert <app-microphone-array> component exists in DOM
        app_mic = page.locator("app-microphone-array")
        expect(app_mic).to_be_visible(timeout=UI_TIMEOUT_MS)

        # 5. Assert 360° DOA Radar Compass SVG or Image exists
        radar = page.locator(
            "app-microphone-array svg, app-microphone-array img[alt*='compass']"
        ).first
        expect(radar).to_be_visible(timeout=UI_TIMEOUT_MS)

        # 6. Assert VAD status text and channel level meters exist
        expect(page.locator("app-microphone-array")).to_contain_text(
            "VAD:", timeout=UI_TIMEOUT_MS
        )

        progressbars = page.locator(
            "app-microphone-array progressbar, app-microphone-array .progress-bar, app-microphone-array [role='progressbar']"
        ).first
        expect(progressbars).to_be_visible(timeout=UI_TIMEOUT_MS)

    def test_02_preset_selection_and_dsp_sliders(self, page: Page):
        """
        Navigates to the Microphone Array tab and applies a DSP preset:
        1. Sets 'Noisy Environment / ASR' in the DSP Preset field. In the current UI that field is
           a text input (data-test SEL_Microphone_Array_Preset), not a <select>; Angular reacts to
           the change event.
        2. Verifies the AGC Max Gain slider follows the preset: the preset defines AGCMAXGAIN 31.6
           on a linear 1..1000 slider.
        The preset that was active before is restored at the end, so the run leaves the robot on the
        tuning it found.
        """
        page.locator("#system-nav").click()
        mic_tab = page.locator(
            "a:has-text('Microphone Array'), a:has-text('Microphone')"
        ).first
        mic_tab.click()
        expect(page).to_have_url(
            re.compile(r".*/system/microphone-array$"), timeout=UI_TIMEOUT_MS
        )

        # The field shows the live tuning, so it stays disabled until that has arrived.
        preset_input = page.locator("input[data-test='SEL_Microphone_Array_Preset']")
        expect(preset_input).to_be_visible(timeout=UI_TIMEOUT_MS)
        expect(preset_input).to_be_enabled(timeout=UI_TIMEOUT_MS)
        previous_preset = preset_input.input_value()

        target_preset = "Noisy Environment / ASR"
        preset_input.fill(target_preset)
        preset_input.evaluate(
            "el => el.dispatchEvent(new Event('change', { bubbles: true }))"
        )

        # The preset sets AGCMAXGAIN = 31.6; the slider below is linear (min 1, max 1000, step 0.1).
        agc_slider = page.locator("input[data-test='SLD_AGC_Max_Gain']")
        expect(agc_slider).to_be_visible(timeout=UI_TIMEOUT_MS)
        expected_gain = 31.6
        value = float("nan")
        for _ in range(40):
            value = float(agc_slider.input_value() or "nan")
            if abs(value - expected_gain) < 0.2:
                break
            page.wait_for_timeout(250)
        assert (
            abs(value - expected_gain) < 0.2
        ), f"AGC Max Gain is {value} after applying '{target_preset}', expected {expected_gain}"

        if previous_preset and previous_preset != target_preset:
            preset_input.fill(previous_preset)
            preset_input.evaluate(
                "el => el.dispatchEvent(new Event('change', { bubbles: true }))"
            )

    def test_03_led_ring_controls_and_custom_tuning(self, page: Page):
        """
        Tests LED Ring mode and brightness controls end-to-end against /doa_publisher:
        1. Selects a new LED mode via #led-mode and verifies /doa_publisher:led_mode updates.
        2. Adjusts LED Brightness slider (#led-brightness, range 0..31) and verifies
           /doa_publisher:led_brightness updates on the live ROS node.
        3. Restores the previous LED mode and brightness.
        """
        page.locator("#system-nav").click()
        mic_tab = page.locator(
            "a:has-text('Microphone Array'), a:has-text('Microphone')"
        ).first
        mic_tab.click()

        led_mode_select = page.locator(
            "select#led-mode, select[data-test='SEL_LED_Mode']"
        ).first
        expect(led_mode_select).to_be_visible(timeout=UI_TIMEOUT_MS)
        expect(led_mode_select).to_be_enabled(timeout=UI_TIMEOUT_MS)

        brightness_slider = page.locator(
            "input#led-brightness, input[data-test='SLD_LED_Brightness']"
        ).first
        expect(brightness_slider).to_be_visible(timeout=UI_TIMEOUT_MS)

        initial_mode = _get_doa_param("led_mode")
        initial_brightness = int(_get_doa_param("led_brightness"))

        # Cerebra disables the brightness slider while the ring is off
        # ([disabled]="saving || tuning.ledMode === 'off' || ledBrightness === undefined"),
        # so "enabled" is only the right expectation once a mode is active - the
        # live robot sits in 'off' (PR-2170).
        if initial_mode != "off":
            expect(brightness_slider).to_be_enabled(timeout=UI_TIMEOUT_MS)

        target_mode_label = "Spin" if initial_mode == "trace" else "DOA Trace"
        target_mode_val = "spin" if initial_mode == "trace" else "trace"
        target_brightness = 28 if initial_brightness != 28 else 14

        try:
            led_mode_select.select_option(label=target_mode_label)
            expect(
                page.locator("[data-test='MSG_Microphone_Array_Success']")
            ).to_be_visible(timeout=UI_TIMEOUT_MS)
            expect(led_mode_select).to_be_enabled(timeout=UI_TIMEOUT_MS)
            assert _get_doa_param("led_mode") == target_mode_val, (
                f"Expected /doa_publisher:led_mode to be {target_mode_val!r}, "
                f"got {_get_doa_param('led_mode')!r}"
            )

            expect(brightness_slider).to_be_enabled(timeout=UI_TIMEOUT_MS)
            brightness_slider.evaluate(f"""el => {{
                    el.value = '{target_brightness}';
                    el.dispatchEvent(new Event('input', {{ bubbles: true }}));
                    el.dispatchEvent(new Event('change', {{ bubbles: true }}));
                }}""")
            deadline = time.monotonic() + 10.0
            live_brightness = initial_brightness
            while time.monotonic() < deadline:
                live_brightness = int(_get_doa_param("led_brightness"))
                if live_brightness == target_brightness:
                    break
                page.wait_for_timeout(250)
            assert live_brightness == target_brightness, (
                f"Expected /doa_publisher:led_brightness to become {target_brightness}, "
                f"got {live_brightness}"
            )
        finally:
            expect(led_mode_select).to_be_enabled(timeout=UI_TIMEOUT_MS)
            mode_label_map = {
                "off": "Off",
                "listen": "Listen",
                "speak": "Speak",
                "think": "Think",
                "spin": "Spin",
                "trace": "DOA Trace",
                "mono": "Solid Color",
            }
            if initial_mode in mode_label_map:
                led_mode_select.select_option(label=mode_label_map[initial_mode])
                expect(led_mode_select).to_be_enabled(timeout=UI_TIMEOUT_MS)
            if initial_mode != "off":
                # The slider is disabled by design while the ring is off.
                expect(brightness_slider).to_be_enabled(timeout=UI_TIMEOUT_MS)
            brightness_slider.evaluate(f"""el => {{
                    el.value = '{initial_brightness}';
                    el.dispatchEvent(new Event('input', {{ bubbles: true }}));
                    el.dispatchEvent(new Event('change', {{ bubbles: true }}));
                }}""")
            page.wait_for_timeout(500)
