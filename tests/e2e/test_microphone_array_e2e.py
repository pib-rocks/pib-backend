import os
import re
import pytest
from playwright.sync_api import sync_playwright, Page, expect

BASE_URL = os.getenv("PIB_ROBOT_URL", "http://localhost")


@pytest.fixture(scope="function")
def page():
    with sync_playwright() as p:
        browser = p.chromium.launch(headless=True)
        context = browser.new_context(viewport={"width": 1400, "height": 900})
        page = context.new_page()
        page.on("dialog", lambda dialog: dialog.accept())
        page.goto(f"{BASE_URL}/system/diagnostics", wait_until="domcontentloaded")
        page.wait_for_selector("#system-nav", timeout=15000)
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
        page.wait_for_selector("ul.nav-tabs", timeout=15000)

        # 2. Click Microphone Array tab
        mic_tab = page.locator(
            "a:has-text('Microphone Array'), a:has-text('Microphone')"
        ).first
        expect(mic_tab).to_be_visible(timeout=10000)
        mic_tab.click()

        # 3. Assert URL is /system/microphone-array
        expect(page).to_have_url(
            re.compile(r".*/system/microphone-array$"), timeout=10000
        )

        # 4. Assert <app-microphone-array> component exists in DOM
        app_mic = page.locator("app-microphone-array")
        expect(app_mic).to_be_visible(timeout=10000)

        # 5. Assert 360° DOA Radar Compass SVG or Image exists
        radar = page.locator(
            "app-microphone-array svg, app-microphone-array img[alt*='compass']"
        ).first
        expect(radar).to_be_visible(timeout=10000)

        # 6. Assert VAD status text and channel level meters exist
        expect(page.locator("app-microphone-array")).to_contain_text(
            "VAD:", timeout=10000
        )

        progressbars = page.locator(
            "app-microphone-array progressbar, app-microphone-array .progress-bar, app-microphone-array [role='progressbar']"
        ).first
        expect(progressbars).to_be_visible(timeout=10000)

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
            re.compile(r".*/system/microphone-array$"), timeout=10000
        )

        # The field shows the live tuning, so it stays disabled until that has arrived.
        preset_input = page.locator("input[data-test='SEL_Microphone_Array_Preset']")
        expect(preset_input).to_be_visible(timeout=15000)
        expect(preset_input).to_be_enabled(timeout=15000)
        previous_preset = preset_input.input_value()

        target_preset = "Noisy Environment / ASR"
        preset_input.fill(target_preset)
        preset_input.evaluate(
            "el => el.dispatchEvent(new Event('change', { bubbles: true }))"
        )

        # The preset sets AGCMAXGAIN = 31.6; the slider below is linear (min 1, max 1000, step 0.1).
        agc_slider = page.locator("input[data-test='SLD_AGC_Max_Gain']")
        expect(agc_slider).to_be_visible(timeout=10000)
        expected_gain = 31.6
        value = float("nan")
        for _ in range(40):
            value = float(agc_slider.input_value() or "nan")
            if abs(value - expected_gain) < 0.2:
                break
            page.wait_for_timeout(250)
        assert abs(value - expected_gain) < 0.2, (
            f"AGC Max Gain is {value} after applying '{target_preset}', expected {expected_gain}"
        )

        if previous_preset and previous_preset != target_preset:
            preset_input.fill(previous_preset)
            preset_input.evaluate(
                "el => el.dispatchEvent(new Event('change', { bubbles: true }))"
            )

    def test_03_led_ring_controls_and_custom_tuning(self, page: Page):
        """
        Tests LED Ring mode and brightness controls:
        1. Selects LED mode 'DOA Trace'.
        2. Adjusts LED Brightness slider to 90%.
        """
        page.locator("#system-nav").click()
        mic_tab = page.locator(
            "a:has-text('Microphone Array'), a:has-text('Microphone')"
        ).first
        mic_tab.click()

        # Select LED mode dropdown
        led_mode_select = page.locator(
            "select#led-mode, select[data-test='DDN_LED_Mode']"
        ).first
        expect(led_mode_select).to_be_visible(timeout=10000)
        led_mode_select.select_option(label="DOA Trace")

        # Set LED brightness slider
        brightness_slider = page.locator(
            "input#led-brightness, input[data-test='SLD_LED_Brightness']"
        ).first
        expect(brightness_slider).to_be_visible(timeout=10000)
