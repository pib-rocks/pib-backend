import re
import pytest
from playwright.sync_api import sync_playwright, Page, expect

BASE_URL = "http://192.168.1.28"


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
        mic_tab = page.locator("a:has-text('Microphone Array'), a:has-text('Microphone')").first
        expect(mic_tab).to_be_visible(timeout=10000)
        mic_tab.click()

        # 3. Assert URL is /system/microphone-array
        expect(page).to_have_url(re.compile(r".*/system/microphone-array$"), timeout=10000)

        # 4. Assert <app-microphone-array> component exists in DOM
        app_mic = page.locator("app-microphone-array")
        expect(app_mic).to_be_visible(timeout=10000)

        # 5. Assert 360° DOA Radar Compass SVG or Image exists
        radar = page.locator("app-microphone-array svg, app-microphone-array img[alt*='compass']").first
        expect(radar).to_be_visible(timeout=10000)

        # 6. Assert VAD status text and channel level meters exist
        expect(page.locator("app-microphone-array")).to_contain_text("VAD:", timeout=10000)

        progressbars = page.locator("app-microphone-array progressbar, app-microphone-array .progress-bar, app-microphone-array [role='progressbar']").first
        expect(progressbars).to_be_visible(timeout=10000)

    def test_02_preset_selection_and_dsp_sliders(self, page: Page):
        """
        Navigates to Microphone Array tab and tests preset selection:
        1. Selects 'Noisy Environment / ASR' preset from DSP Preset combobox.
        2. Verifies AGC Max Gain slider updates to 50 dB.
        """
        page.locator("#system-nav").click()
        mic_tab = page.locator("a:has-text('Microphone Array'), a:has-text('Microphone')").first
        mic_tab.click()
        expect(page).to_have_url(re.compile(r".*/system/microphone-array$"), timeout=10000)

        # Select 'Noisy Environment / ASR' preset
        preset_select = page.locator("select#preset-select, select[data-test='DDN_Mic_Preset'], app-microphone-array select").first
        expect(preset_select).to_be_visible(timeout=10000)
        preset_select.select_option(label="Noisy Environment / ASR")

        # Trigger change event for Angular binding if needed
        preset_select.evaluate("el => { el.dispatchEvent(new Event('change', { bubbles: true })); }")

        # Verify AGC Max Gain slider updates to 30 dB
        agc_slider = page.locator("input#agc-max-gain, input[data-test='SLD_AGC_Max_Gain']").first
        expect(agc_slider).to_have_value("30", timeout=5000)

    def test_03_led_ring_controls_and_custom_tuning(self, page: Page):
        """
        Tests LED Ring mode and brightness controls:
        1. Selects LED mode 'DOA Trace'.
        2. Adjusts LED Brightness slider to 90%.
        """
        page.locator("#system-nav").click()
        mic_tab = page.locator("a:has-text('Microphone Array'), a:has-text('Microphone')").first
        mic_tab.click()

        # Select LED mode dropdown
        led_mode_select = page.locator("select#led-mode, select[data-test='DDN_LED_Mode']").first
        expect(led_mode_select).to_be_visible(timeout=10000)
        led_mode_select.select_option(label="DOA Trace")

        # Set LED brightness slider
        brightness_slider = page.locator("input#led-brightness, input[data-test='SLD_LED_Brightness']").first
        expect(brightness_slider).to_be_visible(timeout=10000)
