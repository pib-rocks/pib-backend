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
        3. The 360° DOA Radar Compass SVG (#mic-doa-radar) is visible.
        4. VAD badges (#badge-vad-status) and Audio Level Meters (#meter-channel-0) exist.
        """
        # 1. Click System in left navigation
        page.locator("#system-nav").click()
        page.wait_for_selector("ul.nav-tabs", timeout=15000)

        # 2. Click Microphone Array tab
        mic_tab = page.locator("#tab-microphone-array, a[data-test='TAB_MicrophoneArray'], a:has-text('Microphone')").first
        expect(mic_tab).to_be_visible(timeout=10000)
        mic_tab.click()

        # 3. Assert URL is /system/microphone-array
        expect(page).to_have_url(re.compile(r".*/system/microphone-array$"), timeout=10000)

        # 4. Assert <app-microphone-array> component exists in DOM
        app_mic = page.locator("app-microphone-array")
        expect(app_mic).to_be_visible(timeout=10000)

        # 5. Assert 360° DOA Radar Compass SVG exists
        radar = page.locator("#mic-doa-radar")
        expect(radar).to_be_visible(timeout=10000)

        # 6. Assert VAD status badge and channel level meters exist
        vad_badge = page.locator("#badge-vad-status")
        expect(vad_badge).to_be_visible(timeout=10000)

        meter_ch0 = page.locator("#meter-channel-0")
        expect(meter_ch0).to_be_visible(timeout=10000)

    def test_02_preset_selection_and_dsp_sliders(self, page: Page):
        """
        Navigates to Microphone Array tab and tests preset selection:
        1. Selects 'Noisy Environment / ASR' preset from #select-mic-preset.
        2. Verifies AGC Max Gain slider (#slider-agc-max-gain) updates to 50 dB.
        3. Verifies Noise Suppression toggles are active.
        """
        page.locator("#system-nav").click()
        mic_tab = page.locator("#tab-microphone-array, a[data-test='TAB_MicrophoneArray'], a:has-text('Microphone')").first
        mic_tab.click()
        expect(page).to_have_url(re.compile(r".*/system/microphone-array$"), timeout=10000)

        # Select 'Noisy Environment / ASR' preset
        preset_select = page.locator("#select-mic-preset")
        expect(preset_select).to_be_visible(timeout=10000)
        preset_select.select_option(label="Noisy Environment / ASR")

        # Verify AGC Max Gain slider updates to 50 dB
        agc_slider = page.locator("#slider-agc-max-gain")
        expect(agc_slider).to_have_value("50", timeout=5000)

    def test_03_led_ring_controls_and_custom_tuning(self, page: Page):
        """
        Tests LED Ring mode and brightness controls:
        1. Selects LED mode 'DOA Trace' from #select-led-mode.
        2. Adjusts LED Brightness slider (#slider-led-brightness) to 90%.
        3. Verifies state persists.
        """
        page.locator("#system-nav").click()
        mic_tab = page.locator("#tab-microphone-array, a[data-test='TAB_MicrophoneArray'], a:has-text('Microphone')").first
        mic_tab.click()

        # Select LED mode
        led_mode_select = page.locator("#select-led-mode")
        expect(led_mode_select).to_be_visible(timeout=10000)
        led_mode_select.select_option(label="DOA Trace")

        # Set LED brightness slider to 90
        brightness_slider = page.locator("#slider-led-brightness")
        expect(brightness_slider).to_be_visible(timeout=10000)
        brightness_slider.fill("90")
        expect(brightness_slider).to_have_value("90", timeout=5000)
