import time
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
        page.goto(BASE_URL, wait_until="domcontentloaded")
        page.wait_for_selector("#program-nav", timeout=15000)
        yield page
        context.close()
        browser.close()


class TestProgramsComponentE2E:

    def _open_marimo_tab(self, page: Page):
        page.locator("#program-nav").click()
        page.wait_for_selector('a[data-test="LNK_Marimo"]', timeout=15000)
        page.wait_for_timeout(500)
        page.locator('a[data-test="LNK_Marimo"]').click()
        marimo_li = page.locator('ul.nav-tabs li:has(a[data-test="LNK_Marimo"])')
        expect(marimo_li).to_have_class("nav-item navbar-btn-active", timeout=15000)

    def test_tab_header_rendering_and_navigation(self, page: Page):
        """Verify tab header rendering and route switching across Programs, Marimo, and Assign Buttons."""
        page.locator("#program-nav").click()
        page.wait_for_selector("ul.nav-tabs", timeout=15000)

        programs_tab = page.locator('ul.nav-tabs a[data-test="LNK_Programs"]')
        marimo_tab = page.locator('ul.nav-tabs a[data-test="LNK_Marimo"]')
        assign_tab = page.locator('ul.nav-tabs a[routerlink="/program/rgb-led-button"]')

        expect(programs_tab).to_be_visible()
        expect(marimo_tab).to_be_visible()
        expect(assign_tab).to_be_visible()

        assert "PROGRAMS" in (programs_tab.inner_text()).upper()
        assert "MARIMO" in (marimo_tab.inner_text()).upper()
        assert "ASSIGN BUTTONS" in (assign_tab.inner_text()).upper()

    def test_single_active_tab_class_isolation(self, page: Page):
        """Verify that only the active tab parent `li` has the `navbar-btn-active` class."""
        self._open_marimo_tab(page)

        prog_li = page.locator('ul.nav-tabs li:has(a[data-test="LNK_Programs"])')
        marimo_li = page.locator('ul.nav-tabs li:has(a[data-test="LNK_Marimo"])')
        assign_li = page.locator('ul.nav-tabs li:has(a[routerlink="/program/rgb-led-button"])')

        # Verify Marimo tab is active and Programs tab is deselected
        expect(marimo_li).to_have_class("nav-item navbar-btn-active", timeout=10000)
        expect(prog_li).to_have_class("nav-item")
        expect(assign_li).to_have_class("nav-item")

    def test_marimo_iframe_rendering_theme_dark(self, page: Page):
        """Verify Marimo embedded iframe renders with `theme=dark` query param."""
        self._open_marimo_tab(page)

        iframe = page.locator("iframe")
        expect(iframe).to_be_attached(timeout=15000)

        src = iframe.get_attribute("src")
        assert src is not None, "Marimo iframe src property should not be None"
        assert "theme=dark" in src, f"Expected 'theme=dark' in iframe src, got: {src}"

    def test_right_sidebar_workbook_creation_and_deletion(self, page: Page):
        """Verify workbook creation & deletion via modal in the right sidebar manager."""
        unique_id = str(int(time.time()))
        test_workbook_name = f"e2etest_{unique_id}"

        self._open_marimo_tab(page)

        sidebar_wrapper = page.locator("app-sidebar-right")
        expect(sidebar_wrapper).to_be_visible(timeout=10000)

        # 1. 'New workbook' Button in der rechten Sidebar klicken
        new_wb_btn = page.locator('#sidebar-right-New\\ workbook, [data-test="BTN_New workbook"]')
        expect(new_wb_btn).to_be_visible()
        new_wb_btn.click()

        # 2. Modal prüfen & Namen eingeben
        input_field = page.locator('#input-name, [data-test="INP_Name"]')
        expect(input_field).to_be_visible()
        input_field.fill(test_workbook_name)
        input_field.dispatch_event("input")

        # Click Save button
        save_btn = page.locator('#modal-save-button, [data-test="BTN_Save"]')
        save_btn.click()

        expected_filename = f"{test_workbook_name}.py"

        # 3. Prüfen, ob das Workbook in der Sidebar erscheint
        expect(sidebar_wrapper).to_contain_text(test_workbook_name, timeout=10000, ignore_case=True)

        # 4. Löschen über das Dropdown-Menü des Eintrags
        dropdown_btn = page.locator(f'button[id*="dropdownbutton-{expected_filename}"]')
        expect(dropdown_btn).to_be_visible()
        dropdown_btn.click()

        delete_btn = page.locator(f'button[id*="sidebar-right-delete-{expected_filename}"]')
        expect(delete_btn).to_be_visible()
        delete_btn.click()

        # 5. Bestätigen, dass der Eintrag aus der Sidebar entfernt wurde
        expect(sidebar_wrapper).not_to_contain_text(test_workbook_name, timeout=10000, ignore_case=True)
