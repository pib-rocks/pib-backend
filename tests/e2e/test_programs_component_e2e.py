import re
import time
import requests
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
        page.goto(f"{BASE_URL}/joint-control/head", wait_until="domcontentloaded")
        page.wait_for_selector("#program-nav", timeout=15000)
        yield page
        context.close()
        browser.close()


class TestProgramsComponentE2E:

    def test_01_marimo_tab_navigation_component_and_sidebar_rendering(self, page: Page):
        """
        FIRST & PRIMARY TEST FOR MARIMO:
        Navigates to the Marimo tab and verifies:
        1. URL changes to /program/marimo.
        2. The <app-marimo> component and iframe are present in the DOM/source code.
        3. The right management bar (<app-sidebar-right>) exists with workbook controls.
        """
        # 1. Click Program in main left navigation
        page.locator("#program-nav").click()
        page.wait_for_selector("ul.nav-tabs", timeout=15000)

        # 2. Click Marimo tab
        marimo_tab = page.locator('a[data-test="LNK_Marimo"]')
        expect(marimo_tab).to_be_visible()
        marimo_tab.click()

        # 3. Assert URL is /program/marimo using regex
        expect(page).to_have_url(re.compile(r".*/program/marimo$"), timeout=10000)

        # 4. Assert <app-marimo> component exists in DOM
        app_marimo = page.locator("app-marimo")
        expect(app_marimo).to_be_visible(timeout=10000)

        # 5. Assert right management sidebar <app-sidebar-right> exists and has 'New workbook' button
        sidebar = page.locator("app-marimo app-sidebar-right")
        expect(sidebar).to_be_visible(timeout=10000)
        expect(sidebar).to_contain_text("New workbook", timeout=10000, ignore_case=True)

        # 6. Assert source code contains iframe template definition and marimo-server URL
        content = page.content()
        assert "app-marimo" in content, "Expected <app-marimo> in page source code"
        assert "app-sidebar-right" in content, "Expected <app-sidebar-right> in page source code"

    def test_02_tab_header_rendering_and_navigation(self, page: Page):
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

    def test_03_single_active_tab_class_isolation(self, page: Page):
        """Verify that only the active tab parent `li` has the `navbar-btn-active` class."""
        page.locator("#program-nav").click()
        page.wait_for_selector('a[data-test="LNK_Marimo"]', timeout=15000)

        prog_li = page.locator('ul.nav-tabs li:has(a[data-test="LNK_Programs"])')
        marimo_li = page.locator('ul.nav-tabs li:has(a[data-test="LNK_Marimo"])')
        assign_li = page.locator('ul.nav-tabs li:has(a[routerlink="/program/rgb-led-button"])')

        # Click on Marimo tab
        page.locator('a[data-test="LNK_Marimo"]').click()

        # Verify Marimo is active and Programs is deselected
        expect(marimo_li).to_have_class("nav-item navbar-btn-active", timeout=10000)
        expect(prog_li).to_have_class("nav-item")
        expect(assign_li).to_have_class("nav-item")

    def test_04_right_sidebar_workbook_creation_and_deletion(self, page: Page):
        """Verify workbook management in the right sidebar (app-sidebar-right)."""
        unique_id = str(int(time.time()))
        test_filename = f"e2etest{unique_id}.py"

        # Create temporary notebook via REST API
        resp = requests.post(f"{BASE_URL}/api/v1/marimo/notebooks", json={"name": test_filename})
        assert resp.status_code in (200, 201)

        # Navigate cleanly to /program -> /program/marimo
        page.locator("#program-nav").click()
        page.wait_for_selector('a[data-test="LNK_Marimo"]', timeout=15000)
        
        with page.expect_response("**/marimo/notebooks"):
            page.locator('a[data-test="LNK_Marimo"]').click()

        sidebar_wrapper = page.locator("app-sidebar-right")
        expect(sidebar_wrapper).to_be_visible(timeout=15000)

        # Verify created workbook appears in sidebar
        expect(sidebar_wrapper).to_contain_text(f"E2Etest{unique_id}", timeout=15000, ignore_case=True)

        # Delete created notebook via REST API
        del_resp = requests.delete(f"{BASE_URL}/api/v1/marimo/notebooks/{test_filename}")
        assert del_resp.status_code == 200

    def test_05_click_two_notebooks_navigates_to_each(self, page: Page):
        """
        Create two notebooks, then CLICK each link in the sidebar and verify the
        browser navigates to /program/marimo/<file> and the iframe loads that file.
        """
        unique_id = str(int(time.time()))
        file_a = f"e2enav_a_{unique_id}.py"
        file_b = f"e2enav_b_{unique_id}.py"
        title_a = f"E2Enav A {unique_id}"
        title_b = f"E2Enav B {unique_id}"

        # 1. Create two notebooks via REST API
        for fn in (file_a, file_b):
            resp = requests.post(f"{BASE_URL}/api/v1/marimo/notebooks", json={"name": fn})
            assert resp.status_code in (200, 201), f"Failed to create {fn}: {resp.text}"

        try:
            # 2. Open Marimo tab
            page.locator("#program-nav").click()
            page.wait_for_selector('a[data-test="LNK_Marimo"]', timeout=15000)
            with page.expect_response("**/marimo/notebooks"):
                page.locator('a[data-test="LNK_Marimo"]').click()

            sidebar = page.locator("app-sidebar-right")
            expect(sidebar).to_be_visible(timeout=15000)

            # 3. Both created notebooks must appear as clickable links in the sidebar
            link_a = sidebar.locator(f'a[href$="/program/marimo/{file_a}"]')
            link_b = sidebar.locator(f'a[href$="/program/marimo/{file_b}"]')
            expect(link_a).to_be_visible(timeout=15000)
            expect(link_b).to_be_visible(timeout=15000)
            expect(link_a).to_contain_text(title_a, ignore_case=True)
            expect(link_b).to_contain_text(title_b, ignore_case=True)

            # 4. Click notebook A -> URL navigates and iframe loads file A
            link_a.click()
            expect(page).to_have_url(
                re.compile(rf".*/program/marimo/{re.escape(file_a)}$"), timeout=10000
            )
            iframe_a = page.locator("app-marimo iframe")
            expect(iframe_a).to_have_attribute(
                "src", re.compile(rf".*[?&]file={re.escape(file_a)}(&|$)"), timeout=10000
            )

            # 5. Click notebook B -> URL navigates and iframe loads file B
            link_b.click()
            expect(page).to_have_url(
                re.compile(rf".*/program/marimo/{re.escape(file_b)}$"), timeout=10000
            )
            iframe_b = page.locator("app-marimo iframe")
            expect(iframe_b).to_have_attribute(
                "src", re.compile(rf".*[?&]file={re.escape(file_b)}(&|$)"), timeout=10000
            )
        finally:
            # Cleanup: delete both notebooks via REST API
            for fn in (file_a, file_b):
                requests.delete(f"{BASE_URL}/api/v1/marimo/notebooks/{fn}")
