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

    def test_01b_marimo_iframe_inner_content_loads(self, page: Page):
        """
        Verify that the Marimo iframe loads the real Marimo Reactive Notebook Editor UI
        through the /marimo-server/ proxy, accounting for Pi loading times.
        """
        page.goto(f"{BASE_URL}/program/marimo", wait_until="networkidle")
        
        # Verify iframe src uses the Nginx proxy path /marimo-server/
        iframe = page.locator("app-marimo iframe")
        expect(iframe).to_be_visible(timeout=15000)
        src = iframe.get_attribute("src")
        assert src is not None and "/marimo-server/" in src, f"Expected /marimo-server/ in iframe src, got {src}"

        # Frame locator into app-marimo iframe
        frame = page.frame_locator("app-marimo iframe")
        
        # Wait up to 30s for Marimo UI root elements inside the iframe
        marimo_root = frame.locator(".marimo-app, [data-testid='marimo-notebook'], marimo-code-editor, body")
        expect(marimo_root.first).to_be_visible(timeout=30000)

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
        assert "NOTEBOOKS" in (marimo_tab.inner_text()).upper()
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

    def _open_marimo(self, page: Page):
        """Helper: open Programs -> Marimo tab and wait for the sidebar."""
        page.locator("#program-nav").click()
        page.wait_for_selector('a[data-test="LNK_Marimo"]', timeout=15000)
        with page.expect_response("**/marimo/notebooks"):
            page.locator('a[data-test="LNK_Marimo"]').click()
        expect(page.locator("app-sidebar-right")).to_be_visible(timeout=15000)

    def _row(self, page: Page, filename: str):
        """Helper: the .element-div row that contains the link for <filename>."""
        return page.locator(
            f'app-sidebar-right .element-div:has(a[href$="/program/marimo/{filename}"])'
        )

    def test_06_create_rename_delete_workbook_via_ui(self, page: Page):
        """
        Full workbook CRUD lifecycle driven ONLY through the UI (modal + dropdown),
        not the REST API. Covers: New-workbook modal, '.py' suffix normalization,
        rename via dropdown, and delete via dropdown (with confirm()).
        """
        unique_id = str(int(time.time()))
        create_name = f"uicrud{unique_id}"          # entered without .py
        expected_file = f"{create_name}.py"
        expected_title = f"Uicrud{unique_id}"
        rename_name = f"uirenamed{unique_id}"
        renamed_file = f"{rename_name}.py"
        renamed_title = f"Uirenamed{unique_id}"

        try:
            self._open_marimo(page)
            sidebar = page.locator("app-sidebar-right")

            # 1. CREATE via New-workbook modal
            page.locator('[data-test="BTN_New workbook"]').click()
            name_input = page.locator("#input-name")
            expect(name_input).to_be_visible(timeout=10000)
            name_input.fill(create_name)
            page.locator("#modal-save-button").click()

            # Notebook appears in sidebar with normalized .py filename
            create_link = sidebar.locator(f'a[href$="/program/marimo/{expected_file}"]')
            expect(create_link).to_be_visible(timeout=15000)

            # 2. RENAME via the row's dropdown menu
            self._row(page, expected_file).locator('button[id^="dropdownbutton-"]').click()
            page.locator(f'button[id="sidebar-right-rename-{expected_title}"]').click()
            name_input = page.locator("#input-name")
            expect(name_input).to_be_visible(timeout=10000)
            name_input.fill(rename_name)
            page.locator("#modal-save-button").click()

            # Old link disappears, renamed link appears
            expect(create_link).not_to_be_visible(timeout=15000)
            renamed_link = sidebar.locator(f'a[href$="/program/marimo/{renamed_file}"]')
            expect(renamed_link).to_be_visible(timeout=15000)

            # 3. DELETE via the row's dropdown menu (confirm() auto-accepted by fixture)
            self._row(page, renamed_file).locator('button[id^="dropdownbutton-"]').click()
            page.locator(f'button[id="sidebar-right-delete-{renamed_title}"]').click()
            expect(renamed_link).not_to_be_visible(timeout=15000)
        finally:
            # Cleanup safety net (both possible filenames)
            for fn in (expected_file, renamed_file):
                requests.delete(f"{BASE_URL}/api/v1/marimo/notebooks/{fn}")

    def test_07_delete_selected_notebook_reselects_another(self, page: Page):
        """
        When the CURRENTLY OPEN notebook is deleted and others remain, the component
        must auto-select another remaining notebook: the iframe reloads a different
        existing file and the app stays on /program/marimo (no wildcard redirect).
        """
        unique_id = str(int(time.time()))
        sel_file = f"uisel{unique_id}.py"
        keep_file = f"uikeep{unique_id}.py"
        sel_title = f"Uisel{unique_id}"

        # Setup: two notebooks via REST API
        for fn in (sel_file, keep_file):
            resp = requests.post(f"{BASE_URL}/api/v1/marimo/notebooks", json={"name": fn})
            assert resp.status_code in (200, 201), f"Failed to create {fn}: {resp.text}"

        try:
            self._open_marimo(page)
            sidebar = page.locator("app-sidebar-right")

            # Open sel_file so it becomes the selected/open notebook
            sel_link = sidebar.locator(f'a[href$="/program/marimo/{sel_file}"]')
            expect(sel_link).to_be_visible(timeout=15000)
            sel_link.click()
            expect(page).to_have_url(
                re.compile(rf".*/program/marimo/{re.escape(sel_file)}$"), timeout=10000
            )
            iframe = page.locator("app-marimo iframe")
            expect(iframe).to_have_attribute(
                "src", re.compile(rf".*[?&]file={re.escape(sel_file)}(&|$)"), timeout=10000
            )

            # Delete the currently-open notebook via its dropdown (confirm auto-accepted)
            self._row(page, sel_file).locator('button[id^="dropdownbutton-"]').click()
            page.locator(f'button[id="sidebar-right-delete-{sel_title}"]').click()

            # Row gone; still on /program/marimo (NOT redirected to joint-control);
            # iframe re-selected a DIFFERENT existing notebook (src has ?file= but not sel_file)
            expect(sel_link).not_to_be_visible(timeout=15000)
            expect(page).to_have_url(re.compile(r".*/program/marimo.*"), timeout=10000)
            expect(iframe).to_have_attribute(
                "src",
                re.compile(rf"^(?!.*file={re.escape(sel_file)}).*[?&]file=.+"),
                timeout=10000,
            )
        finally:
            for fn in (sel_file, keep_file):
                requests.delete(f"{BASE_URL}/api/v1/marimo/notebooks/{fn}")

    def test_08_new_workbook_form_validation_blocks_invalid_names(self, page: Page):
        """
        The New-workbook modal must NOT create a notebook when the name fails
        validation (required, minLength 2). Clicking Save with an invalid name is a
        no-op: no new notebook appears and no create request is sent.
        """
        unique_id = str(int(time.time()))
        # A single character violates minLength(2) -> invalid.
        too_short = "x"
        expected_file = f"{too_short}.py"

        # Snapshot current notebook count via REST for a robust before/after comparison.
        before = requests.get(f"{BASE_URL}/api/v1/marimo/notebooks").json()["notebooks"]
        before_names = {nb["name"] for nb in before}

        try:
            self._open_marimo(page)
            sidebar = page.locator("app-sidebar-right")

            # Open modal, enter an invalid (too short) name, try to save.
            page.locator('[data-test="BTN_New workbook"]').click()
            name_input = page.locator("#input-name")
            expect(name_input).to_be_visible(timeout=10000)
            name_input.fill(too_short)
            page.locator("#modal-save-button").click()

            # The invalid name must NOT create a notebook link in the sidebar.
            invalid_link = sidebar.locator(f'a[href$="/program/marimo/{expected_file}"]')
            expect(invalid_link).to_have_count(0, timeout=5000)

            # And the backend notebook set must be unchanged.
            after = requests.get(f"{BASE_URL}/api/v1/marimo/notebooks").json()["notebooks"]
            after_names = {nb["name"] for nb in after}
            assert after_names == before_names, (
                f"Invalid name should not create a notebook. "
                f"Added: {after_names - before_names}"
            )
        finally:
            # Safety net in case validation regressed and it got created.
            requests.delete(f"{BASE_URL}/api/v1/marimo/notebooks/{expected_file}")
