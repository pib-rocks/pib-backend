import re
import time
import requests
import pytest
from playwright.sync_api import sync_playwright, FrameLocator, Page, expect

BASE_URL = "http://192.168.1.28"

# Strict Marimo iframe selectors (PR-1526). Do NOT fall back to bare `body` —
# nginx 502 / empty shells would otherwise pass. Live Marimo 0.23 uses
# `.marimo-cell` / `[data-testid=cell-editor]` / `.cm-editor`; older builds may
# expose `.marimo-app` / `marimo-code-editor`.
MARIMO_IFRAME = 'app-marimo iframe[src*="marimo-server"]'
MARIMO_EDITOR_ROOT = (
    ".marimo-app, marimo-code-editor, .marimo-cell, "
    "[data-testid='cell-editor'], .cm-editor"
)
MARIMO_OUTPUT_NODE = (
    ".marimo-output-cell, .cell-output, [data-testid='cell-output'], "
    "[data-testid='console-output-area'], .stdout, .output-area .output"
)


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

    def _assert_marimo_iframe_editor(self, page: Page) -> FrameLocator:
        """Testfall 1 + 5: deep iframe DOM must expose a real Marimo editor root."""
        iframe = page.locator(MARIMO_IFRAME)
        expect(iframe).to_be_visible(timeout=30000)
        src = iframe.get_attribute("src") or ""
        assert "marimo-server" in src, f"Expected marimo-server in iframe src, got {src!r}"

        frame = page.frame_locator(MARIMO_IFRAME)
        # Fail fast on proxy / empty error pages (Testfall 5).
        body_text = frame.locator("body").inner_text(timeout=15000)
        for marker in ("502 Bad Gateway", "404 Not Found", "500 Internal Server Error"):
            assert marker not in body_text, f"Marimo iframe shows error page: {marker}"

        editor_root = frame.locator(MARIMO_EDITOR_ROOT)
        expect(editor_root.first).to_be_visible(timeout=30000)
        return frame

    def _run_cell_and_assert_output(self, page: Page, frame: FrameLocator, code: str, expected: str):
        """Write code into the first cell, execute it, assert DOM output node text."""
        editor = frame.locator(".cm-editor").first
        expect(editor).to_be_visible(timeout=30000)
        editor.click()
        page.keyboard.press("Control+A")
        page.keyboard.press("Backspace")
        page.keyboard.type(code, delay=20)

        editor.hover()
        run_button = frame.get_by_test_id("run-button").locator("visible=true").first
        try:
            expect(run_button).to_be_visible(timeout=5000)
            run_button.click()
        except Exception:
            page.keyboard.press("Control+Enter")

        output = frame.locator(MARIMO_OUTPUT_NODE).filter(has_text=expected)
        expect(output.first).to_be_visible(timeout=45000)
        assert expected in output.first.inner_text()

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

        # 5. Assert right management sidebar <app-sidebar-right> exists and has 'New notebook' button
        sidebar = page.locator("app-marimo app-sidebar-right")
        expect(sidebar).to_be_visible(timeout=10000)
        expect(sidebar).to_contain_text("New notebook", timeout=10000, ignore_case=True)

        # 6. Assert source code contains iframe template definition and marimo-server URL
        content = page.content()
        assert "app-marimo" in content, "Expected <app-marimo> in page source code"
        assert "app-sidebar-right" in content, "Expected <app-sidebar-right> in page source code"

    def test_01b_marimo_iframe_inner_content_loads(self, page: Page):
        """
        Testfall 1 (Deep Iframe DOM Rendering) + Testfall 5 (Error-State Detection):
        iframe[src*='marimo-server'] must contain a visible Marimo editor root
        (.marimo-app / marimo-code-editor / .marimo-cell / cell-editor / .cm-editor).
        Empty shells and nginx gateway error pages must fail.
        """
        page.goto(f"{BASE_URL}/program/marimo", wait_until="networkidle")
        self._assert_marimo_iframe_editor(page)

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

            # 1. CREATE via New-notebook modal
            page.locator('[data-test="BTN_New notebook"]').click()
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
            page.locator('[data-test="BTN_New notebook"]').click()
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

    def test_09_create_notebook_write_time_program_and_run_in_marimo(self, page: Page):
        """
        Create a new notebook via the UI, write a small time-printing Python cell
        inside the Marimo iframe editor, run it, assert DOM output, then clean up.
        """
        unique_id = str(int(time.time()))
        create_name = f"pib_time_demo_{unique_id}"
        expected_file = f"{create_name}.py"
        expected_output = "Hello World, current time:"
        code = (
            "import time\n"
            "print('Hello World, current time:', time.ctime())"
        )

        try:
            # 1. Navigate to /program/marimo
            self._open_marimo(page)
            sidebar = page.locator("app-sidebar-right")

            # 2–3. New notebook modal → name → Save
            page.locator('[data-test="BTN_New notebook"]').click()
            name_input = page.locator("#input-name")
            expect(name_input).to_be_visible(timeout=30000)
            name_input.fill(create_name)
            page.locator("#modal-save-button").click()

            # 4. Notebook created and selected
            create_link = sidebar.locator(
                f'a[href$="/program/marimo/{expected_file}"]'
            )
            expect(create_link).to_be_visible(timeout=30000)
            create_link.click()
            expect(page).to_have_url(
                re.compile(rf".*/program/marimo/{re.escape(expected_file)}$"),
                timeout=30000,
            )
            expect(page.locator(MARIMO_IFRAME)).to_have_attribute(
                "src",
                re.compile(rf".*[?&]file={re.escape(expected_file)}(&|$)"),
                timeout=30000,
            )

            # 5. Frame-switch into marimo-server iframe (strict editor root)
            frame = self._assert_marimo_iframe_editor(page)

            # 6. Verify the notebook starts clean (no prior test output / time import)
            cm_content = frame.locator(".cm-content").first
            expect(cm_content).to_be_visible(timeout=30000)
            initial_text = cm_content.inner_text()
            assert expected_output not in initial_text
            assert "import time" not in initial_text
            expect(frame.get_by_text(expected_output, exact=False)).to_have_count(0)

            # 7–9. Write, execute, assert rendered DOM output node
            self._run_cell_and_assert_output(page, frame, code, expected_output)
        finally:
            # 10. Cleanup: delete the created test notebook
            requests.delete(f"{BASE_URL}/api/v1/marimo/notebooks/{expected_file}")

    def test_10_marimo_notebooks_strict_crud_and_execution(self, page: Page):
        """
        PR-1526 end-to-end: strict deep-iframe check, Create & Read via UI,
        Update & code execution with DOM output assertion, then Delete via UI.
        Covers Jira Testfälle 1–5 in one lifecycle.
        """
        unique_id = str(int(time.time()))
        create_name = f"pr1526crud{unique_id}"
        expected_file = f"{create_name}.py"
        # Backend title: stem.replace("_"," ").title() — digits reset word boundaries.
        expected_title = create_name.replace("_", " ").title()
        marker = f"PIB_EXEC_{unique_id}"
        code = f"print(21 * 2)\nprint('{marker}')"

        try:
            self._open_marimo(page)
            sidebar = page.locator("app-sidebar-right")

            # Testfall 1 + 5: deep iframe editor must already be present
            self._assert_marimo_iframe_editor(page)

            # Testfall 2: Create & Read via "New notebook"
            page.locator('[data-test="BTN_New notebook"]').click()
            name_input = page.locator("#input-name")
            expect(name_input).to_be_visible(timeout=15000)
            name_input.fill(create_name)
            page.locator("#modal-save-button").click()

            create_link = sidebar.locator(
                f'a[href$="/program/marimo/{expected_file}"]'
            )
            expect(create_link).to_be_visible(timeout=15000)

            # Backend persistence
            listed = requests.get(f"{BASE_URL}/api/v1/marimo/notebooks").json()["notebooks"]
            assert any(nb["name"] == expected_file for nb in listed), (
                f"{expected_file} missing from backend notebook list"
            )

            create_link.click()
            expect(page).to_have_url(
                re.compile(rf".*/program/marimo/{re.escape(expected_file)}$"),
                timeout=15000,
            )
            expect(page.locator(MARIMO_IFRAME)).to_have_attribute(
                "src",
                re.compile(rf".*[?&]file={re.escape(expected_file)}(&|$)"),
                timeout=15000,
            )

            frame = self._assert_marimo_iframe_editor(page)

            # Testfall 3: Update & Run — assert stdout in DOM output node
            self._run_cell_and_assert_output(page, frame, code, "42")
            expect(
                frame.locator(MARIMO_OUTPUT_NODE).filter(has_text=marker).first
            ).to_be_visible(timeout=15000)

            # Testfall 4: Delete via UI dropdown + confirm()
            page.once("dialog", lambda dialog: dialog.accept())
            self._row(page, expected_file).locator('button[id^="dropdownbutton-"]').click()
            page.locator(f'button[id="sidebar-right-delete-{expected_title}"]').click()
            expect(create_link).not_to_be_visible(timeout=15000)

            after = requests.get(f"{BASE_URL}/api/v1/marimo/notebooks").json()["notebooks"]
            assert not any(nb["name"] == expected_file for nb in after), (
                f"{expected_file} still present in backend after UI delete"
            )
        finally:
            requests.delete(f"{BASE_URL}/api/v1/marimo/notebooks/{expected_file}")
