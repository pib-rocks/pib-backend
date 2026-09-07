import os
import time

import pytest
import requests
from playwright.sync_api import sync_playwright, expect

ROBOT_URL = os.getenv("PIB_ROBOT_URL", "http://192.168.1.28")
API_URL = os.getenv("PIB_API_URL", "http://192.168.1.28/api")

# Timeouts are generous by default because the suite runs on the Pi itself, where
# Angular bootstrap and Bricklet round-trips are slow under load. Override per host.
UI_TIMEOUT_MS = int(os.getenv("PIB_E2E_UI_TIMEOUT_MS", "30000"))
NAV_TIMEOUT_MS = int(os.getenv("PIB_E2E_NAV_TIMEOUT_MS", "60000"))
REQUEST_TIMEOUT_S = 10
BACKEND_SETTLE_TIMEOUT_S = int(os.getenv("PIB_E2E_BACKEND_SETTLE_TIMEOUT_S", "30"))


def _get_chromium_launch_kwargs():
    for path in ["/usr/bin/chromium", "/usr/bin/chromium-browser"]:
        if os.path.exists(path):
            return {"executable_path": path, "headless": True}
    return {"headless": True}


def _bricklet_uids():
    response = requests.get(f"{API_URL}/bricklet", timeout=REQUEST_TIMEOUT_S)
    response.raise_for_status()
    return [entry.get("uid") for entry in response.json().get("bricklets", [])]


def _wait_for_persisted_uid(uid, timeout_s=BACKEND_SETTLE_TIMEOUT_S):
    """Block until the backend reports `uid` for one of the bricklets.

    Clicking "Update" fires one async PUT per Bricklet; polling the API is what makes
    the following export/import steps deterministic instead of racing a fixed sleep.
    """
    deadline = time.monotonic() + timeout_s
    last_seen = None
    while time.monotonic() < deadline:
        try:
            last_seen = _bricklet_uids()
            if uid in last_seen:
                return
        except (requests.RequestException, ValueError):
            pass
        time.sleep(0.5)
    raise AssertionError(
        f"Backend did not persist Bricklet UID {uid!r} within {timeout_s}s "
        f"(last read from /bricklet: {last_seen})"
    )


class TestHardwareIDsE2E:

    def test_hardware_ids_export_import_full_lifecycle(self):
        """
        Full E2E UI test verifying the Hardware-IDs lifecycle:
        1. Navigate to /system/hardware-ids with SmartConnect credentials in browser context.
        2. Set initial Bricklet UIDs via UI input (e.g. TXT_Bricklet_UID_1="E2E001") and click Update.
        3. Click "Export IDs" button and capture the exported JSON payload.
        4. Set different reference Bricklet UIDs (e.g. TXT_Bricklet_UID_1="DIFF99") and click Update.
        5. Click "Import IDs" button, upload the initial JSON backup via FileChooser.
        6. Wait for import preview, then click "Confirm import".
        7. Verify that the UIDs displayed in the UI form inputs are restored to the initially set UIDs ("E2E001").
        """
        # 1. Ensure SmartConnect is active on backend
        try:
            requests.post(
                f"{API_URL}/system/smart-connect",
                json={"token": "12345678", "password": "12345678"},
                timeout=REQUEST_TIMEOUT_S,
            )
        except Exception:
            pass

        exported_file_path = os.path.expanduser("~/pib_e2e_hardware_import_test.json")

        with sync_playwright() as p:
            browser = p.chromium.launch(**_get_chromium_launch_kwargs())
            context = browser.new_context(viewport={"width": 1400, "height": 900})
            context.add_init_script("""
                localStorage.setItem('token', '12345678');
                localStorage.setItem('password', '12345678');
            """)
            page = context.new_page()
            page.set_default_timeout(UI_TIMEOUT_MS)
            page.set_default_navigation_timeout(NAV_TIMEOUT_MS)
            page.on("dialog", lambda dialog: dialog.accept())

            try:
                # Open Hardware-IDs tab. "networkidle" is unusable here: Cerebra keeps a
                # rosbridge socket and polls, so the network never goes idle on the Pi.
                page.goto(
                    f"{ROBOT_URL}/system/hardware-ids", wait_until="domcontentloaded"
                )
                page.wait_for_selector(
                    "app-hardware-id", state="visible", timeout=NAV_TIMEOUT_MS
                )

                # Locate form input for Servo Bricklet 1
                input_1 = page.locator("input[data-test='TXT_Bricklet_UID_1']")
                expect(input_1).to_be_visible(timeout=UI_TIMEOUT_MS)
                expect(input_1).to_be_enabled(timeout=UI_TIMEOUT_MS)

                # Step 1: Set initial Bricklet UID
                initial_uid = "E2E001"
                input_1.fill(initial_uid)
                update_btn = page.locator("[data-test='BTN_Update_bricklet_UIDs']")
                expect(update_btn).to_be_enabled(timeout=UI_TIMEOUT_MS)
                update_btn.click()

                # Verify input_1 holds initial_uid and the write reached the backend
                expect(input_1).to_have_value(initial_uid, timeout=UI_TIMEOUT_MS)
                _wait_for_persisted_uid(initial_uid)

                # Step 2: Click "Export IDs" button and capture exported JSON content
                export_btn = page.locator("[data-test='BTN_Export_Hardware_IDs']")
                expect(export_btn).to_be_visible(timeout=UI_TIMEOUT_MS)
                expect(export_btn).to_be_enabled(timeout=UI_TIMEOUT_MS)

                with page.expect_response(
                    "**/hardware-config/export", timeout=UI_TIMEOUT_MS
                ) as resp_info:
                    export_btn.click()

                exported_content = resp_info.value.text()
                assert initial_uid in exported_content

                with open(exported_file_path, "w", encoding="utf-8") as f:
                    f.write(exported_content)

                # Step 3: Set different reference UID ("DIFF99")
                diff_uid = "DIFF99"
                input_1.fill(diff_uid)
                expect(update_btn).to_be_enabled(timeout=UI_TIMEOUT_MS)
                update_btn.click()
                expect(input_1).to_have_value(diff_uid, timeout=UI_TIMEOUT_MS)
                _wait_for_persisted_uid(diff_uid)

                # Step 4: Click "Import IDs" button
                import_btn = page.locator("[data-test='BTN_Import_Hardware_IDs']")
                expect(import_btn).to_be_visible(timeout=UI_TIMEOUT_MS)
                expect(import_btn).to_be_enabled(timeout=UI_TIMEOUT_MS)
                import_btn.click()

                # Wait for import modal
                modal = page.locator("#hardware-ids-import-modal")
                expect(modal).to_be_visible(timeout=UI_TIMEOUT_MS)

                # Upload the exported JSON file via Choose file button
                choose_btn = page.locator("#btn-choose-hardware-import-file")
                expect(choose_btn).to_be_visible(timeout=UI_TIMEOUT_MS)
                expect(choose_btn).to_be_enabled(timeout=UI_TIMEOUT_MS)
                with page.expect_file_chooser(timeout=UI_TIMEOUT_MS) as fc_info:
                    choose_btn.click()
                file_chooser = fc_info.value
                file_chooser.set_files(exported_file_path)

                # Step 5: Wait for import preview to parse and render
                page.wait_for_selector(
                    ".import-preview", state="visible", timeout=UI_TIMEOUT_MS
                )

                # Step 6: Click "Confirm import" button. The click goes through
                # evaluate() to stay immune to the modal's overlay/animation, so the
                # element has to be waited for explicitly first.
                confirm_btn = page.locator(
                    "[data-test='BTN_Import_Hardware_IDs_Confirm']"
                )
                expect(confirm_btn).to_be_visible(timeout=UI_TIMEOUT_MS)
                expect(confirm_btn).to_be_enabled(timeout=UI_TIMEOUT_MS)

                with page.expect_response(
                    "**/hardware-config/import", timeout=UI_TIMEOUT_MS
                ) as import_resp_info:
                    page.evaluate(
                        "() => document.querySelector('[data-test=\"BTN_Import_Hardware_IDs_Confirm\"]').click()"
                    )

                assert import_resp_info.value.status == 200

                # Step 7: Verify success alert and that UI input is restored to initial_uid
                # ".alert-success" is a shared Cerebra style, so scope to the first
                # match: a second alert elsewhere in the page would otherwise make
                # expect() fail on a strict-mode violation instead of on behaviour.
                success_alert = page.locator(".alert-success").first
                expect(success_alert).to_be_visible(timeout=UI_TIMEOUT_MS)

                _wait_for_persisted_uid(initial_uid)
                expect(input_1).to_have_value(initial_uid, timeout=UI_TIMEOUT_MS)

            finally:
                if os.path.exists(exported_file_path):
                    try:
                        os.remove(exported_file_path)
                    except Exception:
                        pass
                context.close()
                browser.close()
