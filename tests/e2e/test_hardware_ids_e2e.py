import os
import pytest
import requests
from playwright.sync_api import sync_playwright, expect

ROBOT_URL = os.getenv("PIB_ROBOT_URL", "http://192.168.1.28")
API_URL = os.getenv("PIB_API_URL", "http://192.168.1.28/api")


def _get_chromium_launch_kwargs():
    for path in ["/usr/bin/chromium", "/usr/bin/chromium-browser"]:
        if os.path.exists(path):
            return {"executable_path": path, "headless": True}
    return {"headless": True}


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
                timeout=10,
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
            page.on("dialog", lambda dialog: dialog.accept())

            try:
                # Open Hardware-IDs tab
                page.goto(f"{ROBOT_URL}/system/hardware-ids", wait_until="networkidle")
                page.wait_for_selector("app-hardware-id", timeout=15000)

                # Locate form input for Servo Bricklet 1
                input_1 = page.locator("input[data-test='TXT_Bricklet_UID_1']")
                expect(input_1).to_be_visible(timeout=10000)

                # Step 1: Set initial Bricklet UID
                initial_uid = "E2E001"
                input_1.fill(initial_uid)
                update_btn = page.locator("[data-test='BTN_Update_bricklet_UIDs']")
                expect(update_btn).to_be_enabled(timeout=5000)
                update_btn.click()
                page.wait_for_timeout(2000)

                # Verify input_1 holds initial_uid
                expect(input_1).to_have_value(initial_uid, timeout=10000)

                # Step 2: Click "Export IDs" button and capture exported JSON content
                export_btn = page.locator("[data-test='BTN_Export_Hardware_IDs']")
                expect(export_btn).to_be_visible(timeout=10000)

                with page.expect_response("**/hardware-config/export", timeout=15000) as resp_info:
                    export_btn.click()

                exported_content = resp_info.value.text()
                assert initial_uid in exported_content

                with open(exported_file_path, "w", encoding="utf-8") as f:
                    f.write(exported_content)

                # Step 3: Set different reference UID ("DIFF99")
                diff_uid = "DIFF99"
                input_1.fill(diff_uid)
                update_btn.click()
                page.wait_for_timeout(2000)
                expect(input_1).to_have_value(diff_uid, timeout=10000)

                # Step 4: Click "Import IDs" button
                import_btn = page.locator("[data-test='BTN_Import_Hardware_IDs']")
                expect(import_btn).to_be_visible(timeout=10000)
                import_btn.click()

                # Wait for import modal
                modal = page.locator("#hardware-ids-import-modal")
                expect(modal).to_be_visible(timeout=10000)

                # Upload the exported JSON file via Choose file button
                with page.expect_file_chooser() as fc_info:
                    page.locator("#btn-choose-hardware-import-file").click()
                file_chooser = fc_info.value
                file_chooser.set_files(exported_file_path)

                # Step 5: Wait for import preview to parse and render
                page.wait_for_selector(".import-preview", timeout=10000)

                # Step 6: Click "Confirm import" button
                with page.expect_response("**/hardware-config/import", timeout=15000) as import_resp_info:
                    page.evaluate("() => document.querySelector('[data-test=\"BTN_Import_Hardware_IDs_Confirm\"]').click()")

                assert import_resp_info.value.status == 200

                # Step 7: Verify success alert and that UI input is restored to initial_uid
                success_alert = page.locator(".alert-success")
                expect(success_alert).to_be_visible(timeout=10000)
                page.wait_for_timeout(1000)

                expect(input_1).to_have_value(initial_uid, timeout=10000)

            finally:
                if os.path.exists(exported_file_path):
                    try:
                        os.remove(exported_file_path)
                    except Exception:
                        pass
                context.close()
                browser.close()
