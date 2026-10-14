import json
import os
import time

import pytest
import requests
from playwright.sync_api import sync_playwright, expect

from robot_address import api_url, robot_base_url

ROBOT_URL = robot_base_url()
API_URL = api_url()

# Timeouts are generous by default because the suite runs on the Pi itself, where
# Angular bootstrap and Bricklet round-trips are slow under load. Override per host.
UI_TIMEOUT_MS = int(os.getenv("PIB_E2E_UI_TIMEOUT_MS", "30000"))
NAV_TIMEOUT_MS = int(os.getenv("PIB_E2E_NAV_TIMEOUT_MS", "60000"))
REQUEST_TIMEOUT_S = 10
BACKEND_SETTLE_TIMEOUT_S = int(os.getenv("PIB_E2E_BACKEND_SETTLE_TIMEOUT_S", "30"))

SERVO_DEVICE_TYPE = "Servo Bricklet"


def _get_chromium_launch_kwargs():
    for path in ["/usr/bin/chromium", "/usr/bin/chromium-browser"]:
        if os.path.exists(path):
            return {"executable_path": path, "headless": True}
    return {"headless": True}


def _hardware_config():
    """The store the Hardware-IDs page reads and writes.

    Not /bricklet: that legacy endpoint answers with empty UIDs even when the page
    shows a fully configured robot, so a snapshot taken there restores nothing.
    """
    response = requests.get(
        f"{API_URL}/system/hardware-config/export", timeout=REQUEST_TIMEOUT_S
    )
    response.raise_for_status()
    return response.json()


def _import_hardware_config(config):
    response = requests.post(
        f"{API_URL}/system/hardware-config/import",
        json=config,
        timeout=REQUEST_TIMEOUT_S,
    )
    response.raise_for_status()
    return response.json()


def _controllers(config):
    return [c for c in config.get("controllers", []) if c.get("number") is not None]


def _address_of(config, number):
    for controller in _controllers(config):
        if controller["number"] == number:
            return controller.get("address") or ""
    raise AssertionError(f"No controller {number} in hardware config: {config}")


def _positions(config):
    """{number: address} - what the restore asserts against."""
    return {c["number"]: (c.get("address") or "") for c in _controllers(config)}


def _connected_devices():
    """The Bricklets brickd actually sees, as the page's dropdowns get them."""
    response = requests.get(f"{API_URL}/bricklet/connected", timeout=REQUEST_TIMEOUT_S)
    response.raise_for_status()
    return response.json().get("bricklets", [])


def _restore_hardware_config(snapshot):
    """Put the robot's own hardware config back and assert it landed that way.

    This test runs against a live robot whose motors container reads these rows on
    start, so a leftover test UID keeps the machine broken long after the run ended
    (PR-1797).
    """
    target = _positions(snapshot)
    _import_hardware_config(snapshot)

    for number, address in sorted(target.items()):
        _wait_for_stored_uid(number, address)

    restored = _positions(_hardware_config())
    mismatched = {
        number: (target.get(number), restored.get(number))
        for number in target
        if target.get(number) != restored.get(number)
    }
    assert not mismatched, (
        f"Robot hardware config was not restored: {mismatched} "
        f"(expected addresses {target}, got {restored})"
    )


def _wait_for_stored_uid(number, uid, timeout_s=BACKEND_SETTLE_TIMEOUT_S):
    """Block until the backend reports `uid` for controller `number`.

    Clicking "Update" fires async writes; polling the API is what makes the following
    export/import steps deterministic instead of racing a fixed sleep. Read per number
    and not "is the UID somewhere in the config": an empty address is a legal stored
    value, so a membership check passes for every cleared controller and proves nothing.
    """
    deadline = time.monotonic() + timeout_s
    last_seen = None
    while time.monotonic() < deadline:
        try:
            last_seen = _positions(_hardware_config())
            if last_seen.get(number) == uid:
                return
        except (requests.RequestException, ValueError):
            pass
        time.sleep(0.5)
    raise AssertionError(
        f"Backend did not store address {uid!r} for controller {number} within "
        f"{timeout_s}s (last read: {last_seen})"
    )


def _option_values(select_locator):
    """The <option> values of a <select>.

    Mapping the select's own .value would return just its current value - one entry,
    which is what made an earlier version of this test report a dropdown as missing
    its options while the page rendered every one of them.
    """
    return select_locator.evaluate("el => Array.from(el.options).map(o => o.value)")


class TestHardwareIDsE2E:

    def test_hardware_ids_export_import_full_lifecycle(self):
        """
        Full E2E UI test verifying the Hardware-IDs lifecycle.

        The UID fields are dropdowns of the Bricklets brickd actually sees (PR-1959),
        so the test can no longer type arbitrary UIDs - it selects detected devices.
        The lifecycle it covers:

        1. Open /system/hardware-ids with SmartConnect credentials in the browser context.
        2. Assert the UID dropdown offers this robot's detected devices plus "not
           configured", and that the connected-Bricklets table lists them.
        3. Swap two detected Servo Bricklets between their slots in one save - a real
           change that never leaves a duplicate assignment - and wait until the backend
           stored both.
        4. Export the IDs and assert the exported JSON carries the swapped addresses.
        5. Clear the first slot ("- not configured -") and click Update - the write path
           for the empty value that used to come back as 400.
        6. Import the exported JSON back through the modal and confirm it.
        7. Assert the addresses are what the export said, in the UI and in the backend.

        The robot's own hardware config is snapshotted before the first write and
        restored afterwards, including when the test fails.
        """
        # 1. Ensure SmartConnect is active on backend
        try:
            requests.post(
                f"{API_URL}/system/smart-connect",
                json={"token": "12345678"},
                timeout=REQUEST_TIMEOUT_S,
            )
        except Exception:
            pass

        exported_file_path = os.path.expanduser("~/pib_e2e_hardware_import_test.json")

        # Snapshot the robot's own config before anything is written, so the finally
        # block can put it back.
        original_config = _hardware_config()

        # Detected Servo Bricklets fill the slots; two configured ones get swapped below.
        detected_servos = sorted(
            (
                device
                for device in _connected_devices()
                if SERVO_DEVICE_TYPE.lower() in (device.get("name") or "").lower()
            ),
            key=lambda device: device.get("port") or "",
        )
        assert len(detected_servos) >= 2, (
            "Need at least two detected Servo Bricklets for the swap step, "
            f"/bricklet/connected reported: {detected_servos}"
        )

        configured = [
            (controller["number"], controller.get("address") or "")
            for controller in _controllers(original_config)
            if SERVO_DEVICE_TYPE in (controller.get("deviceType") or "")
        ]
        configured = [(number, address) for number, address in configured if address]
        assert (
            len(configured) >= 2
        ), f"Need two configured Servo Bricklet controllers to swap, config has: {configured}"
        slot_a, uid_a = configured[0]
        slot_b, uid_b = configured[1]

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

                # The UID field of a controller is a <select> since PR-1959, not an input.
                def select_for(number):
                    return page.locator(
                        f"select[data-test='TXT_Bricklet_UID_{number}']"
                    )

                select_a = select_for(slot_a)
                expect(select_a).to_be_visible(timeout=UI_TIMEOUT_MS)
                expect(select_a).to_be_enabled(timeout=UI_TIMEOUT_MS)

                # Step 2: the connected Bricklets are listed, and the dropdown carries
                # the detected devices plus the empty choice. Refresh first: the list is
                # fetched on interaction, and this test may be the first thing that
                # touches the page after a container restart.
                refresh_btn = page.locator(
                    "[data-test='BTN_Refresh_Connected_Bricklets']"
                )
                if refresh_btn.count():
                    refresh_btn.click()
                page.wait_for_selector(
                    f"[data-test='ROW_Connected_Bricklet_{uid_a}']",
                    state="visible",
                    timeout=UI_TIMEOUT_MS,
                )

                option_values = _option_values(select_a)
                assert "" in option_values, (
                    f"Dropdown for controller {slot_a} has no 'not configured' option: "
                    f"{option_values}"
                )
                for device in detected_servos:
                    assert device["uid"] in option_values, (
                        f"Detected Servo Bricklet {device['uid']} (port {device.get('port')}) "
                        f"is missing from the dropdown of controller {slot_a}: {option_values}"
                    )

                update_btn = page.locator("[data-test='BTN_Update_bricklet_UIDs']")
                expect(update_btn).to_be_enabled(timeout=UI_TIMEOUT_MS)

                # Step 3: swap the two Servo Bricklets and save once, so no intermediate
                # state has one UID assigned to two slots.
                select_b = select_for(slot_b)
                select_a.select_option(uid_b)
                select_b.select_option(uid_a)
                update_btn.click()

                expect(select_a).to_have_value(uid_b, timeout=UI_TIMEOUT_MS)
                expect(select_b).to_have_value(uid_a, timeout=UI_TIMEOUT_MS)
                _wait_for_stored_uid(slot_a, uid_b)
                _wait_for_stored_uid(slot_b, uid_a)

                # Step 4: click "Export IDs" and capture the exported JSON content
                export_btn = page.locator("[data-test='BTN_Export_Hardware_IDs']")
                expect(export_btn).to_be_visible(timeout=UI_TIMEOUT_MS)
                expect(export_btn).to_be_enabled(timeout=UI_TIMEOUT_MS)

                with page.expect_response(
                    "**/hardware-config/export", timeout=UI_TIMEOUT_MS
                ) as resp_info:
                    export_btn.click()

                exported_content = resp_info.value.text()
                exported_config = json.loads(exported_content)
                assert _address_of(exported_config, slot_a) == uid_b, (
                    "Exported hardware config does not carry the address that was just "
                    f"saved for controller {slot_a}: "
                    f"{_address_of(exported_config, slot_a)!r}"
                )
                assert _address_of(exported_config, slot_b) == uid_a

                with open(exported_file_path, "w", encoding="utf-8") as f:
                    f.write(exported_content)

                # Step 5: clear the first slot - exercises the empty-value write path
                # that PR-1959 fixed (the backend accepts "", rejects null).
                select_a.select_option("")
                expect(update_btn).to_be_enabled(timeout=UI_TIMEOUT_MS)
                update_btn.click()
                expect(select_a).to_have_value("", timeout=UI_TIMEOUT_MS)
                _wait_for_stored_uid(slot_a, "")

                # Step 6: click "Import IDs" button
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

                # Step 7: Wait for import preview to parse and render
                page.wait_for_selector(
                    ".import-preview", state="visible", timeout=UI_TIMEOUT_MS
                )

                # Step 8: Click "Confirm import". The click goes through evaluate() to
                # stay immune to the modal's overlay/animation, so the element has to be
                # waited for explicitly first.
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

                # Step 9: Verify success alert and that the imported addresses are back,
                # in the form and in the backend.
                # ".alert-success" is a shared Cerebra style, so scope to the first
                # match: a second alert elsewhere in the page would otherwise make
                # expect() fail on a strict-mode violation instead of on behaviour.
                success_alert = page.locator(".alert-success").first
                expect(success_alert).to_be_visible(timeout=UI_TIMEOUT_MS)

                _wait_for_stored_uid(slot_a, uid_b)
                expect(select_a).to_have_value(uid_b, timeout=UI_TIMEOUT_MS)

            finally:
                try:
                    if os.path.exists(exported_file_path):
                        try:
                            os.remove(exported_file_path)
                        except Exception:
                            pass
                    context.close()
                    browser.close()
                finally:
                    # Last, so a failing browser teardown cannot skip it: the test
                    # addresses must never survive the run on a live robot (PR-1797).
                    _restore_hardware_config(original_config)

        assert not os.path.exists(exported_file_path)
