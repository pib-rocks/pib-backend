"""End-to-end check of the connected-Bricklets table on a live robot (PR-1854).

Read-only. This test reads the enumeration endpoint and the rendered table and
compares the two; it writes nothing to the robot, so unlike the Hardware-IDs test
beside it no state has to be restored afterwards.

Where the expectations come from: the endpoint's own answer, not a list of serial
numbers compiled into this file. Which Bricklets hang on a given robot is a
hardware fact, and a test that pins them fails on every other robot without
proving anything extra. What has to hold is that the table shows exactly what the
hardware reports, in the presentation the contract promises:

  * one row per device the endpoint answered with,
  * the port cell carries the printed letter in upper case,
  * a device without a parent board is named as the carrier and its cell is never
    empty - it has no port, and an empty cell reads as a broken row,
  * a Bricklet's tooltip names the board its port sits on.
"""

import contextlib
import os

import requests
from playwright.sync_api import sync_playwright

ROBOT_URL = os.getenv("PIB_ROBOT_URL", "http://localhost")
API_URL = os.getenv("PIB_API_URL", "http://localhost/api")
UI_TIMEOUT_MS = int(os.getenv("PIB_E2E_UI_TIMEOUT_MS", "30000"))
NAV_TIMEOUT_MS = int(os.getenv("PIB_E2E_NAV_TIMEOUT_MS", "60000"))
REQUEST_TIMEOUT_S = 10

# The wording the component uses for a device that has no port of its own.
CARRIER_CELL = "\u2014 carrier board"


def _chromium_executable():
    """The browser binary to drive, or None for the Playwright default.

    PIB_E2E_CHROMIUM overrides the search, which is what makes this file runnable
    from a workstation: there the system names point at a snap wrapper Playwright
    cannot use, while on the robot /usr/bin/chromium is the real package. Unset,
    the order below is the one the sibling tests use.
    """
    override = os.getenv("PIB_E2E_CHROMIUM")
    if override and os.path.exists(override):
        return override
    for path in ["/usr/bin/chromium", "/usr/bin/chromium-browser"]:
        if os.path.exists(path):
            return path
    return None


def _connected_bricklets():
    response = requests.get(f"{API_URL}/bricklet/connected", timeout=REQUEST_TIMEOUT_S)
    response.raise_for_status()
    return response.json().get("bricklets", [])


def _expected_port_cell(device):
    """What the table must show in the port column for this device."""
    return device["port"].upper() if device["port"] else CARRIER_CELL


def _carriers(devices):
    """Devices the enumeration reports without a parent board."""
    return [device for device in devices if device["parentUid"] in ("", "0")]


@contextlib.contextmanager
def _hardware_ids_page():
    """The Hardware-IDs tab, open in a real browser.

    "networkidle" is unusable here: Cerebra holds a rosbridge socket and polls, so
    the network never goes idle on the robot.
    """
    executable = _chromium_executable()
    with sync_playwright() as playwright:
        if executable:
            browser = playwright.chromium.launch(
                headless=True, executable_path=executable
            )
        else:
            browser = playwright.chromium.launch(headless=True)
        try:
            # The robot's own screen size, so this looks at what the display shows.
            page = browser.new_page(viewport={"width": 1024, "height": 600})
            page.set_default_timeout(UI_TIMEOUT_MS)
            page.set_default_navigation_timeout(NAV_TIMEOUT_MS)
            page.goto(f"{ROBOT_URL}/system/hardware-ids", wait_until="domcontentloaded")
            # The endpoint spends a few seconds in the enumeration before the table appears.
            page.wait_for_selector(
                '[data-test="TBL_Connected_Bricklets"]', timeout=NAV_TIMEOUT_MS
            )
            yield page
        finally:
            browser.close()


def _rows(page):
    """The rendered rows as {uid: {name, port, port_title}}."""
    rows = {}
    for row in page.query_selector_all('[data-test^="ROW_Connected_Bricklet_"]'):
        port_cell = row.query_selector('[data-test="TXT_Connected_Bricklet_Port"]')
        uid = (
            row.query_selector('[data-test="TXT_Connected_Bricklet_UID"]')
            .inner_text()
            .strip()
        )
        rows[uid] = {
            "name": row.query_selector('[data-test="TXT_Connected_Bricklet_Name"]')
            .inner_text()
            .strip(),
            "port": port_cell.inner_text().strip(),
            "port_title": port_cell.get_attribute("title") or "",
        }
    return rows


def test_the_table_shows_exactly_the_devices_the_hardware_reports():
    devices = _connected_bricklets()
    assert devices, (
        f"The endpoint at {API_URL}/bricklet/connected reported no devices, so this test "
        "would pass without checking anything. Is a robot reachable?"
    )

    with _hardware_ids_page() as page:
        rows = _rows(page)
        assert len(rows) == len(devices), (
            f"The table has {len(rows)} rows for {len(devices)} reported devices: "
            f"rows {sorted(rows)}, devices {sorted(d['uid'] for d in devices)}"
        )
        for device in devices:
            row = rows.get(device["uid"])
            assert (
                row is not None
            ), f"{device['uid']} ({device['name']}) is missing from the table"
            assert (
                row["name"] == device["name"]
            ), f"{device['uid']}: table says {row['name']!r}, endpoint says {device['name']!r}"
            assert row["port"] == _expected_port_cell(device), (
                f"{device['uid']}: port cell is {row['port']!r}, expected "
                f"{_expected_port_cell(device)!r} for port {device['port']!r}"
            )


def test_the_carrier_is_named_and_no_port_cell_is_ever_empty():
    devices = _connected_bricklets()
    carriers = _carriers(devices)

    with _hardware_ids_page() as page:
        rows = _rows(page)
        for uid, row in rows.items():
            assert (
                row["port"] != ""
            ), f"{uid}: the port cell is empty, which reads as a broken row"
        for carrier in carriers:
            row = rows.get(carrier["uid"])
            assert (
                row is not None
            ), f"the carrier {carrier['uid']} is missing from the table"
            assert row["port"] == CARRIER_CELL, (
                f"{carrier['uid']} is reported without a parent board, so its position "
                f"({carrier['port']!r}) is not a socket; the cell says {row['port']!r}"
            )
            assert "carrier board" in row["port_title"].lower(), (
                f"{carrier['uid']}: the tooltip {row['port_title']!r} does not explain that "
                "this is the carrier"
            )


def test_a_bricklet_names_the_board_its_port_sits_on():
    devices = _connected_bricklets()
    names_by_uid = {device["uid"]: device["name"] for device in devices}
    bricklets = [device for device in devices if device["parentUid"] not in ("", "0")]
    assert (
        bricklets
    ), "No Bricklet with a parent board was reported, so there is nothing to check"

    with _hardware_ids_page() as page:
        rows = _rows(page)
        for device in bricklets:
            title = rows[device["uid"]]["port_title"]
            assert title.startswith(
                f"Port {device['port'].upper()} on "
            ), f"{device['uid']}: tooltip {title!r} does not name the socket"
            parent_name = names_by_uid.get(device["parentUid"])
            if parent_name:
                assert parent_name in title, (
                    f"{device['uid']}: tooltip {title!r} does not name the board it sits "
                    f"on ({parent_name})"
                )


def test_the_error_branch_and_the_table_are_never_shown_together():
    """A failed read must not look like an empty table, and vice versa."""
    with _hardware_ids_page() as page:
        assert (
            page.query_selector('[data-test="TXT_Connected_Bricklets_Error"]') is None
        ), "The table is rendered and the error message at the same time"
        assert (
            page.query_selector('[data-test="BTN_Refresh_Connected_Bricklets"]')
            is not None
        ), "The refresh button is missing"
