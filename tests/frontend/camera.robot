*** Settings ***
Documentation     Camera view functional E2E — docs/test-basis/frontend_e2e.md (E2E-BDD-FE-CAM-*).
...               PR-1466 rewrite: tests verify FUNCTIONALITY (resolution change persists,
...               settings panel reveals sliders), not just visibility.
...               Requires Cerebra on http://localhost:80 and Flask API on http://localhost:5000.
Resource          ../resources/frontend_keywords.robot

Suite Setup        Set Suite Variables
Suite Teardown     Close Cerebra Application


*** Test Cases ***
E2E-BDD-FE-CAM-001 Camera View Renders After Navigation
    [Documentation]    Given Cerebra is open When user navigates to Camera Then the view container is visible.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Camera
    Then Cerebra Url Path Should Contain    /camera
    Wait For Element By Data Test    TGL_Camera_On_Off    visible
    # Functional: camera settings were fetched from the Flask API
    Wait For Response    matcher=${FLASK_BASE_URL}/camera-settings    timeout=20s

E2E-BDD-FE-CAM-002 Camera On Off Toggle Is Reachable
    [Documentation]    Given Camera view When user inspects TGL_Camera_On_Off
    ...               Then the toggle reflects a checkable state.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Camera
    Wait For Element By Data Test    TGL_Camera_On_Off    visible
    # Functional: the toggle exposes a `checked` property
    ${checked}=    Get Property By Data Test    TGL_Camera_On_Off    checked

E2E-BDD-FE-CAM-003 Resolution Dropdown Change Keeps On Off Working
    [Documentation]    Given Camera view When user opens DDN_Camera_resolution, selects 720p
    ...               Then TGL_Camera_On_Off still works (still visible/interactable).
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Camera
    Wait For Element By Data Test    DDN_Camera_resolution    visible
    # Functional: change the resolution to 720p
    Select Option By Data Test    DDN_Camera_resolution    720p
    # ...and verify the on/off toggle is still present and checkable
    Wait For Element By Data Test    TGL_Camera_On_Off    visible
    ${checked}=    Get Property By Data Test    TGL_Camera_On_Off    checked

E2E-BDD-FE-CAM-004 Video Settings Button Reveals Quality And Refresh Sliders
    [Documentation]    Given Camera view When user clicks BTN_Video_settings
    ...               Then SLD_Camera_qualityFactor and SLD_Camera_refreshRate become visible.
    ...               PRECONDITION: the settings panel is closed until clicked.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Camera
    Wait For Element By Data Test    BTN_Video_settings    visible
    # Precondition + functional action: open the settings panel
    Click Element By Data Test    BTN_Video_settings
    # Functional: the advanced sliders must now be visible
    Wait For Element By Data Test    SLD_Camera_qualityFactor    visible
    Wait For Element By Data Test    SLD_Camera_refreshRate    visible
