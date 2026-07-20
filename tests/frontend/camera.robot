*** Settings ***
Documentation     Camera view E2E — docs/test-basis/frontend_e2e.md (E2E-BDD-FE-CAM-*).
...               Verifies the video stream element, resolution dropdown, and settings button.
...               Requires Cerebra on http://localhost:4200.
Resource          ../resources/frontend_keywords.robot

Suite Setup        Set Suite Variables
Suite Teardown     Close Cerebra Application


*** Test Cases ***
E2E-BDD-FE-CAM-001 Camera View Renders After Navigation
    [Documentation]    Given Cerebra is open When user navigates to Camera Then the view container is visible.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Camera
    Then Cerebra Url Path Should Contain    /camera
    Then Camera View Is Visible

E2E-BDD-FE-CAM-002 Camera Video Stream Element Is Visible
    [Documentation]    Given Camera view When loaded Then the video stream element is rendered.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Camera
    Then Camera View Is Visible
    Then Camera Video Stream Is Visible

E2E-BDD-FE-CAM-003 Camera Resolution Dropdown Is Visible
    [Documentation]    Given Camera view When loaded Then the resolution dropdown selector is present.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Camera
    Then Camera View Is Visible
    Then Camera Resolution Dropdown Is Visible

E2E-BDD-FE-CAM-004 Camera Settings Button Is Visible
    [Documentation]    Given Camera view When loaded Then the Video Settings button is present.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Camera
    Then Camera View Is Visible
    Then Camera Settings Button Is Visible
