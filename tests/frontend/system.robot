*** Settings ***
Documentation     System view E2E — docs/test-basis/frontend_e2e.md (E2E-BDD-FE-SYS-*).
...               Verifies hardware IDs, bricklet update, smart connect, and relay controls.
...               Requires Cerebra on http://localhost:4200.
Resource          ../resources/frontend_keywords.robot

Suite Setup        Set Suite Variables
Suite Teardown     Close Cerebra Application


*** Test Cases ***
E2E-BDD-FE-SYS-001 System View Renders After Navigation
    [Documentation]    Given Cerebra is open When user navigates to System Then the view container is visible.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_System
    Then Cerebra Url Path Should Contain    /system
    Then System View Is Visible

E2E-BDD-FE-SYS-002 Hardware IDs Section Is Visible
    [Documentation]    Given System view When loaded Then the hardware IDs section is rendered.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_System
    Then System View Is Visible
    Then Hardware Id Section Is Visible

E2E-BDD-FE-SYS-003 Update Bricklet UIDs Button Is Visible
    [Documentation]    Given System view When loaded Then the Update Bricklet UIDs button is present.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_System
    Then System View Is Visible
    Then Bricklet Update Button Is Visible

E2E-BDD-FE-SYS-004 Smart Connect Button Is Visible
    [Documentation]    Given System view When loaded Then the Smart Connect button is present.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_System
    Then System View Is Visible
    Then Smart Connect Button Is Visible

E2E-BDD-FE-SYS-005 Relay Control Toggle Is Visible
    [Documentation]    Given System view When loaded Then the relay control toggle is present.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_System
    Then System View Is Visible
    Then Relay Control Is Visible    TGL_Solid_State_Relay
