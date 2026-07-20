*** Settings ***
Documentation     Program Manager view E2E — docs/test-basis/frontend_e2e.md (E2E-BDD-FE-PM-*).
...               Verifies program list, workspace, and run/stop/save/export buttons.
...               Requires Cerebra on http://localhost:4200.
Resource          ../resources/frontend_keywords.robot

Suite Setup        Set Suite Variables
Suite Teardown     Close Cerebra Application


*** Test Cases ***
E2E-BDD-FE-PM-001 Program Manager View Renders After Navigation
    [Documentation]    Given Cerebra is open When user navigates to Program Then the view container is visible.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Program
    Then Cerebra Url Path Should Be    /program
    Then Program Manager View Is Visible

E2E-BDD-FE-PM-002 Program List Is Visible
    [Documentation]    Given Program Manager view When loaded Then the program list is rendered.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Program
    Then Program Manager View Is Visible
    Then Program List Is Visible

E2E-BDD-FE-PM-003 Program Workspace Is Visible
    [Documentation]    Given Program Manager view When loaded Then the Blockly workspace is rendered.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Program
    Then Program Manager View Is Visible
    Then Program Workspace Is Visible

E2E-BDD-FE-PM-004 Program Run Stop Button Is Visible
    [Documentation]    Given Program Manager view When loaded Then the Run/Stop button is present.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Program
    Then Program Manager View Is Visible
    Then Program Button Is Visible    BTN_Program_Run_Stop

E2E-BDD-FE-PM-005 Program Save Button Is Visible
    [Documentation]    Given Program Manager view When loaded Then the Save button is present.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Program
    Then Program Manager View Is Visible
    Then Program Button Is Visible    BTN_Program_Save

E2E-BDD-FE-PM-006 Program Export Button Is Visible
    [Documentation]    Given Program Manager view When loaded Then the Export button is present.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Program
    Then Program Manager View Is Visible
    Then Program Button Is Visible    BTN_Program_Export
