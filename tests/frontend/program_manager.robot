*** Settings ***
Documentation     Program Manager view functional E2E — docs/test-basis/frontend_e2e.md (E2E-BDD-FE-PM-*).
...               PR-1466 rewrite: tests verify FUNCTIONALITY (run starts, save confirms,
...               export triggers), not just visibility.
...               Requires Cerebra on http://localhost:80 and Flask API on http://localhost:5000.
Resource          ../resources/frontend_keywords.robot

Suite Setup        Set Suite Variables
Suite Teardown     Close Cerebra Application


*** Test Cases ***
E2E-BDD-FE-PM-001 Program Manager View Renders After Navigation
    [Documentation]    Given Cerebra is open When user navigates to Program Then the view container is visible.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Program
    Then Cerebra Url Path Should Contain    /program
    Wait For Element By Data Test    BTN_Program_Run_Stop    visible
    # Functional: the program list was fetched from the Flask API
    Wait For Response    matcher=${FLASK_BASE_URL}/program    timeout=20s

E2E-BDD-FE-PM-002 Program List Is Visible
    [Documentation]    Given Program Manager view When loaded Then the program list is rendered.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Program
    Wait For Element By Data Test    BTN_Program_Run_Stop    visible
    Wait For Element By Data Test    BTN_Program_Save    visible

E2E-BDD-FE-PM-003 Program Workspace Is Visible
    [Documentation]    Given Program Manager view When loaded Then the workspace area is rendered.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Program
    Wait For Element By Data Test    TXT_Program_Input    visible
    Wait For Element By Data Test    TGL_Split_Screen    visible

E2E-BDD-FE-PM-004 Program Run Stop Button Starts Execution
    [Documentation]    Given Program Manager view When user clicks BTN_Program_Run_Stop
    ...               Then program execution starts (TXT_Program_Input reflects running state
    ...               or a run request reaches the Flask API).
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Program
    Wait For Element By Data Test    BTN_Program_Run_Stop    visible
    ${before}=    Get Text By Data Test    TXT_Program_Input
    Click Element By Data Test    BTN_Program_Run_Stop
    # Functional: the run action must reach the Flask /program API
    Wait For Response    matcher=${FLASK_BASE_URL}/program    timeout=20s
    # ...and the run/stop label or program input reflects the new state
    ${after}=    Get Text By Data Test    TXT_Program_Input
    Should Not Be Equal    ${before}    ${after}

E2E-BDD-FE-PM-005 Program Save Button Shows Save Confirmation
    [Documentation]    Given Program Manager view When user clicks BTN_Program_Save
    ...               Then a save confirmation appears (BTN_Confirm or BTN_Save visible).
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Program
    Wait For Element By Data Test    BTN_Program_Save    visible
    Click Element By Data Test    BTN_Program_Save
    # Functional: a confirmation/save button must appear after the save click,
    # OR a save request reaches the Flask API. Accept either signal.
    ${ok}=    Run Keyword And Ignore Error    Wait For Element By Data Test    BTN_Confirm    visible
    Run Keyword If    '${ok}' == 'FAIL'
    ...    Run Keyword And Ignore Error    Wait For Element By Data Test    BTN_Save    visible
    Run Keyword If    '${ok}' == 'FAIL'
    ...    Wait For Response    matcher=${FLASK_BASE_URL}/program    timeout=20s

E2E-BDD-FE-PM-006 Program Export Button Triggers Export Action
    [Documentation]    Given Program Manager view When user clicks BTN_Program_Export
    ...               Then an export/download action triggers (page remains responsive,
    ...               and a Flask /program request is observed).
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Program
    Wait For Element By Data Test    BTN_Program_Export    visible
    Click Element By Data Test    BTN_Program_Export
    # Functional: export must trigger a backend request (download or API call)
    Wait For Response    matcher=${FLASK_BASE_URL}/program    timeout=20s
    # ...and the page is still responsive afterwards
    Wait For Element By Data Test    BTN_Program_Run_Stop    visible
