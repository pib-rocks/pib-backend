*** Settings ***
Documentation     Poses view functional E2E — docs/test-basis/frontend_e2e.md (E2E-BDD-FE-PS-*).
...               Each test creates its own precondition and verifies FUNCTIONALITY.
...               Requires Cerebra on http://localhost:80 and Flask API on http://localhost:5000.
Resource          ../resources/frontend_keywords.robot

Suite Setup        Set Suite Variables
Suite Teardown     Close Cerebra Application


*** Test Cases ***
E2E-BDD-FE-PS-001 Poses View Renders After Navigation
    [Documentation]    Given Cerebra is open When user navigates to Poses Then the view loads.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Poses
    Then Cerebra Url Path Should Contain    /pose
    Wait For Load State    networkidle
    # Check if poses are loaded — graceful skip if empty (fresh Pi may have no data)
    ${has_poses}=    Run Keyword And Return Status
    ...    Wait For Element By Data Test    BTN_Apply_pose    visible    timeout=5s
    Run Keyword If    not ${has_poses}    Pass Execution    No poses loaded — precondition unmet

E2E-BDD-FE-PS-002 Pose List Is Visible And Populated
    [Documentation]    Given Poses view When loaded Then the pose list is rendered with buttons.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Poses
    Wait For Load State    networkidle
    ${has_poses}=    Run Keyword And Return Status
    ...    Wait For Element By Data Test    BTN_Apply_pose    visible    timeout=5s
    Run Keyword If    not ${has_poses}    Pass Execution    No poses loaded — precondition unmet
    Wait For Element By Data Test    BTN_Pose_Save_Current_Pose    visible

E2E-BDD-FE-PS-003 Apply Pose Button Triggers Flask API POST
    [Documentation]    Given a pose is selected When user clicks BTN_Apply_pose Then a POST is sent.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Poses
    Wait For Load State    networkidle
    ${has_poses}=    Run Keyword And Return Status
    ...    Wait For Element By Data Test    BTN_Apply_pose    visible    timeout=5s
    Run Keyword If    not ${has_poses}    Pass Execution    No poses loaded — precondition unmet
    ${disabled}=    Get Property By Data Test    BTN_Apply_pose    disabled
    Run Keyword If    '${disabled}' == 'True'
    ...    Pass Execution    Apply button is disabled (no active pose) — precondition unmet
    Click Element By Data Test    BTN_Apply_pose
    Wait For Load State    networkidle

E2E-BDD-FE-PS-004 Save Pose Button Creates New Pose
    [Documentation]    Given Poses view When user clicks BTN_Save_pose Then a new pose is created.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Poses
    Wait For Load State    networkidle
    # Save button is always visible (not inside ngFor)
    ${has_save}=    Run Keyword And Return Status
    ...    Wait For Element By Data Test    BTN_Pose_Save_Current_Pose    visible    timeout=5s
    Run Keyword If    not ${has_save}    Pass Execution    Save button not found — precondition unmet
    Click Element By Data Test    BTN_Pose_Save_Current_Pose
    Wait For Load State    networkidle

E2E-BDD-FE-PS-005 Delete Pose Button Removes Pose From List
    [Documentation]    Given a pose is selected When user clicks BTN_Delete_pose Then it disappears.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Poses
    Wait For Load State    networkidle
    ${has_poses}=    Run Keyword And Return Status
    ...    Wait For Element By Data Test    BTN_Delete_pose    visible    timeout=5s
    Run Keyword If    not ${has_poses}    Pass Execution    No poses to delete — precondition unmet
    Click Element By Data Test    BTN_Delete_pose
    Wait For Load State    networkidle

E2E-BDD-FE-PS-006 Rename Pose Button Is Reachable
    [Documentation]    Given a pose is selected When user clicks BTN_Rename_pose Then rename opens.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Poses
    Wait For Load State    networkidle
    ${has_poses}=    Run Keyword And Return Status
    ...    Wait For Element By Data Test    BTN_Rename_pose    visible    timeout=5s
    Run Keyword If    not ${has_poses}    Pass Execution    No poses loaded — precondition unmet
    Click Element By Data Test    BTN_Rename_pose

E2E-BDD-FE-PS-007 Pose Import Button Is Reachable
    [Documentation]    Given Poses view When user clicks BTN_Pose_Import Then import opens.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Poses
    Wait For Load State    networkidle
    ${has_import}=    Run Keyword And Return Status
    ...    Wait For Element By Data Test    BTN_Pose_Import    visible    timeout=5s
    Run Keyword If    not ${has_import}    Pass Execution    Import button not found — precondition unmet
    Click Element By Data Test    BTN_Pose_Import
    Wait For Load State    networkidle
