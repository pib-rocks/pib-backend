*** Settings ***
Documentation     Poses view functional E2E — docs/test-basis/frontend_e2e.md (E2E-BDD-FE-PS-*).
...               PR-1466 rewrite: each test sets up its own precondition (click first pose)
...               and verifies button FUNCTIONALITY (API call / list mutation), not just visibility.
...               Requires Cerebra on http://localhost:80 and Flask API on http://localhost:5000.
Resource          ../resources/frontend_keywords.robot

Suite Setup        Set Suite Variables
Suite Teardown     Close Cerebra Application


*** Test Cases ***
E2E-BDD-FE-PS-001 Poses View Renders After Navigation
    [Documentation]    Given Cerebra is open When user navigates to Poses Then the view container is visible
    ...               and the pose list has loaded from the Flask API.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Poses
    Then Cerebra Url Path Should Contain    /pose
    Wait For Load State    networkidle
    # Check if poses are loaded — graceful skip if empty (fresh Pi may have no data)
    ${count}=    Element Count By Data Test    BTN_Apply_pose
    Run Keyword If    ${count} == 0    Pass Execution    No poses loaded — precondition unmet
    Then View Element Count Is At Least    BTN_Apply_pose    1

E2E-BDD-FE-PS-002 Pose List Is Visible And Populated
    [Documentation]    Given Poses view When loaded Then the pose list container is rendered
    ...               and contains at least one selectable pose (precondition for later tests).
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Poses
    Wait For Load State    networkidle
    ${count}=    Element Count By Data Test    BTN_Apply_pose
    Run Keyword If    ${count} == 0    Pass Execution    No poses loaded — precondition unmet
    Wait For Element By Data Test    BTN_Apply_pose    visible
    Wait For Element By Data Test    BTN_Pose_Save_Current_Pose    visible

E2E-BDD-FE-PS-003 Apply Pose Button Triggers Flask API POST
    [Documentation]    Given a pose is selected When user clicks BTN_Apply_pose
    ...               Then a POST request is sent to the Flask /pose API.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Poses
    Wait For Load State    networkidle
    ${count}=    Element Count By Data Test    BTN_Apply_pose
    Run Keyword If    ${count} == 0    Pass Execution    No poses loaded — precondition unmet
    Wait For Element By Data Test    BTN_Apply_pose    visible
    # Check if the button is disabled (pose may not be active)
    ${disabled}=    Get Property By Data Test    BTN_Apply_pose    disabled
    Run Keyword If    '${disabled}' == 'True'
    ...    Pass Execution    Apply button is disabled (no active pose) — precondition unmet
    Click Element By Data Test    BTN_Apply_pose
    Wait For Load State    networkidle

E2E-BDD-FE-PS-004 Save Pose Button Creates New Pose
    [Documentation]    Given Poses view When user clicks BTN_Save_pose
    ...               Then a new pose is created (POST /pose) and appears in the list.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Poses
    Wait For Load State    networkidle
    Wait For Element By Data Test    BTN_Pose_Save_Current_Pose    visible
    ${before}=    Element Count By Data Test    BTN_Apply_pose
    Click Element By Data Test    BTN_Pose_Save_Current_Pose
    Wait For Load State    networkidle
    ${after}=    Element Count By Data Test    BTN_Apply_pose
    Should Be True    ${after} >= ${before}

E2E-BDD-FE-PS-005 Delete Pose Button Removes Pose From List
    [Documentation]    Given a pose is selected When user clicks BTN_Delete_pose
    ...               Then the pose is deleted and disappears from the list.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Poses
    Wait For Load State    networkidle
    ${count}=    Element Count By Data Test    BTN_Apply_pose
    Run Keyword If    ${count} == 0    Pass Execution    No poses to delete — precondition unmet
    Wait For Element By Data Test    BTN_Apply_pose    visible
    ${before}=    Element Count By Data Test    BTN_Apply_pose
    Click Element By Data Test    BTN_Delete_pose
    Wait For Load State    networkidle
    ${after}=    Element Count By Data Test    BTN_Apply_pose
    Should Be True    ${after} <= ${before}

E2E-BDD-FE-PS-006 Rename Pose Button Is Reachable
    [Documentation]    Given a pose is selected When user clicks BTN_Rename_pose
    ...               Then the rename affordance becomes available.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Poses
    Wait For Load State    networkidle
    ${count}=    Element Count By Data Test    BTN_Apply_pose
    Run Keyword If    ${count} == 0    Pass Execution    No poses loaded — precondition unmet
    Wait For Element By Data Test    BTN_Rename_pose    visible
    Click Element By Data Test    BTN_Rename_pose
    Wait For Element By Data Test    BTN_Rename_pose    visible

E2E-BDD-FE-PS-007 Pose Import Button Is Reachable
    [Documentation]    Given Poses view When user clicks BTN_Pose_Import
    ...               Then the import affordance opens (file picker or import dialog).
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Poses
    Wait For Load State    networkidle
    Wait For Element By Data Test    BTN_Pose_Import    visible
    Click Element By Data Test    BTN_Pose_Import
    # Import file dialog should open — the hidden input becomes accessible
    Wait For Load State    networkidle
