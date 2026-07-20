*** Settings ***
Documentation     Poses view E2E — docs/test-basis/frontend_e2e.md (E2E-BDD-FE-PS-*).
...               Verifies the pose list and apply/save/delete/rename/import buttons are present.
...               Requires Cerebra on http://localhost:4200.
Resource          ../resources/frontend_keywords.robot

Suite Setup        Set Suite Variables
Suite Teardown     Close Cerebra Application


*** Test Cases ***
E2E-BDD-FE-PS-001 Poses View Renders After Navigation
    [Documentation]    Given Cerebra is open When user navigates to Poses Then the view container is visible.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Poses
    Then Cerebra Url Path Should Contain    /pose
    Then Poses View Is Visible

E2E-BDD-FE-PS-002 Pose List Is Visible
    [Documentation]    Given Poses view When loaded Then the pose list container is rendered.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Poses
    Then Poses View Is Visible
    Then Pose List Is Visible

E2E-BDD-FE-PS-003 Apply Pose Button Is Visible
    [Documentation]    Given Poses view When loaded Then the Apply Pose button is present.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Poses
    Then Poses View Is Visible
    Then Pose Button Is Visible    BTN_Apply_pose

E2E-BDD-FE-PS-004 Save Pose Button Is Visible
    [Documentation]    Given Poses view When loaded Then the Save Pose button is present.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Poses
    Then Poses View Is Visible
    Then Pose Button Is Visible    BTN_Save_pose

E2E-BDD-FE-PS-005 Delete Pose Button Is Visible
    [Documentation]    Given Poses view When loaded Then the Delete Pose button is present.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Poses
    Then Poses View Is Visible
    Then Pose Button Is Visible    BTN_Delete_pose

E2E-BDD-FE-PS-006 Rename Pose Button Is Visible
    [Documentation]    Given Poses view When loaded Then the Rename Pose button is present.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Poses
    Then Poses View Is Visible
    Then Pose Button Is Visible    BTN_Rename_pose

E2E-BDD-FE-PS-007 Pose Import Button Is Visible
    [Documentation]    Given Poses view When loaded Then the Pose Import button is present.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Poses
    Then Poses View Is Visible
    Then Pose Button Is Visible    BTN_Pose_Import
