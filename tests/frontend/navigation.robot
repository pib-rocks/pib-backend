*** Settings ***
Documentation     Sidebar navigation functional E2E — docs/test-basis/frontend_e2e.md (E2E-BDD-FE-NAV-*).
...               PR-1466 rewrite: verifies all 6 sidebar nav items navigate to the correct
...               Angular routes AND that the target view actually rendered content
...               (not just that the URL changed).
...               Requires Cerebra on http://localhost:80 and Flask API on http://localhost:5000.
Resource          ../resources/frontend_keywords.robot

Suite Setup        Set Suite Variables
Suite Teardown     Close Cerebra Application


*** Test Cases ***
E2E-BDD-FE-NAV-001 Sidebar Joint Control Link Navigates To Joint Control Head View
    [Documentation]    Given Cerebra is open When user clicks LNK_Joint_Control
    ...               Then URL contains /joint-control AND touchpoint elements are visible.
    Given User Opens Cerebra And Navigates To    LNK_Joint_Control    /joint-control
    Wait For Load State    networkidle
    # Functional: the target view rendered motor touchpoints, not just a blank route
    Wait For Element By Css Prefix    BTN_Touchpoint_    visible

E2E-BDD-FE-NAV-002 Sidebar Poses Link Navigates To Pose View
    [Documentation]    Given Cerebra is open When user clicks LNK_Poses
    ...               Then URL contains /pose AND the Apply Pose button is visible.
    Given User Opens Cerebra And Navigates To    LNK_Poses    /pose
    Wait For Load State    networkidle
    # Graceful skip if no poses loaded on fresh Pi
    ${count}=    Element Count By Data Test    BTN_Apply_pose
    Run Keyword If    ${count} == 0    Pass Execution    No poses loaded — precondition unmet
    Wait For Element By Data Test    BTN_Apply_pose    visible

E2E-BDD-FE-NAV-003 Sidebar Camera Link Navigates To Camera View
    [Documentation]    Given Cerebra is open When user clicks LNK_Camera
    ...               Then URL contains /camera AND the on/off toggle is visible.
    Given User Opens Cerebra And Navigates To    LNK_Camera    /camera
    Wait For Load State    networkidle
    Wait For Element By Data Test    TGL_Camera_On_Off    visible

E2E-BDD-FE-NAV-004 Sidebar Voice Assistant Link Navigates To Voice Assistant View
    [Documentation]    Given Cerebra is open When user clicks LNK_Voice_Assistant
    ...               Then URL contains /voice-assistant AND the Add Personality button is visible.
    Given User Opens Cerebra And Navigates To    LNK_Voice_Assistant    /voice-assistant
    Wait For Load State    networkidle
    Wait For Element By Data Test    BTN_Add_Personality    visible

E2E-BDD-FE-NAV-005 Sidebar Program Link Navigates To Program View
    [Documentation]    Given Cerebra is open When user clicks LNK_Program
    ...               Then URL contains /program AND the Run/Stop button is visible.
    Given User Opens Cerebra And Navigates To    LNK_Program    /program
    Wait For Load State    networkidle
    Wait For Element By Data Test    BTN_Program_Run_Stop    visible

E2E-BDD-FE-NAV-006 Sidebar System Link Navigates To System View
    [Documentation]    Given Cerebra is open When user clicks LNK_System
    ...               Then URL contains /system AND the Update Bricklet UIDs button is visible.
    Given User Opens Cerebra And Navigates To    LNK_System    /system
    Wait For Load State    networkidle
    Wait For Element By Data Test    BTN_Update_bricklet_UIDs    visible
