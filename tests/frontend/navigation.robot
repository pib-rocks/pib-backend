*** Settings ***
Documentation     Sidebar navigation E2E — docs/test-basis/frontend_e2e.md (E2E-BDD-FE-NAV-*).
...               Verifies all 6 sidebar nav items navigate to the correct Angular routes.
...               Requires Cerebra on http://localhost:4200.
Resource          ../resources/frontend_keywords.robot

Suite Setup        Set Suite Variables
Suite Teardown     Close Cerebra Application


*** Test Cases ***
E2E-BDD-FE-NAV-001 Sidebar Joint Control Link Navigates To Joint Control Head View
    [Documentation]    Given Cerebra is open When user clicks LNK_Joint_Control Then URL is /joint-control/head.
    Given User Opens Cerebra And Navigates To    LNK_Joint_Control    /joint-control
    Then View Element Is Visible    BTN_Motor_Settings_Toggle_Extended

E2E-BDD-FE-NAV-002 Sidebar Poses Link Navigates To Pose View
    [Documentation]    Given Cerebra is open When user clicks LNK_Poses Then URL is /pose.
    Given User Opens Cerebra And Navigates To    LNK_Poses    /pose
    Then View Element Is Visible    BTN_Apply_pose

E2E-BDD-FE-NAV-003 Sidebar Camera Link Navigates To Camera View
    [Documentation]    Given Cerebra is open When user clicks LNK_Camera Then URL is /camera.
    Given User Opens Cerebra And Navigates To    LNK_Camera    /camera
    Then View Element Is Visible    TGL_Camera_On_Off

E2E-BDD-FE-NAV-004 Sidebar Voice Assistant Link Navigates To Voice Assistant View
    [Documentation]    Given Cerebra is open When user clicks LNK_Voice_Assistant Then URL is /voice-assistant.
    Given User Opens Cerebra And Navigates To    LNK_Voice_Assistant    /voice-assistant
    Then View Element Is Visible    BTN_Add_Personality

E2E-BDD-FE-NAV-005 Sidebar Program Link Navigates To Program View
    [Documentation]    Given Cerebra is open When user clicks LNK_Program Then URL is /program.
    Given User Opens Cerebra And Navigates To    LNK_Program    /program
    Then View Element Is Visible    BTN_Program_Run_Stop

E2E-BDD-FE-NAV-006 Sidebar System Link Navigates To System View
    [Documentation]    Given Cerebra is open When user clicks LNK_System Then URL is /system.
    Given User Opens Cerebra And Navigates To    LNK_System    /system
    Then View Element Is Visible    BTN_Update_bricklet_UIDs
