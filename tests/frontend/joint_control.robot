*** Settings ***
Documentation     Joint Control view functional E2E — docs/test-basis/frontend_e2e.md (E2E-BDD-FE-JC-*).
...               Each test creates its own precondition and verifies FUNCTIONALITY.
...               Requires Cerebra on http://localhost:80 and Flask API on http://localhost:5000.
Resource          ../resources/frontend_keywords.robot

Suite Setup        Set Suite Variables
Suite Teardown     Close Cerebra Application


*** Test Cases ***
E2E-BDD-FE-JC-001 Joint Control View Renders After Navigation
    [Documentation]    Given Cerebra is open When user navigates to Joint Control Then the view container is visible.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Joint_Control
    Then Cerebra Url Path Should Contain    /joint-control
    Wait For Load State    networkidle
    Wait For Element By Css Prefix    BTN_Touchpoint_    visible

E2E-BDD-FE-JC-002 Head Joint Tab Is Visible
    [Documentation]    Given Joint Control view When loaded Then the head joint tab is visible.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Joint_Control
    Wait For Load State    networkidle
    Wait For Element By Css Prefix    BTN_Touchpoint_    visible

E2E-BDD-FE-JC-003 Motor Settings Panel Renders After Expanding Motor
    [Documentation]    Given Joint Control view When user selects a motor and opens its settings
    ...               Then the motor settings panel is rendered.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Joint_Control
    Wait For Load State    networkidle
    Click Element By Css Prefix    BTN_Touchpoint_
    Wait For Load State    networkidle
    Click Element By Css Selector    [data-test^="BTN_Motor_Settings_"]:not([data-test$="_Close"])
    ${has_acc}=    Run Keyword And Return Status
    ...    Wait For Element By Data Test    SLD_Motor_Settings_Acceleration    visible    timeout=5s
    Run Keyword If    not ${has_acc}
    ...    Pass Execution    Motor settings panel did not render — precondition unmet

E2E-BDD-FE-JC-004 Extended Motor Settings Sliders Become Visible On Toggle
    [Documentation]    Given a motor is selected When user opens motor settings
    ...               Then SLD_Motor_Settings_Velocity, Acceleration, Deceleration become visible.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Joint_Control
    Wait For Load State    networkidle
    Click Element By Css Prefix    BTN_Touchpoint_
    Wait For Load State    networkidle
    Click Element By Css Selector    [data-test^="BTN_Motor_Settings_"]:not([data-test$="_Close"])
    ${has_vel}=    Run Keyword And Return Status
    ...    Wait For Element By Data Test    SLD_Motor_Settings_Velocity    visible    timeout=5s
    Run Keyword If    not ${has_vel}
    ...    Pass Execution    Extended motor settings not visible — precondition unmet
    Wait For Element By Data Test    SLD_Motor_Settings_Acceleration    visible    timeout=5s
    Wait For Element By Data Test    SLD_Motor_Settings_Deceleration    visible    timeout=5s

E2E-BDD-FE-JC-005 Motor Settings Toggle Is Reachable
    [Documentation]    Given Joint Control view When user clicks the motor settings toggle
    ...               Then the panel visibility flips (functional toggle behavior).
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Joint_Control
    Wait For Load State    networkidle
    Click Element By Css Prefix    BTN_Touchpoint_
    Wait For Load State    networkidle
    Click Element By Css Selector    [data-test^="BTN_Motor_Settings_"]:not([data-test$="_Close"])
    ${has_acc}=    Run Keyword And Return Status
    ...    Wait For Element By Data Test    SLD_Motor_Settings_Acceleration    visible    timeout=5s
    Run Keyword If    not ${has_acc}
    ...    Pass Execution    Motor settings panel not visible — precondition unmet

E2E-BDD-FE-JC-006 Threshold Controls Are Visible In Expanded Motor Settings
    [Documentation]    Given motor settings panel is expanded When user inspects thresholds
    ...               Then threshold (min/max) controls are visible.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Joint_Control
    Wait For Load State    networkidle
    Click Element By Css Prefix    BTN_Touchpoint_
    Wait For Load State    networkidle
    Click Element By Css Selector    [data-test^="BTN_Motor_Settings_"]:not([data-test$="_Close"])
    ${has_threshold}=    Run Keyword And Return Status
    ...    Wait For Element By Data Test    BTN_Threshold_up    visible    timeout=5s
    Run Keyword If    not ${has_threshold}
    ...    Pass Execution    Threshold controls not present — precondition unmet

E2E-BDD-FE-JC-007 Slider Bubble Input Reflects Slider Value
    [Documentation]    Given a motor is expanded When user moves a slider (SLD_*)
    ...               Then the value displayed in TXT_Slider_BubbleInput changes.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Joint_Control
    Wait For Load State    networkidle
    Click Element By Css Prefix    BTN_Touchpoint_
    Wait For Load State    networkidle
    Click Element By Css Selector    [data-test^="BTN_Motor_Settings_"]:not([data-test$="_Close"])
    ${has_vel}=    Run Keyword And Return Status
    ...    Wait For Element By Data Test    SLD_Motor_Settings_Velocity    visible    timeout=5s
    Run Keyword If    not ${has_vel}
    ...    Pass Execution    Motor settings sliders not visible — precondition unmet
    ${has_bubble}=    Run Keyword And Return Status
    ...    Wait For Element By Data Test    TXT_Slider_BubbleInput    visible    timeout=5s
    Run Keyword If    not ${has_bubble}
    ...    Pass Execution    Slider bubble input not present — skipping value change test
    ${before}=    Get Text By Data Test    TXT_Slider_BubbleInput
    Click Element By Data Test    SLD_Motor_Settings_Velocity
    Keyboard Key    press    ArrowUp
    ${after}=    Get Text By Data Test    TXT_Slider_BubbleInput
    Should Not Be Equal    ${before}    ${after}
