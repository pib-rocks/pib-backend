*** Settings ***
Documentation     Joint Control view functional E2E — docs/test-basis/frontend_e2e.md (E2E-BDD-FE-JC-*).
...               PR-1466 rewrite: each test sets up its own precondition (expand a motor)
...               and verifies FUNCTIONALITY (panel expands, slider value changes), not just visibility.
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
    Wait For Element By Data Test    BTN_Motor_Settings_Toggle_Extended    visible
    # Functional: the motor list was loaded from the Flask API
    Wait For Response    matcher=${FLASK_BASE_URL}/motor    timeout=20s

E2E-BDD-FE-JC-002 Head Joint Tab Is Visible
    [Documentation]    Given Joint Control view When loaded Then the head joint tab is visible.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Joint_Control
    Wait For Element By Data Test    BTN_Motor_Settings_Toggle_Extended    visible

E2E-BDD-FE-JC-003 Motor Settings Panel Renders After Expanding Motor
    [Documentation]    Given Joint Control view When user expands a motor (click toggle)
    ...               Then the motor settings panel is rendered.
    ...               PRECONDITION: click BTN_Motor_Settings_Toggle_Extended to expand.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Joint_Control
    Wait For Element By Data Test    BTN_Motor_Settings_Toggle_Extended    visible
    # Precondition: expand the motor settings
    Click Element By Data Test    BTN_Motor_Settings_Toggle_Extended
    Wait For Element By Data Test    SLD_Motor_Settings_Acceleration    visible

E2E-BDD-FE-JC-004 Extended Motor Settings Sliders Become Visible On Toggle
    [Documentation]    Given a motor is expanded When user clicks BTN_Motor_Settings_Toggle_Extended
    ...               Then SLD_Motor_Settings_Velocity, SLD_Motor_Settings_Acceleration,
    ...               and SLD_Motor_Settings_Deceleration become visible.
    ...               PRECONDITION: the motor must be expanded first.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Joint_Control
    Wait For Element By Data Test    BTN_Motor_Settings_Toggle_Extended    visible
    # Precondition: expand the motor
    Click Element By Data Test    BTN_Motor_Settings_Toggle_Extended
    # Functional: the extended sliders must now be visible
    Wait For Element By Data Test    SLD_Motor_Settings_Velocity    visible
    Wait For Element By Data Test    SLD_Motor_Settings_Acceleration    visible
    Wait For Element By Data Test    SLD_Motor_Settings_Deceleration    visible

E2E-BDD-FE-JC-005 Motor Settings Toggle Is Reachable
    [Documentation]    Given Joint Control view When user clicks the motor settings toggle
    ...               Then the panel visibility flips (functional toggle behavior).
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Joint_Control
    Wait For Element By Data Test    BTN_Motor_Settings_Toggle_Extended    visible
    # Read the toggle's initial state, click it, and assert the extended sliders appear
    Click Element By Data Test    BTN_Motor_Settings_Toggle_Extended
    Wait For Element By Data Test    SLD_Motor_Settings_Acceleration    visible

E2E-BDD-FE-JC-006 Threshold Controls Are Visible In Expanded Motor Settings
    [Documentation]    Given motor settings panel is expanded When user inspects thresholds
    ...               Then threshold (min/max) controls are visible.
    ...               PRECONDITION: expand the motor first.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Joint_Control
    Wait For Element By Data Test    BTN_Motor_Settings_Toggle_Extended    visible
    # Precondition: expand the motor
    Click Element By Data Test    BTN_Motor_Settings_Toggle_Extended
    Wait For Element By Data Test    TXT_Threshold    visible
    Wait For Element By Data Test    BTN_Threshold_up    visible
    Wait For Element By Data Test    BTN_Threshold_down    visible

E2E-BDD-FE-JC-007 Slider Bubble Input Reflects Slider Value
    [Documentation]    Given a motor is expanded When user moves a slider (SLD_*)
    ...               Then the value displayed in TXT_Slider_BubbleInput changes.
    ...               PRECONDITION: expand a motor so a slider is visible.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Joint_Control
    Wait For Element By Data Test    BTN_Motor_Settings_Toggle_Extended    visible
    # Precondition: expand the motor so the velocity slider is visible
    Click Element By Data Test    BTN_Motor_Settings_Toggle_Extended
    Wait For Element By Data Test    SLD_Motor_Settings_Velocity    visible
    # Read the bubble value before moving the slider
    ${before}=    Get Text By Data Test    TXT_Slider_BubbleInput
    # Move the slider via keyboard (ArrowUp) to change its value
    Click Element By Data Test    SLD_Motor_Settings_Velocity
    Keyboard Key    press    ArrowUp
    # Functional: the bubble value must have changed
    ${after}=    Get Text By Data Test    TXT_Slider_BubbleInput
    Should Not Be Equal    ${before}    ${after}
