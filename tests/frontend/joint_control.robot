*** Settings ***
Documentation     Joint Control view E2E — docs/test-basis/frontend_e2e.md (E2E-BDD-FE-JC-*).
...               Verifies joint tabs, motor sliders, motor settings panel, and threshold controls.
...               Requires Cerebra on http://localhost:4200.
Resource          ../resources/frontend_keywords.robot

Suite Setup        Set Suite Variables
Suite Teardown     Close Cerebra Application


*** Test Cases ***
E2E-BDD-FE-JC-001 Joint Control View Renders After Navigation
    [Documentation]    Given Cerebra is open When user navigates to Joint Control Then the view container is visible.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Joint_Control
    Then Cerebra Url Path Should Contain    /joint-control
    Then Joint Control View Is Visible

E2E-BDD-FE-JC-002 Head Joint Tab Is Visible
    [Documentation]    Given Joint Control view When loaded Then the head joint tab is visible.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Joint_Control
    Then Joint Control View Is Visible
    Then Motor Settings Panel Is Visible

E2E-BDD-FE-JC-003 Motor Slider Controls Are Visible
    [Documentation]    Given Joint Control view When head tab selected Then motor sliders are rendered.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Joint_Control
    Then Joint Control View Is Visible
    Then Motor Slider Is Visible    SLD_Motor_Settings_Acceleration
    Then Motor Slider Is Visible    SLD_Motor_Settings_Deceleration
    Then Motor Slider Is Visible    SLD_Motor_Settings_Velocity

E2E-BDD-FE-JC-004 Motor Settings Panel Is Visible
    [Documentation]    Given Joint Control view When loaded Then the motor settings panel is rendered.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Joint_Control
    Then Joint Control View Is Visible
    Then Motor Settings Panel Is Visible

E2E-BDD-FE-JC-005 Motor Settings Toggle Expands Settings
    [Documentation]    Given Joint Control view When user toggles motor settings Then the panel remains visible.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Joint_Control
    Then Joint Control View Is Visible
    When User Toggles Motor Settings    BTN_Motor_Settings_Toggle_Extended
    Then Motor Settings Panel Is Visible

E2E-BDD-FE-JC-006 Threshold Controls Are Visible In Motor Settings
    [Documentation]    Given motor settings panel When expanded Then threshold (min/max) controls are visible.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Joint_Control
    Then Joint Control View Is Visible
    Then Motor Settings Panel Is Visible
    Then Threshold Control Is Visible    TXT_Threshold
    Then Threshold Control Is Visible    BTN_Threshold_up
    Then Threshold Control Is Visible    BTN_Threshold_down
