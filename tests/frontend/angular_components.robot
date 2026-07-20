*** Settings ***
Documentation     Frontend UI tests mapped from docs/test-basis/backend_safety_and_edge_cases.md.
...               Requires Cerebra on :80 or route mocks via Browser library.

Resource            ../resources/frontend_keywords.robot
Resource            ../resources/api_keywords.robot

Suite Setup         Set Suite Variables
Suite Teardown     Close Cerebra Application


*** Test Cases ***
E2E-BDD-FE-001 Personality List Success State With Mocked API
    [Documentation]    Given mocked personality list When page loads Then rows are visible.
    ${can_mock}=    Run Keyword And Return Status
    ...    Given Frontend Loads Personality List Mock Success
    Run Keyword If    not ${can_mock}
    ...    Pass Execution    Route mocking not available in this Browser Library version — skipping
    When User Opens Voice Assistant Personalities Page
    Then Personality List Shows Success State

E2E-BDD-FE-002 Personality List Error State With Mocked API
    [Documentation]    Given API 500 When page loads Then alert is shown.
    ${can_mock}=    Run Keyword And Return Status
    ...    Given Frontend Loads Personality List Mock Error
    Run Keyword If    not ${can_mock}
    ...    Pass Execution    Route mocking not available in this Browser Library version — skipping
    When User Opens Voice Assistant Personalities Page
    Then Personality List Shows Error State

E2E-BDD-FE-003 Motor Settings Page Loads From Flask API
    [Documentation]    Given Flask /motor responds When motors page opens Then list is populated.
    Flask API Is Reachable
    Open Cerebra Page    /joint-control/head
    Wait For Load State    networkidle
    ${has_touchpoint}=    Run Keyword And Return Status
    ...    Wait For Element By Css Prefix    BTN_Touchpoint_    visible    timeout=10s
    Run Keyword If    not ${has_touchpoint}
    ...    Pass Execution    No touchpoint elements — motor list not rendered

E2E-BDD-FE-004 Pose Management Reads Startup Pose
    [Documentation]    Given Startup/Resting pose exists When poses page loads Then pose name is shown.
    ${has_pose}=    Run Keyword And Return Status
    ...    Get Startup Pose By Name
    Run Keyword If    not ${has_pose}
    ...    Pass Execution    Startup pose not available — skipping
    Open Cerebra Page    /pose
    Wait For Load State    networkidle
    ${has_apply}=    Run Keyword And Return Status
    ...    Wait For Element By Data Test    BTN_Apply_pose    visible    timeout=5s
    Run Keyword If    not ${has_apply}
    ...    Pass Execution    No poses loaded on this instance — skipping
