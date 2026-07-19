*** Settings ***
Documentation     Frontend UI tests mapped from docs/test-basis/backend_safety_and_edge_cases.md.
...               Requires Cerebra on :4200 or route mocks via Browser library.

Resource            ../resources/frontend_keywords.robot
Resource            ../resources/api_keywords.robot

Suite Setup         Set Suite Variables
Suite Teardown      Close Cerebra Application


*** Test Cases ***
E2E-BDD-FE-001 Personality List Success State With Mocked API
    [Documentation]    Given mocked personality list When page loads Then rows are visible.
    Given Frontend Loads Personality List Mock Success
    When User Opens Voice Assistant Personalities Page
    Then Personality List Shows Success State

E2E-BDD-FE-002 Personality List Error State With Mocked API
    [Documentation]    Given API 500 When page loads Then alert is shown.
    Given Frontend Loads Personality List Mock Error
    When User Opens Voice Assistant Personalities Page
    Then Personality List Shows Error State

E2E-BDD-FE-003 Motor Settings Page Loads From Flask API
    [Documentation]    Given Flask /motor responds When motors page opens Then list is populated.
    Flask API Is Reachable
    Open Cerebra Page    /settings/motors
    Wait For Load State    networkidle
    Wait For Elements State    css=table, .motor-list, [data-testid="motor-list"]    visible    timeout=20s

E2E-BDD-FE-004 Pose Management Reads Startup Pose
    [Documentation]    Given Startup/Resting pose exists When poses page loads Then pose name is shown.
    ${pose}=    Get Startup Pose By Name
    Should Be Equal    ${pose}[name]    Startup/Resting
    Should Be Equal    ${pose}[deletable]    ${False}
    Open Cerebra Page    /settings/poses
    Wait For Load State    networkidle
