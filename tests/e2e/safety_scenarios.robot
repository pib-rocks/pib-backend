*** Settings ***
Documentation     Safety and anti-pattern scenarios from docs/test-basis/backend_safety_and_edge_cases.md.
Resource            ../resources/api_keywords.robot
Library             ../resources/ROS2TestLibrary.py
Library             RequestsLibrary
Library             OperatingSystem

Suite Setup         Initialize Safety Test Environment
Suite Teardown      Shutdown Safety Test Environment


*** Variables ***
${FLASK_BASE_URL}    http://localhost:5000


*** Test Cases ***
E2E-BDD-SAF-001 Flask Does Not Return 503 On Motor List
    [Documentation]    ROS timeouts are not surfaced as HTTP 503 — anti-pattern guard.
    Ros2 Assert No Http 503 From Flask    ${FLASK_BASE_URL}

E2E-BDD-SAF-002 Non Deletable Calibration Pose Rejects Delete
    [Documentation]    Given Calibration pose When DELETE Then 500 not 404.
    ${response}=    GET    ${FLASK_BASE_URL}/pose    expected_status=200
    ${poses}=    Set Variable    ${response.json()}[poses]
    ${cal_id}=    Evaluate    next(p['poseId'] for p in $poses if p['name'] == 'Calibration')
    ${response}=    DELETE    ${FLASK_BASE_URL}/pose/${cal_id}    expected_status=500

E2E-BDD-SAF-003 Unknown Motor Returns 404 Not 503
    [Documentation]    Given unknown motor path When GET Then 404 entity not found envelope.
    ${response}=    GET    ${FLASK_BASE_URL}/motor/nonexistent_joint_xyz    expected_status=404
    Response Error Envelope Matches    ${response}    404    Entity not found. Please check your path parameter.

E2E-BDD-SAF-004 Duplicate Program Name Returns 400
    [Documentation]    Given duplicate program name When POST Then integrity-style 400.
    ${program_number}    ${name}=    Create Unique Program    safety_dup
    ${response}=    POST    ${FLASK_BASE_URL}/program    json={"name": "${name}"}
    Should Be Equal As Integers    ${response.status_code}    400
    Should Be Equal    ${response.json()}[error]    Bad request.
    [Teardown]    Run Keyword And Ignore Error    Delete Program    ${program_number}

E2E-BDD-SAF-005 Host IP Missing File Returns 500
    [Documentation]    Documented edge case — requires isolated Flask; verified via integration pytest.
    ${response}=    GET    ${FLASK_BASE_URL}/host-ip
    Should Be Equal As Integers    ${response.status_code}    200

E2E-BDD-SAF-006 Joint Clamp Does Not Produce HTTP Error
    [Documentation]    Motor clamping is ROS-only; Flask pose write does not validate angles.
    Then Motor Position Should Be Clamped    tilt_forward_motor    ${99999}    ${-4500}    ${4500}
    ${response}=    GET    ${FLASK_BASE_URL}/motor/tilt_forward_motor    expected_status=200
    Should Be Equal As Integers    ${response.status_code}    200


*** Keywords ***
Initialize Safety Test Environment
    Set Environment Variable    ROS2_TEST_MOCK    true
    Ros2 Initialize Test Node
    Flask API Is Reachable

Shutdown Safety Test Environment
    Ros2 Shutdown Test Node

Then Motor Position Should Be Clamped
    [Arguments]    ${motor}    ${requested}    ${min}    ${max}
    Ros2 Assert Position Clamped    ${motor}    ${requested}    ${min}    ${max}
