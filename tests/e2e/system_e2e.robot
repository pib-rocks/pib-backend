*** Settings ***
Documentation     Full-system E2E tests — docs/test-basis/ros2_interfaces.md and backend_safety_and_edge_cases.md.
Resource            ../resources/api_keywords.robot
Library             ../resources/ROS2TestLibrary.py
Library             RequestsLibrary
Library             Collections
Library             OperatingSystem

Suite Setup         Initialize E2E Environment
Suite Teardown      Shutdown E2E Environment


*** Variables ***
${FLASK_BASE_URL}       http://localhost:5000
${ROSBRIDGE_URL}        ws://localhost:9090


*** Test Cases ***
E2E-BDD-SYS-001 Flask REST Independent Of Rosbridge
    [Documentation]    Given rosbridge may be down When Flask /motor is called Then 200 is returned.
    ${response}=    GET    ${FLASK_BASE_URL}/motor    expected_status=200
    Dictionary Should Contain Key    ${response.json()}    motors
    Ros2 Assert No Http 503 From Flask    ${FLASK_BASE_URL}

E2E-BDD-SYS-002 Motor Trajectory Clamped To Global Envelope
    [Documentation]    Given apply_joint_trajectory requests 12000 When clamp applied Then effective is 9000.
    Then Motor Position Should Be Clamped    turn_head_motor    ${12000}    ${-9000}    ${9000}

E2E-BDD-SYS-003 Tilt Forward Motor Clamped To Seeded Range
    [Documentation]    Given tilt_forward_motor range -4500..4500 When 5000 requested Then clamped to 4500.
    ${min}    ${max}=    Get Motor Rotation Range    tilt_forward_motor
    Should Be Equal As Integers    ${min}    ${-4500}
    Should Be Equal As Integers    ${max}    ${4500}
    Then Motor Position Should Be Clamped    tilt_forward_motor    ${5000}    ${min}    ${max}

E2E-BDD-SYS-004 Startup Pose Available For Motor Boot Sequence
    [Documentation]    Given Startup/Resting pose When fetched by name Then deletable is false.
    ${pose}=    Get Startup Pose By Name
    Should Be Equal    ${pose}[name]    Startup/Resting
    Should Be Equal    ${pose}[deletable]    ${False}

E2E-BDD-SYS-005 Button Program Mapping Round Trip
    [Documentation]    Given program created When mapped to bricklet 5 Then PUT /button-programs succeeds.
    ${program_number}    ${name}=    Create Unique Program    e2e_button_map
    Put Button Program Mapping    5    ${program_number}
    Put Button Program Mapping    5    ${None}
    [Teardown]    Run Keyword And Ignore Error    Delete Program    ${program_number}

E2E-BDD-SYS-006 Rosbridge WebSocket Port Accepts Connection
    [Documentation]    Given rosbridge-ws on :9090 When TCP connect Then socket opens.
    ${result}=    Evaluate    __import__('socket').create_connection(('localhost', 9090), timeout=5) or True
    Should Be True    ${result}


*** Keywords ***
Initialize E2E Environment
    Set Environment Variable    ROS2_TEST_MOCK    true
    Ros2 Initialize Test Node
    Flask API Is Reachable

Shutdown E2E Environment
    Ros2 Shutdown Test Node

Then Motor Position Should Be Clamped
    [Arguments]    ${motor}    ${requested}    ${min}    ${max}
    Ros2 Assert Position Clamped    ${motor}    ${requested}    ${min}    ${max}
