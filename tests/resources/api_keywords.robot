*** Settings ***
Documentation     Shared REST keywords for Flask API contract tests (docs/test-basis/api_contracts.md).
Library           RequestsLibrary
Library           Collections
Library           JSONLibrary


*** Variables ***
${FLASK_BASE_URL}       http://localhost:5000
${JSON_HEADERS}         Content-Type=application/json


*** Keywords ***
Flask API Is Reachable
    ${response}=    GET    ${FLASK_BASE_URL}/motor    expected_status=200
    Dictionary Should Contain Key    ${response.json()}    motors

Create Program
    [Arguments]    ${name}
    ${payload}=    Create Dictionary    name=${name}
    ${response}=    POST    ${FLASK_BASE_URL}/program    json=${payload}    expected_status=201
    ${body}=    Set Variable    ${response.json()}
    RETURN    ${body}[programNumber]

Create Unique Program
    [Arguments]    ${prefix}
    ${suffix}=    Evaluate    __import__('uuid').uuid4().hex[:8]
    ${name}=    Set Variable    ${prefix}_${suffix}
    ${program_number}=    Create Program    ${name}
    RETURN    ${program_number}    ${name}

Delete Program
    [Arguments]    ${program_number}
    DELETE    ${FLASK_BASE_URL}/program/${program_number}    expected_status=204

Get Startup Pose By Name
    ${response}=    GET    ${FLASK_BASE_URL}/pose/by-name/Startup%2FResting    expected_status=200
    RETURN    ${response.json()}

Response Error Envelope Matches
    [Arguments]    ${response}    ${expected_status}    ${expected_error}
    Should Be Equal As Integers    ${response.status_code}    ${expected_status}
    Should Be Equal    ${response.json()}[error]    ${expected_error}

Get Motor Rotation Range
    [Arguments]    ${motor_name}
    ${response}=    GET    ${FLASK_BASE_URL}/motor/${motor_name}    expected_status=200
    ${body}=    Set Variable    ${response.json()}
    RETURN    ${body}[rotationRangeMin]    ${body}[rotationRangeMax]

Put Button Program Mapping
    [Arguments]    ${bricklet_number}    ${program_number}=${None}
    ${updates}=    Create List
    ${update}=    Create Dictionary    brickletNumber=${bricklet_number}
    Run Keyword If    '${program_number}' != '${None}' and '${program_number}' != 'None'
    ...    Set To Dictionary    ${update}    programNumber=${program_number}
    ...    ELSE    Set To Dictionary    ${update}    programNumber=${None}
    Append To List    ${updates}    ${update}
    ${payload}=    Create Dictionary    buttonProgramUpdates=${updates}
    PUT    ${FLASK_BASE_URL}/button-programs    json=${payload}    expected_status=200

Get Host IP
    ${response}=    GET    ${FLASK_BASE_URL}/host-ip
    RETURN    ${response}
