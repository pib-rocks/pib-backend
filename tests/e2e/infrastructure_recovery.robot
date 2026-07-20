*** Settings ***
Documentation     Infrastructure recovery BDD — docs/test-basis Docker container lifecycle scenarios.
Resource            ../resources/api_keywords.robot
Library             RequestsLibrary
Library             Process
Library             OperatingSystem


*** Variables ***
${COMPOSE_FILE}         ${CURDIR}/../../docker-compose.yaml
${FLASK_BASE_URL}       http://localhost:5000
${RUN_DOCKER_TESTS}     ${EMPTY}


*** Test Cases ***
E2E-BDD-INF-001 Default Profile Services Only Without All Profile
    [Documentation]    Given compose without --profile all Then ros-motors is not required for Flask.
    Skip If Docker Tests Disabled
    ${result}=    Run Process    docker    compose    -f    ${COMPOSE_FILE}    config    --services
    ...    cwd=${CURDIR}/../..
    Should Be Equal As Integers    ${result.rc}    0
    Should Contain    ${result.stdout}    flask-app
    Should Contain    ${result.stdout}    rosbridge-ws
    Should Contain    ${result.stdout}    pib-blockly-server

E2E-BDD-INF-002 Flask Responds While Rosbridge Port Checked Independently
    [Documentation]    Given rosbridge may fail When Flask /motor called Then REST still works.
    Skip If Docker Tests Disabled
    Flask API Is Reachable
    ${sock}=    Evaluate    __import__('socket').socket()
    ${connected}=    Evaluate    $sock.connect_ex(('localhost', 9090)) == 0    modules=socket
    Run Keyword If    not ${connected}    Log    rosbridge not running — Flask independence still holds
    ${response}=    GET    ${FLASK_BASE_URL}/motor    expected_status=200

E2E-BDD-INF-003 Compose Project Name Matches Env File
    [Documentation]    Given .env When COMPOSE_PROJECT_NAME Then multirepo.
    ${env}=    Get File    ${CURDIR}/../../.env
    Should Contain    ${env}    COMPOSE_PROJECT_NAME=multirepo

E2E-BDD-INF-004 Flask Container Restart Policy Documented As Always
    [Documentation]    Given docker-compose.yaml When flask-app inspected Then restart is always.
    ${compose}=    Get File    ${COMPOSE_FILE}
    Should Match Regexp    ${compose}    flask-app:[\\s\\S]*restart:\\s*always

E2E-BDD-INF-005 No Healthcheck In Compose For Cold Boot Race
    [Documentation]    depends_on does not guarantee Flask readiness — no healthcheck keys.
    ${compose}=    Get File    ${COMPOSE_FILE}
    Should Not Contain    ${compose}    healthcheck


*** Keywords ***
Skip If Docker Tests Disabled
    ${enabled}=    Get Environment Variable    RUN_DOCKER_TESTS    0
    Skip If    '${enabled}' != '1'    Set RUN_DOCKER_TESTS=1 for live infrastructure recovery tests
