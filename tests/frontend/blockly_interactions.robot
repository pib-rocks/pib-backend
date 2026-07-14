*** Settings ***
Documentation     Blockly UI interaction tests — docs/test-basis pib-blockly-server failures.
Resource            ../resources/frontend_keywords.robot
Resource            ../resources/api_keywords.robot

Suite Setup         Set Suite Variables
Suite Teardown      Close Cerebra Application


*** Test Cases ***
E2E-BDD-BLY-001 Blockly Workspace Renders In Program Editor
    [Documentation]    Given Cerebra program editor When Blockly loads Then workspace canvas is visible.
    Flask API Is Reachable
    When User Opens Blockly Program Editor
    Then Blockly Workspace Is Visible

E2E-BDD-BLY-002 Blockly Compile Failure Surfaces As Flask 500
    [Documentation]    Given invalid visual code When PUT /program/{id}/code Then Flask returns 500.
    Flask API Is Reachable
    ${program_number}=    Create Program    blockly_compile_fail_robot
    When User Saves Program Visual Code    ${program_number}
    [Teardown]    Run Keyword And Ignore Error    Delete Program    ${program_number}

E2E-BDD-BLY-003 Blockly Server Port Is Reachable When Stack Running
    [Documentation]    Given pib-blockly-server on :2442 When POST invalid JSON Then 400 compile error.
    ${response}=    Evaluate    __import__('requests').post('http://localhost:2442', data='not-json', headers={'Content-Type':'text/plain'}, timeout=5)
    Should Be Equal As Integers    ${response.status_code}    400
    Should Be Equal    ${response.text}    failed to compile visual-code.
