*** Settings ***
Documentation     Browser (Playwright) keywords for Cerebra frontend tests.
Library           Browser
Library           Collections


*** Variables ***
${CEREBRA_BASE_URL}     http://localhost:4200
${FLASK_BASE_URL}       http://localhost:5000
${HEADLESS}             ${True}


*** Keywords ***
Set Suite Variables
    New Browser    headless=${HEADLESS}    timeout=30s
    New Context    viewport={'width': 1280, 'height': 720}

Close Cerebra Application
    Close Browser

Open Cerebra Page
    [Arguments]    ${path}=/voice-assistant/personalities
    New Page    ${CEREBRA_BASE_URL}${path}

Mock Flask Route
    [Arguments]    ${path_pattern}    ${status}=200    ${json_body}={}
    ${route}=    Set Variable    ${FLASK_BASE_URL}${path_pattern}
    Route    ${route}    fulfill    status=${status}    body=${json_body}    contentType=application/json

Given Frontend Loads Personality List Mock Success
    ${body}=    Catenate    SEPARATOR=
    ...    {"personalities":[{"personalityId":"00000000-0000-0000-0000-000000000001","name":"Test Personality","gender":"Female"}]}
    Mock Flask Route    /voice-assistant/personality    json_body=${body}

Given Frontend Loads Personality List Mock Error
    Mock Flask Route    /voice-assistant/personality    status=500    json_body={"error":"Internal Server Error, please try later again."}

When User Opens Voice Assistant Personalities Page
    Open Cerebra Page    /voice-assistant/personalities
    Wait For Load State    networkidle

Then Personality List Shows Success State
    Wait For Elements State    css=[data-testid="personality-row"], .personality-row, table tbody tr    visible    timeout=15s

Then Personality List Shows Error State
    Wait For Elements State    css=[role="alert"], .alert-danger, mat-error    visible    timeout=15s

When User Opens Blockly Program Editor
    Open Cerebra Page    /programs
    Wait For Load State    networkidle
    Wait For Elements State    css=.blocklyMainBackground, #blocklyDiv    visible    timeout=20s

Then Blockly Workspace Is Visible
    Get Element Count    css=.blocklyMainBackground, #blocklyDiv >> visible=true    >    0

When User Saves Program Visual Code
    [Arguments]    ${program_number}
    ${response}=    Evaluate    __import__('requests').put('${FLASK_BASE_URL}/program/${program_number}/code', json={'codeVisual': '{}'}, timeout=10)
    Should Be Equal As Integers    ${response.status_code}    500
