*** Settings ***
Documentation       Full CRUD lifecycle test for the Voice Assistant Personality endpoint
...                 (/voice-assistant/personality) of the pib-backend Flask API.
...
...                 This is the first Robot Framework suite for pib-backend. It is non-destructive:
...                 it creates its own personality, exercises it, and deletes it, leaving the
...                 seeded data intact. The test cases are ordered and share the created id via a
...                 suite variable.

Resource            resources/api_keywords.robot

Suite Setup         Set Up Suite
Suite Teardown      Tear Down Suite


*** Variables ***
${PERSONALITY_NAME}             RobotTester
${PERSONALITY_GENDER}           Female
${PERSONALITY_PAUSE}            ${0.8}
${PERSONALITY_HISTORY}          ${5}

${UPDATED_NAME}                 RobotTesterUpdated
${UPDATED_GENDER}               Male
${UPDATED_PAUSE}                ${1.2}
${UPDATED_HISTORY}              ${10}


*** Test Cases ***
Create Personality Returns 201 And Body
    [Documentation]    POST creates a personality and returns it with a generated personalityId.
    ${body}=    Create Personality
    ...    ${PERSONALITY_NAME}    ${PERSONALITY_GENDER}    ${PERSONALITY_PAUSE}
    ...    ${PERSONALITY_HISTORY}    ${ASSISTANT_MODEL_ID}
    Dictionary Should Contain Key    ${body}    personalityId
    Should Not Be Empty    ${body}[personalityId]
    Should Be Equal    ${body}[name]    ${PERSONALITY_NAME}
    Should Be Equal    ${body}[assistantModelId]    ${ASSISTANT_MODEL_ID}
    Set Suite Variable    ${PERSONALITY_ID}    ${body}[personalityId]

Get Created Personality Returns 200
    [Documentation]    GET by id returns the personality created above.
    ${response}=    GET On Session    ${SESSION}
    ...    /voice-assistant/personality/${PERSONALITY_ID}    expected_status=200
    ${body}=    Set Variable    ${response.json()}
    Should Be Equal    ${body}[personalityId]    ${PERSONALITY_ID}
    Should Be Equal    ${body}[name]    ${PERSONALITY_NAME}
    Should Be Equal As Numbers    ${body}[messageHistory]    ${PERSONALITY_HISTORY}

Personality Appears In List
    [Documentation]    GET list includes the created personality id.
    ${response}=    GET On Session    ${SESSION}    /voice-assistant/personality    expected_status=200
    ${personalities}=    Set Variable    ${response.json()}[voiceAssistantPersonalities]
    ${ids}=    Evaluate    [p['personalityId'] for p in $personalities]
    Should Contain    ${ids}    ${PERSONALITY_ID}

Update Personality Returns 200
    [Documentation]    PUT updates the personality fields.
    ${payload}=    Create Dictionary
    ...    name=${UPDATED_NAME}
    ...    gender=${UPDATED_GENDER}
    ...    pauseThreshold=${UPDATED_PAUSE}
    ...    messageHistory=${UPDATED_HISTORY}
    ...    assistantModelId=${ASSISTANT_MODEL_ID}
    ${response}=    PUT On Session    ${SESSION}
    ...    /voice-assistant/personality/${PERSONALITY_ID}    json=${payload}    expected_status=200
    ${body}=    Set Variable    ${response.json()}
    Should Be Equal    ${body}[name]    ${UPDATED_NAME}
    Should Be Equal    ${body}[gender]    ${UPDATED_GENDER}
    Should Be Equal As Numbers    ${body}[messageHistory]    ${UPDATED_HISTORY}

Delete Personality Returns 204
    [Documentation]    DELETE removes the personality and returns 204 No Content.
    ${response}=    DELETE On Session    ${SESSION}
    ...    /voice-assistant/personality/${PERSONALITY_ID}    expected_status=204

Get Deleted Personality Returns Error
    [Documentation]    GET on a deleted id no longer returns 200 (the service raises on a missing row).
    ${response}=    GET On Session    ${SESSION}
    ...    /voice-assistant/personality/${PERSONALITY_ID}    expected_status=any
    Should Be True    ${response.status_code} >= 400
    ...    Expected an error status for a deleted personality but got ${response.status_code}


*** Keywords ***
Set Up Suite
    [Documentation]    Opens the HTTP session and resolves a valid assistant model id (FK).
    Create API Session
    ${assistant_model_id}=    Get An Assistant Model Id
    Set Suite Variable    ${ASSISTANT_MODEL_ID}    ${assistant_model_id}
    Set Suite Variable    ${PERSONALITY_ID}    ${EMPTY}

Tear Down Suite
    [Documentation]    Removes the test personality if it still exists, so the suite is
    ...    non-destructive even when a test fails mid-lifecycle.
    Run Keyword If    '${PERSONALITY_ID}' != '${EMPTY}'    Delete Personality    ${PERSONALITY_ID}
