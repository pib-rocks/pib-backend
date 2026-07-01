*** Settings ***
Documentation       Shared keywords and HTTP session setup for the pib-backend Flask API.
...                 Tests run black-box over HTTP against a live, seeded API instance.

Library             RequestsLibrary
Library             Collections


*** Variables ***
${BASE_URL}         http://localhost:5000
${SESSION}          pib_api


*** Keywords ***
Create API Session
    [Documentation]    Opens a reusable RequestsLibrary session to the Flask API.
    Create Session    ${SESSION}    ${BASE_URL}    verify=${True}

Get An Assistant Model Id
    [Documentation]    Returns the id of a seeded assistant model, used as a valid FK
    ...    when creating personalities.
    ${response}=    GET On Session    ${SESSION}    /assistant-model    expected_status=200
    ${models}=    Set Variable    ${response.json()}[assistantModels]
    Should Not Be Empty    ${models}    No seeded assistant models found - is the DB seeded?
    ${assistant_model_id}=    Set Variable    ${models}[0][id]
    RETURN    ${assistant_model_id}

Create Personality
    [Documentation]    Creates a personality and returns the parsed response body.
    [Arguments]    ${name}    ${gender}    ${pause_threshold}    ${message_history}    ${assistant_model_id}
    ${payload}=    Create Dictionary
    ...    name=${name}
    ...    gender=${gender}
    ...    pauseThreshold=${pause_threshold}
    ...    messageHistory=${message_history}
    ...    assistantModelId=${assistant_model_id}
    ${response}=    POST On Session    ${SESSION}    /voice-assistant/personality
    ...    json=${payload}    expected_status=201
    RETURN    ${response.json()}

Delete Personality
    [Documentation]    Deletes a personality by id. Tolerates an already-deleted id so it is
    ...    safe to use in teardowns.
    [Arguments]    ${personality_id}
    ${response}=    DELETE On Session    ${SESSION}    /voice-assistant/personality/${personality_id}
    ...    expected_status=any
