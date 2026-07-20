*** Settings ***
Documentation     Browser (Playwright) keywords for Cerebra frontend tests.
...               PR-1466 rewrite: keywords are PRIMITIVES only (open page, click/wait/get by
...               data-test). Test logic — preconditions and functional assertions — lives
...               in the .robot test files under tests/frontend/.
Library           Browser
Library           Collections


*** Variables ***
${CEREBRA_BASE_URL}     http://localhost:80
${FLASK_BASE_URL}       http://localhost:5000
${HEADLESS}             ${True}
${DEFAULT_TIMEOUT}      20s


*** Keywords ***
# ===========================================================================
# Suite setup / teardown
# ===========================================================================
Set Suite Variables
    New Browser    headless=${HEADLESS}    timeout=30s
    New Context    viewport={'width': 1280, 'height': 720}

Close Cerebra Application
    Close Browser

# ===========================================================================
# Page primitives
# ===========================================================================
Open Cerebra Page
    [Arguments]    ${path}=/voice-assistant/personalities
    [Documentation]    Opens a Cerebra page at the given path and waits for network idle.
    New Page    ${CEREBRA_BASE_URL}${path}
    Wait For Load State    networkidle

Open Cerebra Home
    [Documentation]    Opens the Cerebra root and waits for the SPA to settle.
    Open Cerebra Page    /

# ===========================================================================
# Data-test primitives — used by every test file
# ===========================================================================
Wait For Element By Data Test
    [Arguments]    ${data_test}    ${state}=visible    ${timeout}=${DEFAULT_TIMEOUT}
    [Documentation]    Waits for the element identified by `data-test="${data_test}"`
    ...               to reach `state` (visible | hidden | attached | detached | enabled | disabled).
    Wait For Elements State    css=[data-test="${data_test}"]    ${state}    timeout=${timeout}

Click Element By Data Test
    [Arguments]    ${data_test}    ${timeout}=${DEFAULT_TIMEOUT}
    [Documentation]    Waits for the element to be visible then clicks it.
    ...               Does NOT wait for network idle afterwards — callers chain
    ...               `Wait For Response ...` or `Wait For Element By Data Test`
    ...               to assert the effect of the click.
    Wait For Elements State    css=[data-test="${data_test}"]    visible    timeout=${timeout}
    Click    css=[data-test="${data_test}"]

Get Text By Data Test
    [Arguments]    ${data_test}    ${timeout}=${DEFAULT_TIMEOUT}
    [Documentation]    Returns the textContent of the element matching `data-test`.
    ...               Waits for visibility first.
    Wait For Elements State    css=[data-test="${data_test}"]    visible    timeout=${timeout}
    ${text}=    Get Text    css=[data-test="${data_test}"]
    RETURN    ${text}

Get Property By Data Test
    [Arguments]    ${data_test}    ${property}    ${timeout}=${DEFAULT_TIMEOUT}
    [Documentation]    Returns a DOM property of the element matching `data-test`
    ...               (e.g. `checked`, `value`, `className`). Useful for toggle
    ...               state assertions (Get Property    checked).
    Wait For Elements State    css=[data-test="${data_test}"]    visible    timeout=${timeout}
    ${value}=    Get Property    css=[data-test="${data_test}"]    ${property}
    RETURN    ${value}

Type Into Element By Data Test
    [Arguments]    ${data_test}    ${text}    ${timeout}=${DEFAULT_TIMEOUT}
    [Documentation]    Clears the input and types `text` into an element identified by data-test.
    Wait For Elements State    css=[data-test="${data_test}"]    visible    timeout=${timeout}
    Clear Text    css=[data-test="${data_test}"]
    Type Text    css=[data-test="${data_test}"]    ${text}

Select Option By Data Test
    [Arguments]    ${data_test}    ${option_label}    ${timeout}=${DEFAULT_TIMEOUT}
    [Documentation]    Selects an option from a `<select>` / dropdown identified by data-test
    ...               by visible label.
    Wait For Elements State    css=[data-test="${data_test}"]    visible    timeout=${timeout}
    Select Options By    css=[data-test="${data_test}"]    label    ${option_label}

Element Count By Data Test
    [Arguments]    ${data_test}    ${timeout}=${DEFAULT_TIMEOUT}
    [Documentation]    Returns the count of elements matching `data-test` (attached or not).
    ...               Useful for "item appears/disappears from list" assertions.
    Wait For Elements State    css=[data-test="${data_test}"]    attached    timeout=${timeout}
    ${count}=    Get Element Count    css=[data-test="${data_test}"]
    RETURN    ${count}

Wait For Element By Css Prefix
    [Arguments]    ${css_prefix}    ${state}=visible    ${timeout}=${DEFAULT_TIMEOUT}
    [Documentation]    Waits for an element matching a CSS prefix selector
    ...               (e.g. `[data-test^="BTN_Touchpoint_"]`) to reach `state`.
    Wait For Elements State    css=[data-test^="${css_prefix}"] >> nth=0    ${state}    timeout=${timeout}

Click Element By Css Prefix
    [Arguments]    ${css_prefix}    ${timeout}=${DEFAULT_TIMEOUT}
    [Documentation]    Waits for the first element matching a CSS prefix selector
    ...               to be visible, then clicks it. Used for dynamic data-test
    ...               attributes like `BTN_Touchpoint_{motorName}`.
    Wait For Elements State    css=[data-test^="${css_prefix}"]    visible    timeout=${timeout}
    Click    css=[data-test^="${css_prefix}"] >> nth=0

Wait For Element By Css Selector
    [Arguments]    ${css_selector}    ${state}=visible    ${timeout}=${DEFAULT_TIMEOUT}
    [Documentation]    Waits for an element matching an arbitrary CSS selector.
    ...               Used for complex selectors with :not() etc.
    Wait For Elements State    css=${css_selector} >> nth=0    ${state}    timeout=${timeout}

Click Element By Css Selector
    [Arguments]    ${css_selector}    ${timeout}=${DEFAULT_TIMEOUT}
    [Documentation]    Waits for the first element matching an arbitrary CSS selector
    ...               to be visible, then clicks it.
    Wait For Elements State    css=${css_selector} >> nth=0    visible    timeout=${timeout}
    Click    css=${css_selector} >> nth=0

Get Property By Css Selector
    [Arguments]    ${css_selector}    ${property}    ${timeout}=${DEFAULT_TIMEOUT}
    [Documentation]    Returns a DOM property of the first element matching an
    ...               arbitrary CSS selector.
    Wait For Elements State    css=${css_selector}    visible    timeout=${timeout}
    ${value}=    Get Property    css=${css_selector} >> nth=0    ${property}
    RETURN    ${value}

# ===========================================================================
# Navigation helpers (thin wrappers over primitives)
# ===========================================================================
Click Sidebar Nav Item
    [Arguments]    ${data_test}
    [Documentation]    Clicks a sidebar nav link and waits for the route to settle.
    Click Element By Data Test    ${data_test}
    Wait For Load State    networkidle

When User Clicks Sidebar Nav Item
    [Arguments]    ${data_test}
    Click Sidebar Nav Item    ${data_test}

Given User Opens Cerebra And Navigates To
    [Arguments]    ${data_test}    ${expected_path}
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    ${data_test}
    Then Cerebra Url Path Should Contain    ${expected_path}

Then Cerebra Url Path Should Contain
    [Arguments]    ${expected_path}
    ${url}=    Get Url
    Should Contain    ${url}    ${expected_path}

Then Cerebra Url Path Should Be
    [Arguments]    ${expected_path}
    Then Cerebra Url Path Should Contain    ${expected_path}

Then View Element Is Visible
    [Arguments]    ${data_test}
    Wait For Element By Data Test    ${data_test}    visible

Then View Element Count Is At Least
    [Arguments]    ${data_test}    ${min_count}
    ${count}=    Element Count By Data Test    ${data_test}
    Should Be True    ${count} >= ${min_count}
    ...    Expected at least ${min_count} element(s) for [data-test="${data_test}"], got ${count}

# ===========================================================================
# Compatibility shims — keep the sibling suites (angular_components,
# blockly_interactions) working. These DO NOT assert functional behavior;
# new functional assertions live in the test files.
# ===========================================================================
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
    Wait For Elements State    css=[data-test="TXT_Peronality"]    visible    timeout=15s

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
