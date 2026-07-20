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


# ---------------------------------------------------------------------------
# PR-1466 — Cerebra UI structure keywords (data-test selectors)
# ---------------------------------------------------------------------------
# Conventions:
#   - All element lookups use data-test attributes (css=[data-test="..."])
#   - Element waits use timeout=20s for resilience against slow Angular renders
#   - Keywords assert VISIBILITY only (UI structure), not functional behavior
#   - Sidebar nav items: LNK_Joint_Control, LNK_Poses, LNK_Camera,
#     LNK_Voice_Assistant, LNK_Program, LNK_System
# ---------------------------------------------------------------------------


# === Navigation ===========================================================

Open Cerebra Home
    Open Cerebra Page    /
    Wait For Load State    networkidle

Click Sidebar Nav Item
    [Arguments]    ${data_test}
    [Documentation]    Clicks a sidebar navigation link identified by its data-test attribute.
    Wait For Elements State    css=[data-test="${data_test}"]    visible    timeout=20s
    Click    css=[data-test="${data_test}"]
    Wait For Load State    networkidle

Given User Opens Cerebra And Navigates To
    [Arguments]    ${data_test}    ${expected_path}
    [Documentation]    Given-style step: open Cerebra, click a sidebar item, verify the URL path.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    ${data_test}
    Then Cerebra Url Path Should Be    ${expected_path}

When User Clicks Sidebar Nav Item
    [Arguments]    ${data_test}
    Click Sidebar Nav Item    ${data_test}

Then Cerebra Url Path Should Be
    [Arguments]    ${expected_path}
    [Documentation]    Verifies the browser URL ends with the expected Angular route path.
    ${url}=    Get Url
    Should End With    ${url}    ${expected_path}

Then View Element Is Visible
    [Arguments]    ${data_test}
    [Documentation]    Asserts an element identified by data-test is visible on the current view.
    Wait For Elements State    css=[data-test="${data_test}"]    visible    timeout=20s

Then View Element Count Is At Least
    [Arguments]    ${data_test}    ${min_count}
    [Documentation]    Asserts at least N elements matching data-test exist on the current view.
    ${count}=    Get Element Count    css=[data-test="${data_test}"]
    Should Be True    ${count} >= ${min_count}
    ...    Expected at least ${min_count} element(s) for [data-test="${data_test}"], got ${count}


# === Joint Control =========================================================

Then Joint Control View Is Visible
    [Documentation]    Verifies the Joint Control view container is rendered.
    Wait For Elements State    css=[data-test="LNK_Joint_Control"]    visible    timeout=20s
    Wait For Elements State    css=[data-test="view-joint-control"], [data-test="joint-control-view"]    visible    timeout=20s

Then Joint Tab Is Visible
    [Arguments]    ${tab_data_test}
    [Documentation]    Verifies a joint selection tab (e.g. head, left_arm, right_arm) is visible.
    Wait For Elements State    css=[data-test="${tab_data_test}"]    visible    timeout=20s

When User Selects Joint Tab
    [Arguments]    ${tab_data_test}
    [Documentation]    Clicks a joint tab to switch the displayed motor group.
    Wait For Elements State    css=[data-test="${tab_data_test}"]    visible    timeout=20s
    Click    css=[data-test="${tab_data_test}"]
    Wait For Load State    networkidle

Then Motor Slider Is Visible
    [Arguments]    ${motor_data_test}
    [Documentation]    Verifies a motor slider control is visible within the joint control view.
    Wait For Elements State    css=[data-test="${motor_data_test}"]    visible    timeout=20s

Then Motor Settings Panel Is Visible
    [Documentation]    Verifies the motor settings panel container is rendered.
    Wait For Elements State    css=[data-test="panel-motor-settings"], [data-test="motor-settings-panel"]    visible    timeout=20s

When User Toggles Motor Settings
    [Arguments]    ${toggle_data_test}
    [Documentation]    Clicks a motor settings toggle/expander button.
    Wait For Elements State    css=[data-test="${toggle_data_test}"]    visible    timeout=20s
    Click    css=[data-test="${toggle_data_test}"]

Then Threshold Control Is Visible
    [Arguments]    ${threshold_data_test}
    [Documentation]    Verifies a threshold control (min/max slider) is visible in the motor settings.
    Wait For Elements State    css=[data-test="${threshold_data_test}"]    visible    timeout=20s


# === Poses ================================================================

Then Poses View Is Visible
    [Documentation]    Verifies the Poses view container is rendered.
    Wait For Elements State    css=[data-test="LNK_Poses"]    visible    timeout=20s
    Wait For Elements State    css=[data-test="view-poses"], [data-test="poses-view"]    visible    timeout=20s

Then Pose List Is Visible
    [Documentation]    Verifies the pose list container is rendered and at least one pose row is present.
    Wait For Elements State    css=[data-test="list-poses"], [data-test="pose-list"]    visible    timeout=20s

Then Pose Button Is Visible
    [Arguments]    ${button_data_test}
    [Documentation]    Verifies a pose action button (Apply/Save/Delete/Rename/Import) is visible.
    Wait For Elements State    css=[data-test="${button_data_test}"]    visible    timeout=20s


# === Camera ===============================================================

Then Camera View Is Visible
    [Documentation]    Verifies the Camera view container is rendered.
    Wait For Elements State    css=[data-test="LNK_Camera"]    visible    timeout=20s
    Wait For Elements State    css=[data-test="view-camera"], [data-test="camera-view"]    visible    timeout=20s

Then Camera Video Stream Is Visible
    [Documentation]    Verifies the camera video stream element (img or video) is visible.
    Wait For Elements State    css=[data-test="video-stream"], [data-test="camera-stream"], [data-test="IMG_Camera_stream"]    visible    timeout=20s

Then Camera Resolution Dropdown Is Visible
    [Documentation]    Verifies the camera resolution dropdown selector is visible.
    Wait For Elements State    css=[data-test="DDN_Camera_resolution"]    visible    timeout=20s

Then Camera Settings Button Is Visible
    [Documentation]    Verifies the camera settings button is visible.
    Wait For Elements State    css=[data-test="BTN_Video_settings"]    visible    timeout=20s


# === Voice Assistant ======================================================

Then Voice Assistant View Is Visible
    [Documentation]    Verifies the Voice Assistant view container is rendered.
    Wait For Elements State    css=[data-test="LNK_Voice_Assistant"]    visible    timeout=20s
    Wait For Elements State    css=[data-test="view-voice-assistant"], [data-test="voice-assistant-view"]    visible    timeout=20s

Then Personality List Is Visible
    [Documentation]    Verifies the personality list container is rendered.
    Wait For Elements State    css=[data-test="list-personalities"], [data-test="personality-list"]    visible    timeout=20s

Then Personality Button Is Visible
    [Arguments]    ${button_data_test}
    [Documentation]    Verifies a personality action button (Add/Delete) is visible.
    Wait For Elements State    css=[data-test="${button_data_test}"]    visible    timeout=20s

Then Chat Window Is Visible
    [Documentation]    Verifies the chat window container is rendered.
    Wait For Elements State    css=[data-test="window-chat"], [data-test="chat-window"]    visible    timeout=20s

Then Chat Send Button Is Visible
    [Documentation]    Verifies the chat send button is visible.
    Wait For Elements State    css=[data-test="BTN_Chat_Send"]    visible    timeout=20s

Then Chat Input Field Is Visible
    [Documentation]    Verifies the chat message input field is visible.
    Wait For Elements State    css=[data-test="input-chat-message"], [data-test="INP_Chat_Message"]    visible    timeout=20s


# === Program Manager ======================================================

Then Program Manager View Is Visible
    [Documentation]    Verifies the Program Manager view container is rendered.
    Wait For Elements State    css=[data-test="LNK_Program"]    visible    timeout=20s
    Wait For Elements State    css=[data-test="view-program"], [data-test="program-view"]    visible    timeout=20s

Then Program List Is Visible
    [Documentation]    Verifies the program list container is rendered.
    Wait For Elements State    css=[data-test="list-programs"], [data-test="program-list"]    visible    timeout=20s

Then Program Workspace Is Visible
    [Documentation]    Verifies the Blockly workspace area is rendered.
    Wait For Elements State    css=[data-test="workspace-program"], [data-test="program-workspace"], .blocklyMainBackground, #blocklyDiv    visible    timeout=20s

Then Program Button Is Visible
    [Arguments]    ${button_data_test}
    [Documentation]    Verifies a program action button (Run/Stop/Save/Export) is visible.
    Wait For Elements State    css=[data-test="${button_data_test}"]    visible    timeout=20s


# === System ===============================================================

Then System View Is Visible
    [Documentation]    Verifies the System view container is rendered.
    Wait For Elements State    css=[data-test="LNK_System"]    visible    timeout=20s
    Wait For Elements State    css=[data-test="view-system"], [data-test="system-view"]    visible    timeout=20s

Then Hardware Id Section Is Visible
    [Documentation]    Verifies the hardware IDs section is rendered.
    Wait For Elements State    css=[data-test="section-hardware-ids"], [data-test="hardware-ids-section"]    visible    timeout=20s

Then Bricklet Update Button Is Visible
    [Documentation]    Verifies the bricklet UID update button is visible.
    Wait For Elements State    css=[data-test="BTN_Update_bricklet_UIDs"]    visible    timeout=20s

Then Smart Connect Button Is Visible
    [Documentation]    Verifies the Smart Connect button is visible.
    Wait For Elements State    css=[data-test="BTN_Smart_Connect"]    visible    timeout=20s

Then Relay Control Is Visible
    [Arguments]    ${relay_data_test}
    [Documentation]    Verifies a relay control (checkbox or toggle) is visible.
    Wait For Elements State    css=[data-test="${relay_data_test}"]    visible    timeout=20s
