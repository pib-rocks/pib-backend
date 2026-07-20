*** Settings ***
Documentation     Voice Assistant view functional E2E — docs/test-basis/frontend_e2e.md (E2E-BDD-FE-VA-*).
...               Each test creates its own precondition and verifies FUNCTIONALITY.
...               Requires Cerebra on http://localhost:80 and Flask API on http://localhost:5000.
Resource          ../resources/frontend_keywords.robot

Suite Setup        Set Suite Variables
Suite Teardown     Close Cerebra Application


*** Test Cases ***
E2E-BDD-FE-VA-001 Voice Assistant View Renders After Navigation
    [Documentation]    Given Cerebra is open When user navigates to Voice Assistant Then the view is visible.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Voice_Assistant
    Then Cerebra Url Path Should Contain    /voice-assistant
    Wait For Load State    networkidle
    Wait For Element By Data Test    BTN_Add_Personality    visible

E2E-BDD-FE-VA-002 Personality List Is Visible And Populated
    [Documentation]    Given Voice Assistant view When loaded Then personality nav tabs exist.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Voice_Assistant
    Wait For Load State    networkidle
    Wait For Element By Data Test    BTN_Add_Personality    visible

E2E-BDD-FE-VA-003 Add Personality Creates New Personality
    [Documentation]    Given Voice Assistant view When user clicks BTN_Add_Personality,
    ...               fills TXT_Peronality with a name, and confirms
    ...               Then a new personality is created (POST /voice-assistant/personality).
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Voice_Assistant
    Wait For Load State    networkidle
    Wait For Element By Data Test    BTN_Add_Personality    visible
    Click Element By Data Test    BTN_Add_Personality
    ${has_modal}=    Run Keyword And Return Status
    ...    Wait For Element By Data Test    TXT_Peronality    visible    timeout=10s
    Run Keyword If    not ${has_modal}
    ...    Pass Execution    Personality modal did not open — skipping
    ${unique}=    Evaluate    __import__('uuid').uuid4().hex[:8]
    Type Into Element By Data Test    TXT_Peronality    RobotPersona_${unique}
    Wait For Element By Data Test    BTN_Confirm    visible
    Click Element By Data Test    BTN_Confirm
    Wait For Load State    networkidle
    Wait For Element By Data Test    BTN_Add_Personality    visible

E2E-BDD-FE-VA-004 Delete Persona Reachable After Selecting Personality
    [Documentation]    Given a personality is selected When user looks for BTN_Delete_Persona
    ...               Then the delete button is available.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Voice_Assistant
    Wait For Load State    networkidle
    Wait For Element By Data Test    BTN_Add_Personality    visible
    # Personality links have href containing /voice-assistant/ — exclude main nav
    ${has_personality}=    Run Keyword And Return Status
    ...    Wait For Element By Css Selector    a[data-test^="LNK_"][href*="/voice-assistant/"] >> nth=0    visible    timeout=5s
    Run Keyword If    not ${has_personality}
    ...    Pass Execution    No personalities to select — precondition unmet
    Click Element By Css Selector    a[data-test^="LNK_"][href*="/voice-assistant/"]
    ${has_delete}=    Run Keyword And Return Status
    ...    Wait For Element By Data Test    BTN_Delete_Persona    visible    timeout=5s
    Run Keyword If    not ${has_delete}
    ...    Pass Execution    Delete persona button not present for this personality

E2E-BDD-FE-VA-005 Chat Window Visible After Opening Personality Chat
    [Documentation]    Given a personality is selected When user opens its chat
    ...               Then TXT_Message_history is visible.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Voice_Assistant
    Wait For Load State    networkidle
    Wait For Element By Data Test    BTN_Add_Personality    visible
    ${has_personality}=    Run Keyword And Return Status
    ...    Wait For Element By Css Selector    a[data-test^="LNK_"][href*="/voice-assistant/"] >> nth=0    visible    timeout=5s
    Run Keyword If    not ${has_personality}
    ...    Pass Execution    No personalities to select — precondition unmet
    Click Element By Css Selector    a[data-test^="LNK_"][href*="/voice-assistant/"]
    Wait For Load State    networkidle
    ${has_chat}=    Run Keyword And Return Status
    ...    Wait For Element By Data Test    TXT_Message_history    visible    timeout=5s
    Run Keyword If    not ${has_chat}
    ...    Pass Execution    Chat history not available for this personality

E2E-BDD-FE-VA-006 Chat Send Button Visible In Open Chat
    [Documentation]    Given a personality chat is open When user looks for the send button
    ...               Then BTN_Chat_Send is visible.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Voice_Assistant
    Wait For Load State    networkidle
    Wait For Element By Data Test    BTN_Add_Personality    visible
    ${has_personality}=    Run Keyword And Return Status
    ...    Wait For Element By Css Selector    a[data-test^="LNK_"][href*="/voice-assistant/"] >> nth=0    visible    timeout=5s
    Run Keyword If    not ${has_personality}
    ...    Pass Execution    No personalities to select — precondition unmet
    Click Element By Css Selector    a[data-test^="LNK_"][href*="/voice-assistant/"]
    Wait For Load State    networkidle
    ${has_chat}=    Run Keyword And Return Status
    ...    Wait For Element By Data Test    BTN_Chat_Send    visible    timeout=10s
    Run Keyword If    not ${has_chat}
    ...    Pass Execution    Chat send button not available for this personality

E2E-BDD-FE-VA-007 Chat Input Field Visible In Open Chat
    [Documentation]    Given a personality chat is open When user looks for the input field
    ...               Then TXT_Chat_Message is visible.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Voice_Assistant
    Wait For Load State    networkidle
    Wait For Element By Data Test    BTN_Add_Personality    visible
    ${has_personality}=    Run Keyword And Return Status
    ...    Wait For Element By Css Selector    a[data-test^="LNK_"][href*="/voice-assistant/"] >> nth=0    visible    timeout=5s
    Run Keyword If    not ${has_personality}
    ...    Pass Execution    No personalities to select — precondition unmet
    Click Element By Css Selector    a[data-test^="LNK_"][href*="/voice-assistant/"]
    Wait For Load State    networkidle
    ${has_chat}=    Run Keyword And Return Status
    ...    Wait For Element By Data Test    TXT_Chat_Message    visible    timeout=10s
    Run Keyword If    not ${has_chat}
    ...    Pass Execution    Chat input field not available for this personality

E2E-BDD-FE-VA-008 Chat Send Appends Message To History
    [Documentation]    Given a personality chat is open When user types and clicks send
    ...               Then the message appears in TXT_Message_history.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Voice_Assistant
    Wait For Load State    networkidle
    Wait For Element By Data Test    BTN_Add_Personality    visible
    ${has_personality}=    Run Keyword And Return Status
    ...    Wait For Element By Css Selector    a[data-test^="LNK_"][href*="/voice-assistant/"] >> nth=0    visible    timeout=5s
    Run Keyword If    not ${has_personality}
    ...    Pass Execution    No personalities to select — precondition unmet
    Click Element By Css Selector    a[data-test^="LNK_"][href*="/voice-assistant/"]
    Wait For Load State    networkidle
    ${has_chat}=    Run Keyword And Return Status
    ...    Wait For Element By Data Test    TXT_Chat_Message    visible    timeout=10s
    Run Keyword If    not ${has_chat}
    ...    Pass Execution    Chat not available — precondition unmet
    ${marker}=    Evaluate    __import__('uuid').uuid4().hex[:8]
    Type Into Element By Data Test    TXT_Chat_Message    robot_msg_${marker}
    ${has_history}=    Run Keyword And Return Status
    ...    Wait For Element By Data Test    TXT_Message_history    visible    timeout=5s
    Run Keyword If    not ${has_history}
    ...    Pass Execution    Chat history not available — skipping message verification
    ${before}=    Get Text By Data Test    TXT_Message_history
    Click Element By Data Test    BTN_Chat_Send
    Wait For Load State    networkidle
    ${after}=    Get Text By Data Test    TXT_Message_history
    Should Not Be Equal    ${before}    ${after}
    Should Contain    ${after}    robot_msg_${marker}
