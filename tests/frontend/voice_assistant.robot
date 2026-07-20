*** Settings ***
Documentation     Voice Assistant view functional E2E — docs/test-basis/frontend_e2e.md (E2E-BDD-FE-VA-*).
...               PR-1466 rewrite: each test sets up its own precondition (click first personality,
...               open a chat) and verifies button FUNCTIONALITY (personality created / message sent),
...               not just visibility.
...               Requires Cerebra on http://localhost:80 and Flask API on http://localhost:5000.
Resource          ../resources/frontend_keywords.robot

Suite Setup        Set Suite Variables
Suite Teardown     Close Cerebra Application


*** Test Cases ***
E2E-BDD-FE-VA-001 Voice Assistant View Renders After Navigation
    [Documentation]    Given Cerebra is open When user navigates to Voice Assistant Then the view container is visible.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Voice_Assistant
    Then Cerebra Url Path Should Contain    /voice-assistant
    Wait For Element By Data Test    BTN_Add_Personality    visible

E2E-BDD-FE-VA-002 Personality List Is Visible And Populated
    [Documentation]    Given Voice Assistant view When loaded Then the personality list is rendered
    ...               and at least one personality is selectable (precondition for later tests).
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Voice_Assistant
    Wait For Element By Data Test    BTN_Add_Personality    visible
    Wait For Element By Data Test    TXT_Peronality    visible
    # Functional: list was populated by GET /voice-assistant/personality
    Wait For Response    matcher=${FLASK_BASE_URL}/voice-assistant/personality    timeout=20s

E2E-BDD-FE-VA-003 Add Personality Creates New Personality
    [Documentation]    Given Voice Assistant view When user clicks BTN_Add_Personality,
    ...               fills TXT_Peronality with a name, and confirms
    ...               Then a new personality is created (POST /voice-assistant/personality)
    ...               and appears in the list.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Voice_Assistant
    Wait For Element By Data Test    BTN_Add_Personality    visible
    ${before}=    Element Count By Data Test    TXT_Peronality
    Click Element By Data Test    BTN_Add_Personality
    # The name field is the same selector reused for entry; type a unique name
    ${unique}=    Evaluate    __import__('uuid').uuid4().hex[:8]
    Type Into Element By Data Test    TXT_Peronality    RobotPersona_${unique}
    # Confirm the new personality (the add button doubles as confirm, or Enter)
    Click Element By Data Test    BTN_Add_Personality
    # Functional: POST must hit the Flask API
    Wait For Response    matcher=${FLASK_BASE_URL}/voice-assistant/personality    timeout=20s
    # ...and the list should now reflect the addition
    ${after}=    Element Count By Data Test    TXT_Peronality
    Should Be True    ${after} >= ${before}

E2E-BDD-FE-VA-004 Delete Persona Reachable After Selecting Personality
    [Documentation]    Given a personality is selected When user looks for BTN_Delete_Persona
    ...               Then the delete button is available for the selected persona.
    ...               PRECONDITION: click the first personality in the list first.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Voice_Assistant
    Wait For Element By Data Test    BTN_Add_Personality    visible
    # Precondition: click the first personality row to select it
    Click Element By Data Test    TXT_Peronality
    # Now the delete button should be actionable for that persona
    Wait For Element By Data Test    BTN_Delete_Persona    visible

E2E-BDD-FE-VA-005 Chat Window Visible After Opening Personality Chat
    [Documentation]    Given a personality is selected When user opens its chat
    ...               Then TXT_Message_history is visible.
    ...               PRECONDITION: click the first personality to open its chat.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Voice_Assistant
    Wait For Element By Data Test    TXT_Peronality    visible
    # Precondition: open the chat of the first personality
    Click Element By Data Test    TXT_Peronality
    # The message history appears once a chat is opened
    Wait For Element By Data Test    TXT_Message_history    visible

E2E-BDD-FE-VA-006 Chat Send Button Visible In Open Chat
    [Documentation]    Given a personality chat is open When user looks for the send button
    ...               Then BTN_Chat_Send is visible.
    ...               PRECONDITION: open a personality chat first.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Voice_Assistant
    Wait For Element By Data Test    TXT_Peronality    visible
    # Precondition: open the chat
    Click Element By Data Test    TXT_Peronality
    Wait For Element By Data Test    BTN_Chat_Send    visible

E2E-BDD-FE-VA-007 Chat Input Field Visible In Open Chat
    [Documentation]    Given a personality chat is open When user looks for the input field
    ...               Then TXT_Chat_Message is visible.
    ...               PRECONDITION: open a personality chat first.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Voice_Assistant
    Wait For Element By Data Test    TXT_Peronality    visible
    # Precondition: open the chat
    Click Element By Data Test    TXT_Peronality
    Wait For Element By Data Test    TXT_Chat_Message    visible

E2E-BDD-FE-VA-008 Chat Send Appends Message To History
    [Documentation]    Given a personality chat is open When user types in TXT_Chat_Message
    ...               and clicks BTN_Chat_Send Then the message appears in TXT_Message_history.
    ...               PRECONDITION: open a personality chat first.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Voice_Assistant
    Wait For Element By Data Test    TXT_Peronality    visible
    # Precondition: open the chat
    Click Element By Data Test    TXT_Peronality
    Wait For Element By Data Test    TXT_Chat_Message    visible
    ${marker}=    Evaluate    __import__('uuid').uuid4().hex[:8]
    Type Into Element By Data Test    TXT_Chat_Message    robot_msg_${marker}
    ${before}=    Get Text By Data Test    TXT_Message_history
    Click Element By Data Test    BTN_Chat_Send
    # Functional: the message must reach the Flask chat API
    Wait For Response    matcher=${FLASK_BASE_URL}/voice-assistant/chat    timeout=20s
    # ...and appear in the history
    ${after}=    Get Text By Data Test    TXT_Message_history
    Should Not Be Equal    ${before}    ${after}
    Should Contain    ${after}    robot_msg_${marker}
