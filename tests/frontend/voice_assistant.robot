*** Settings ***
Documentation     Voice Assistant view E2E — docs/test-basis/frontend_e2e.md (E2E-BDD-FE-VA-*).
...               Verifies personality list, add/delete buttons, and chat window elements.
...               Requires Cerebra on http://localhost:4200.
Resource          ../resources/frontend_keywords.robot

Suite Setup        Set Suite Variables
Suite Teardown     Close Cerebra Application


*** Test Cases ***
E2E-BDD-FE-VA-001 Voice Assistant View Renders After Navigation
    [Documentation]    Given Cerebra is open When user navigates to Voice Assistant Then the view container is visible.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Voice_Assistant
    Then Cerebra Url Path Should Be    /voice-assistant
    Then Voice Assistant View Is Visible

E2E-BDD-FE-VA-002 Personality List Is Visible
    [Documentation]    Given Voice Assistant view When loaded Then the personality list is rendered.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Voice_Assistant
    Then Voice Assistant View Is Visible
    Then Personality List Is Visible

E2E-BDD-FE-VA-003 Add Personality Button Is Visible
    [Documentation]    Given Voice Assistant view When loaded Then the Add Personality button is present.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Voice_Assistant
    Then Voice Assistant View Is Visible
    Then Personality Button Is Visible    BTN_Add_Personality

E2E-BDD-FE-VA-004 Delete Persona Button Is Visible
    [Documentation]    Given Voice Assistant view When loaded Then the Delete Persona button is present.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Voice_Assistant
    Then Voice Assistant View Is Visible
    Then Personality Button Is Visible    BTN_Delete_Persona

E2E-BDD-FE-VA-005 Chat Window Is Visible
    [Documentation]    Given Voice Assistant view When loaded Then the chat window is rendered.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Voice_Assistant
    Then Voice Assistant View Is Visible
    Then Chat Window Is Visible

E2E-BDD-FE-VA-006 Chat Send Button Is Visible
    [Documentation]    Given Voice Assistant view When loaded Then the chat send button is present.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Voice_Assistant
    Then Voice Assistant View Is Visible
    Then Chat Send Button Is Visible

E2E-BDD-FE-VA-007 Chat Input Field Is Visible
    [Documentation]    Given Voice Assistant view When loaded Then the chat message input field is present.
    Open Cerebra Home
    When User Clicks Sidebar Nav Item    LNK_Voice_Assistant
    Then Voice Assistant View Is Visible
    Then Chat Input Field Is Visible
