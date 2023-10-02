# coding: utf-8

from __future__ import absolute_import

from flask import json
from six import BytesIO

from swagger_server.models.add_voice_assistant_chat_message_request import AddVoiceAssistantChatMessageRequest  # noqa: E501
from swagger_server.models.add_voice_assistant_chat_message_response import AddVoiceAssistantChatMessageResponse  # noqa: E501
from swagger_server.models.create_voice_assistant_chat import CreateVoiceAssistantChat  # noqa: E501
from swagger_server.models.error_message import ErrorMessage  # noqa: E501
from swagger_server.models.get_all_chats import GetAllChats  # noqa: E501
from swagger_server.models.get_voice_assistant_chat import GetVoiceAssistantChat  # noqa: E501
from swagger_server.models.get_voice_assistant_chat_messages import GetVoiceAssistantChatMessages  # noqa: E501
from swagger_server.models.successful_voice_assistant_chat_creation_response import SuccessfulVoiceAssistantChatCreationResponse  # noqa: E501
from swagger_server.models.update_voice_assistant_chat import UpdateVoiceAssistantChat  # noqa: E501
from swagger_server.models.update_voice_assistant_chat_request import UpdateVoiceAssistantChatRequest  # noqa: E501
from swagger_server.test import BaseTestCase


class TestChatController(BaseTestCase):
    """ChatController integration test stubs"""

    def test_voice_assistant_chat_chat_id_delete(self):
        """Test case for voice_assistant_chat_chat_id_delete

        Deletes voice assistant personality by id
        """
        response = self.client.open(
            '/voice-assistant/chat/{chat_id}'.format(chat_id='chat_id_example'),
            method='DELETE')
        self.assert200(response,
                       'Response body is : ' + response.data.decode('utf-8'))

    def test_voice_assistant_chat_chat_id_get(self):
        """Test case for voice_assistant_chat_chat_id_get

        Get voice assistant chat by id
        """
        response = self.client.open(
            '/voice-assistant/chat/{chat_id}'.format(chat_id='chat_id_example'),
            method='GET')
        self.assert200(response,
                       'Response body is : ' + response.data.decode('utf-8'))

    def test_voice_assistant_chat_chat_id_messages_get(self):
        """Test case for voice_assistant_chat_chat_id_messages_get

        Get voice assistant chat messages by chat id
        """
        query_string = [('timestamp', 'timestamp_example'),
                        ('count', 10)]
        response = self.client.open(
            '/voice-assistant/chat/{chat_id}/messages'.format(chat_id='chat_id_example'),
            method='GET',
            query_string=query_string)
        self.assert200(response,
                       'Response body is : ' + response.data.decode('utf-8'))

    def test_voice_assistant_chat_chat_id_messages_post(self):
        """Test case for voice_assistant_chat_chat_id_messages_post

        Get voice assistant chat by id
        """
        body = AddVoiceAssistantChatMessageRequest()
        response = self.client.open(
            '/voice-assistant/chat/{chat_id}/messages'.format(chat_id='chat_id_example'),
            method='POST',
            data=json.dumps(body),
            content_type='application/json')
        self.assert200(response,
                       'Response body is : ' + response.data.decode('utf-8'))

    def test_voice_assistant_chat_chat_id_put(self):
        """Test case for voice_assistant_chat_chat_id_put

        Create new voice assistant personality
        """
        body = UpdateVoiceAssistantChatRequest()
        response = self.client.open(
            '/voice-assistant/chat/{chat_id}'.format(chat_id='chat_id_example'),
            method='PUT',
            data=json.dumps(body),
            content_type='application/json')
        self.assert200(response,
                       'Response body is : ' + response.data.decode('utf-8'))

    def test_voice_assistant_chat_get(self):
        """Test case for voice_assistant_chat_get

        Get all voice-assistant chats
        """
        response = self.client.open(
            '/voice-assistant/chat',
            method='GET')
        self.assert200(response,
                       'Response body is : ' + response.data.decode('utf-8'))

    def test_voice_assistant_chat_post(self):
        """Test case for voice_assistant_chat_post

        Create new voice assistant chat
        """
        body = CreateVoiceAssistantChat()
        response = self.client.open(
            '/voice-assistant/chat',
            method='POST',
            data=json.dumps(body),
            content_type='application/json')
        self.assert200(response,
                       'Response body is : ' + response.data.decode('utf-8'))


if __name__ == '__main__':
    import unittest
    unittest.main()
