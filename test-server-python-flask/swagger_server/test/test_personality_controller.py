# coding: utf-8

from __future__ import absolute_import

from flask import json
from six import BytesIO

from swagger_server.models.create_voice_assistant_personality import CreateVoiceAssistantPersonality  # noqa: E501
from swagger_server.models.error_message import ErrorMessage  # noqa: E501
from swagger_server.models.get_all_personalities import GetAllPersonalities  # noqa: E501
from swagger_server.models.successful_voice_assistant_personality_creation_response import SuccessfulVoiceAssistantPersonalityCreationResponse  # noqa: E501
from swagger_server.models.update_voice_assistant_personality import UpdateVoiceAssistantPersonality  # noqa: E501
from swagger_server.models.update_voice_assistant_personality_request import UpdateVoiceAssistantPersonalityRequest  # noqa: E501
from swagger_server.models.voice_assistant_personality import VoiceAssistantPersonality  # noqa: E501
from swagger_server.test import BaseTestCase


class TestPersonalityController(BaseTestCase):
    """PersonalityController integration test stubs"""

    def test_voice_assistant_personality_get(self):
        """Test case for voice_assistant_personality_get

        Get all voice-assistant personalities
        """
        response = self.client.open(
            '/voice-assistant/personality',
            method='GET')
        self.assert200(response,
                       'Response body is : ' + response.data.decode('utf-8'))

    def test_voice_assistant_personality_personality_id_delete(self):
        """Test case for voice_assistant_personality_personality_id_delete

        Deletes voice assistant personality by id
        """
        response = self.client.open(
            '/voice-assistant/personality/{personality_id}'.format(personality_id='personality_id_example'),
            method='DELETE')
        self.assert200(response,
                       'Response body is : ' + response.data.decode('utf-8'))

    def test_voice_assistant_personality_personality_id_get(self):
        """Test case for voice_assistant_personality_personality_id_get

        Get voice assistant by id
        """
        response = self.client.open(
            '/voice-assistant/personality/{personality_id}'.format(personality_id='personality_id_example'),
            method='GET')
        self.assert200(response,
                       'Response body is : ' + response.data.decode('utf-8'))

    def test_voice_assistant_personality_personality_id_put(self):
        """Test case for voice_assistant_personality_personality_id_put

        Create new voice assistant personality
        """
        body = UpdateVoiceAssistantPersonalityRequest()
        response = self.client.open(
            '/voice-assistant/personality/{personality_id}'.format(personality_id='personality_id_example'),
            method='PUT',
            data=json.dumps(body),
            content_type='application/json')
        self.assert200(response,
                       'Response body is : ' + response.data.decode('utf-8'))

    def test_voice_assistant_personality_post(self):
        """Test case for voice_assistant_personality_post

        Create new voice assistant personality
        """
        body = CreateVoiceAssistantPersonality()
        response = self.client.open(
            '/voice-assistant/personality',
            method='POST',
            data=json.dumps(body),
            content_type='application/json')
        self.assert200(response,
                       'Response body is : ' + response.data.decode('utf-8'))


if __name__ == '__main__':
    import unittest
    unittest.main()
