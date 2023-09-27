# coding: utf-8

from __future__ import absolute_import

from flask import json
from six import BytesIO

from swagger_server.models.create_voice_assistant_personality import CreateVoiceAssistantPersonality  # noqa: E501
from swagger_server.models.delete_voice_assistant_personality import DeleteVoiceAssistantPersonality  # noqa: E501
from swagger_server.models.error_message import ErrorMessage  # noqa: E501
from swagger_server.models.get_personalities import GetPersonalities  # noqa: E501
from swagger_server.models.successful_voice_assistant_personality_creation_response import SuccessfulVoiceAssistantPersonalityCreationResponse  # noqa: E501
from swagger_server.models.update_voice_assistant_personality import UpdateVoiceAssistantPersonality  # noqa: E501
from swagger_server.models.voice_assistant_personality import VoiceAssistantPersonality  # noqa: E501
from swagger_server.test import BaseTestCase


class TestPibDatabaseController(BaseTestCase):
    """PibDatabaseController integration test stubs"""

    def test_voice_assistant_get(self):
        """Test case for voice_assistant_get

        Get all voice-assistant personalities
        """
        response = self.client.open(
            '/voice-assistant',
            method='GET')
        self.assert200(response,
                       'Response body is : ' + response.data.decode('utf-8'))

    def test_voice_assistant_id_delete(self):
        """Test case for voice_assistant_id_delete

        Deletes voice assistant personality by id
        """
        response = self.client.open(
            '/voice-assistant/{id}'.format(id='id_example'),
            method='DELETE')
        self.assert200(response,
                       'Response body is : ' + response.data.decode('utf-8'))

    def test_voice_assistant_id_get(self):
        """Test case for voice_assistant_id_get

        Get voice assistant by id
        """
        response = self.client.open(
            '/voice-assistant/{id}'.format(id='id_example'),
            method='GET')
        self.assert200(response,
                       'Response body is : ' + response.data.decode('utf-8'))

    def test_voice_assistant_id_put(self):
        """Test case for voice_assistant_id_put

        Create new voice assistant personality
        """
        body = UpdateVoiceAssistantPersonality()
        response = self.client.open(
            '/voice-assistant/{id}'.format(id='id_example'),
            method='PUT',
            data=json.dumps(body),
            content_type='application/json')
        self.assert200(response,
                       'Response body is : ' + response.data.decode('utf-8'))

    def test_voice_assistant_post(self):
        """Test case for voice_assistant_post

        Create new voice assistant personality
        """
        body = CreateVoiceAssistantPersonality()
        response = self.client.open(
            '/voice-assistant',
            method='POST',
            data=json.dumps(body),
            content_type='application/json')
        self.assert200(response,
                       'Response body is : ' + response.data.decode('utf-8'))


if __name__ == '__main__':
    import unittest
    unittest.main()
