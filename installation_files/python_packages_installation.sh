#!/bin/bash
#
# This script installs all python packages necessary for running pib

# Package installs for voice assistant
pip install openai==0.28
pip install google-cloud-texttospeech
pip install --upgrade google-cloud-speech
sudo apt install -y python3-pyaudio
pip install SpeechRecognition