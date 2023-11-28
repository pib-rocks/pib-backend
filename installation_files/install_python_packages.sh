#!/bin/bash
#
# This script installs all python packages necessary for running pib

echo -e "$YELLOW_TEXT_COLOR""-- Installing python packages --""$RESET_TEXT_COLOR"		

# Package installs for voice assistant
pip install openai==0.28
pip install google-cloud-texttospeech
pip install --upgrade google-cloud-speech
sudo apt install -y python3-pyaudio
pip install SpeechRecognition

echo -e "$NEW_LINE""$GREEN_TEXT_COLOR""-- Python package installation completed --""$RESET_TEXT_COLOR""$NEW_LINE"